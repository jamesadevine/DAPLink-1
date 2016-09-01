/**
 * @file    vfs_user.c
 * @brief   Implementation of vfs_user.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stdbool.h"
#include "ctype.h"
#include "string.h"
#include "stdlib.h"

#include "vfs_manager.h"
#include "macro.h"
#include "error.h"
#include "util.h"
#include "settings.h"
#include "target_reset.h"
#include "daplink.h"
#include "IO_config.h"      // for NVIC_SystemReset
#include "version_git.h"
#include "info.h"
#include "gpio.h"           // for gpio_get_sw_reset
#include "swd_host.h"
#include "daplink_debug.h"

#include "jmx_packets.h"
#include "jmx.h"
#include "uart.h"
#include "main.h"
#include "Base64.h"
#include "microbit_util.h"

#define MINV(a,b) (a<b?a:b)

#define MICROBIT_FS_FILENAME_LEN    16

#define TABLE_WIDTH                 16

#define TABLE_SIZE                  1024/TABLE_WIDTH
#define FILENAME_LEN                16

#define UNKNOWN_SIZE_BIT            0x1000

static const UART_Configuration init_config = {
    .Baudrate = 115200,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE
};


/**
  * Micro:Bit specific VFS files:
  *  - MBITFS.HTM filesystem webpage interface.
  *  - FLASH.BIN binary representation of the filesystem.
  *  - FLASHJS.TXT javascript array of the filesystem
  *
  * Files are provided by three read file handles: read_microbit_flash_bin, 
  * read_microbit_flash_js, and read_file_microbit_htm.
  *
  * The location of the file system in flash is not known at compile time, being
  * set dynamically by the DAL. The location and size are stored in a key-value pair
  * structure, at a known page. Before anything else, these need to be established by calling 
  * microbit_get_flash_meta_data.
  *
  * board_vfs_add_files() is called in vfs_user.c provided BOARD_VFS_ADD_FILES is set,
  * and calls the relevant vfs_manager.h functions to add these files to the VFS.
  *
  */

// The location in flash of the NRF51822 key/value pair set.
#define KEY_VALUE_FLASH_LOC 0x3BC08

// The fixed size of each key/value pair entry.
#define KEY_VALUE_LEN       48

// The number of key value pairs.
#define KEY_VALUE_SIZE      21

// which translates to ~32 bytes of base 64
#define MICROBIT_MTU        24 

// The key for the flash storage location
#define KEY_STRING "MBFS_START"


// Read functions for the MicroBit VFS.
static uint32_t read_microbit_flash_bin(uint32_t sector_offset, uint8_t* data, uint32_t num_sectors);
static uint32_t read_microbit_flash_js(uint32_t sector_offset, uint8_t* data, uint32_t num_sectors);
static uint32_t read_file_microbit_htm(uint32_t sector, uint8_t *data, uint32_t n);
static uint32_t board_read_subdir(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors);
static void board_write_subdir(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors);

// MBITFS.HTM contents (in microbit_html.c)
extern char microbit_html[];

// Initialization function, to obtain the flash start (microbit_flash_start) and size (microbit_flash_size).
static int microbit_get_flash_meta(uint32_t * flash_start, uint32_t * flash_size);

static uint32_t microbit_flash_start = 0;
static uint32_t microbit_flash_size = 0;

static int first = 0;
static int entries_count = 0;

static const FatDirectoryEntry_t test_file_entry = {
    /*uint8_t[11] */ .filename = {""},
    /*uint8_t */ .attributes = 0x00,
    /*uint8_t */ .reserved = 0x00,
    /*uint8_t */ .creation_time_ms = 0x00,
    /*uint16_t*/ .creation_time = 0x0000,
    /*uint16_t*/ .creation_date = 0x4876,
    /*uint16_t*/ .accessed_date = 0x4876,
    /*uint16_t*/ .first_cluster_high_16 = 0x0000,
    /*uint16_t*/ .modification_time = 0x83dc,
    /*uint16_t*/ .modification_date = 0x4876,
    /*uint16_t*/ .first_cluster_low_16 = 0x0000,
    /*uint32_t*/ .filesize = 0x00000000
};

static volatile uint8_t initialised = 0;
static volatile uint8_t flag = 1;

#define JUNK_CLUSTER_COUNT          12

static uint16_t junk_clusters[JUNK_CLUSTER_COUNT] = { 0 };
static uint8_t junk_cluster_index = 0;

int print_flag = 0;

typedef struct LWDirectoryEntry
{
    uint8_t filename[16];
    uint8_t last_char;
    uint16_t first_cluster;
    uint32_t size;
    uint16_t last_char_count;
    //wasting one byte...
}LWDirectoryEntry_t;

typedef struct FileTransferState
{
    LWDirectoryEntry_t* currentEntry;
    uint16_t next_sector;
}FileTransferState_t;

FileTransferState_t* fts = NULL;

char version[9];

OS_MUT jmx_mutex;

#ifdef __cplusplus
extern "C" {
#endif
    
#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
    
#define BUFF_SIZE 1024
    
static char tx_rx_buffer[BUFF_SIZE];
static int tx_rx_buffer_offset = 0;
#endif    
void tx_rx_buff_write(char c)
{
#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
    if(tx_rx_buffer_offset > BUFF_SIZE-1)
        return;
    
    tx_rx_buffer[tx_rx_buffer_offset++] = c;
#endif 
}

int sync_jmx_state_track(char c)
{
    os_mut_wait(&jmx_mutex, 0xFFFF);
    
    int ret = jmx_state_track(c);
    
    os_mut_release(&jmx_mutex);
    return ret;
}

void user_putc(char c)
{
    if(print_flag)
        tx_rx_buff_write(c);
    
    uart_write_data((uint8_t*)&c,1);
}    

void jmx_packet_received(char* identifier)
{
    flag = 0;
}

void initialise_req(void* data)
{
    //uart_debug('R');
    
    JMXInitPacket* p = (JMXInitPacket*)data;
    
    if(p->enable)
    {
        initialised = 1;
        memcpy(version, p->v, 9);
    }
}
#ifdef __cplusplus
}
#endif

static uint32_t read_tx_rx_buff(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{
#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
    
    int buffer_offset = sector_offset * VFS_SECTOR_SIZE;
    int copy_amount = num_sectors * VFS_SECTOR_SIZE;
    
    if(buffer_offset >= BUFF_SIZE)
        return 0;
    
    memcpy(data + buffer_offset, tx_rx_buffer + buffer_offset, copy_amount);
    
    return copy_amount;
#else
    return 0;
#endif
}

void add_junk_cluster(int cluster_number)
{
    junk_clusters[junk_cluster_index] = cluster_number;
    junk_cluster_index = (junk_cluster_index + 1) % JUNK_CLUSTER_COUNT;
}

int is_junk_cluster(int cluster_number)
{
    for(int i = 0; i < JUNK_CLUSTER_COUNT; i++)
        if(junk_clusters[i] == cluster_number)
            return 1;
        
    return 0;
}

int wait_for_reply(int cd)
{ 
    uint8_t c[2] = { 0, 0};
	while (cd > 0 && flag)
	{
        int n = uart_read_data(c,1);
        if (n == 1)
        {
            //tx_rx_buff_write('K');
            sync_jmx_state_track(c[0]);
        }
        cd--;
	}
    
    if(cd <= 0)
        return 0;
    
    return 1;
}

void target_get_entry(int entry, DIRRequestPacket* dir)
{
    
    uart_clear_rx();
    //setup the packet
    DIRRequestPacket local_dir_send;
	memset(&local_dir_send, 0, sizeof(local_dir_send));
	local_dir_send.entry = entry;
    
	//configure the FS packet buffer to point to the user given buffer
	int ret = jmx_configure_buffer("dir", dir);
	
    if(ret < 0)
	{
        dir->entry = ret;
        return;
    }

    jmx_send("dir", &local_dir_send);
    
    if(!wait_for_reply(10000000))
        dir->entry = -4;
    
	flag = 1;
}

void target_get_file(char* filename, int size, int offset, FSRequestPacket* fsr)
{
	FSRequestPacket local_fsr_send;
    
    memset(&local_fsr_send, 0, sizeof(FSRequestPacket));

    for(int i = 0; i <  strnlen(filename, 16); i++)
        tx_rx_buff_write(filename[i]);
    
	memcpy(local_fsr_send.filename, filename, strnlen(filename, 16));
	strcpy(local_fsr_send.mode, "read");
	strcpy(local_fsr_send.format, "B64");
	local_fsr_send.offset = offset;
	local_fsr_send.len = size;
	local_fsr_send.base64 = NULL;

    //configure the FS packet buffer to point to the user given buffer
	int ret = jmx_configure_buffer("fs", fsr);
	
    if(ret < 0)
	{
        fsr->len = ret;
        return;
    }

    //print_flag = 1;
    jmx_send("fs", &local_fsr_send);
    print_flag = 0;
    
    if(!wait_for_reply(10000000))
        fsr->len = -4;
    
	flag = 1;
}

void target_write_file(char* filename, int size, int offset, char* base64, FSRequestPacket* fsr)
{
	FSRequestPacket local_fsr_send;
    
    memset(&local_fsr_send,0,sizeof(FSRequestPacket));

	memcpy(local_fsr_send.filename,filename, strnlen(filename, 16));
	strcpy(local_fsr_send.mode, "w");
	strcpy(local_fsr_send.format, "B64");
	local_fsr_send.offset = offset;
	local_fsr_send.len = size;
	local_fsr_send.base64 = base64;

    //configure the FS packet buffer to point to the user given buffer
	int ret = jmx_configure_buffer("fs", fsr);
	
    if(ret < 0)
	{
        fsr->len = ret;
        return;
    }
    
    //print_flag = 1;
    jmx_send("fs", &local_fsr_send);
    print_flag = 0;
    
    if(!wait_for_reply(10000000))
        fsr->len = -4;
    
	flag = 1;
}

/**
  * called in vfs_user.c: vfs_build().
  *
  * Adds the files specific to the microbit.
  * Requires that BOARD_VFS_ADD_FILES
  */
void board_vfs_add_files() {
 
    int cd = 2000000;
    main_blink_cdc_led(MAIN_LED_OFF);
    jmx_init();
    swd_init();
    
    //initialise our mutex, incase our thread is paged out.
    os_mut_init(&jmx_mutex);
    
    //reset the microbit, setup the initalisation of UART
    target_set_state(RESET_HOLD);
    uart_initialize();
    uart_set_configuration((UART_Configuration*)&init_config);
    uart_clear_rx();
    target_set_state(RESET_RUN);
    swd_off();
    
    //tx_rx_buff_write('S');
    
    main_blink_cdc_led(MAIN_LED_FLASH);
    
    // wait for our init packet, if any.
    while(cd > 0 && !initialised)
    {
        
        uint8_t c[2] = { 0, 0};
        int res = uart_read_data(c,1);
        //uart_debug(c);
        if(res == 1)
        {
            //if this returns one, we break, jmx should not be used.
            if(sync_jmx_state_track(c[0]))
                break;
        }
        
        cd--;
    }
    
    main_blink_cdc_led(MAIN_LED_OFF);
    flag = 1;
    
    //uart_debug('I');
    
#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
    memset(tx_rx_buffer, 0, BUFF_SIZE);
    vfs_create_file("RXTX    TXT", read_tx_rx_buff, 0, BUFF_SIZE);
#endif   
    
    if(initialised)
    {
        vfs_create_subdir("FILES      ", 85, board_read_subdir, board_write_subdir);
        //created = true;
        
        if(strncmp(version,"0.0.0",5) == 0)
        {
            if(microbit_get_flash_meta(&microbit_flash_start, &microbit_flash_size)) {
                microbit_flash_size *= 1024;
                vfs_create_file("FLASH   BIN", read_microbit_flash_bin, 0, microbit_flash_size);
                vfs_create_file("FLASHJS TXT", read_microbit_flash_js, 0, (microbit_flash_size*5)+12);

                int file_size = strlen(microbit_html);
                vfs_create_file("MBITFS  HTM", read_file_microbit_htm, 0, file_size);
            } 
            
            initialised = 0;
        }
        else
        {
            memset(microbit_page_buffer, 0, MICROBIT_PAGE_BUFFER_SIZE);
            //uart_debug('P');
            DIRRequestPacket dir;
            
            bool created = false;
            bool done = false;
            
            while (!done)
            {
                target_get_entry(entries_count, &dir);
                
                if (dir.entry < 0)
                    done = true;
                else
                {
                    if(!created)
                    {
                        // a maximum of 85 entries, allocate our cluster, add our hooks
                        //vfs_create_subdir("FILES      ", 85, board_read_subdir, board_write_subdir);
                        //created = true;
                    }
                    
                    for(int i = 0; i < 16; i++)
                    {
                        
                        tx_rx_buff_write(dir.filename[i]);
                        tx_rx_buff_write('[');
                        tx_rx_buff_write('0' + (((dir.filename[i])/10)%10));
                        tx_rx_buff_write('0' + (((dir.filename[i]))%10));
                        tx_rx_buff_write(']');
                        if(dir.filename[i] == 0)
                            break;
                    }
                    LWDirectoryEntry_t lw;
                    memset(&lw, 0, sizeof(LWDirectoryEntry_t));
                    memcpy(lw.filename, dir.filename, MIN(strlen(dir.filename),16));
                    lw.size = dir.size;
                    lw.first_cluster = write_clusters((dir.size/VFS_CLUSTER_SIZE) + 1);
                    
                    if(entries_count == 0)
                        first = lw.first_cluster;
                    
                    memcpy(microbit_page_buffer + (entries_count * sizeof(LWDirectoryEntry_t)), &lw, sizeof(LWDirectoryEntry_t));
                    
                    entries_count++;
                }
            }
            tx_rx_buff_write('0' + entries_count);
        }
    }
}

/*
* cache meta-data: filename, cluster.
* READ_ONLY
* creates are allowed
* delete followed by a read, cause re-enum
*
* WRITE MISSING BYTES - CHECK CODE!!

* GIT COMMIT!!!!
*
* translate sector to cluster, then look to see if it is within array...
*/

void transform_name(char* orig, char* dest)
{
    int orig_len = strnlen(orig,16);
    memset(dest, ' ', 11);
    
    int orig_offset = 0;
    int dest_offset = 0;
    while(orig_offset < orig_len && orig[orig_offset] != '.' && dest_offset < 8)
    {
        if (orig[orig_offset] != ' ')   
            dest[dest_offset++] = orig[orig_offset];
        
        orig_offset++;
    }

    int basis_len = dest_offset;
    
    memcpy(dest,orig,dest_offset);
    
    int extension = 0;
    
    for(int i = 0; i < orig_len; i++)
        if(orig[i] == '.')
        {
            extension = i;
            break;
        }
        
    if(extension)
    {
        dest_offset = 8;
        orig_offset = extension + 1;
        while(orig_offset < orig_len && dest_offset < 11 && orig[orig_offset] != 0)
        {
            dest[dest_offset++] = orig[orig_offset];
            orig_offset++;
        }
    }
        
    int name_count = 0;
    
    // I should really compare against the transformed name, but this could take a long time...
    // i will always end up > 1
    for(int i = 0; i < entries_count; i++)
    {
        LWDirectoryEntry_t* lw = ((LWDirectoryEntry_t*)microbit_page_buffer) + i;
        if(strncmp((char *)lw->filename,orig, MIN(basis_len, strnlen((char*)lw->filename, 16))) == 0)
            name_count++;
    }
    
    if(name_count > 1)
    {
        dest[6] = '~';
        if(name_count > 9)
            dest[5] ='~';
    
        dest[7] = '0' + (name_count % 9);
    }
}

int retrieve_dirent(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{   
    //tx_rx_buff_write('D');
    
}

//intercept????
static uint32_t board_read_subdir(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors){
    //debug_msg("read %d %d\n", sector_offset, num_sectors);
    memset(data,0,VFS_SECTOR_SIZE * num_sectors);
    
    int size = 0;
    FatDirectoryEntry_t de;
    memcpy(&de, &test_file_entry, sizeof(FatDirectoryEntry_t));
    de.attributes = VFS_FILE_ATTR_READ_ONLY; 
    
    uint32_t dirs_per_sector = VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t); 
    int dir_offset = dirs_per_sector * sector_offset;
    
    for(int i = dir_offset; i < dir_offset + dirs_per_sector; i++)
    {
        if(i >= entries_count)
            break;
        
        LWDirectoryEntry_t* lw = ((LWDirectoryEntry_t*)microbit_page_buffer) + i;
        
        //tx_rx_buff_write('V');
        memset(de.filename,' ',11);
        
        //for(int i = 0; i < 16; i++)
        //    tx_rx_buff_write(lw->filename[i]);
        
        transform_name((char*)lw->filename,de.filename);
        
        de.filesize = lw->size;
            
        uint32_t cluster = (uint32_t)lw->first_cluster;
            
        de.first_cluster_high_16 = cluster >> 16;
        de.first_cluster_low_16 = cluster >> 0;
        
        memcpy(data + size, &de, sizeof(FatDirectoryEntry_t));
        size += sizeof(FatDirectoryEntry_t);
    }
    
    return size;
}

LWDirectoryEntry_t* getEntry(uint32_t requested_cluster)
{ 
    LWDirectoryEntry_t* lw;
    LWDirectoryEntry_t* file_transfer = NULL;
    
    for(int i = 0; i < entries_count; i++)
    {
        lw = ((LWDirectoryEntry_t*)microbit_page_buffer) + i;
        
        if(!file_transfer && lw->first_cluster & UNKNOWN_SIZE_BIT)
            file_transfer = lw;
        
        uint32_t clust = lw->first_cluster & ~0xF000;
        
        uint32_t last_cluster = clust + ((lw->size / VFS_CLUSTER_SIZE));
        
        //if the requested sector is between this entries first cluster and last cluster...
        if(requested_cluster >= clust && requested_cluster  <= last_cluster)
            return lw;
    }
    
    return file_transfer;
}

LWDirectoryEntry_t* getEntryByShortName(const char* name)
{ 
    LWDirectoryEntry_t* lw;
    
    char given_name[12];
    memcpy(given_name, name, 11);
    given_name[11] = 0;
    
    for(int i = 0; i < entries_count; i++)
    {
        lw = ((LWDirectoryEntry_t*)microbit_page_buffer) + i;
        
        char name_buf[12];
        memset(name_buf, 0, 12);
        
        transform_name((char*)lw->filename,name_buf);
        
        if(strcmp(name_buf, given_name) == 0)
            return lw;
    }
    
    return NULL;
}

int board_vfs_read(uint32_t requested_sector, uint8_t *buf, uint32_t num_sectors)
{    
    if(!initialised)
        return 0;
    
    //get the usage of the DAPLink VFS.
    uint32_t total_vfs_sectors = vfs_get_virtual_usage() / VFS_SECTOR_SIZE;
    
    //calculates the cluster offset into our file system
    int microbit_fs_off = (requested_sector - total_vfs_sectors) / 8;
    
    //calculates the block offset into the file
    int microbit_block_offset = (requested_sector - total_vfs_sectors) % 8;
    
    // find correct entry
    LWDirectoryEntry_t* entry = getEntry(first + microbit_fs_off); 
    
    if(entry == NULL)
        return 0;
    
    int bytes_read = 0;
    int cluster_offset = ((first + microbit_fs_off) - entry->first_cluster) * VFS_CLUSTER_SIZE;
    
    int byte_offset = cluster_offset + (microbit_block_offset * VFS_SECTOR_SIZE);
    int read_end = VFS_SECTOR_SIZE * num_sectors;
    
    while((byte_offset + bytes_read) < entry->size && bytes_read < read_end)
    {
        FSRequestPacket fsr;
        int desired_bytes = MIN(MICROBIT_MTU,MIN(read_end - bytes_read, entry->size - (byte_offset + bytes_read)));
        
        tx_rx_buff_write('*');
        tx_rx_buff_write('0' + (((byte_offset + bytes_read)/100)%10));
        tx_rx_buff_write('0' + (((byte_offset + bytes_read)/10)%10));
        tx_rx_buff_write('0' + (((byte_offset + bytes_read))%10));
        tx_rx_buff_write('/');
        tx_rx_buff_write('0' + (((desired_bytes)/10)%10));
        tx_rx_buff_write('0' + (((desired_bytes))%10));
        tx_rx_buff_write('*');
        
        target_get_file((char*)entry->filename, MIN(MICROBIT_MTU, desired_bytes), (byte_offset + bytes_read), &fsr);
        
        if(fsr.len <= 0)
            break;
        
        int data_len = base64_dec_len(fsr.base64, fsr.len);
        
        char ret_buf[data_len + 1];
        base64_decode(ret_buf, fsr.base64 , fsr.len); 
        
        free(fsr.base64);
        fsr.base64 = NULL;
        
        data_len = MIN(desired_bytes, data_len);
        
        memcpy(buf + bytes_read, ret_buf, data_len);
        
        bytes_read += data_len;
    }
    
    
    return bytes_read;
}


int base64_send(char* filename, uint32_t offset, uint8_t* data_raw, uint32_t data_raw_size)
{
    FSRequestPacket fsr;
    
    int data_len = base64_enc_len(data_raw_size);
    char base_buf[data_len + 1];

    memset(base_buf,0,data_len + 1);
    base64_encode(base_buf, (char*)data_raw, data_raw_size);
    
    target_write_file(filename, data_len, offset, base_buf, &fsr);

    int cd = 100000;

    while(cd > 0)
        cd--;

    if(fsr.len < 0)
    {
        tx_rx_buff_write('-');
        int pos_ret = -fsr.len;

        tx_rx_buff_write('0' + ((pos_ret/100)%10));
        tx_rx_buff_write('0' + ((pos_ret/10)%10));
        tx_rx_buff_write('0' + ((pos_ret)%10));
    }
}

int board_vfs_write(uint32_t requested_sector, uint8_t *buf, uint32_t num_sectors)
{
    if(!initialised)
        return 0;
    
    //get the usage of the DAPLink VFS.
    uint32_t total_vfs_sectors = vfs_get_virtual_usage() / VFS_SECTOR_SIZE;
    
    //calculates the cluster offset into our file system
    int microbit_fs_off = (requested_sector - total_vfs_sectors) / 8;
    
    //calculates the block offset into the file
    int microbit_block_offset = (requested_sector - total_vfs_sectors) % 8;
    
    if(is_junk_cluster(first + microbit_fs_off))
    {
        //tx_rx_buff_write('R');
        return 0;
    }
    // find correct entry
    LWDirectoryEntry_t* entry = NULL;
    
    if(fts)
    {
        // first time processing this file transfer request.
        if(requested_sector == fts->next_sector || fts->next_sector == 0)
        {
            
            // track our sector for next time
            fts->next_sector = requested_sector + 1;
            entry = fts->currentEntry;
        }
        else if(requested_sector != fts->next_sector)
        {
            if(fts->currentEntry->size > 0)
            {
                // TODO:
                //we should probably delete the entry as well... and decrement the entry count;
                //re-enumerate?
            }
            tx_rx_buff_write('O');
           
            free(fts);
            fts = NULL;
        }

        
        tx_rx_buff_write('I');
        if(requested_sector >= 100)
            tx_rx_buff_write('0' + ((requested_sector/100)%10));
        tx_rx_buff_write('0' + ((requested_sector/10)%10));
        tx_rx_buff_write('0' + (((requested_sector))%10));
    }
    else
        entry = getEntry(first + microbit_fs_off);
    
    if(entry == NULL)
    {
        //tx_rx_buff_write('0' + (((first + microbit_fs_off)/10)%10));
        //tx_rx_buff_write('0' + (((first + microbit_fs_off))%10));
        //tx_rx_buff_write('A');
        add_junk_cluster(first + microbit_fs_off);
        return 0;
    }
    
    //if we haven't yet got a size, we also probably don't have a first_cluster...
	if ((entry->first_cluster & ~UNKNOWN_SIZE_BIT) == 0)
    {
		entry->first_cluster = first + microbit_fs_off | UNKNOWN_SIZE_BIT;
    }
	
	int bytes_written = 0;
    
    int cluster_offset = ((first + microbit_fs_off) - entry->first_cluster) * VFS_CLUSTER_SIZE;
    
	int byte_offset = cluster_offset + (microbit_block_offset * VFS_SECTOR_SIZE);
	int write_end = VFS_SECTOR_SIZE * num_sectors;
	int bytes_parsed = 0;
    
	if (entry->first_cluster & UNKNOWN_SIZE_BIT)
	{
		int write_end = VFS_SECTOR_SIZE * num_sectors;

		char data_buf[MICROBIT_MTU];
		int data_buf_offset = 0;
		memset(data_buf, 0, MICROBIT_MTU);

		while (bytes_parsed < write_end)
		{	
			if (entry->last_char == buf[bytes_parsed])
			{
				entry->last_char_count++;
				bytes_parsed++;
			}
			else
			{
				//there's a difference, flush the last last_char_count characters.
				while (entry->last_char_count > 0)
				{
					int character_count = MIN(entry->last_char_count, MICROBIT_MTU - data_buf_offset);

					memset(data_buf + data_buf_offset, entry->last_char, character_count);

					data_buf_offset += character_count;

					// if we have filled the buffer, send it....
					if (data_buf_offset == MICROBIT_MTU)
					{

                        base64_send((char *)entry->filename, entry->size, (uint8_t *)data_buf, data_buf_offset);
                        
                        bytes_written += data_buf_offset;
                        
                        entry->size += data_buf_offset;  
                        
						memset(data_buf, 0, MICROBIT_MTU);
						data_buf_offset = 0;
					}

					entry->last_char_count -= character_count;
				}

				entry->last_char = buf[bytes_parsed++];
				entry->last_char_count = 1;
			}
		}
		
		if (data_buf_offset > 0)
		{
            base64_send((char *)entry->filename, entry->size, (uint8_t *)data_buf, data_buf_offset);
			entry->size += data_buf_offset;
		}
        
        return bytes_parsed;
	}
    else
    {
        tx_rx_buff_write('W');
        if(requested_sector >= 100)
            tx_rx_buff_write('0' + ((requested_sector/100)%10));
        tx_rx_buff_write('0' + ((requested_sector/10)%10));
        tx_rx_buff_write('0' + (((requested_sector))%10));
        
        while((byte_offset + bytes_written) < entry->size && bytes_written < write_end)
        {
            int tx_size = MIN(MICROBIT_MTU, entry->size - bytes_written);
            
            tx_rx_buff_write('*');
            tx_rx_buff_write('0' + (((bytes_written)/100)%10));
            tx_rx_buff_write('0' + (((bytes_written)/10)%10));
            tx_rx_buff_write('0' + (((bytes_written))%10));
            tx_rx_buff_write('*');
           
            for(int i = bytes_written; i < bytes_written + tx_size; i++)
                tx_rx_buff_write(buf[i]);
            
            base64_send((char*) entry->filename, (byte_offset + bytes_written), buf + bytes_written, tx_size);
            bytes_written += tx_size;
        }
        
        return bytes_written;
    }
}

//intercept???
static void board_write_subdir(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors){
    FatDirectoryEntry_t *new_entry = (FatDirectoryEntry_t *)data;
   
    uint32_t dirs_per_sector = VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t); 

    int entry_offset = dirs_per_sector * sector_offset;
    
    /*tx_rx_buff_write('P');
    tx_rx_buff_write('0' + ((entry_offset / 10)%10));
    tx_rx_buff_write('0' + ((entry_offset)%10));
    tx_rx_buff_write('P');*/
    
    //if(entry_offset > entries_count)
    //{
        //tx_rx_buff_write('Q');    
    //    return;
    //}
    
    int validated_entries = 0;
    
    uint32_t num_entries = num_sectors * VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t);
    
    while(entry_offset < num_entries)
    {
        FatDirectoryEntry_t current_entry = new_entry[entry_offset];
        
        if(current_entry.attributes & VFS_FILE_ATTR_LONG_NAME)
        {
            tx_rx_buff_write(':');
            entry_offset++;
            continue;
        }
        
        uint32_t cluster = ((uint32_t)current_entry.first_cluster_high_16 << 16) |  current_entry.first_cluster_low_16;
        
        if(0xe5 == (uint8_t)current_entry.filename[0])
        {
            tx_rx_buff_write('D');
            tx_rx_buff_write('0' + ((cluster / 100)%10));
            tx_rx_buff_write('0' + ((cluster / 10)%10));
            tx_rx_buff_write('0' + ((cluster)%10));
            entry_offset++;
            continue;
        }
        
        if(!current_entry.attributes)
        {
            tx_rx_buff_write('@');
            break;
        }

        
        for(int j = 0; j < 11; j++)
        {
            tx_rx_buff_write(current_entry.filename[j]);
            tx_rx_buff_write('0' + ((current_entry.filename[j] / 10)%10));
            tx_rx_buff_write('0' + ((current_entry.filename[j])%10));
        }
        
        LWDirectoryEntry_t* lw = getEntryByShortName(current_entry.filename);
        
        // This is a new entry...
        if(!lw)
        {
            tx_rx_buff_write('0' + ((current_entry.filesize / 100)%10));
            tx_rx_buff_write('0' + ((current_entry.filesize / 10)%10));
            tx_rx_buff_write('0' + ((current_entry.filesize)%10));
            tx_rx_buff_write('-');
            tx_rx_buff_write('0' + ((cluster / 100)%10));
            tx_rx_buff_write('0' + ((cluster / 10)%10));
            tx_rx_buff_write('0' + ((cluster)%10));
            
            LWDirectoryEntry_t new_lw;
            
            memset(&new_lw, 0, sizeof(LWDirectoryEntry_t));
            
            tx_rx_buff_write('E');
            //new SHORT dirent, capture the filename and transform it to 8-[period]-3.
            int filename_offset = 0;
       
            for(int j = 0; j < 11; j++)
            {
                if(current_entry.filename[j] == 0)
                    break;
                
                if(j == 8 && current_entry.filename[j] != ' ')
                    new_lw.filename[filename_offset++] = '.';
                
                if(current_entry.filename[j] != ' ')
                    new_lw.filename[filename_offset++] = current_entry.filename[j];
            }
            
            for(int i = 0 ; i < filename_offset; i++)
                tx_rx_buff_write(new_lw.filename[i]);
            
            new_lw.size = current_entry.filesize;
            new_lw.first_cluster = cluster;
            
            if(new_lw.size == 0)
                new_lw.first_cluster |= UNKNOWN_SIZE_BIT;
            
            LWDirectoryEntry_t* dest = (LWDirectoryEntry_t*)(microbit_page_buffer + (entries_count * sizeof(LWDirectoryEntry_t)));
            
            memcpy(dest, &new_lw, sizeof(LWDirectoryEntry_t));
            
            // account for MACOS
            if(!cluster)
            {
                //if we don't have a cluster and our file transfer state is not in progress.
                if(!fts)
                {
                    fts = malloc(sizeof(FileTransferState_t));
                    memset(fts, 0, sizeof(FileTransferState_t));
                    
                    fts->currentEntry = dest;
                    fts->next_sector = 0;
                }
                else
                {
                    // blow up, File transfer in progress.
                }
            }
            
            entries_count++;
            
            tx_rx_buff_write('L');
        }
        else
        {   
            //if we don't know the filesize, update the filesize. That is all.
            if(lw->first_cluster & UNKNOWN_SIZE_BIT)
            {
                if(current_entry.filesize)
                {
                    tx_rx_buff_write('U');
                    tx_rx_buff_write('0' + ((current_entry.filesize / 100)%10));
                    tx_rx_buff_write('0' + ((current_entry.filesize / 10)%10));
                    tx_rx_buff_write('0' + ((current_entry.filesize)%10));
                    tx_rx_buff_write('C');
                    tx_rx_buff_write('0' + ((cluster / 100)%10));
                    tx_rx_buff_write('0' + ((cluster / 10)%10));
                    tx_rx_buff_write('0' + ((cluster)%10));
                    
                    // if our optimisation was too harsh, we need to write the appropriate amount bytes to the file
                    // now we know the size...
                    if(lw->size < current_entry.filesize)
                    {
                        int bytes_to_write = current_entry.filesize - lw->size;
                        
                        char data_buf[MICROBIT_MTU];
                        memset(data_buf, 0, MICROBIT_MTU);
                        
                        while(bytes_to_write)
                        {
                            int chars_to_write = MIN(MICROBIT_MTU, bytes_to_write);
                            memset(data_buf, lw->last_char, chars_to_write);
                            
                            base64_send((char*)lw->filename, lw->size, (uint8_t*)data_buf, chars_to_write);
                        
                            lw->size += chars_to_write;
                            bytes_to_write -= chars_to_write;
                        }
                        
                        lw->last_char = 0;
                        lw->last_char_count = 0;
                    }
                    
                    lw->size = current_entry.filesize;
                    lw->first_cluster &= ~UNKNOWN_SIZE_BIT;
                    
                    if(fts)
                    {
                        free(fts);
                        fts = NULL;
                    }
                }
            }
            // for some inconvenient reason, mac OS likes to move things around...
            else if(lw->first_cluster != cluster)
            {
                lw->first_cluster = cluster;
                tx_rx_buff_write('M');
                tx_rx_buff_write('0' + ((cluster / 10)%10));
                tx_rx_buff_write('0' + ((cluster)%10));
            }
        }
        entry_offset++;
        validated_entries++;
    }
}

/**
  * Function to initialize the microbit_flash_start and microbit_flash_size internal variables.
  * These must be set before any other functions can be called.
  *
  * These values are set by the MicroBit DAL in the keyvalue store.
  *
  * @param flash_start pointer to the start of flash location variable to set.
  * @param flash_size pointer to the size of flash variable to set.
  * @return non-zero on error.
  */
static int microbit_get_flash_meta(uint32_t * flash_start, uint32_t * flash_size) {

    swd_init();
    target_set_state(RESET_HOLD);

    static uint8_t buffer[48];

    for(int i=0;i<KEY_VALUE_SIZE;i++) {
        swd_read_memory(KEY_VALUE_FLASH_LOC, buffer, KEY_VALUE_LEN);

        if(strncmp((char*)((uint8_t*)buffer), KEY_STRING, strlen(KEY_STRING)) != 0) {
            continue;
        }
    
        memcpy(flash_start, ((uint8_t*)buffer)+16, sizeof(uint32_t));
        memcpy(flash_size, ((uint8_t*)buffer)+20, sizeof(uint32_t));

        swd_off();
        target_set_state(RESET_RUN);

        return 1;
    }

    swd_off();
    target_set_state(RESET_RUN);

    return 0;
}

/**
  * Callback read Function to read the microbit flash contents as a raw binary file.
  * 
  * @param sector_offset sector offset to read from.
  * @param data buffer to read into.
  * @param num_sectors number of sectors to read.
  * @return number of bytes read.
  */
static uint32_t read_microbit_flash_bin(uint32_t sector_offset, uint8_t* data, uint32_t num_sectors) {
    
    swd_init();
    target_set_state(RESET_HOLD);
   
    uint32_t loc = microbit_flash_start + (VFS_SECTOR_SIZE * sector_offset);

    swd_read_memory(loc, data, (num_sectors * VFS_SECTOR_SIZE)); 
    
    swd_off();

    target_set_state(RESET_RUN);
    
    return (VFS_SECTOR_SIZE * num_sectors);
}

/**
  * Callback read function to read the microbit flash contents as a javascript array.
  * Each byte is represented as "0x??," - 5 bytes. 
  * The array is prepended with "var fl = [" - 10 bytes.
  * The array is appended with "];" - 2 bytes.
  *
  * The total size of the javascript file is 5n+12 (n = flash size).
  *
  * @param sector_offset sector offset to read from.
  * @param data buffer to read into
  * @param num_sectors number of sectors to read.
  * @return number of bytes read.
  */
static uint32_t read_microbit_flash_js(uint32_t sector, uint8_t* data, uint32_t num_sectors) {
  
    swd_init();
    target_set_state(RESET_HOLD);

    int max_file_size = (microbit_flash_size*5)+12; 

    int buffer_offset = 0; 

    for(int k=0; k < num_sectors && (sector * VFS_SECTOR_SIZE) < max_file_size; k++, sector++) {

        int wr = 0;

        int write = MINV( VFS_SECTOR_SIZE, 
                          (max_file_size - (sector * VFS_SECTOR_SIZE)) ) - 2;

        // Flash address to read from.
        int addr = microbit_flash_start + (sector * 102) +
                   (sector == 0 ? 0 : -2);

        if(sector == 0) {   
            
            // start of file, print header.
            sprintf((char*)data, "var fl = [");
            wr += 10;
            buffer_offset += 10;
        }

        for(; wr < write; wr += 5) {

            // Copy each byte from flash, print to *data

            uint8_t byte = 0;
            swd_read_memory(addr++, &byte, 1);

            sprintf((char*)&data[buffer_offset], "0x%02x,", byte);
            buffer_offset+=5;
        }

        if(sector != (max_file_size / VFS_SECTOR_SIZE)) {
            sprintf((char*)&data[buffer_offset], " \n");
            buffer_offset+=2;
        }
        else {

            // EOF, print array end.
            sprintf((char*)&data[buffer_offset-1], " ];");
            buffer_offset+=2;
        }
    }

    target_set_state(RESET_RUN);
    swd_off();

    // buffer_offset is the no. bytes written.
    return buffer_offset;
}

/**
  * Callback read function to read the MBITFS.HTM file
  *
  * Reads from microbit_html array.
  * @param sector offset sector to read from
  * @param data buffer to read into
  * @param n number of sectors to read
  * @return number of bytes read.
  */
static uint32_t read_file_microbit_htm(uint32_t sector, uint8_t *data, uint32_t n)
{
    uint32_t offset = sector * VFS_SECTOR_SIZE;
    int len = MIN( (n*VFS_SECTOR_SIZE) , (strlen(microbit_html) - offset) );
    if(len < 0) return 0;

    memcpy(data, microbit_html+offset, len);
    return len;
}
