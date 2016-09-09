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

#include "tasks.h"
#include "vfs_manager.h"
#include "virtual_fs.h"
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
#define FILE_DELETED_BIT            0x2000


static const UART_Configuration init_config = {
    .Baudrate = 115200,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE
};

#define MICROBIT_JMX_SUCCESS        1
#define MICROBIT_JMX_REPEAT         0
#define MICROBIT_JMX_ERROR          -1
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
static uint32_t read_file_fs_error(uint32_t sector, uint8_t *data, uint32_t n);
static uint32_t read_microbit_flash_js(uint32_t sector_offset, uint8_t* data, uint32_t num_sectors);
static uint32_t read_file_microbit_htm(uint32_t sector, uint8_t *data, uint32_t n);
static uint32_t board_read_subdir(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors);
static void board_write_subdir(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors);

// MBITFS.HTM contents (in microbit_html.c)
extern char microbit_html[];

extern uint8_t usb_tx_flag;

// Initialization function, to obtain the flash start (microbit_flash_start) and size (microbit_flash_size).
static int microbit_get_flash_meta(uint32_t * flash_start, uint32_t * flash_size);

static uint32_t microbit_flash_start = 0;
static uint32_t microbit_flash_size = 0;



static int first = 0;
/*static*/ int entries_count = 0;

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

#define JMX_STATUS_PACKET           0x01
#define JMX_INIT_PACKET             0x02
#define JMX_DIR_PACKET              0x04
#define JMX_WINDOW_RET              0x08

#define BOARD_VFS_STATE_BOOT        0x01
#define BOARD_VFS_STATE_INVALID     0x02
#define BOARD_VFS_STATE_FLASH       0x04
#define BOARD_VFS_STATE_INIT        0x08

#define BOARD_VFS_ERROR_IDX_MULTI   1
#define BOARD_VFS_ERROR_IDX_DEL     2
#define BOARD_VFS_ERROR_IDX_FULL    3
#define BOARD_VFS_ERROR_IDX_CONF    4

#define MSC_SPIN_TIMEOUT            10000000
#define USB_PROCESS_TIMEOUT_MS      100
#define TIMEOUT_SCALAR               5

#define WINDOW_RET_ACK              1
#define WINDOW_RET_RETRY            2
#define WINDOW_RET_CANCEL           3

/*static*/ uint8_t board_vfs_state = BOARD_VFS_STATE_BOOT;

static const char* microbit_board_errors[4] = {
    "Sorry - I can't handle writing more than one file at a time from this computer.",
    "That file copy didn't seem to work - try again?",
    "That was one byte too many - I'm really full!",
    "I don't what was wrong with me, but I feel better now :)"
};

/*static*/ uint8_t jmx_flag = 0;

#define JUNK_CLUSTER_COUNT          12

static uint16_t junk_clusters[JUNK_CLUSTER_COUNT] = { 0 };
static uint8_t junk_cluster_index = 0;

uint8_t window_ret_status = 0;

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

extern OS_TID main_task_id;
extern OS_TID serial_task_id;



char version[9];

OS_MUT jmx_mutex;

#ifdef __cplusplus
extern "C" {
#endif
    
#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
    
#define BUFF_SIZE 512
    
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
    uart_write_data((uint8_t*)&c,1);
}    

void jmx_packet_received(char* identifier)
{
    char ident[3] = {identifier[0],identifier[1],identifier[2]};
    
    if(strcmp(identifier, "status") == 0)
        jmx_flag |= JMX_STATUS_PACKET;
    
    if(strcmp(identifier, "dir") == 0)
        jmx_flag |= JMX_DIR_PACKET;
    
    if(strcmp(identifier, "stat_ack") == 0)
    {
        window_ret_status = WINDOW_RET_ACK;
        jmx_flag |= JMX_WINDOW_RET;
    }
    if(strcmp(identifier, "stat_retry") == 0)
    {
        window_ret_status = WINDOW_RET_RETRY;
        jmx_flag |= JMX_WINDOW_RET;
    }
    
    if(strcmp(identifier, "stat_cancel") == 0)
    {
        window_ret_status = WINDOW_RET_CANCEL;
        jmx_flag |= JMX_WINDOW_RET;
    }
}
    
void initialise_req(void* data)
{
    JMXInitPacket* p = (JMXInitPacket*)data;
    
    if(p->enable)
    {
        board_vfs_state |= BOARD_VFS_STATE_INIT;
        memcpy(version, p->v, 9);
    }else
        board_vfs_state &= ~BOARD_VFS_STATE_INIT;
    
    
    // this is a weird one, we could receive a valid jmx init packet after a boot or a flash, but also during normal operation.
    if(board_vfs_state & (BOARD_VFS_STATE_FLASH | BOARD_VFS_STATE_BOOT))
        jmx_flag |= JMX_INIT_PACKET;
}

void uart_req(void* data)
{
    UARTConfigPacket* p = (UARTConfigPacket*)data;
 
    UART_Configuration config = init_config;
    
    config.Baudrate = p->baud;
   
    uart_set_configuration(&config);
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

void reset_junk_clusters(void)
{
    for(int i = 0; i < JUNK_CLUSTER_COUNT; i++)
        junk_clusters[i] = 0;
}

int wait_for_reply(uint8_t bit, bool shorter)
{ 
    uint8_t c = 0;
    
    int cd = MSC_SPIN_TIMEOUT;
    
    bool usb = false;
    
    if(serial_task_id)
    {
        usb = true;
        cd = 10;//USB_PROCESS_TIMEOUT_MS;
    }
    
    if(shorter)
        cd = cd / TIMEOUT_SCALAR;
    
    
    
	while (cd > 0 && !(jmx_flag & bit))
	{
        // check if our usb serial task is up and running yet, if not process here
        if(!serial_task_id)
        {
            if (uart_read_data(&c,1))
                jmx_parse(c);
        }
        else
        {
            //if we've moved from spinning, to usb serial processing uart
            if(!usb)
            {
                usb = true;
                cd = USB_PROCESS_TIMEOUT_MS;
            }
            
            os_tsk_prio(serial_task_id, MAIN_TASK_PRIORITY);
            os_tsk_prio(main_task_id, SERIAL_TASK_PRIORITY);
            os_dly_wait(0);
        }
        cd--;
	}
    
    jmx_flag &= ~bit;
    
    if(cd <= 0)
        return 0;
    
    return 1;
}

void usb_tx_block()
{
    usb_tx_flag = 0;
}

void usb_tx_resume()
{
    usb_tx_flag = 1;
}

int jmx_file_request(char* filename, int size, int offset, char mode, bool status_set, int window, uint8_t* buf)
{
    FSRequestPacket local_fsr_send;
    
    memset(&local_fsr_send, 0, sizeof(FSRequestPacket));

	memcpy(local_fsr_send.filename, filename, strnlen(filename, 16));
    
    char mode_formatted[2] = {mode,0};
	strcpy(local_fsr_send.mode, mode_formatted);
	strcpy(local_fsr_send.format, "BIN");
	local_fsr_send.offset = offset;
	local_fsr_send.len = size;
    
    if(status_set)
    {
        StatusPacket status;
        memset(&status, 0, sizeof(StatusPacket));
        status.code = 1;
        status.window = window;
        status.receipt = buf;

        jmx_configure_buffer("status", &status);
    }

    // gain absolute control over the hardware uart
    usb_tx_block();
    
    jmx_send("fs", &local_fsr_send);
    
    if(status_set)
        wait_for_reply(JMX_STATUS_PACKET, false);
    
    return 1;//status.code;
}

#define TOTAL_BUFFER_SIZE 36
#define EFFECTIVE_BUFF_SIZE 32

int target_get_file(char* filename, int size, int offset, uint8_t* buf)
{
	int ret = jmx_file_request(filename, size, offset, 'r', true, size, buf);
    
    usb_tx_resume();
    
    if(ret == MICROBIT_JMX_SUCCESS)
        return size;
    
    return 0;
}

int target_write_buf(char* filename, int size, int offset, uint8_t* buf)
{
	jmx_file_request(filename, size,offset,'w', false, TOTAL_BUFFER_SIZE, NULL);
    
    int response = -1;
    
    int ret = wait_for_reply(JMX_WINDOW_RET, false);
    
    if(ret)
        response = window_ret_status;
    
    uint32_t byte_count = 0;
    
    if(response == WINDOW_RET_ACK)
    {
        uint8_t tx_buf[TOTAL_BUFFER_SIZE];
        
        while(byte_count < size)
        {
            
            int tx_size = MIN(EFFECTIVE_BUFF_SIZE, size - byte_count);
            
            memset(tx_buf,0,TOTAL_BUFFER_SIZE);
            
            uint32_t *checksum = (uint32_t *)&tx_buf[EFFECTIVE_BUFF_SIZE];
            
            for(int i = 0; i < tx_size; i++)
            {
                tx_buf[i] = *(buf + byte_count + i);
                *checksum += tx_buf[i];
            }

            uart_write_data(tx_buf, TOTAL_BUFFER_SIZE);
            
            if(!wait_for_reply(JMX_WINDOW_RET, false) || window_ret_status == WINDOW_RET_CANCEL)
            {
                board_vfs_state |= (BOARD_VFS_ERROR_IDX_FULL << 4);
                board_vfs_state |= BOARD_VFS_STATE_INVALID;
                break;
            }
            
            if(window_ret_status == WINDOW_RET_ACK)
                byte_count += tx_size;
        }
    }
    
    usb_tx_resume();
    
    return byte_count;
}

int target_write_byte(char* filename, int size, int offset, uint8_t byte)
{
    jmx_file_request(filename,size,offset,'w', false, TOTAL_BUFFER_SIZE, NULL);
    
    int response = -1;
    
    int ret = wait_for_reply(JMX_WINDOW_RET, false);
    
    if(ret)
        response = window_ret_status;
    
    uint32_t byte_count = 0;
    
    if(response == WINDOW_RET_ACK)
    {
        uint8_t tx_buf[TOTAL_BUFFER_SIZE];
        
        memset(tx_buf, byte, EFFECTIVE_BUFF_SIZE);
        
        uint32_t *checksum = (uint32_t *)&tx_buf[EFFECTIVE_BUFF_SIZE];
        
        *checksum = 0;
        
        for(int i = 0; i < EFFECTIVE_BUFF_SIZE; i++)
            *checksum += tx_buf[i];

        StatusPacket status;
        
        while(byte_count < size)
        {
            int tx_size = MIN(EFFECTIVE_BUFF_SIZE, size - byte_count);
            
            status.code = 1;
            status.window = TOTAL_BUFFER_SIZE;
            status.receipt = NULL;
            
            jmx_configure_buffer("status", &status);
            
            status.code = 1;
            
            uart_write_data(tx_buf, TOTAL_BUFFER_SIZE);
            
            if(!wait_for_reply(JMX_STATUS_PACKET, false) || status.code < MICROBIT_JMX_REPEAT)
            {
                board_vfs_state |= (BOARD_VFS_ERROR_IDX_FULL << 4);
                board_vfs_state |= BOARD_VFS_STATE_INVALID;
                break;
            }
            
            if(status.code == MICROBIT_JMX_SUCCESS)
                byte_count += tx_size;
        }
    }
    
    usb_tx_resume();
    
    return byte_count;
}

void target_get_entry(int entry, DIRRequestPacket* dir)
{
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
    
    dir->entry = -4;
    
    wait_for_reply(JMX_DIR_PACKET, false);
}

void ls()
{
    memset(microbit_page_buffer, 0, MICROBIT_PAGE_BUFFER_SIZE);
    entries_count = 0;
    
    // a maximum of 85 entries, allocate our cluster, add our hooks
    vfs_create_subdir("FILES      ", 85, board_read_subdir, board_write_subdir);
    //uart_debug('P');
    DIRRequestPacket dir;
    
    bool done = false;
    
    usb_tx_block();
    
    while (!done)
    {
        target_get_entry(entries_count, &dir);
        
        if (dir.entry < 0 || dir.size < 0)
            done = true;
        else
        {
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
    
    usb_tx_resume();
}

void sync_init()
{
    jmx_init();
    swd_init();
    
    //initialise our mutex, incase our thread is paged out.
    os_mut_init(&jmx_mutex);
    
    //reset our state bits, but not our error indicator.
    board_vfs_state &= ~0x0F;
    entries_count = 0;
    
    //reset the microbit, setup the initalisation of UART
    target_set_state(RESET_HOLD);
    uart_initialize();
    uart_set_configuration((UART_Configuration*)&init_config);
    uart_clear_rx();
    target_set_state(RESET_RUN);
    swd_off();
    
    main_blink_cdc_led(MAIN_LED_OFF);
    
#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
    memset(tx_rx_buffer, 0, BUFF_SIZE);
    vfs_create_file("RXTX    TXT", read_tx_rx_buff, 0, BUFF_SIZE);
#endif   
}

/**
  * called in vfs_user.c: vfs_build().
  *
  * Adds the files specific to the microbit.
  * Requires that BOARD_VFS_ADD_FILES
  */
void board_vfs_add_files() {
    main_blink_cdc_led(MAIN_LED_OFF);
    
    bool first_init = false;
    
    // this is the first entrance, we have to reset the target, and synchronously read.
    if(board_vfs_state & BOARD_VFS_STATE_BOOT)
    {
        sync_init();
        first_init = true;
    }
    
    if(!(board_vfs_state & BOARD_VFS_STATE_INVALID))
        // 2000000
        wait_for_reply(JMX_INIT_PACKET, true);
    else
    {
        reset_junk_clusters();
        
        uint8_t error_index = (board_vfs_state & 0xF0) >> 4;
        
        if(error_index > 0)
        {
            //account for zero based indexing...
            error_index--;
            
            vfs_create_file("FS_ERRORTXT", read_file_fs_error, 0, strlen(microbit_board_errors[error_index]));
        }
    }
    
    
    if(board_vfs_state & BOARD_VFS_STATE_INIT && !first_init)
    {
        if(strncmp(version,"0.0.0",5) == 0)
        {
            if(microbit_get_flash_meta(&microbit_flash_start, &microbit_flash_size)) {
                microbit_flash_size *= 1024;
                vfs_create_file("FLASH   BIN", read_microbit_flash_bin, 0, microbit_flash_size);
                vfs_create_file("FLASHJS TXT", read_microbit_flash_js, 0, (microbit_flash_size*5)+12);

                int file_size = strlen(microbit_html);
                vfs_create_file("MBITFS  HTM", read_file_microbit_htm, 0, file_size);
            } 
            
            board_vfs_state &= ~BOARD_VFS_STATE_INIT;
        }
        else
            ls();
    }
    
    // reset our state indicators
    board_vfs_state &= ~(BOARD_VFS_STATE_INVALID | BOARD_VFS_STATE_FLASH);
}

/*

I WROTE SOME CODE TO ACCOUNT FOR A RESET BUTTON BEING PUSHED DURING RUNNING....

IN THE INIT PACKET HANDLER, AND THE USB THREAD.



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
            
        uint32_t cluster = (uint32_t)lw->first_cluster & ~0xF000;
            
        de.first_cluster_high_16 = cluster >> 16;
        de.first_cluster_low_16 = cluster >> 0;
        
        memcpy(data + size, &de, sizeof(FatDirectoryEntry_t));
        size += sizeof(FatDirectoryEntry_t);
    }
    
    return size;
}

// REFACTOR, USB BEHAVIOUR, EMPTY TX BUFF BEFORE SIGNALLING JMX?, TEST DELETION HANDLING, USB BUG

LWDirectoryEntry_t* getEntry(uint32_t requested_cluster)
{ 
    LWDirectoryEntry_t* lw = NULL;
    
    for(int i = 0; i < entries_count; i++)
    {
        lw = ((LWDirectoryEntry_t*)microbit_page_buffer) + i;
        
        uint32_t clust = lw->first_cluster & ~0xF000;
        
        uint32_t last_cluster = clust + ((lw->size / VFS_CLUSTER_SIZE));
        
        //if the requested sector is between this entries first cluster and last cluster...
        if(requested_cluster >= clust && requested_cluster  <= last_cluster)
            return lw;
    }
    
    return NULL;
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


int board_vfs_get_size(void)
{
    //for(int i = 0; i < entries_count; i++)
        
    return 64000;
}



void board_vfs_reset(void)
{
    board_vfs_state &= ~BOARD_VFS_STATE_INIT;
    board_vfs_state |= BOARD_VFS_STATE_FLASH;
    entries_count = 0;
    
    reset_junk_clusters();
}

int board_vfs_remount_req(void)
{
    return (board_vfs_state & BOARD_VFS_STATE_INVALID);
}

int board_vfs_enabled(void)
{
    return (board_vfs_state & BOARD_VFS_STATE_INIT);
}

int board_vfs_read(uint32_t requested_sector, uint8_t *buf, uint32_t num_sectors)
{    
    if(!(board_vfs_state & BOARD_VFS_STATE_INIT) || board_vfs_state & BOARD_VFS_STATE_INVALID)
        return 0;
    
    //get the usage of the DAPLink VFS.
    uint32_t total_vfs_sectors = vfs_get_virtual_usage() / VFS_SECTOR_SIZE;
    
    //calculates the cluster offset into our file system
    int microbit_fs_off = (requested_sector - total_vfs_sectors) / 8;
    
    //calculates the block offset into the file
    int microbit_block_offset = (requested_sector - total_vfs_sectors) % 8;
    
    if(is_junk_cluster(first + microbit_fs_off))
        return 0;
    
    // find correct entry
    LWDirectoryEntry_t* entry = getEntry(first + microbit_fs_off); 
    
    if(entry == NULL)
        return 0;
    
    int bytes_read = 0;
    int cluster_offset = ((first + microbit_fs_off) - entry->first_cluster) * VFS_CLUSTER_SIZE;
    
    int byte_offset = cluster_offset + (microbit_block_offset * VFS_SECTOR_SIZE);
    
    if(byte_offset > entry->size)
        return 0;
    
    int read_end = VFS_SECTOR_SIZE * num_sectors;
    
    //a small optimisation to read only what is recorded...
    int size = MIN(entry->size - (byte_offset + bytes_read), read_end);

    return target_get_file((char *)entry->filename, size, byte_offset, buf);
}

int board_vfs_write(uint32_t requested_sector, uint8_t *buf, uint32_t num_sectors)
{
    if(!(board_vfs_state & BOARD_VFS_STATE_INIT) || board_vfs_state & BOARD_VFS_STATE_INVALID)
        return -1;
    
    //get the usage of the DAPLink VFS.
    uint32_t total_vfs_sectors = vfs_get_virtual_usage() / VFS_SECTOR_SIZE;
    
    //calculates the cluster offset into our file system
    int microbit_fs_off = (requested_sector - total_vfs_sectors) / 8;
    
    //calculates the block offset into the file
    int microbit_block_offset = (requested_sector - total_vfs_sectors) % 8;
    
    if(is_junk_cluster(first + microbit_fs_off))
    {
        //tx_rx_buff_write('R');
        return -1;
    }

    LWDirectoryEntry_t* entry = NULL;
    
    // if we have an ongoing file transfer.
    if(fts)
    {
        bool valid = true;
        
        entry = fts->currentEntry;
        
        // first time processing this file transfer request.
        if(requested_sector == fts->next_sector || fts->next_sector == 0)
            // track our sector for next time
            fts->next_sector = requested_sector + 1;
        // this is an out of sequence write, it breaks our assumption of contiguity with writes, and structure.
        else if(requested_sector != fts->next_sector)
        {
            // compute whether we have already processed this sector:
            uint32_t start_sector = cluster_to_sector(entry->first_cluster);
            
            if(requested_sector - start_sector > fts->next_sector - start_sector)
                valid = false;
            else
                return VFS_SECTOR_SIZE * num_sectors;
        }
        
        //if we are overflowing into another dirent, or we haven't seen a contiguous write...
        if(!valid || (getEntry(first + microbit_fs_off) != NULL && fts->currentEntry != getEntry(first + microbit_fs_off)))
        {
            board_vfs_state |= (BOARD_VFS_ERROR_IDX_CONF << 4);
            board_vfs_state |= BOARD_VFS_STATE_INVALID;
            
            entry = NULL;
            free(fts);
            fts = NULL;
        }
    }
    else
        entry = getEntry(first + microbit_fs_off);
    
    if(entry == NULL)
    {
        add_junk_cluster(first + microbit_fs_off);
        return -1;
    }
    
    // we are receiving writes for a deleted file, or we are receiving writes after a file has been deleted, something has gone wrong...
    if(entry->first_cluster & FILE_DELETED_BIT)
    {
        board_vfs_state |= (BOARD_VFS_ERROR_IDX_DEL << 4);
        board_vfs_state |= BOARD_VFS_STATE_INVALID;
        return -1;
    }
    
    //if we haven't yet got a size, we also probably don't have a first_cluster...
	if ((entry->first_cluster & ~0xF000) == 0)
    {
        uint32_t flags = (entry->first_cluster & 0xF000) | UNKNOWN_SIZE_BIT;
		entry->first_cluster = (first + microbit_fs_off) | flags;
	}
    
	int bytes_written = 0;
    int cluster_offset = ((first + microbit_fs_off) - entry->first_cluster) * VFS_CLUSTER_SIZE;
    int byte_offset = cluster_offset + (microbit_block_offset * VFS_SECTOR_SIZE);
    
	int write_end = VFS_SECTOR_SIZE * num_sectors;
	int bytes_parsed = 0;
    
	if (entry->first_cluster & UNKNOWN_SIZE_BIT)
	{
        uint32_t valid_bytes = 0;
        
        //preserve our previous char, and counter for the future.
        uint8_t previous_char = entry->last_char;
        uint32_t previous_char_count = entry->last_char_count;
        
        int valid_start = -1;
        
        //set our count to zero, so we can get an accurate count of the characters in this sector only...
        entry->last_char_count = 0;
        
        // parse the sector
        // for each byte:
            // compare it to the previous, if it is different, change the last char, increment valid_bytes by the last_char_count;
            // if it is the same, increment last_char_count
            //increment bytes_parsed
		while (bytes_parsed < write_end)
		{

			if (entry->last_char == buf[bytes_parsed])
				entry->last_char_count++;
			else
			{
				// if this is our first transition
				if (valid_start == -1)
				{
					// add our parsed counter to the PREVIOUS character count and set our start
					previous_char_count += entry->last_char_count;
					valid_start = bytes_parsed;
				}
				else
					valid_bytes += entry->last_char_count;
				
				entry->last_char = buf[bytes_parsed];
				entry->last_char_count = 1;
			}

			bytes_parsed++;
		}
        
        // if this sector has different bytes in it, we can assume it's valid, write our previous n characters, and this sector's valid bytes.
        if(valid_bytes > 0 && valid_start >= 0)
        {
            entry->size += target_write_byte((char*)entry->filename, previous_char_count, entry->size, previous_char);
            entry->size += target_write_buf((char*)entry->filename, valid_bytes, entry->size, buf + valid_start);
        }
        else
            // add our previous character count back on...
            entry->last_char_count += previous_char_count;
        
        return valid_bytes + previous_char_count;
	}
    else
    {
        int size = MIN(entry->size - (byte_offset + bytes_written), write_end);
        
        return target_write_buf((char*) entry->filename, size, byte_offset, buf);
    }
}

static void board_write_subdir(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors){
    FatDirectoryEntry_t *new_entry = (FatDirectoryEntry_t *)data;
   
    uint32_t dirs_per_sector = VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t); 

    int entry_offset = dirs_per_sector * sector_offset;
    
    int validated_entries = 0;
    
    uint32_t num_entries = num_sectors * VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t);
    
    while(entry_offset < num_entries)
    {
        FatDirectoryEntry_t current_entry = new_entry[entry_offset];
        
        // we aren't interested in Long filenames, they add unneccessary complexity.
        if(current_entry.attributes & VFS_FILE_ATTR_LONG_NAME)
        {
            entry_offset++;
            continue;
        }
        
        // transform to a complete cluster number.
        uint32_t cluster = ((uint32_t)current_entry.first_cluster_high_16 << 16) |  current_entry.first_cluster_low_16;
        
        LWDirectoryEntry_t* lw = getEntry(cluster);
        
        // this is a deleted entry.
        if(0xe5 == (uint8_t)current_entry.filename[0])
        {   
            // set our bit, delete the file and continue
            if(lw)
            {
                int ret = jmx_file_request((char *)lw->filename, 0, 0, 'd', true, 0, NULL);
                
                lw->first_cluster |= FILE_DELETED_BIT;
                
                usb_tx_resume();
            }
            
            entry_offset++;
            continue;
        }
        
        if(!current_entry.attributes)
            break;
        
        // this is almost certainly a new file, but we should check the filename just to be sure...
        if(!lw)
            lw = getEntryByShortName(current_entry.filename);
        
        // This is a new entry, or we could be replacing a deleted file.
        if(!lw || lw->first_cluster & FILE_DELETED_BIT)
        {
            LWDirectoryEntry_t new_lw;
            memset(&new_lw, 0, sizeof(LWDirectoryEntry_t));

            int filename_offset = 0;
            
            // capture the filename and transform it to 8-[period]-3, we only support 8.3 filenames when writing.
            for(int j = 0; j < 11; j++)
            {
                if(current_entry.filename[j] == 0)
                    break;
                
                if(j == 8 && current_entry.filename[j] != ' ')
                    new_lw.filename[filename_offset++] = '.';
                
                if(current_entry.filename[j] != ' ')
                    new_lw.filename[filename_offset++] = current_entry.filename[j];
            }
            
            new_lw.size = current_entry.filesize;
            new_lw.first_cluster = cluster;
            
            if(new_lw.size == 0)
                new_lw.first_cluster |= UNKNOWN_SIZE_BIT;
            
            LWDirectoryEntry_t* dest ;
            
            // if this is true, we are allocating over an existing, deleted, entry...
            if(lw)
            {
                uint32_t file_size_clust = (lw->size / VFS_CLUSTER_SIZE) + 1;
                uint32_t new_size_clust = (new_lw.size / VFS_CLUSTER_SIZE) + 1;
                
                // if we are allocating within the existing directory entry's cluster allocation
                // that's fine, however, if we overflow into a new cluster, blow up...
                if(new_size_clust <= file_size_clust)
                    dest = lw;
                else
                {
                    board_vfs_state |= (BOARD_VFS_ERROR_IDX_DEL << 4);
                    board_vfs_state |= BOARD_VFS_STATE_INVALID;
                    continue;
                }
            }else
                dest = ((LWDirectoryEntry_t*)microbit_page_buffer) + entries_count;
            
            *dest = new_lw;
            
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
                    // blow up, file transfer in progress, we can't handle simulataneous writes of unknown sizes.
                    board_vfs_state |= (BOARD_VFS_ERROR_IDX_MULTI << 4);
                    board_vfs_state |= BOARD_VFS_STATE_INVALID;
                    
                    free(fts);
                    fts = NULL;
                }
            }
            entries_count++;
        }
        else
        {   
            //if we don't know the filesize, update the filesize. That is all.
            if(lw->first_cluster & UNKNOWN_SIZE_BIT)
            {
                if(current_entry.filesize)
                {
                    lw->first_cluster &= ~UNKNOWN_SIZE_BIT;
                    
                    // if our optimisation was too harsh, we need to write the appropriate amount bytes to the file
                    // now we know the size...
                    
                    // if we have buffered some characters, and we have received the first cluster...
                    if(lw->size < current_entry.filesize && lw->last_char_count > 0 && (lw->first_cluster & ~0xF000) > 0)
                    {
                        //write the remaining bytes.
                        target_write_byte((char*)lw->filename,lw->last_char_count, lw->size, lw->last_char);
                        
                        lw->last_char = 0;
                        lw->last_char_count = 0;
                    }
                    
                    lw->size = current_entry.filesize;
                    
                    // we've received a final size for a file of unknown length, mark our file transfer status as finished.
                    // We've filled in our unknowns.
                    if(fts)
                    {
                        free(fts);
                        fts = NULL;
                    }
                }
            }
            // for some inconvenient reason, mac OS likes to move things around...
            else if(lw->first_cluster != cluster)
                lw->first_cluster = cluster;
        }
        entry_offset++;
        validated_entries++;
    }
}


static uint32_t read_file_fs_error(uint32_t sector, uint8_t *data, uint32_t n)
{
    uint8_t error_index = (board_vfs_state & 0xF0) >> 4;
        
    if(error_index == 0 || sector > 0)
        return 0;
    
    //account for zero based indexing...
    error_index--;
    
    //return one sector only...
    int len = MIN(strlen(microbit_board_errors[error_index]), VFS_SECTOR_SIZE);
    
    memcpy(data, microbit_board_errors[error_index], len);
    
    return len;
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
