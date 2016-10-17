/**
 * @file    virtual_fs.c
 * @brief   Implementation of virtual_fs.h
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

#include "string.h"

#include "virtual_fs.h"
#include "vfs_manager.h"
#include "info.h"
#include "settings.h"
#include "compiler.h"
#include "macro.h"
#include "util.h"
#include "daplink_debug.h"

COMPILER_ASSERT(sizeof(FatDirectoryEntry_t) == 32);

typedef struct root_dir {
    FatDirectoryEntry_t f[32];
} root_dir_t;

// to save RAM all files must be in the first root dir entry (512 bytes)
//  but 2 actually exist on disc (32 entries) to accomodate hidden OS files,
//  folders and metadata
typedef struct virtual_media {
    vfs_read_cb_t read_cb;
    vfs_write_cb_t write_cb;
    uint32_t length;
} virtual_media_t;

static uint32_t read_zero(uint32_t offset, uint8_t *data, uint32_t size);
static void write_none(uint32_t offset, const uint8_t *data, uint32_t size);

static uint32_t read_mbr(uint32_t offset, uint8_t *data, uint32_t size);
static uint32_t read_fat(uint32_t offset, uint8_t *data, uint32_t size);
static void write_fat_fs(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors);
static uint32_t read_dir(uint32_t offset, uint8_t *data, uint32_t size);
static void write_dir(uint32_t offset, const uint8_t *data, uint32_t size);
static void file_change_cb_stub(const vfs_filename_t filename, vfs_file_change_t change,
                                vfs_file_t file, vfs_file_t new_file_data);
static bool filename_valid(const vfs_filename_t filename);
static bool filename_character_valid(char character);
static void set_init_done(void);

// If sector size changes update comment below
COMPILER_ASSERT(0x0200 == VFS_SECTOR_SIZE);
// If root directory size changes update max_root_dir_entries
COMPILER_ASSERT(0x0020 == sizeof(root_dir_t) / sizeof(FatDirectoryEntry_t));
static const mbr_t mbr_tmpl = {
    /*uint8_t[11]*/.boot_sector = {
        0xEB, 0x3C, 0x90,
        'M', 'S', 'D', '0', 'S', '4', '.', '1' // OEM Name in text (8 chars max)
    },
    /*uint16_t*/.bytes_per_sector           = 0x0200,       // 512 bytes per sector
    /*uint8_t */.sectors_per_cluster        = 0x08,         // 4k cluser
    /*uint16_t*/.reserved_logical_sectors   = 0x0001,       // mbr is 1 sector
    /*uint8_t */.num_fats                   = 0x02,         // 2 FATs
    /*uint16_t*/.max_root_dir_entries       = 0x0020,       // 32 dir entries (max)
    /*uint16_t*/.total_logical_sectors      = 0x1f50,       // sector size * # of sectors = drive size
    /*uint8_t */.media_descriptor           = 0xf8,         // fixed disc = F8, removable = F0
    /*uint16_t*/.logical_sectors_per_fat    = 0x0001,       // FAT is 1k - ToDO:need to edit this
    /*uint16_t*/.physical_sectors_per_track = 0x0001,       // flat
    /*uint16_t*/.heads                      = 0x0001,       // flat
    /*uint32_t*/.hidden_sectors             = 0x00000000,   // before mbt, 0
    /*uint32_t*/.big_sectors_on_drive       = 0x00000000,   // 4k sector. not using large clusters
    /*uint8_t */.physical_drive_number      = 0x00,
    /*uint8_t */.not_used                   = 0x00,         // Current head. Linux tries to set this to 0x1
    /*uint8_t */.boot_record_signature      = 0x29,         // signature is present
    /*uint32_t*/.volume_id                  = 0x27021974,   // serial number
    // needs to match the root dir label
    /*char[11]*/.volume_label               = {'D', 'A', 'P', 'L', 'I', 'N', 'K', '-', 'D', 'N', 'D'},
    // unused by msft - just a label (FAT, FAT12, FAT16)
    /*char[8] */.file_system_type           = {'F', 'A', 'T', '1', '2', ' ', ' ', ' '},

    /* Executable boot code that starts the operating system */
    /*uint8_t[448]*/.bootstrap = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    },
    // Set signature to 0xAA55 to make drive bootable
    /*uint16_t*/.signature = 0x0000,
};

enum virtual_media_idx_t {
    MEDIA_IDX_MBR = 0,
    MEDIA_IDX_FAT1,
    MEDIA_IDX_FAT2,
    MEDIA_IDX_ROOT_DIR,
    
    MEDIA_IDX_COUNT
};

static void write_fat_fs(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors);

// Note - everything in virtual media must be a multiple of VFS_SECTOR_SIZE
const virtual_media_t virtual_media_tmpl[] = {
    /*  Read CB         Write CB        Region Size                 Region Name     */
    {   read_mbr,       write_none,     VFS_SECTOR_SIZE         },  /* MBR          */
    {   read_fat,       write_fat_fs,     0 /* Set at runtime */  },  /* FAT1         */
    {   read_fat,       write_fat_fs,     0 /* Set at runtime */  },  /* FAT2         */
    {   read_dir,       write_dir,      VFS_SECTOR_SIZE * 2     },  /* Root Dir     */
    /* Raw filesystem contents follow */
};
// Keep virtual_media_idx_t in sync with virtual_media_tmpl
COMPILER_ASSERT(MEDIA_IDX_COUNT == ELEMENTS_IN_ARRAY(virtual_media_tmpl));

static const FatDirectoryEntry_t root_dir_entry = {
    /*uint8_t[11] */ .filename = {""},
    /*uint8_t */ .attributes = VFS_FILE_ATTR_VOLUME_LABEL | VFS_FILE_ATTR_ARCHIVE,
    /*uint8_t */ .reserved = 0x00,
    /*uint8_t */ .creation_time_ms = 0x00,
    /*uint16_t*/ .creation_time = 0x0000,
    /*uint16_t*/ .creation_date = 0x0000,
    /*uint16_t*/ .accessed_date = 0x0000,
    /*uint16_t*/ .first_cluster_high_16 = 0x0000,
    /*uint16_t*/ .modification_time = 0x8E41,
    /*uint16_t*/ .modification_date = 0x32bb,
    /*uint16_t*/ .first_cluster_low_16 = 0x0000,
    /*uint32_t*/ .filesize = 0x00000000
};

static const FatDirectoryEntry_t sub_dir_entry = {
    /*uint8_t[11] */ .filename = {""},
    /*uint8_t */ .attributes = VFS_FILE_ATTR_SUB_DIR,
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

static const FatDirectoryEntry_t dir_entry_tmpl = {
    /*uint8_t[11] */ .filename = {""},
    /*uint8_t */ .attributes = VFS_FILE_ATTR_READ_ONLY,
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

mbr_t mbr;
file_allocation_table_t fat;
virtual_media_t virtual_media[16];
root_dir_t dir_current;
FatDirectoryEntry_t dir_initial[VFS_MAX_FILES];
uint8_t file_count;
vfs_file_change_cb_t file_change_cb;
uint32_t virtual_media_idx;
uint32_t fat_idx;
uint32_t dir_idx;
uint32_t data_start;
bool init_complete;

// Virtual media must be larger than the template
COMPILER_ASSERT(sizeof(virtual_media) > sizeof(virtual_media_tmpl));
static void write_fat(file_allocation_table_t *fat, uint32_t idx, uint16_t val)
{
    uint32_t low_idx;
    uint32_t high_idx;
    uint8_t low_data;
    uint8_t high_data;
    low_idx = idx * 3 / 2;
    high_idx = idx * 3 / 2 + 1;

    // Assert that this is still within the fat table
    if (high_idx >= ELEMENTS_IN_ARRAY(fat->f)) {
        util_assert(0);
        return;
    }

    
    if (idx & 1) {
        // Odd - lower byte shared
        low_data = (val << 4) & 0xF0;
        high_data = (val >> 4) & 0xFF;
        fat->f[low_idx] = fat->f[low_idx] | low_data;
        fat->f[high_idx] = high_data;
    } else {
        // Even - upper byte shared
        low_data = (val >> 0) & 0xFF;
        high_data = (val >> 8) & 0x0F;
        fat->f[low_idx] =  low_data;
        fat->f[high_idx] = fat->f[high_idx] | high_data;
    }
}

int get_fat_entry(uint32_t cluster)
{
    uint32_t low_idx;
    uint32_t high_idx;
    low_idx = cluster * 3 / 2;
    high_idx = cluster * 3 / 2 + 1;
    
    // Assert that this is still within the fat table
    if (high_idx >= ELEMENTS_IN_ARRAY(fat.f)) {
        util_assert(0);
        return -1;
    }

    
    if (cluster & 1) {
        // Odd - lower byte shared
        return ((uint16_t)fat.f[high_idx] << 4) | (fat.f[low_idx] & 0xF0) >> 4;
    } else {
        // Even - upper byte shared
        return ((uint16_t)(fat.f[high_idx] &0x0F) << 8) | fat.f[low_idx];
    }
}

void vfs_init(const vfs_filename_t drive_name, uint32_t disk_size)
{
    uint32_t i;
    // Clear everything
    memset(&mbr, 0, sizeof(mbr));
    memset(&fat, 0, sizeof(fat));
    fat_idx = 0;
    memset(&virtual_media, 0, sizeof(virtual_media));
    memset(&dir_current, 0, sizeof(dir_current));
    memset(&dir_initial, 0, sizeof(dir_initial));
    dir_idx = 0;
    file_count = 0;
    file_change_cb = file_change_cb_stub;
    virtual_media_idx = 0;
    data_start = 0;
    init_complete = false;
    // Initialize MBR
    memcpy(&mbr, &mbr_tmpl, sizeof(mbr_t));
    mbr.total_logical_sectors = ((disk_size + KB(64)) / mbr.bytes_per_sector);
    mbr.logical_sectors_per_fat = (3 * (((mbr.total_logical_sectors / mbr.sectors_per_cluster) + 1023) / 1024));
    // Initailize virtual media
    memcpy(&virtual_media, &virtual_media_tmpl, sizeof(virtual_media_tmpl));
    virtual_media[MEDIA_IDX_FAT1].length = VFS_SECTOR_SIZE * mbr.logical_sectors_per_fat;
    virtual_media[MEDIA_IDX_FAT2].length = VFS_SECTOR_SIZE * mbr.logical_sectors_per_fat;
    // Initialize indexes
    virtual_media_idx = MEDIA_IDX_COUNT;
    data_start = 0;

    for (i = 0; i < ELEMENTS_IN_ARRAY(virtual_media_tmpl); i++) {
        data_start += virtual_media[i].length;
    }

    // Initialize FAT
    fat_idx = 0;
    write_fat(&fat, fat_idx, 0xFF8);    // Media type "media_descriptor"
    fat_idx++;
    write_fat(&fat, fat_idx, 0xFFF);    // No meaning
    fat_idx++;
    // Initialize root dir
    dir_idx = 0;
    dir_current.f[dir_idx] = root_dir_entry;
    memcpy(dir_current.f[dir_idx].filename, drive_name, sizeof(dir_current.f[0].filename));
    dir_idx++;
}

uint32_t vfs_get_total_size()
{
    return mbr.bytes_per_sector * mbr.total_logical_sectors;
}

uint32_t vfs_get_virtual_usage()
{
    uint32_t vfs_total = 0;
    
    for(int i = 0; i < ELEMENTS_IN_ARRAY(virtual_media); i++)
        vfs_total += virtual_media[i].length;

    return vfs_total;
}

uint32_t write_clusters(uint32_t clusters_to_write)
{
    uint32_t first_cluster;
    uint32_t i;
    
    // Write the cluster chain to the fat table
    first_cluster = 0;
    
    first_cluster = fat_idx;

    for (i = 0; i < clusters_to_write - 1; i++) {
        write_fat(&fat, fat_idx, fat_idx + 1);
        fat_idx++;
    }

    write_fat(&fat, fat_idx, 0xFFF);
    fat_idx++;
    
    return first_cluster;
}

int add_to_virtual_media(uint32_t clusters, vfs_read_cb_t read_cb, vfs_write_cb_t write_cb)
{
    // Update virtual media
    if (virtual_media_idx >= ELEMENTS_IN_ARRAY(virtual_media)) {
        util_assert(0);
        return VFS_FILE_INVALID;
    }

    virtual_media[virtual_media_idx].read_cb = read_zero;
    virtual_media[virtual_media_idx].write_cb = write_none;

    if (0 != read_cb) {
        virtual_media[virtual_media_idx].read_cb = read_cb;
    }

    if (0 != write_cb) {
        virtual_media[virtual_media_idx].write_cb = write_cb;
    }

    virtual_media[virtual_media_idx].length = clusters * mbr.bytes_per_sector * mbr.sectors_per_cluster;
    virtual_media_idx++;
    file_count += 1;
    
    return 1;
}

vfs_file_t vfs_create(FatDirectoryEntry_t *fde, vfs_read_cb_t read_cb, vfs_write_cb_t write_cb, uint32_t len)
{  
    dir_idx++;
    
    uint32_t clusters = vfs_create_entry(fde, len);
    
    // Update directory entry
    if (dir_idx >= ELEMENTS_IN_ARRAY(dir_current.f)) {
        util_assert(0);
        return VFS_FILE_INVALID;
    }
    
    if(add_to_virtual_media(clusters, read_cb, write_cb) == VFS_FILE_INVALID)
        return VFS_FILE_INVALID;
    
    return fde;
}

uint32_t vfs_create_entry(FatDirectoryEntry_t *de, uint32_t len)
{   
    if(NULL == de)
       return 0;
    
    // Compute the number of clusters in the file
    uint32_t cluster_size = mbr.bytes_per_sector * mbr.sectors_per_cluster;
    uint32_t clusters = (len + cluster_size - 1) / cluster_size;
    
    uint32_t first_cluster = 0;
    
    if (len > 0)
        first_cluster = write_clusters(clusters);
    
    de->first_cluster_high_16 = (first_cluster >> 16) & 0xFFFF;
    de->first_cluster_low_16 = (first_cluster >> 0) & 0xFFFF;

    return clusters;
}

/*
    create a directory entry... and allocate a single cluster... we need some way of taking in a pointer to a directory array...
*/
vfs_file_t vfs_create_subdir(const vfs_filename_t dir_name, uint32_t entries, vfs_read_cb_t read_cb, vfs_write_cb_t write_cb)
{
    util_assert(filename_valid(dir_name));
    FatDirectoryEntry_t *de = &dir_current.f[dir_idx];
    memcpy(de, &sub_dir_entry, sizeof(sub_dir_entry));
    memcpy(de->filename, dir_name, 11);
    
    return vfs_create(de, read_cb, write_cb, entries * sizeof(FatDirectoryEntry_t));
}

vfs_file_t vfs_create_file(const vfs_filename_t filename, vfs_read_cb_t read_cb, vfs_write_cb_t write_cb, uint32_t len)
{
    util_assert(filename_valid(filename));
    FatDirectoryEntry_t *de = &dir_current.f[dir_idx];
    memcpy(de, &dir_entry_tmpl, sizeof(dir_entry_tmpl));
    memcpy(de->filename, filename, 11);
    
    de->filesize = len;
    
    return vfs_create(de, read_cb, write_cb, de->filesize);
}

void vfs_file_set_attr(vfs_file_t file, vfs_file_attr_bit_t attr)
{
    FatDirectoryEntry_t *de = file;
    de->attributes = attr;
}

vfs_sector_t vfs_file_get_start_sector(vfs_file_t file)
{
    FatDirectoryEntry_t *de = file;

    if (vfs_file_get_size(file) == 0) {
        return VFS_INVALID_SECTOR;
    }

    return cluster_to_sector(de->first_cluster_low_16);
}

uint32_t vfs_file_get_size(vfs_file_t file)
{
    FatDirectoryEntry_t *de = file;
    return de->filesize;
}

vfs_file_attr_bit_t vfs_file_get_attr(vfs_file_t file)
{
    FatDirectoryEntry_t *de = file;
    return (vfs_file_attr_bit_t)de->attributes;
}

void vfs_set_file_change_callback(vfs_file_change_cb_t cb)
{
    file_change_cb = cb;
}

int vfs_read(uint32_t requested_sector, uint8_t *buf, uint32_t num_sectors)
{
    set_init_done();
    
    int read_success = -1;

    uint8_t i = 0;
    uint32_t current_sector;
    // Zero out the buffer
    memset(buf, 0, num_sectors * VFS_SECTOR_SIZE);
    current_sector = 0;
    for (i = 0; i < ELEMENTS_IN_ARRAY(virtual_media); i++) {
        uint32_t vm_sectors = virtual_media[i].length / VFS_SECTOR_SIZE;
        uint32_t vm_start = current_sector;
        uint32_t vm_end = current_sector + vm_sectors;

        // Data can be used in this sector
        if ((requested_sector >= vm_start) && (requested_sector < vm_end)) {
            uint32_t sector_offset;
            uint32_t sectors_to_write = vm_end - requested_sector;
            sectors_to_write = MIN(sectors_to_write, num_sectors);
            sector_offset = requested_sector - current_sector;
            virtual_media[i].read_cb(sector_offset, buf, sectors_to_write);
            // Update requested sector
            requested_sector += sectors_to_write;
            num_sectors -= sectors_to_write;
        }

        // If there is no more data to be read then break
        if (num_sectors == 0) {
            read_success = 0;
            break;
        }

        // Move to the next virtual media entry
        current_sector += vm_sectors;
    }
    
    return read_success;
}

int vfs_write(uint32_t requested_sector, const uint8_t *buf, uint32_t num_sectors)
{
    uint8_t i = 0;
    uint32_t current_sector;
    current_sector = 0;

    set_init_done();
    
    int write_success = -1;

    for (i = 0; i < virtual_media_idx; i++) {
        uint32_t vm_sectors = virtual_media[i].length / VFS_SECTOR_SIZE;
        uint32_t vm_start = current_sector;
        uint32_t vm_end = current_sector + vm_sectors;

        // Data can be used in this sector
        if ((requested_sector >= vm_start) && (requested_sector < vm_end)) {
            uint32_t sector_offset;
            uint32_t sectors_to_read = vm_end - requested_sector;
            sectors_to_read = MIN(sectors_to_read, num_sectors);
            sector_offset = requested_sector - current_sector;
            virtual_media[i].write_cb(sector_offset, buf, sectors_to_read);
            // Update requested sector
            requested_sector += sectors_to_read;
            num_sectors -= sectors_to_read;
        }

        // If there is no more data to be read then break
        if (num_sectors == 0) {
            write_success = 0;
            break;
        }

        // Move to the next virtual media entry
        current_sector += vm_sectors;
    }
    
    return write_success;
}

static uint32_t read_zero(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{
    uint32_t read_size = VFS_SECTOR_SIZE * num_sectors;
    memset(data, 0, read_size);
    return read_size;
}

static void write_none(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors)
{
    // Do nothing
}

static uint32_t read_mbr(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{
    uint32_t read_size = sizeof(mbr_t);
    COMPILER_ASSERT(sizeof(mbr_t) <= VFS_SECTOR_SIZE);

    if (sector_offset != 0) {
        // Don't worry about reading other sectors
        return 0;
    }

    memcpy(data, &mbr, read_size);
    return read_size;
}

/* No need to handle writes to the mbr */
extern void tx_rx_buff_write(char c);
static uint32_t read_fat(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{
    uint32_t read_size = sizeof(file_allocation_table_t);
    COMPILER_ASSERT(sizeof(file_allocation_table_t) <= VFS_SECTOR_SIZE);

    if (sector_offset != 0) {
        debug_msg("OOB");
        // Don't worry about reading other sectors
        return 0;
    }
    //tx_rx_buff_write(' ');
    //tx_rx_buff_write('R');

    memcpy(data, &fat, read_size);
    return read_size;
}


static void write_fat_fs(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors)
{
    //uint32_t cluster = write_clusters(num_sectors);
    //tx_rx_buff_write(' ');
    //tx_rx_buff_write('W');
}

/* No need to handle writes to the fat */

static uint32_t read_dir(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{
    uint32_t start_index;
    uint32_t copy_size;

    if ((sector_offset + num_sectors) * VFS_SECTOR_SIZE > sizeof(dir_current)) {
        // Trying to read too much of the root directory
        util_assert(0);
        return 0;
    }

    // Zero buffer
    memset(data, 0, num_sectors * VFS_SECTOR_SIZE);
    start_index = sector_offset * VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t);

    // Copy data if anything can be copied
    if (start_index < ELEMENTS_IN_ARRAY(dir_initial)) {
        util_assert(sizeof(dir_initial) > sector_offset * VFS_SECTOR_SIZE);
        copy_size = sizeof(dir_initial) - sector_offset * VFS_SECTOR_SIZE;
        memcpy(data, &dir_initial[start_index], copy_size);
    }

    return num_sectors * VFS_SECTOR_SIZE;
}

static void write_dir(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors)
{
    FatDirectoryEntry_t *old_entry;
    FatDirectoryEntry_t *new_entry;
    uint32_t start_index;
    uint32_t num_entries;
    uint32_t i;

    if ((sector_offset + num_sectors) * VFS_SECTOR_SIZE > sizeof(dir_current)) {
        // Trying to write too much of the root directory
        util_assert(0);
        return;
    }

    start_index = sector_offset * VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t);
    num_entries = num_sectors * VFS_SECTOR_SIZE / sizeof(FatDirectoryEntry_t);
    old_entry = &dir_current.f[start_index];
    new_entry = (FatDirectoryEntry_t *)data;
    // If this is the first sector start at index 1 to get past drive name
    i = 0 == sector_offset ? 1 : 0;

    for (; i < num_entries; i++) {
        bool same_name;

        if (0 == memcmp(&old_entry[i], &new_entry[i], sizeof(FatDirectoryEntry_t))) {
            continue;
        }

        // If were at this point then something has changed in the file
        same_name = (0 == memcmp(old_entry[i].filename, new_entry[i].filename, sizeof(new_entry[i].filename))) ? 1 : 0;
        // Changed
        file_change_cb(new_entry[i].filename, VFS_FILE_CHANGED, (vfs_file_t)&old_entry[i], (vfs_file_t)&new_entry[i]);

        // Deleted
        if (0xe5 == (uint8_t)new_entry[i].filename[0]) {
            file_change_cb(old_entry[i].filename, VFS_FILE_DELETED, (vfs_file_t)&old_entry[i], (vfs_file_t)&new_entry[i]);
            continue;
        }

        // Created
        if (!same_name && filename_valid(new_entry[i].filename)) {
            file_change_cb(new_entry[i].filename, VFS_FILE_CREATED, (vfs_file_t)&old_entry[i], (vfs_file_t)&new_entry[i]);
            continue;
        }
    }

    memcpy(&dir_current.f[start_index], data, num_sectors * VFS_SECTOR_SIZE);
}

static void file_change_cb_stub(const vfs_filename_t filename, vfs_file_change_t change, vfs_file_t file, vfs_file_t new_file_data)
{
    // Do nothing
}

uint32_t cluster_to_sector(uint32_t cluster_idx)
{
    uint32_t sectors_before_data = data_start / mbr.bytes_per_sector;
    return sectors_before_data + (cluster_idx - 2) * mbr.sectors_per_cluster;
}

static bool filename_valid(const vfs_filename_t  filename)
{
    // Information on valid 8.3 filenames can be found in
    // the microsoft hardware whitepaper:
    //
    // Microsoft Extensible Firmware Initiative
    // FAT32 File System Specification
    // FAT: General Overview of On-Disk Format
    const char invalid_starting_chars[] = {
        0xE5, // Deleted
        0x00, // Deleted (and all following entries are free)
        0x20, // Space not allowed as first character
    };
    uint32_t i;

    // Check for invalid starting characters
    for (i = 0; i < sizeof(invalid_starting_chars); i++) {
        if (invalid_starting_chars[i] == filename[0]) {
            return false;
        }
    }

    // Make sure all the characters are valid
    for (i = 0; i < sizeof(vfs_filename_t); i++) {
        if (!filename_character_valid(filename[i])) {
            return false;
        }
    }

    // All checks have passed so filename is valid
    return true;
}

static bool filename_character_valid(char character)
{
    const char invalid_chars[] = {0x22, 0x2A, 0x2B, 0x2C, 0x2E, 0x2F, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x5B, 0x5C, 0x5D, 0x7C};
    uint32_t i;

    // Lower case characters are not allowed
    if ((character >= 'a') && (character <= 'z')) {
        return false;
    }

    // Values less than 0x20 are not allowed except 0x5
    if ((character < 0x20) && (character != 0x5)) {
        return false;
    }

    // Check for special characters that are not allowed
    for (i = 0; i < sizeof(invalid_chars); i++) {
        if (invalid_chars[i] == character) {
            return false;
        }
    }

    // All of the checks have passed so this is a valid file name character
    return true;
}

static void set_init_done(void)
{
    if (!init_complete) {
        memcpy(&dir_initial, &dir_current, MIN(sizeof(dir_initial), sizeof(dir_current)));
        init_complete = true;
    }
}
