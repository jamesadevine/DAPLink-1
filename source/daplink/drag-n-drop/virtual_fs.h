/**
 * @file    virtual_fs.h
 * @brief   FAT 12/16 filesystem handling
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

#ifndef VIRTUAL_FS_H
#define VIRTUAL_FS_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VFS_CLUSTER_SIZE        0x1000
#define VFS_SECTOR_SIZE         512
#define VFS_INVALID_SECTOR      0xFFFFFFFF
#define VFS_FILE_INVALID        0
#define VFS_MAX_FILES           16

typedef char vfs_filename_t[11];

typedef enum {
    VFS_FILE_ATTR_READ_ONLY     = (1 << 0),
    VFS_FILE_ATTR_HIDDEN        = (1 << 1),
    VFS_FILE_ATTR_SYSTEM        = (1 << 2),
    VFS_FILE_ATTR_VOLUME_LABEL  = (1 << 3),
    VFS_FILE_ATTR_SUB_DIR       = (1 << 4),
    VFS_FILE_ATTR_ARCHIVE       = (1 << 5),
    VFS_FILE_ATTR_LONG_NAME     = (VFS_FILE_ATTR_READ_ONLY | VFS_FILE_ATTR_HIDDEN | VFS_FILE_ATTR_SYSTEM | VFS_FILE_ATTR_VOLUME_LABEL)
} vfs_file_attr_bit_t;

typedef enum {
    VFS_FILE_CREATED = 0,   /*!< A new file was created */
    VFS_FILE_DELETED,       /*!< An existing file was deleted */
    VFS_FILE_CHANGED,       /*!< Some attribute of the file changed.
                                  Note: when a file is deleted or
                                  created a file changed
                                  notification will also occur*/
} vfs_file_change_t;

// Virtual file system driver
// Limitations:
//   - files must be contiguous
//   - data written cannot be read back
//   - data should only be read once

typedef struct {
    uint8_t boot_sector[11];
    /* DOS 2.0 BPB - Bios Parameter Block, 11 bytes */
    uint16_t bytes_per_sector;
    uint8_t  sectors_per_cluster;
    uint16_t reserved_logical_sectors;
    uint8_t  num_fats;
    uint16_t max_root_dir_entries;
    uint16_t total_logical_sectors;
    uint8_t  media_descriptor;
    uint16_t logical_sectors_per_fat;
    /* DOS 3.31 BPB - Bios Parameter Block, 12 bytes */
    uint16_t physical_sectors_per_track;
    uint16_t heads;
    uint32_t hidden_sectors;
    uint32_t big_sectors_on_drive;
    /* Extended BIOS Parameter Block, 26 bytes */
    uint8_t  physical_drive_number;
    uint8_t  not_used;
    uint8_t  boot_record_signature;
    uint32_t volume_id;
    char     volume_label[11];
    char     file_system_type[8];
    /* bootstrap data in bytes 62-509 */
    uint8_t  bootstrap[448];
    /* These entries in place of bootstrap code are the *nix partitions */
    //uint8_t  partition_one[16];
    //uint8_t  partition_two[16];
    //uint8_t  partition_three[16];
    //uint8_t  partition_four[16];
    /* Mandatory value at bytes 510-511, must be 0xaa55 */
    uint16_t signature;
} __attribute__((packed)) mbr_t;

typedef struct file_allocation_table {
    uint8_t f[512];
} file_allocation_table_t;

typedef struct FatDirectoryEntry {
    vfs_filename_t filename;
    uint8_t attributes;
    uint8_t reserved;
    uint8_t creation_time_ms;
    uint16_t creation_time;
    uint16_t creation_date;
    uint16_t accessed_date;
    uint16_t first_cluster_high_16;
    uint16_t modification_time;
    uint16_t modification_date;
    uint16_t first_cluster_low_16;
    uint32_t filesize;
} __attribute__((packed)) FatDirectoryEntry_t;

typedef struct LongFatDirectoryEntry {
    uint8_t order; //If masked with 0x40 (LAST_LONG_ENTRY), this indicates the entry is the last long dir entry in a set of long dir entries. All valid sets of long dir entries must begin with an entry having this mask.
    uint16_t name1[5]; // Characters 1-5 
    uint8_t attributes; // ATTR_LONG_NAME
    uint8_t type; //If zero, indicates a directory entry that is a sub-component of a long name.  NOTE: Other values reserved for future extensions. Non-zero implies other dirent types.
    uint8_t chksum;
    uint16_t name2[6]; // chars 6-11
    uint16_t first_cluster_low_16; // always 0
    uint16_t name3[2]; // chars 12-13
} __attribute__((packed)) LongFatDirectoryEntry_t;


typedef void *vfs_file_t;
typedef uint32_t vfs_sector_t;

// Callback for when data is written to a file on the virtual filesystem
typedef void (*vfs_write_cb_t)(uint32_t sector_offset, const uint8_t *data, uint32_t num_sectors);
// Callback for when data is ready from the virtual filesystem
typedef uint32_t (*vfs_read_cb_t)(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors);
// Callback for when a file's attributes are changed on the virtual filesystem.  Note that the 'file' parameter
// can be saved and compared to other files to see if they are referencing the same object.  The
// same cannot be done with new_file_data since it points to a temporary buffer.
typedef void (*vfs_file_change_cb_t)(const vfs_filename_t filename, vfs_file_change_t change,
                                     vfs_file_t file, vfs_file_t new_file_data);

// Initialize the filesystem with the given size and name
void vfs_init(const vfs_filename_t drive_name, uint32_t disk_size);

// Get the total size of the virtual filesystem
uint32_t vfs_get_total_size(void);

uint32_t vfs_get_virtual_usage(void);

// Add a file to the virtual FS and return a handle to this file.
// This must be called before vfs_read or vfs_write are called.
// Adding a new file after vfs_read or vfs_write have been called results in undefined behavior.
vfs_file_t vfs_create_file(const vfs_filename_t filename, vfs_read_cb_t read_cb, vfs_write_cb_t write_cb, uint32_t len);


vfs_file_t vfs_create_subdir(const vfs_filename_t dir_name, uint32_t entries, vfs_read_cb_t read_cb, vfs_write_cb_t write_cb);

uint32_t vfs_create_entry(FatDirectoryEntry_t *de, uint32_t len);

// Set the attributes of a file
void vfs_file_set_attr(vfs_file_t file, vfs_file_attr_bit_t attr);

// Get the starting sector of this file.
// NOTE - If the file size is 0 there is no starting
// sector so VFS_INVALID_SECTOR will be returned.
vfs_sector_t vfs_file_get_start_sector(vfs_file_t file);

uint32_t write_clusters(uint32_t clusters_to_write);
int get_fat_entry(uint32_t cluster);

// Get the size of the file.
uint32_t vfs_file_get_size(vfs_file_t file);

// Get the attributes of a file
vfs_file_attr_bit_t vfs_file_get_attr(vfs_file_t file);

// Set the callback when a file is created, deleted or has atributes changed.
void vfs_set_file_change_callback(vfs_file_change_cb_t cb);

// Read one or more sectors from the virtual filesystem
int vfs_read(uint32_t sector, uint8_t *buf, uint32_t num_of_sectors);

// Write one or more sectors to the virtual filesystem
int vfs_write(uint32_t sector, const uint8_t *buf, uint32_t num_of_sectors);

uint32_t cluster_to_sector(uint32_t cluster_idx);

#ifdef __cplusplus
}
#endif

#endif
