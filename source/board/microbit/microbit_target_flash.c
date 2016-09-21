/**
 * @file    target_flash.c
 * @brief   Implementation of target_flash.h
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

#include "target_config.h"
#include "target_reset.h"
#include "gpio.h"
#include "validation.h"
#include "target_config.h"
#include "intelhex.h"
#include "swd_host.h"
#include "flash_intf.h"
#include "util.h"
#include "settings.h"
#include "microbit_util.h"

#define PERSIST_X                   0x0003BC00
#define PERSIST_Y                   0x0003B800

#define PAGE_CHECK_BUFF_SIZE        16

uint32_t flash_persist_x = 0;
uint32_t flash_persist_y = 0;

uint8_t microbit_page_buffer[MICROBIT_PAGE_BUFFER_SIZE];

extern const flash_intf_t *const flash_intf_target;

// this is a naiive assumption, a page could be empty, but could be actively
// used at runtime, however this is better than nothing.
int page_empty(uint32_t page_num)
{
    uint8_t empty_byte = 0xFF;
    
    uint16_t byte_offset = 0;
    
    uint8_t bytes[PAGE_CHECK_BUFF_SIZE];
    
    while(byte_offset < 1024)
    {
        swd_read_memory(page_num + byte_offset, bytes, PAGE_CHECK_BUFF_SIZE);
        
        for(int i = 0; i < PAGE_CHECK_BUFF_SIZE; i++)
            if(bytes[i] != empty_byte)
                return 0;
        
        byte_offset += PAGE_CHECK_BUFF_SIZE;
    }
    
    return 1;
}

error_t board_target_flash_erase_chip(void)
{   
    if(flash_persist_x)
        swd_read_memory(flash_persist_x, microbit_page_buffer, 1024);
    
    if(flash_persist_y)
        swd_read_memory(flash_persist_y, microbit_page_buffer + 1024, 1024);
	
    error_t status = ERROR_SUCCESS;
    const program_target_t * const flash = target_device.flash_algo;
    if (0 == swd_flash_syscall_exec(&flash->sys_call_s, flash->erase_chip, 0, 0, 0, 0)) {
        return ERROR_ERASE_ALL;
    }

    // Reset and re-initialize the target after the erase if required
    if (target_device.erase_reset) {
        status = flash_intf_target->init();
    }
    
    if(flash_persist_x)
        flash_intf_target->program_page(flash_persist_x, microbit_page_buffer, 1024);
    
    if(flash_persist_y)
        flash_intf_target->program_page(flash_persist_y, microbit_page_buffer + 1024, 1024);
    
    return status;
}
