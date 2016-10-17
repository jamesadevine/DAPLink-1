/**
 * @file    main.c
 * @brief   Entry point for interface program logic
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
#include "stdio.h"
#include "stdint.h"

#include "RTL.h"
#include "rl_usb.h"
#include "main.h"
#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "serial.h"
#include "tasks.h"
#include "target_reset.h"
#include "swd_host.h"
#include "info.h"
#include "vfs_manager.h"
#include "settings.h"
#include "daplink.h"
#include "util.h"
#include "DAP.h"
#include "jmx.h"

// Reference to our main task
OS_TID main_task_id = 0;
OS_TID serial_task_id = 0;

//used for controlling when the usb serial thread can transmit
uint8_t usb_tx_flag = 1;

// USB busy LED state; when TRUE the LED will flash once using 30mS clock tick
static uint8_t hid_led_usb_activity = 0;
static uint8_t cdc_led_usb_activity = 0;
static uint8_t msc_led_usb_activity = 0;
static main_led_state_t hid_led_state = MAIN_LED_FLASH;
static main_led_state_t cdc_led_state = MAIN_LED_FLASH;
static main_led_state_t msc_led_state = MAIN_LED_FLASH;

// Global state of usb
main_usb_connect_t usb_state;

static U64 stk_timer_30_task[TIMER_TASK_30_STACK / sizeof(U64)];
static U64 stk_dap_task[DAP_TASK_STACK / sizeof(U64)];
static U64 stk_serial_task[SERIAL_TASK_STACK / sizeof(U64)];
static U64 stk_main_task[MAIN_TASK_STACK / sizeof(U64)];

// Timer task, set flags every 30mS and 90mS
__task void timer_task_30mS(void)
{
    uint8_t i = 0;
    os_itv_set(3); // 30mS

    while (1) {
        os_itv_wait();
        os_evt_set(FLAGS_MAIN_30MS, main_task_id);

        if (!(i++ % 3)) {
            os_evt_set(FLAGS_MAIN_90MS, main_task_id);
        }
    }
}

// Forward reset from the user pressing the reset button
// Boards which tie the reset pin directly to the target
// should override this function with a stub that does nothing
__attribute__((weak))
void target_forward_reset(bool assert_reset)
{
    if (assert_reset) {
        target_set_state(RESET_HOLD);
    } else {
        target_set_state(RESET_RUN);
    }
}

// Functions called from other tasks to trigger events in the main task
// parameter should be reset type??
void main_reset_target(uint8_t send_unique_id)
{
    os_evt_set(FLAGS_MAIN_RESET, main_task_id);
    return;
}

// Flash HID LED using 30mS tick
void main_blink_hid_led(main_led_state_t permanent)
{
    hid_led_usb_activity = 1;
    hid_led_state = (permanent) ? MAIN_LED_FLASH_PERMANENT : MAIN_LED_FLASH;
    return;
}

// Flash CDC LED using 30mS tick
void main_blink_cdc_led(main_led_state_t permanent)
{
    cdc_led_usb_activity = 1;
    cdc_led_state = (permanent) ? MAIN_LED_FLASH_PERMANENT : MAIN_LED_FLASH;
    return;
}

// Flash MSC LED using 30mS tick
void main_blink_msc_led(main_led_state_t permanent)
{
    msc_led_usb_activity = 1;
    msc_led_state = (permanent) ? MAIN_LED_FLASH_PERMANENT : MAIN_LED_FLASH;
    return;
}

// Power down the interface
void main_powerdown_event(void)
{
    os_evt_set(FLAGS_MAIN_POWERDOWN, main_task_id);
    return;
}

// Disable debug on target
void main_disable_debug_event(void)
{
    os_evt_set(FLAGS_MAIN_DISABLEDEBUG, main_task_id);
    return;
}

void USBD_SignalHandler()
{
    isr_evt_set(FLAGS_MAIN_PROC_USB, main_task_id);
}

void HardFault_Handler()
{
    util_assert(0);
    NVIC_SystemReset();

    while (1); // Wait for reset
}

os_mbx_declare(serial_mailbox, 20);
#define SIZE_DATA (64)
static uint8_t data[SIZE_DATA];

extern void jmx_packet_received(char*);


__task void serial_process()
{
    UART_Configuration config;
    int32_t len_data = 0;
    int jmx_result = -1;
    int jmx_prev = -1;
    void *msg;

    uint8_t board_vfs = board_vfs_enabled();
    uint8_t usb_connected = terminal_connected();
    
    while (1) {
        
        board_vfs = board_vfs_enabled();
        usb_connected = terminal_connected();
		
        // Check our mailbox to see if we need to set anything up with the UART
        // before we do any sending or receiving
        if (os_mbx_wait(&serial_mailbox, &msg, 0) == OS_R_OK) {
            
            // if our extended vfs is enabled, we pass control of the uart
            // to the user side application running on the target.
            // JMX is in use, and any change of state of the uart interface 
            // would be bad...
            
            
            switch ((SERIAL_MSG)(unsigned)msg) {
                case SERIAL_INITIALIZE:
                    if(!board_vfs)
                        uart_initialize();
                    
                    //
                    break;

                case SERIAL_UNINITIALIZE:
                    if(!board_vfs)
                        uart_uninitialize();
                    break;
                    
                case SERIAL_RESET:
                    
                    if(!board_vfs)
                        uart_reset();
					
                    break;

                case SERIAL_SET_CONFIGURATION:
                    if(!board_vfs)
                    {
                        serial_get_configuration(&config);
                        uart_set_configuration(&config);
                    }
                    break;

                default:
                    break;
            }
        }

        len_data = USBD_CDC_ACM_DataFree();

        if (len_data > SIZE_DATA) {
            len_data = SIZE_DATA;
        }
        
        uint8_t c = 0;
        int read_count = 0;
        
        // if we aren't connected and our VFS is enabled, we digest anyway...
        // if we are connected and our VFS is enabled, we digest
        // if we aren't connected and our VFS is disabled, do nothing
        // this read is not contrained by len_data...
        // we do a best effort copy into our usb serial buffer
        if(board_vfs && !usb_connected)
        {
            while(read_count < SIZE_DATA && uart_read_data(&c,1))
            {
                jmx_prev = jmx_result;
                
                char characters[2] = {jmx_previous(), c};
                
                jmx_result = jmx_state_track(c);
                
                if(jmx_prev != 0 && characters[0] == SLIP_ESC && (characters[1] == 'A' || characters[1] == 'R' || characters[1] == 'C'))
                {
                    if(characters[1] == 'A')
                        jmx_packet_received("stat_ack");
                    
                    if(characters[1] == 'R')
                        jmx_packet_received("stat_retry");
                    
                    if(characters[1] == 'C')
                        jmx_packet_received("stat_cancel");
                    
                    os_evt_set(FLAGS_JMX_PACKET, main_task_id);
                    continue;
                }
                else if(jmx_result == 0)
                    os_evt_set(FLAGS_JMX_PACKET, main_task_id);
                
                if(jmx_result == -1)
                    continue;
                
                // SLIP escape sequence detection...
                if(jmx_result == 2 && is_slip_character(characters[0]) == ESC)
                {
                    int ret = is_slip_character(characters[1]);
                    
                    if(ret == ESC_END)
                    {
                        characters[1] = SLIP_END;
                        jmx_result = 1;
                    }
                    
                    if(ret == ESC_ESC)
                    {
                        characters[1] = SLIP_ESC;
                        jmx_result = 1;
                    }
                }
                
                for(int i = 2 - jmx_result; i < 2; i++)
                    if(read_count + 1 < SIZE_DATA)
                        data[read_count++] = characters[i];
            }
        }
        // this read is constrained by len_data
        else if(board_vfs && usb_connected)
        {
            while(read_count < len_data && (uart_read_data(&c, 1)))
            {
                char characters[2] = {jmx_previous(), c};
                
                jmx_result = jmx_state_track(c);
                
                if(characters[0] == SLIP_ESC && (characters[1] == 'A' || characters[1] == 'R' || characters[1] == 'C'))
                {
                    if(characters[1] == 'A')
                        jmx_packet_received("stat_ack");
                    
                    if(characters[1] == 'R')
                        jmx_packet_received("stat_retry");
                    
                    if(characters[1] == 'C')
                        jmx_packet_received("stat_cancel");
                    
                    os_evt_set(FLAGS_JMX_PACKET, main_task_id);
                    continue;
                }
                else if(jmx_result == 0)
                    os_evt_set(FLAGS_JMX_PACKET, main_task_id);
                
                if(jmx_result == -1)
                    continue;
                
                // SLIP escape sequence detection...
                if(jmx_result == 2 && is_slip_character(characters[0]) == ESC)
                {
                    int ret = is_slip_character(characters[1]);
                    
                    if(ret == ESC_END)
                    {
                        characters[1] = SLIP_END;
                        jmx_result = 1;
                    }
                    
                    if(ret == ESC_ESC)
                    {
                        characters[1] = SLIP_ESC;
                        jmx_result = 1;
                    }
                }
                
                for(int i = 2 - jmx_result; i < 2; i++)
                    if(read_count + 1 < SIZE_DATA)
                        data[read_count++] = characters[i];
            }
                
        }
        // this read caters for legacy firmware
        else if(len_data && !board_vfs)
            read_count = uart_read_data(data, len_data);
        
        if(read_count)
        {
            if (USBD_CDC_ACM_DataSend(data, read_count)) {
                main_blink_cdc_led(MAIN_LED_OFF);
            }
        }

        //returns the number of buffered characters 
        len_data = uart_write_free();
        
        if(!usb_tx_flag)
            continue;

        if (len_data > SIZE_DATA) {
            len_data = SIZE_DATA;
        }

        if (len_data) {
            //read from usb
            len_data = USBD_CDC_ACM_DataRead(data, len_data);
        }

        if (len_data) {
            
            if(board_vfs)
            {
                uint8_t forward[2] = {SLIP_ESC, 0};
                
                int forward_count = 1;
                
                for(int i = 0; i < len_data; i++)
                {
                    forward_count = 1;
                    int ret = is_slip_character(data[i]);
                        
                    if(ret == END)
                    {
                        forward_count = 2;
                        forward[1] = SLIP_ESC_END;
                    } 
                    else if(ret == ESC)
                    {
                        forward_count = 2;
                        forward[1] = SLIP_ESC_ESC;
                    }
                    else
                        forward[1] = data[i];
                    
                    if(uart_write_data(forward + (2 - forward_count), forward_count))
                    {
                        main_blink_cdc_led(MAIN_LED_OFF);
                    }
                }
                
            }
            else
            {
                //forward to target.
                if (uart_write_data(data, len_data)) {
                    main_blink_cdc_led(MAIN_LED_OFF);
                }
            }
            
        }
    }
}

extern __task void hid_process(void);
__attribute__((weak)) void prerun_target_config(void) {}

__task void main_task(void)
{
    // State processing
    uint16_t flags = 0;
    // LED
    gpio_led_state_t hid_led_value = GPIO_LED_ON;
    gpio_led_state_t cdc_led_value = GPIO_LED_ON;
    gpio_led_state_t msc_led_value = GPIO_LED_ON;
    // USB
    uint32_t usb_state_count = USB_BUSY_TIME;
    // thread running after usb connected started
    uint8_t thread_started = 0;
    // button state
    main_reset_state_t main_reset_button_state = MAIN_RESET_RELEASED;
    // Initialize settings
    config_init();
    // Initialize our serial mailbox
    os_mbx_init(&serial_mailbox, sizeof(serial_mailbox));
    // Get a reference to this task
    main_task_id = os_tsk_self();
    // leds
    gpio_init();
    // Turn off LED
    gpio_set_hid_led(GPIO_LED_ON);
    gpio_set_cdc_led(GPIO_LED_ON);
    gpio_set_msc_led(GPIO_LED_ON);
    // Initialize the DAP
    DAP_Setup();
    // do some init with the target before USB and files are configured
    prerun_target_config();
    // Update versions and IDs
    info_init();
    // USB
    usbd_init();
    vfs_mngr_fs_enable(true);
    usbd_connect(0);
    usb_state = USB_CONNECTING;
    usb_state_count = USB_CONNECT_DELAY;
    // Start timer tasks
    os_tsk_create_user(timer_task_30mS, TIMER_TASK_30_PRIORITY, (void *)stk_timer_30_task, TIMER_TASK_30_STACK);

    while (1) {
        os_evt_wait_or(FLAGS_MAIN_RESET             // Put target in reset state
                       | FLAGS_MAIN_90MS            // 90mS tick
                       | FLAGS_MAIN_30MS            // 30mS tick
                       | FLAGS_MAIN_POWERDOWN       // Power down interface
                       | FLAGS_MAIN_DISABLEDEBUG    // Disable target debug
                       | FLAGS_MAIN_PROC_USB        // process usb events
                       , NO_TIMEOUT);
        // Find out what event happened
        flags = os_evt_get();

        if (flags & FLAGS_MAIN_PROC_USB) {
            USBD_Handler();
        }

        if (flags & FLAGS_MAIN_RESET) {
            //target_set_state(RESET_RUN);
        }

        if (flags & FLAGS_MAIN_POWERDOWN) {
            // Disable debug
            target_set_state(NO_DEBUG);
            // Disconnect USB
            usbd_connect(0);
            // Turn off LED
            gpio_set_hid_led(GPIO_LED_OFF);
            gpio_set_cdc_led(GPIO_LED_OFF);
            gpio_set_msc_led(GPIO_LED_OFF);

            // TODO: put the interface chip in sleep mode
            while (1);
        }

        if (flags & FLAGS_MAIN_DISABLEDEBUG) {
            // Disable debug
            target_set_state(NO_DEBUG);
        }

        if (flags & FLAGS_MAIN_90MS) {
            // Update USB busy status
            vfs_mngr_periodic(90); // FLAGS_MAIN_90MS

            // Update USB connect status
            switch (usb_state) {
                case USB_DISCONNECTING:
                    usb_state = USB_DISCONNECTED;
                    usbd_connect(0);
                    break;

                case USB_CONNECTING:

                    // Wait before connecting
                    if (DECZERO(usb_state_count) == 0) {
                        usbd_connect(1);
                        usb_state = USB_CHECK_CONNECTED;
                    }

                    break;

                case USB_CHECK_CONNECTED:
                    if (usbd_configured()) {
                        if (!thread_started) {
                            os_tsk_create_user(hid_process, DAP_TASK_PRIORITY, (void *)stk_dap_task, DAP_TASK_STACK);
                            serial_task_id = os_tsk_create_user(serial_process, SERIAL_TASK_PRIORITY, (void *)stk_serial_task, SERIAL_TASK_STACK);
                            thread_started = 1;
                        }

                        usb_state = USB_CONNECTED;
                    }

                    break;

                case USB_CONNECTED:
                case USB_DISCONNECTED:
                default:
                    break;
            }
        }

        // 30mS tick used for flashing LED when USB is busy
        if (flags & FLAGS_MAIN_30MS) {
            // handle reset button without eventing
            switch (main_reset_button_state) {
                default:
                case MAIN_RESET_RELEASED:
                    if (0 == gpio_get_sw_reset()) {
                        main_reset_button_state = MAIN_RESET_PRESSED;
                        target_forward_reset(true);
                    }

                    break;

                case MAIN_RESET_PRESSED:

                    // ToDo: add a counter to do a mass erase or target recovery after xxx seconds of being held
                    if (1 == gpio_get_sw_reset()) {
                        main_reset_button_state = MAIN_RESET_TARGET;
                    }

                    break;

                case MAIN_RESET_TARGET:
                    target_forward_reset(false);
                    main_reset_button_state = MAIN_RESET_RELEASED;
                    break;
            }

            if (hid_led_usb_activity && ((hid_led_state == MAIN_LED_FLASH) || (hid_led_state == MAIN_LED_FLASH_PERMANENT))) {
                // Flash DAP LED ONCE
                if (hid_led_value) {
                    hid_led_value = GPIO_LED_OFF;
                } else {
                    hid_led_value = GPIO_LED_ON; // Turn on

                    if (hid_led_state == MAIN_LED_FLASH) {
                        hid_led_usb_activity = 0;
                    }
                }

                // Update hardware
                gpio_set_hid_led(hid_led_value);
            }

            if (msc_led_usb_activity && ((msc_led_state == MAIN_LED_FLASH) || (msc_led_state == MAIN_LED_FLASH_PERMANENT))) {
                // Flash MSD LED ONCE
                if (msc_led_value) {
                    msc_led_value = GPIO_LED_OFF;
                } else {
                    msc_led_value = GPIO_LED_ON; // Turn on

                    if (msc_led_state == MAIN_LED_FLASH) {
                        msc_led_usb_activity = 0;
                    }
                }

                // Update hardware
                gpio_set_msc_led(msc_led_value);
            }

            if (cdc_led_usb_activity && ((cdc_led_state == MAIN_LED_FLASH) || (cdc_led_state == MAIN_LED_FLASH_PERMANENT))) {
                // Flash CDC LED ONCE
                if (cdc_led_value) {
                    cdc_led_value = GPIO_LED_OFF;
                } else {
                    cdc_led_value = GPIO_LED_ON; // Turn on

                    if (cdc_led_state == MAIN_LED_FLASH) {
                        cdc_led_usb_activity = 0;
                    }
                }

                // Update hardware
                gpio_set_cdc_led(cdc_led_value);
            }
        }
    }
}

int main(void)
{
    // Explicitly set the vector table since the bootloader might not set
    // it to what we expect.
#if DAPLINK_ROM_BL_SIZE > 0
    SCB->VTOR = SCB_VTOR_TBLOFF_Msk & DAPLINK_ROM_IF_START;
#endif
    os_sys_init_user(main_task, MAIN_TASK_PRIORITY, stk_main_task, MAIN_TASK_STACK);
}
