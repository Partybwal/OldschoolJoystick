/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico.h"
#include "pico/stdio_semihosting.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"

#define ENABLE_SEMIHOSTING 0

#if ENABLE_SEMIHOSTING == 1
  #define printf(...) printf(__VA_ARGS__)
#else
  #define printf(...)
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};


static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(void);

void setup_io() {
    uint input_mask = 0xFFFFFFFF;
    input_mask &= ~(1 << 0); // GP0 is output
    input_mask &= ~(1 << 1); // GP1 is output
    input_mask &= ~(1 << 25); // GP25 is output

    // gpio_init_mask(input_mask);

    for (uint8_t i = 2; i < 32; i++) {
      switch(i) {
        case 23:
        case 24:
        case 25:
            break;
        
        default:
            gpio_init(i);
            gpio_set_dir(i, GPIO_IN);
            gpio_pull_up(i);
            break;
      }
    }

    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

// This buffer contains two values, the first one is the last read values from the pins and
// the second one is the last value sent to the USB host.
uint32_t button_buffer[3] = {0, 0, 0};

void read_joysticks() {
    uint32_t raw_values = gpio_get_all();
    button_buffer[2] = raw_values;

    raw_values = raw_values >> 2;

    uint8_t joys[4];
    joys[0]  = (uint8_t)(raw_values & 63) ^ 63; // GPIO 2-7
    raw_values = raw_values >> 6;
    joys[1] = (uint8_t)(raw_values & 63) ^ 63;  // GPIO 8-13
    raw_values = raw_values >> 6;
    joys[2] = (uint8_t)(raw_values & 63) ^ 63;  // GPIO 14-19
    raw_values = raw_values >> 6;

    joys[3]  = (uint8_t)(raw_values & 7); // GPIO 20-22
    raw_values = raw_values >> 3;
    joys[3] |= (uint8_t)(raw_values & (7 << 3)); // GPIO 26-28
    joys[3] = joys[3] ^ 63;

    uint32_t tmp = 0;
    for (uint8_t i = 0; i < 4; i++) {
      tmp = tmp << 8;
      tmp |= joys[i];
    }

    button_buffer[0] = tmp;
}

void read_task() {
    while (1) {
      read_joysticks();
      sleep_ms(1);
      led_blinking_task();
    }
}

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

#if ENABLE_SEMIHOSTING == 1
  stdio_semihosting_init();
  stdio_set_translate_crlf(&stdio_semihosting, false);
#endif

  setup_io();
  tusb_init();

  printf("Oldschool Joystick\n");

  multicore_reset_core1();
  multicore_launch_core1(read_task);

  while (1)
  {
    tud_task(); // tinyusb device task
    hid_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id)
{

  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  hid_gamepad_report_t report =
  {
    .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
    .hat = 0, .buttons = 0
  };

  uint8_t joy =       (button_buffer[0] >> ((report_id-1) * 8)) & 0xff;
  uint8_t last_send = (button_buffer[1] >> ((report_id-1) * 8)) & 0xff;

  if (joy || joy != last_send) {
    // report.buttons = joy;
    printf("Sending report %d\n", report_id);
    if (joy & 0x1) {
      report.x = INT8_MIN; // left = GPIO (report_id-1)*8 + 2
    } else if (joy & 0x2) {
      report.x = INT8_MAX; // right = GPIO (report_id-1)*8 + 3
    } else {
      report.x = 0;
    }

    if (joy & 0x4) {
      report.y = INT8_MIN; // up = GPIO (report_id-1)*8 + 4
    } else if (joy & 0x8) {
      report.y = INT8_MAX; // down = GPIO (report_id-1)*8 + 5
    } else {
      report.y = 0;
    }

    report.buttons = (joy >> 4) & 0x3; // GPIO (report_id-1)*8 + 6,7

    tud_hid_report(report_id, &report, sizeof(report));
    button_buffer[1] &= ~(0xff << ((report_id-1) * 8));
    button_buffer[1] |=  (joy << ((report_id-1) * 8));
  } else {
    report_id++;
    if (report_id < REPORT_ID_COUNT) {
      send_hid_report(report_id);
    }
  }

#if 0
  if ( btn ) //upBtn )
  switch (report_id)
  {
    case REPORT_ID_GAMEPAD_1:
      report.hat = GAMEPAD_HAT_UP;
      report.y = -127;
      break;
    case REPORT_ID_GAMEPAD_2:
      report.hat = GAMEPAD_HAT_DOWN;
      report.y = 127;
      break;
    case REPORT_ID_GAMEPAD_3:
      report.hat = GAMEPAD_HAT_RIGHT;
      report.x = 127;
      break;
    case REPORT_ID_GAMEPAD_4:
      report.hat = GAMEPAD_HAT_LEFT;
      report.x = -127;
      break;
  }
  
  if (report.buttons != lastReport[report_id-1].buttons 
    || report.x != lastReport[report_id-1].x
    || report.y != lastReport[report_id-1].y
    || report.hat != lastReport[report_id-1].hat)
  {
    tud_hid_report(report_id, &report, sizeof(report));
    lastReport[report_id-1] = report;
  }
#endif
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_GAMEPAD_1);
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;

  uint8_t next_report_id = report[0] + 1;

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id);
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
