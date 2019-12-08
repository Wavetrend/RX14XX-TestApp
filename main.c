/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.145.0
        Device            :  PIC24FJ256GB206
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36b
        MPLAB 	          :  MPLAB X v5.25
 */

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

/**
  Section: Included Files
 */
#include <stdio.h>
#include <stdbool.h>

#include <xc.h>
#include "mcc_generated_files/system.h"

#include "wt_assert.h"
#include "wt_error.h"
#include "wt_task.h"

#include "wtio.h"
#include "wtapi.h"
#include "wtapi_xbee.h"
#include "wtapi_host.h"
#include "wtapi_xbee_receive_packet.h"

#include "wthal_system_pic24.h"
#include "wthal_timer_pic24.h"
#include "wthal_counter.h"
#include "wthal_observers.h"
#include "wthal_isr_pic24.h"
#include "wthal_gpio_pic24.h"
#include "wthal_uart_pic24.h"
#include "wt_rx1400_app_task.h"
#include "wt_rx1400_clock.h"

/*
                         Main application
 */

//////////////////////////////// STDOUT ///////////////////////////////////////

static wthal_uart_t * wthal_stdout_uart = NULL;

int __attribute__((__section__(".libc.write"))) write(int handle, void * buffer, unsigned int len) {
  if (wthal_stdout_uart != NULL) {
      return (int) wthal_uart_write(wthal_stdout_uart, buffer, len, NULL);
  }
  return 0;
}

bool wthal_set_stdout(wthal_uart_t * const uart, wt_error_t * const error) {
  wthal_stdout_uart = uart;
  return true;
}

///////////////////////////////// WTIO_UART ///////////////////////////////////////

typedef struct {

  wtio_t impl;
  wthal_uart_t * uart;

} wtio_uart_t;

static bool wtio_uart_open_impl(void * const context, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  return wthal_uart_open(instance->uart, error);
}

static bool wtio_uart_close_impl(void * const context, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  return wthal_uart_close(instance->uart, error);
}

static size_t wtio_uart_read_impl(void * const context, void * const data, size_t const size, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  return wthal_uart_read(instance->uart, data, size, error);
}

static size_t wtio_uart_write_impl(void * const context, void const * const data, size_t const size, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  return wthal_uart_write(instance->uart, data, size, error);
}

static bool wtio_uart_baudrate_impl(void * const context, uint32_t const baudrate, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  return wthal_uart_baudrate(instance->uart, baudrate, error);
}

static bool wtio_uart_flowcontrol_impl(void * const context, bool const flowcontrol, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  return wthal_uart_flowcontrol(instance->uart, flowcontrol, error);
}

wtio_t * wtio_uart_init(
  wtio_uart_t * const instance,
  wthal_uart_t * const uart,
  wt_error_t * const error
) {
  bool ok = true;

  ok = !ok ? ok : wt_assert_ptr(instance, error);

  if (ok) {
    instance->uart = uart;
    instance->impl.context = instance;
    instance->impl.open = wtio_uart_open_impl;
    instance->impl.close = wtio_uart_close_impl;
    instance->impl.read = wtio_uart_read_impl;
    instance->impl.write = wtio_uart_write_impl;
    instance->impl.baudrate = wtio_uart_baudrate_impl;
    instance->impl.flowcontrol = wtio_uart_flowcontrol_impl;
  }

  return ok ? &instance->impl : NULL;
}



///////////////////////////////// MAIN ///////////////////////////////////////

#define PPS_UNLOCK()    __builtin_write_OSCCONL(OSCCON & 0xbf)
#define PPS_LOCK()      __builtin_write_OSCCONL(OSCCON | 0x40)

#define XTAL (14745600)

WTHAL_UART_PIC24_DECLARE(wt_rx14xx_xbee_uart);
WTHAL_UART_PIC24_DEFINE(wt_rx14xx_xbee_uart, 1, XTAL);
WTHAL_UART_PIC24_DECLARE(wt_rx14xx_debug_uart);
WTHAL_UART_PIC24_DEFINE(wt_rx14xx_debug_uart, 2, XTAL);
WTHAL_UART_PIC24_DECLARE(wt_rx14xx_primary_ethernet_uart);
WTHAL_UART_PIC24_DEFINE(wt_rx14xx_primary_ethernet_uart, 3, XTAL);
WTHAL_UART_PIC24_DECLARE(wt_rx14xx_secondary_ethernet_uart);
WTHAL_UART_PIC24_DEFINE(wt_rx14xx_secondary_ethernet_uart, 4, XTAL);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_led1);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_led1, _RD7, _TRISD7, _LATD7, _ANSD7, _CN16PUE, _CN16PDE, _ODD7);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_led2);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_led2, _RD6, _TRISD6, _LATD6, _ANSD6, _CN15PUE, _CN15PDE, _ODD6);
WTHAL_GPIO_PIC24_DECLARE(wt_rx1400_ethernet_reset);
WTHAL_GPIO_PIC24_DEFINE(wt_rx1400_ethernet_reset, _RB2, _TRISB2, _LATB2, _ANSB2, _CN4PUE, _CN4PDE, _ODB2);
WTHAL_ISR_PIC24_DECLARE(wt_rx14xx_tmr5);
WTHAL_ISR_PIC24_DEFINE(wt_rx14xx_tmr5, _T5Interrupt, _T5IF, _T5IE, _T5IP);
WTHAL_TIMER_PIC24_DECLARE(wt_rx14xx_timer5);
WTHAL_TIMER_PIC24_DEFINE(wt_rx14xx_timer5, XTAL, TMR5, PR5, T5CONbits.TON, T5CONbits.TCKPS, _T5IF);
WTHAL_SYSTEM_PIC24_DECLARE(wt_rx14xx_system);
WTHAL_SYSTEM_PIC24_DEFINE(wt_rx14xx_system, RCON, ClrWdt(), Nop(), asm("reset"));

typedef struct {

  wthal_system_t * system;
  wthal_isr_t * t5_isr;
  wthal_gpio_t * startup_led;
  wthal_gpio_t * activity_led;
  wthal_gpio_t * ethernet_reset;
  wthal_timer_t * timer5;
  wthal_counter_t * counter;
  wthal_clock_t * clock;
  wthal_uart_t * debug_uart;
  wthal_uart_t * xbee_uart;
  wthal_uart_t * primary_ethernet_uart;
  wthal_uart_t * secondary_ethernet_uart;

} wt_hal_t;

typedef struct {

  wt_hal_t hal;

  wthal_observer_t t5_observers[3];

  wt_rx14xx_system_t system;

  wt_rx14xx_tmr5_t t5_isr;
  wt_rx14xx_timer5_t timer5;
  wthal_counter_t counter;
  wt_rx1400_clock_t clock;

  wt_rx14xx_led1_t startup_led;
  wt_rx14xx_led2_t activity_led;

  wt_rx14xx_xbee_uart_t xbee_uart;

  wt_rx14xx_debug_uart_t debug_uart;

  wt_rx1400_ethernet_reset_t ethernet_reset;
  wt_rx14xx_primary_ethernet_uart_t primary_ethernet_uart;
  wt_rx14xx_secondary_ethernet_uart_t secondary_ethernet_uart;

} wt_rx1400_hal_t;

wt_hal_t * const wt_rx1400_hal_init(wt_rx1400_hal_t * const instance, wt_error_t * const error) {

  bool ok = true;

  enum {
    wt_rx1400_isr_priority_timer = 4,
    wt_rx1400_isr_priority_debug_uart_tx = 4,
    wt_rx1400_isr_priority_debug_uart_rx = 4,
    wt_rx1400_isr_priority_debug_uart_err = 4,
    wt_rx1400_isr_priority_xbee_uart_tx = 4,
    wt_rx1400_isr_priority_xbee_uart_rx = 4,
    wt_rx1400_isr_priority_xbee_uart_err = 4,
    wt_rx1400_isr_priority_primary_ethernet_uart_tx = 4,
    wt_rx1400_isr_priority_primary_ethernet_uart_rx = 4,
    wt_rx1400_isr_priority_primary_ethernet_uart_err = 4,
    wt_rx1400_isr_priority_secondary_ethernet_uart_tx = 4,
    wt_rx1400_isr_priority_secondary_ethernet_uart_rx = 4,
    wt_rx1400_isr_priority_secondary_ethernet_uart_err = 4,
  };

  PPS_UNLOCK();

  // UART 1 - XBEE
  _U1RXR = 8; // UART1:U1RX->RP8/RB8
  _RP9R = _RPOUT_U1TX; // RP9/RB9->UART1:U1TX
  _U1CTSR = 1; // UART1:U1CTS->RP1/RB1
  _RP0R = _RPOUT_U1RTS; // RP0/RB0->UART1:U1RTS

  // UART 2 - DEBUG
  _U2RXR = 23; // UART2:U2RX->RP23/RD2
  _RP22R = _RPOUT_U2TX; // RP22/RD3->UART2:U2TX

  // UART 3 - Primary Ethernet
  _U3RXR = 4; // UART3:U3RX->RP4/RD9
  _RP11R = _RPOUT_U3TX; // RP11/RD0->UART3:U3TX
  _U3CTSR = 12; // UART3:U3CTS->RP12/RD11
  _RP3R = _RPOUT_U3RTS; // RP3/RD10->UART3:U3RTS

  // UART 4 - Secondary Ethernet
  _U4RXR = 25; // UART4:U4RX->RP25/RD4
  _RP20R = _RPOUT_U4TX; // RP20/RD5->UART4:U4TX

  PPS_LOCK();

  ok = !ok ? ok : (instance->hal.system = wt_rx14xx_system_init(&instance->system, error)) != NULL;

  ok = !ok ? ok : (instance->hal.t5_isr = wt_rx14xx_tmr5_init(&instance->t5_isr, wt_rx1400_isr_priority_timer, instance->t5_observers, sizeof (instance->t5_observers) / sizeof (instance->t5_observers[0]), error)) != NULL;
  ok = !ok ? ok : (instance->hal.timer5 = wt_rx14xx_timer5_init(&instance->timer5, error)) != NULL;
  ok = !ok ? ok : (instance->hal.counter = wthal_counter_init(&instance->counter, error)) != NULL;
  ok = !ok ? ok : (instance->hal.clock = wt_rx1400_clock_init(&instance->clock, instance->hal.timer5, instance->hal.t5_isr, instance->hal.counter, 10, error)) != NULL;

  ok = !ok ? ok : (instance->hal.startup_led = wt_rx14xx_led1_init(&instance->startup_led, error)) != NULL;
  ok = !ok ? ok : (instance->hal.activity_led = wt_rx14xx_led2_init(&instance->activity_led, error)) != NULL;

  ok = !ok ? ok : (instance->hal.xbee_uart = wt_rx14xx_xbee_uart_init(&instance->xbee_uart, 9600, true, wt_rx1400_isr_priority_xbee_uart_tx, wt_rx1400_isr_priority_xbee_uart_rx, wt_rx1400_isr_priority_xbee_uart_err, error)) != NULL;

  ok = !ok ? ok : (instance->hal.debug_uart = wt_rx14xx_debug_uart_init(&instance->debug_uart, 230400, true, wt_rx1400_isr_priority_debug_uart_tx, wt_rx1400_isr_priority_debug_uart_rx, wt_rx1400_isr_priority_debug_uart_err, error)) != NULL;

  ok = !ok ? ok : (instance->hal.ethernet_reset = wt_rx1400_ethernet_reset_init(&instance->ethernet_reset, error)) != NULL;
  ok = !ok ? ok : (instance->hal.primary_ethernet_uart = wt_rx14xx_primary_ethernet_uart_init(&instance->primary_ethernet_uart, 115200, true, wt_rx1400_isr_priority_primary_ethernet_uart_tx, wt_rx1400_isr_priority_primary_ethernet_uart_rx, wt_rx1400_isr_priority_primary_ethernet_uart_err, error)) != NULL;
  ok = !ok ? ok : (instance->hal.secondary_ethernet_uart = wt_rx14xx_secondary_ethernet_uart_init(&instance->secondary_ethernet_uart, 115200, false, wt_rx1400_isr_priority_secondary_ethernet_uart_tx, wt_rx1400_isr_priority_secondary_ethernet_uart_rx, wt_rx1400_isr_priority_secondary_ethernet_uart_err, error)) != NULL;

  ok = !ok ? ok : wthal_gpio_weak_pull_up(instance->hal.ethernet_reset, true, error);
  ok = !ok ? ok : wthal_gpio_set(instance->hal.ethernet_reset, true, error);

  ok = !ok ? ok : wthal_uart_open(instance->hal.debug_uart, error);

  ok = !ok ? ok : wthal_set_stdout(instance->hal.debug_uart, error);

  return ok ? &instance->hal : NULL;

}

////////////////////////////// XBEE DISPATCH //////////////////////////////////

static wtapi_dispatch_entry_t xbee_opcode_dispatch_table[] = {
    WTAPI_DISPATCH_ENTRY_FINAL,
};

static bool is_xbee_receive_packet(
  void const * const data,
  size_t const size,
  void * const context,
  wt_error_t * const error
) {
  return *((wtapi_xbee_frame_type_t *) data) == WTAPI_XBEE_FRAME_TYPE_RECEIVE_PACKET;
}

static void xbee_packet_handler(
  void const * const data,
  size_t const size,
  void * const context,
  wt_error_t * const error
) {
#ifndef DISABLE_XBEE_PACKET_HANDLER
  wtapi_xbee_receive_packet_t packet;
  wtapi_xbee_receive_packet_init_from_data(&packet, ((uint8_t *) data), size, error);
  //  debug_dump(WT_RX1410_DEBUG_MESH_TASK, "DT <<", packet.header.data, packet.size);
  wtapi_dispatch(xbee_opcode_dispatch_table, &packet.header.fields.address, packet.header.data, packet.size, context, error);
#endif
}

////////////////////////////// HOST DISPATCH //////////////////////////////////

static wtapi_dispatch_entry_t host_opcode_dispatch_table[] = {
  WTAPI_DISPATCH_ENTRY_FINAL,
};

static wt_dispatch_entry_t xbee_dispatch_table[] = {
  { is_xbee_receive_packet, xbee_packet_handler},
  WT_DISPATCH_TABLE_FINAL,
};

static bool is_host_receive_packet(
  void const * const data,
  size_t const size,
  void * const context,
  wt_error_t * const error
) {
    return true;
}

static void host_packet_handler(
  void const * const data,
  size_t const size,
  void * const context,
  wt_error_t * const error
) {
#ifndef DISABLE_HOST_PACKET_HANDLER
  wtapi_host_frame_t const * const frame = data;
  uint8_t buffer[WTAPI_HOST_FRAME_DATA_MAX + 1U];
  buffer[0] = frame->opcode;
  size_t length = frame->length - sizeof (wtapi_address_t);
  memcpy(&buffer[1], frame->data, length);
  wtapi_dispatch(host_opcode_dispatch_table, &frame->address, buffer, length, context, error);
#endif
}

static wt_dispatch_entry_t host_dispatch_table[] = {
  { is_host_receive_packet, host_packet_handler},
  WT_DISPATCH_TABLE_FINAL,
};

static void device_reset(void * const context) {
  wthal_system_t * const system = context;
  (void) wthal_system_reset(system, NULL);
}

int main(void) {
  wt_error_t error;

  // initialize the device - MUST COME FIRST OR WILL OVERRIDE HAL STYLE CONFIG
  SYSTEM_Initialize();

  bool ok = true;

  ok = !ok ? ok : wt_error_init(&error);

  // HAL
  wt_rx1400_hal_t rx1400_hal;
  wt_hal_t * hal = ok ? wt_rx1400_hal_init(&rx1400_hal, &error) : NULL;
  ok = (hal != NULL);

  // XBEE API
  wtio_uart_t xbee_uart;
  wtapi_xbee_t xbee_proto;
  wtapi_t xbee_api;

  ok = !ok ? ok : (wtio_uart_init(&xbee_uart, hal->xbee_uart, &error) != NULL);
  ok = !ok ? ok : wtapi_xbee_init(&xbee_proto, xbee_dispatch_table, NULL, &xbee_uart.impl, 0, 0, hal->clock, &error);
  ok = !ok ? ok : wtapi_init(&xbee_api, &xbee_proto, &xbee_proto.impl, hal->clock, &error);
  ok = !ok ? ok : wtapi_open(&xbee_api, &error);

  // HOST PRIMARY API
  wtio_uart_t host_primary_uart;
  wtapi_host_t host_primary_proto;
  wtapi_t host_primary_api;

  ok = !ok ? ok : (wtio_uart_init(&host_primary_uart, hal->primary_ethernet_uart, &error) != NULL);
  ok = !ok ? ok : wtapi_host_init(&host_primary_proto, host_dispatch_table, NULL, &host_primary_uart.impl, &error);
  ok = !ok ? ok : wtapi_init(&host_primary_api, &host_primary_proto, &host_primary_proto.impl, hal->clock, &error);
  ok = !ok ? ok : wtapi_open(&host_primary_api, &error);

  // HOST SECONDARY API
  wtio_uart_t host_secondary_uart;
  wtapi_host_t host_secondary_proto;
  wtapi_t host_secondary_api;

  ok = !ok ? ok : (wtio_uart_init(&host_secondary_uart, hal->secondary_ethernet_uart, &error) != NULL);
  ok = !ok ? ok : wtapi_host_init(&host_secondary_proto, host_dispatch_table, NULL, &host_secondary_uart.impl, &error);
  ok = !ok ? ok : wtapi_init(&host_secondary_api, &host_secondary_proto, &host_secondary_proto.impl, hal->clock, &error);
  ok = !ok ? ok : wtapi_open(&host_secondary_api, &error);

  wt_rx1400_app_task_t app;

  ok = (wt_rx1400_app_task_init(&app, hal->clock, hal->activity_led, hal->ethernet_reset, &xbee_api, &host_primary_api, &host_secondary_api, &error) != NULL);

  while (ok && wt_task_incomplete(&app.task)) {
    ok = !ok ? ok : wthal_system_clear_watchdog_timer(hal->system, &error);
    ok = !ok ? ok : wt_task_spin(&app.task, &error);
    if (!ok && !error.acknowledged) {
      printf("\nERROR %d, File:%s, Line:%d", error.error_code, error.file, error.line);
      error.acknowledged = true;
      ok = wthal_clock_set_alarm(hal->clock, device_reset, hal->system, 3000, &error);
    }
    wthal_system_nop(hal->system, &error);
  }

  return error.error_code;

}
/**
 End of File
 */

