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
#include "wthal_i2c_pic24.h"
#include "wthal_isr_pic24.h"
#include "wthal_gpio_pic24.h"
#include "wthal_uart_pic24.h"
#include "wt_rx1400_app_task.h"
#include "wt_rx1400_clock.h"
#include "wt_rx14xx_debug.h"
#include "wt_rx1400_reader_cache.h"
#include "wthal_i2c_master_task.h"
#include "wthal_nvm_pic24.h"

/*
                         Main application
 */

///////////////////////////////// WTIO_UART ///////////////////////////////////////

typedef struct {

  wtio_t impl;
  wthal_uart_t * uart;
  
  wt_debug_t * debug;
  wt_module_debug_t module_debug;
  char const * debug_prefix;

} wtio_uart_t;

static bool wtio_uart_open_impl(void * const context, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  bool ok = wthal_uart_open(instance->uart, error);
  if (ok) {
    wt_debug_print(instance->debug, "%s Open", instance->debug_prefix);
  }
  return ok;
}

static bool wtio_uart_close_impl(void * const context, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  bool ok = wthal_uart_close(instance->uart, error);
  if (ok) {
    wt_debug_print(instance->debug, "%s Close", instance->debug_prefix);
  }
  return ok;
}

static size_t wtio_uart_read_impl(void * const context, void * const data, size_t const size, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  size_t read = wthal_uart_read(instance->uart, data, size, error);
  if (read > 0) {
    char msg[16];
    sprintf(msg, "%.13s<<", instance->debug_prefix);
    wt_debug_dump(instance->debug, msg, data, read);
  }
  return read;
}

static size_t wtio_uart_write_impl(void * const context, void const * const data, size_t const size, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  size_t written = wthal_uart_write(instance->uart, data, size, error);
  if (written > 0) {
    char msg[16];
    sprintf(msg, "%.13s>>", instance->debug_prefix);
    wt_debug_dump(instance->debug, msg, data, written);
  }
  return written;
}

static bool wtio_uart_baudrate_impl(void * const context, uint32_t const baudrate, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  bool ok = wthal_uart_baudrate(instance->uart, baudrate, error);
  if (ok) {
    wt_debug_print(instance->debug, "%s baudrate=%lu", instance->debug_prefix, baudrate);
  }
  return ok;
}

static bool wtio_uart_flowcontrol_impl(void * const context, bool const flowcontrol, wt_error_t * const error) {
  wtio_uart_t * const instance = context;
  bool ok = wthal_uart_flowcontrol(instance->uart, flowcontrol, error);
  if (ok) {
    wt_debug_print(instance->debug, "%s flowcontrol=%d", instance->debug_prefix, flowcontrol);
  }
  return ok;
}

wtio_t * wtio_uart_init(
  wtio_uart_t * const instance,
  wthal_uart_t * const uart,
  wt_debug_t * const debug,
  uint32_t const debug_module,
  char const * debug_prefix,
  wt_error_t * const error
) {
  bool ok = true;

  ok = !ok ? ok : wt_assert_ptr(instance, error);

  if (ok) {
    instance->uart = uart;
    
    instance->debug_prefix = debug_prefix;
    instance->debug = wt_module_debug_init(&instance->module_debug, debug_module, &wt_module_debug_impl, debug, error);
    instance->impl.context = instance;
    instance->impl.open = wtio_uart_open_impl;
    instance->impl.close = wtio_uart_close_impl;
    instance->impl.read = wtio_uart_read_impl;
    instance->impl.write = wtio_uart_write_impl;
    instance->impl.baudrate = wtio_uart_baudrate_impl;
    instance->impl.flowcontrol = wtio_uart_flowcontrol_impl;
  }

  ok = !ok ? ok : wt_assert_ptr(instance->debug, error);
  
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

WTHAL_I2C_PIC24_DECLARE(wt_rx14xx_i2c);
WTHAL_I2C_PIC24_DEFINE(wt_rx14xx_i2c, 3);

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_xbee_cts);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_xbee_cts, _RB0, _TRISB0, _LATB0, _ANSB0, _CN2PUE, _CN2PDE, _ODB0);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_xbee_rts);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_xbee_rts, _RB1, _TRISB1, _LATB1, _ANSB1, _CN3PUE, _CN3PDE, _ODB1);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_xbee_tx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_xbee_tx, _RB9, _TRISB9, _LATB9, _ANSB9, _CN27PUE, _CN27PDE, _ODB9);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_xbee_rx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_xbee_rx, _RB8, _TRISB8, _LATB8, _ANSB8, _CN26PUE, _CN26PDE, _ODB8);

static bool no_analogue;

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_debug_tx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_debug_tx, _RD3, _TRISD3, _LATD3, no_analogue, _CN52PUE, _CN52PDE, _ODD3);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_debug_rx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_debug_rx, _RD2, _TRISD2, _LATD2, no_analogue, _CN51PUE, _CN51PDE, _ODD2);

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ethernet_primary_cts);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ethernet_primary_cts, _RD11, _TRISD11, _LATD11, no_analogue, _CN56PUE, _CN56PDE, _ODD11);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ethernet_primary_rts);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ethernet_primary_rts, _RD10, _TRISD10, _LATD10, no_analogue, _CN55PUE, _CN55PDE, _ODD10);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ethernet_primary_tx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ethernet_primary_tx, _RD0, _TRISD0, _LATD0, no_analogue, _CN49PUE, _CN49PDE, _ODD0);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ethernet_primary_rx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ethernet_primary_rx, _RD9, _TRISD9, _LATD9, no_analogue, _CN54PUE, _CN54PDE, _ODD9);

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ethernet_secondary_tx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ethernet_secondary_tx, _RD5, _TRISD5, _LATD5, no_analogue, _CN14PUE, _CN14PDE, _ODD5);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ethernet_secondary_rx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ethernet_secondary_rx, _RD4, _TRISD4, _LATD4, no_analogue, _CN13PUE, _CN13PDE, _ODD4);

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_led1);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_led1, _RD7, _TRISD7, _LATD7, _ANSD7, _CN16PUE, _CN16PDE, _ODD7);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_led2);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_led2, _RD6, _TRISD6, _LATD6, _ANSD6, _CN15PUE, _CN15PDE, _ODD6);

WTHAL_GPIO_PIC24_DECLARE(wt_rx1400_ethernet_reset);
WTHAL_GPIO_PIC24_DEFINE(wt_rx1400_ethernet_reset, _RB2, _TRISB2, _LATB2, _ANSB2, _CN4PUE, _CN4PDE, _ODB2);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_xbee_reset);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_xbee_reset, _RB10, _TRISB10, _LATB10, _ANSB10, _CN28PUE, _CN28PDE, _ODB10);
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
  wthal_gpio_t * xbee_reset;
  wthal_gpio_t * xbee_cts;
  wthal_gpio_t * xbee_rts;
  wthal_gpio_t * xbee_tx;
  wthal_gpio_t * xbee_rx;
  wthal_gpio_t * debug_tx;
  wthal_gpio_t * debug_rx;
  wthal_gpio_t * ethernet_primary_cts;
  wthal_gpio_t * ethernet_primary_rts;
  wthal_gpio_t * ethernet_primary_tx;
  wthal_gpio_t * ethernet_primary_rx;
  wthal_gpio_t * ethernet_secondary_tx;
  wthal_gpio_t * ethernet_secondary_rx;
  wthal_timer_t * timer5;
  wthal_counter_t * counter;
  wthal_clock_t * clock;
  wthal_uart_t * debug_uart;
  wthal_uart_t * xbee_uart;
  wthal_uart_t * primary_ethernet_uart;
  wthal_uart_t * secondary_ethernet_uart;
  wthal_i2c_t * i2c;
  wthal_nvm_t * nvm;

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

  wt_rx14xx_xbee_reset_t xbee_reset;
  wt_rx14xx_xbee_cts_t xbee_cts;
  wt_rx14xx_xbee_rts_t xbee_rts;
  wt_rx14xx_xbee_tx_t xbee_tx;
  wt_rx14xx_xbee_rx_t xbee_rx;
  wt_rx14xx_xbee_uart_t xbee_uart;

  wt_rx14xx_debug_tx_t debug_tx;
  wt_rx14xx_debug_rx_t debug_rx;
  wt_rx14xx_debug_uart_t debug_uart;

  wt_rx1400_ethernet_reset_t ethernet_reset;
  wt_rx14xx_ethernet_primary_cts_t ethernet_primary_cts;
  wt_rx14xx_ethernet_primary_rts_t ethernet_primary_rts;
  wt_rx14xx_ethernet_primary_tx_t ethernet_primary_tx;
  wt_rx14xx_ethernet_primary_rx_t ethernet_primary_rx;

  wt_rx14xx_ethernet_secondary_tx_t ethernet_secondary_tx;
  wt_rx14xx_ethernet_secondary_rx_t ethernet_secondary_rx;

  wt_rx14xx_primary_ethernet_uart_t primary_ethernet_uart;
  wt_rx14xx_secondary_ethernet_uart_t secondary_ethernet_uart;

  wt_rx14xx_i2c_t i2c;
  wthal_i2c_master_task_t i2c_master_task;
  wthal_i2c_queue_t i2c_request_queue;
  wthal_i2c_request_t i2c_request_storage[16];
  wthal_nvm_pic24_t nvm;
  
  
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
    wt_rx1400_isr_priority_i2c = 4,
  };

  PPS_UNLOCK();

  // UART 1 - XBEE
  _U1RXR = 8; // UART1:U1RX->RP8/RB8
  _RP9R = _RPOUT_U1TX; // RP9/RB9->UART1:U1TX
  _U1CTSR = 0; // UART1:U1CTS->RP0/RB0
  _RP1R = _RPOUT_U1RTS; // RP1/RB1->UART1:U1RTS
  
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

  ok = !ok ? ok : (instance->hal.xbee_reset = wt_rx14xx_xbee_reset_init(&instance->xbee_reset, error)) != NULL;
  ok = !ok ? ok : (instance->hal.xbee_cts = wt_rx14xx_xbee_cts_init(&instance->xbee_cts, error)) != NULL;
  ok = !ok ? ok : (instance->hal.xbee_rts = wt_rx14xx_xbee_rts_init(&instance->xbee_rts, error)) != NULL;
  ok = !ok ? ok : (instance->hal.xbee_tx = wt_rx14xx_xbee_tx_init(&instance->xbee_tx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.xbee_rx = wt_rx14xx_xbee_rx_init(&instance->xbee_rx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.xbee_uart = wt_rx14xx_xbee_uart_init(
    &instance->xbee_uart, 
    9600, 
    false, 
    wt_rx1400_isr_priority_xbee_uart_tx, 
    wt_rx1400_isr_priority_xbee_uart_rx, 
    wt_rx1400_isr_priority_xbee_uart_err, 
    instance->hal.xbee_tx,
    instance->hal.xbee_rx,
    instance->hal.xbee_cts,
    instance->hal.xbee_rts,
    error
  )) != NULL;

  ok = !ok ? ok : (instance->hal.debug_tx = wt_rx14xx_debug_tx_init(&instance->debug_tx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.debug_rx = wt_rx14xx_debug_rx_init(&instance->debug_rx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.debug_uart = wt_rx14xx_debug_uart_init(
    &instance->debug_uart, 
    230400, 
    true, 
    wt_rx1400_isr_priority_debug_uart_tx, 
    wt_rx1400_isr_priority_debug_uart_rx, 
    wt_rx1400_isr_priority_debug_uart_err, 
    instance->hal.debug_tx,
    instance->hal.debug_rx,
    NULL,
    NULL,
    error
  )) != NULL;

  ok = !ok ? ok : (instance->hal.ethernet_reset = wt_rx1400_ethernet_reset_init(&instance->ethernet_reset, error)) != NULL;
  ok = !ok ? ok : (instance->hal.ethernet_primary_cts = wt_rx14xx_ethernet_primary_cts_init(&instance->ethernet_primary_cts, error)) != NULL;
  ok = !ok ? ok : (instance->hal.ethernet_primary_rts = wt_rx14xx_ethernet_primary_rts_init(&instance->ethernet_primary_rts, error)) != NULL;
  ok = !ok ? ok : (instance->hal.ethernet_primary_tx = wt_rx14xx_ethernet_primary_tx_init(&instance->ethernet_primary_tx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.ethernet_primary_rx = wt_rx14xx_ethernet_primary_rx_init(&instance->ethernet_primary_rx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.primary_ethernet_uart = wt_rx14xx_primary_ethernet_uart_init(
    &instance->primary_ethernet_uart, 
    115200, 
    true, 
    wt_rx1400_isr_priority_primary_ethernet_uart_tx, 
    wt_rx1400_isr_priority_primary_ethernet_uart_rx, 
    wt_rx1400_isr_priority_primary_ethernet_uart_err, 
    instance->hal.ethernet_primary_tx,
    instance->hal.ethernet_primary_rx,
    instance->hal.ethernet_primary_cts,
    instance->hal.ethernet_primary_rts,
    error
  )) != NULL;
  
  ok = !ok ? ok : (instance->hal.ethernet_secondary_tx = wt_rx14xx_ethernet_secondary_tx_init(&instance->ethernet_secondary_tx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.ethernet_secondary_rx = wt_rx14xx_ethernet_secondary_rx_init(&instance->ethernet_secondary_rx, error)) != NULL;
  ok = !ok ? ok : (instance->hal.secondary_ethernet_uart = wt_rx14xx_secondary_ethernet_uart_init(
    &instance->secondary_ethernet_uart, 
    115200, 
    false, 
    wt_rx1400_isr_priority_secondary_ethernet_uart_tx, 
    wt_rx1400_isr_priority_secondary_ethernet_uart_rx, 
    wt_rx1400_isr_priority_secondary_ethernet_uart_err, 
    instance->hal.ethernet_secondary_tx,
    instance->hal.ethernet_secondary_rx,
    NULL,
    NULL,
    error
  )) != NULL;

  ok = !ok ? ok : wthal_gpio_weak_pull_up(instance->hal.xbee_reset, true, error);
  ok = !ok ? ok : wthal_gpio_set(instance->hal.xbee_reset, true, error);

  ok = !ok ? ok : wthal_gpio_weak_pull_up(instance->hal.ethernet_reset, true, error);
  ok = !ok ? ok : wthal_gpio_set(instance->hal.ethernet_reset, true, error);

  ok = !ok ? ok : wthal_uart_open(instance->hal.debug_uart, error);

  ok = !ok ? ok : (instance->hal.i2c = wt_rx14xx_i2c_init(&instance->i2c, &instance->i2c_master_task.task, wt_rx1400_isr_priority_i2c, error)) != NULL;
  ok = !ok ? ok : wthal_i2c_queue_init(&instance->i2c_request_queue, instance->i2c_request_storage, sizeof(instance->i2c_request_storage) / sizeof(instance->i2c_request_storage[0]), error);
  ok = !ok ? ok : (wthal_i2c_master_task_init(&instance->i2c_master_task, instance->hal.i2c, &instance->i2c_request_queue, instance->hal.clock, error) != NULL);
  ok = !ok ? ok : (instance->hal.nvm = wthal_nvm_pic24_init(&instance->nvm, instance->hal.i2c, &instance->i2c_request_queue, error)) != NULL;
  
  return ok ? &instance->hal : NULL;

}

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

  // DEBUG
  wt_rx14xx_debug_t rx14xx_debug;
  wt_debug_t * debug;  
  ok = !ok ? ok : ((debug = wt_rx14xx_debug_init(&rx14xx_debug, hal->debug_uart, hal->clock, &error)) != NULL);

  if (ok) {
    wt_debug_print(debug, "============================= STARTUP ===========================");
  }
  
  // XBEE API
  wtio_uart_t xbee_uart;
  wtapi_xbee_t xbee_proto;
  wtapi_t xbee_api;

  ok = !ok ? ok : (wtio_uart_init(&xbee_uart, hal->xbee_uart, debug, 1, "XB", &error) != NULL);
  ok = !ok ? ok : wtapi_xbee_init(&xbee_proto, NULL, NULL, &xbee_uart.impl, 0, 0, hal->clock, &error);
  ok = !ok ? ok : wtapi_init(&xbee_api, &xbee_proto, &xbee_proto.impl, hal->clock, &error);

  // HOST PRIMARY API
  wtio_uart_t host_primary_uart;
  wtapi_host_t host_primary_proto;
  wtapi_t host_primary_api;

  ok = !ok ? ok : (wtio_uart_init(&host_primary_uart, hal->primary_ethernet_uart, debug, 1, "E1", &error) != NULL);
  ok = !ok ? ok : wtapi_host_init(&host_primary_proto, NULL, NULL, &host_primary_uart.impl, &error);
  ok = !ok ? ok : wtapi_init(&host_primary_api, &host_primary_proto, &host_primary_proto.impl, hal->clock, &error);

  // HOST SECONDARY API
  wtio_uart_t host_secondary_uart;
  wtapi_host_t host_secondary_proto;
  wtapi_t host_secondary_api;

  ok = !ok ? ok : (wtio_uart_init(&host_secondary_uart, hal->secondary_ethernet_uart, debug, 1, "E2", &error) != NULL);
  ok = !ok ? ok : wtapi_host_init(&host_secondary_proto, NULL, NULL, &host_secondary_uart.impl, &error);
  ok = !ok ? ok : wtapi_init(&host_secondary_api, &host_secondary_proto, &host_secondary_proto.impl, hal->clock, &error);

  wt_rx1400_app_task_t app;
  wt_rx1400_params_t gateway_params;
  
  ok = !ok ? ok : wt_rx1400_params_init(&gateway_params, &error);
  
  // TODO: Load params from NVM (ISSUE #5)
  
  ok = !ok ? ok : (wt_rx1400_app_task_init(&app, hal->clock, hal->activity_led, hal->ethernet_reset, hal->xbee_reset, &xbee_api, &host_primary_api, &host_secondary_api, hal->system, hal->nvm, &gateway_params, debug, &error) != NULL);

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

