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

#include "wthal_system_pic24.h"
#include "wthal_timer_pic24.h"
#include "wthal_counter.h"
#include "wthal_observers.h"
#include "wthal_isr_pic24.h"
#include "wthal_gpio_pic24.h"
#include "wthal_uart_pic24.h"
#include "wt_rx1400_clock.h"

/*
                         Main application
 */

//////////////////////////////// STDOUT ///////////////////////////////////////

static wthal_uart_t * wthal_stdout_uart = NULL;

int __attribute__ ( ( __section__(".libc.write" ) ) ) write(int handle, void * buffer, unsigned int len) {
    if (wthal_stdout_uart != NULL) {
        return (int) wthal_uart_write(wthal_stdout_uart, buffer, len, NULL);
    }
    return 0;
}

bool wthal_set_stdout(wthal_uart_t * const uart, wt_error_t * const error) {
    wthal_stdout_uart = uart;
    return true;
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
WTHAL_SYSTEM_PIC24_DEFINE(wt_rx14xx_system, RCON, ClrWdt(), Nop());

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
    _U1RXR = 8;                 // UART1:U1RX->RP8/RB8
    _RP9R = _RPOUT_U1TX;        // RP9/RB9->UART1:U1TX
    _U1CTSR = 1;                // UART1:U1CTS->RP1/RB1
    _RP0R = _RPOUT_U1RTS;       // RP0/RB0->UART1:U1RTS

    // UART 2 - DEBUG
    _U2RXR = 23;                // UART2:U2RX->RP23/RD2
    _RP22R = _RPOUT_U2TX;       // RP22/RD3->UART2:U2TX
    
    // UART 3 - Primary Ethernet
    _U3RXR = 4;                 // UART3:U3RX->RP4/RD9
    _RP11R = _RPOUT_U3TX;       // RP11/RD0->UART3:U3TX
    _U3CTSR = 12;               // UART3:U3CTS->RP12/RD11
    _RP3R = _RPOUT_U3RTS;       // RP3/RD10->UART3:U3RTS

    // UART 4 - Secondary Ethernet
    _U4RXR = 25;                // UART4:U4RX->RP25/RD4
    _RP20R = _RPOUT_U4TX;       // RP20/RD5->UART4:U4TX

    PPS_LOCK();
    
    ok = !ok ? ok : (instance->hal.system = wt_rx14xx_system_init(&instance->system, error)) != NULL;

    ok = !ok ? ok : (instance->hal.t5_isr = wt_rx14xx_tmr5_init(&instance->t5_isr, wt_rx1400_isr_priority_timer, instance->t5_observers, sizeof(instance->t5_observers) / sizeof(instance->t5_observers[0]), error)) != NULL;
    ok = !ok ? ok : (instance->hal.timer5 = wt_rx14xx_timer5_init(&instance->timer5, error)) != NULL;
    ok = !ok ? ok : (instance->hal.counter = wthal_counter_init(&instance->counter, error)) != NULL;
    ok = !ok ? ok : (instance->hal.clock = wt_rx1400_clock_init(&instance->clock, instance->hal.timer5, instance->hal.t5_isr, instance->hal.counter, 10, error)) != NULL;

    ok = !ok ? ok : (instance->hal.startup_led = wt_rx14xx_led1_init(&instance->startup_led, error)) != NULL;
    ok = !ok ? ok : (instance->hal.activity_led = wt_rx14xx_led2_init(&instance->activity_led, error)) != NULL;

    ok = !ok ? ok : (instance->hal.xbee_uart = wt_rx14xx_xbee_uart_init(&instance->xbee_uart, 9600, true, wt_rx1400_isr_priority_xbee_uart_tx, wt_rx1400_isr_priority_xbee_uart_rx, wt_rx1400_isr_priority_xbee_uart_err, error)) != NULL;

    ok = !ok ? ok : (instance->hal.debug_uart = wt_rx14xx_debug_uart_init(&instance->debug_uart, 230400, true, wt_rx1400_isr_priority_debug_uart_tx, wt_rx1400_isr_priority_debug_uart_rx, wt_rx1400_isr_priority_debug_uart_err, error)) != NULL;

    ok = !ok ? ok : (instance->hal.ethernet_reset = wt_rx1400_ethernet_reset_init(&instance->ethernet_reset, error)) != NULL;
    ok = !ok ? ok : (instance->hal.primary_ethernet_uart = wt_rx14xx_primary_ethernet_uart_init(&instance->primary_ethernet_uart, 115200, true, wt_rx1400_isr_priority_primary_ethernet_uart_tx, wt_rx1400_isr_priority_primary_ethernet_uart_rx, wt_rx1400_isr_priority_primary_ethernet_uart_err, error)) != NULL;
    ok = !ok ? ok : (instance->hal.secondary_ethernet_uart = wt_rx14xx_secondary_ethernet_uart_init(&instance->secondary_ethernet_uart, 115200, true, wt_rx1400_isr_priority_secondary_ethernet_uart_tx, wt_rx1400_isr_priority_secondary_ethernet_uart_rx, wt_rx1400_isr_priority_secondary_ethernet_uart_err, error)) != NULL;
    
    ok = !ok ? ok : wthal_gpio_weak_pull_up(instance->hal.ethernet_reset, true, error);
    
    ok = !ok ? ok : wthal_uart_open(instance->hal.debug_uart, error);
    ok = !ok ? ok : wthal_uart_open(instance->hal.xbee_uart, error);
    ok = !ok ? ok : wthal_uart_open(instance->hal.primary_ethernet_uart, error);
    ok = !ok ? ok : wthal_uart_open(instance->hal.secondary_ethernet_uart, error);
    
    ok = !ok ? ok : wthal_set_stdout(instance->hal.debug_uart, error);
    
    return ok ? &instance->hal : NULL;
    
}

typedef struct {
    
    wt_task_t task;
    wt_hal_t * hal;
//    wt_rx1400_mesh_init_task_t mesh_task;
//    wt_rx1400_host_init_task_t host_task;
    
} wt_rx1400_app_task_t;

bool wt_rx1400_app_task_main(
    wt_task_t * const task,
    void * const context, 
    wt_error_t * const error
) {
    wt_rx1400_app_task_t * const instance = context;
    bool ok = true;
    
    static uint32_t count = 0;
    printf("\n App Task %lu, counter=%lu", count++, (uint32_t)wthal_counter_get(instance->hal->counter, error));
    
    ok = !ok ? ok : wthal_gpio_toggle(instance->hal->activity_led, error);
    ok = !ok ? ok : wt_task_sleep(task, 1000, error);
    return true;
}

wt_task_t * wt_rx1400_app_task_init(
    wt_rx1400_app_task_t * const instance,
    wt_hal_t * const hal,
    wt_error_t * const error
) {
    bool ok = true;

    instance->hal = hal;
    
    ok = !ok ? ok : wt_task_init(&instance->task, wt_rx1400_app_task_main, instance, hal->clock, error);
    
    return ok ? &instance->task : NULL;
}

int main(void)
{
    wt_error_t error;
    
    // initialize the device - MUST COME FIRST OR WILL OVERRIDE HAL STYLE CONFIG
    SYSTEM_Initialize();

    wt_rx1400_hal_t rx1400_hal;
    wt_rx1400_app_task_t app;

    bool ok = true;
    
    ok = !ok ? ok : wt_error_init(&error);
    
    wt_hal_t * hal = ok ? wt_rx1400_hal_init(&rx1400_hal, &error) : NULL;
 
    ok = (hal != NULL);
    
    ok = (wt_rx1400_app_task_init(&app, hal, &error) != NULL);
    
    while (ok && wt_task_incomplete(&app.task)) {
        ok = !ok ? ok : wthal_system_clear_watchdog_timer(hal->system, &error);
        ok = !ok ? ok : wt_task_spin(&app.task, &error);
//        ok = !ok ? wthal_system_reset(hal->system, 3000, &error) : wthal_system_nop(hal);
        wthal_system_nop(hal->system, &error);
    }
    
    return error.error_code;

}
/**
 End of File
*/

