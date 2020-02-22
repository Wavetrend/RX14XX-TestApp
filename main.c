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
#include "ezbl_integration/ezbl.h"

#include "wt_assert.h"
#include "wt_error.h"
#include "wt_task.h"

#include "wt_module_debug.h"

#include "wthal_system_pic24.h"
#include "wthal_timer_pic24.h"
#include "wthal_counter.h"
#include "wthal_observers.h"
#include "wthal_gpio_pic24.h"
#include "wthal_uart_pic24.h"
#include "wt_rx1400_clock.h"
#include "wt_rx14xx_debug.h"
#include "wt_rx14xx_bl_debug.h"
#include "wt_rx14xx_bootloader_task.h"

#ifdef DEBUG_STACK
// https://www.microchip.com/forums/m966141.aspx
extern void fill_stack(void); // Call as early as possible. Does not write over already used stack space.
extern void check_stack(void); // Returns the number of unused stack words, and a pointer to the last used word.
uint16_t volatile MaxStackPointer; //The highest point the stack pointer ever reached
uint16_t volatile UnusedStackBytes; //Number of stack bytes remaining
uint16_t volatile StackLimit; //The stack limit pointer
#else
#define fill_stack()
#define check_stack();
#endif

/*
                         Main application
 */

///////////////////////////////// MAIN ///////////////////////////////////////

#define PPS_UNLOCK()    __builtin_write_OSCCONL(OSCCON & 0xbf)
#define PPS_LOCK()      __builtin_write_OSCCONL(OSCCON | 0x40)

#define XTAL (14745600)

static bool no_analogue;

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_tp1);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_tp1, _RB12, _TRISB12, _LATB12, _ANSB12, _CN30PUE, _CN30PDE, _ODB12);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_tp2);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_tp2, _RF4, _TRISF4, _LATF4, no_analogue, _CN17PUE, _CN17PDE, _ODF4);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_tp3);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_tp3, _RF5, _TRISF5, _LATF5, no_analogue, _CN18PUE, _CN18PDE, _ODF5);

wt_rx14xx_tp1_t tp1;
wt_rx14xx_tp2_t tp2;
wt_rx14xx_tp3_t tp3;

void tp1_set(bool high) {
  wthal_gpio_set(&tp1.gpio, high, NULL);
}

void tp2_set(bool high) {
  wthal_gpio_set(&tp2.gpio, high, NULL);
}

void tp3_set(bool high) {
  wthal_gpio_set(&tp3.gpio, high, NULL);
}

void tp4_set(bool high) {
 
}

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_debug_tx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_debug_tx, _RD3, _TRISD3, _LATD3, no_analogue, _CN52PUE, _CN52PDE, _ODD3);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_debug_rx);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_debug_rx, _RD2, _TRISD2, _LATD2, no_analogue, _CN51PUE, _CN51PDE, _ODD2);

WTHAL_UART_PIC24_DECLARE(wt_rx14xx_debug_uart);
WTHAL_UART_PIC24_DEFINE(wt_rx14xx_debug_uart, 2, XTAL);

WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_led1);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_led1, _RD7, _TRISD7, _LATD7, _ANSD7, _CN16PUE, _CN16PDE, _ODD7);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_led2);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_led2, _RD6, _TRISD6, _LATD6, _ANSD6, _CN15PUE, _CN15PDE, _ODD6);
WTHAL_GPIO_PIC24_DECLARE(wt_rx14xx_ext_led);
WTHAL_GPIO_PIC24_DEFINE(wt_rx14xx_ext_led, _RC14, _TRISC14, _LATC14, _ANSC14, _CN0PUE, _CN0PDE, _ODC14);

WTHAL_ISR_PIC24_DECLARE(wt_rx14xx_tmr5);
WTHAL_ISR_PIC24_DEFINE(wt_rx14xx_tmr5, _T5Interrupt, _T5IF, _T5IE, _T5IP);
WTHAL_TIMER_PIC24_DECLARE(wt_rx14xx_timer5);
WTHAL_TIMER_PIC24_DEFINE(wt_rx14xx_timer5, XTAL, TMR5, PR5, T5CONbits.TON, T5CONbits.TCKPS, _T5IF);

WTHAL_SYSTEM_PIC24_DECLARE(wt_rx14xx_system);
WTHAL_SYSTEM_PIC24_DEFINE(wt_rx14xx_system, RCON, ClrWdt(), Nop(), asm("reset"));

typedef struct {

  wthal_system_t * system;
  wthal_isr_t * t5_isr;
  wthal_gpio_t * led1;
  wthal_gpio_t * led2;
  wthal_gpio_t * ext_led;
  wthal_gpio_t * debug_tx;
  wthal_gpio_t * debug_rx;
  wthal_timer_t * timer5;
  wthal_counter_t * counter;
  wthal_clock_t * clock;
  wthal_uart_t * debug_uart;

} wt_hal_t;

typedef struct {

  wt_hal_t hal;

  wthal_observer_t t5_observers[3];

  wt_rx14xx_system_t system;

  wt_rx14xx_tmr5_t t5_isr;
  wt_rx14xx_timer5_t timer5;
  wthal_counter_t counter;
  wt_rx1400_clock_t clock;

  wt_rx14xx_led1_t led1;
  wt_rx14xx_led2_t led2;
  wt_rx14xx_ext_led_t ext_led;

  wt_rx14xx_debug_tx_t debug_tx;
  wt_rx14xx_debug_rx_t debug_rx;
  wt_rx14xx_debug_uart_t debug_uart;
  
} wt_rx1400_hal_t;

// CONFIG3
#pragma config WPFP = WPFP255    //Write Protection Flash Page Segment Boundary->Highest Page (same as page 170)
#pragma config SOSCSEL = EC    //Secondary Oscillator Power Mode Select->External clock (SCLKI) or Digital I/O mode(
#pragma config WUTSEL = LEG    //Voltage Regulator Wake-up Time Select->Default regulator start-up time is used
#pragma config ALTPMP = ALPMPDIS    //Alternate PMP Pin Mapping->EPMP pins are in default location mode
#pragma config WPDIS = WPDIS    //Segment Write Protection Disable->Segmented code protection is disabled
#pragma config WPCFG = WPCFGDIS    //Write Protect Configuration Page Select->Last page (at the top of program memory) and Flash Configuration Words are not write-protected
#pragma config WPEND = WPENDMEM    //Segment Write Protection End Page Select->Protected code segment upper boundary is at the last page of program memory; the lower boundary is the code page specified by WPFP

// CONFIG2
#pragma config POSCMOD = XT    //Primary Oscillator Select->XT Oscillator mode is selected
#pragma config IOL1WAY = OFF    //IOLOCK One-Way Set Enable->The IOLOCK bit can be set and cleared as needed, provided the unlock sequence has been completed
#pragma config OSCIOFNC = ON    //OSCO Pin Configuration->OSCO/CLKO/RC15 functions as port I/O (RC15)
#pragma config FCKSM = CSECMD    //Clock Switching and Fail-Safe Clock Monitor->Clock switching is enabled, Fail-Safe Clock Monitor is disabled
#pragma config FNOSC = PRIPLL    //Initial Oscillator Select->Primary Oscillator with PLL module (XTPLL, HSPLL, ECPLL)
#pragma config PLL96MHZ = ON    //96MHz PLL Startup Select->96 MHz PLL is enabled automatically on start-up
#pragma config PLLDIV = DIV2    //96 MHz PLL Prescaler Select->Oscillator input is divided by 2 (8 MHz input)
#pragma config IESO = OFF    //Internal External Switchover->IESO mode (Two-Speed Start-up) is disabled

// CONFIG1
#pragma config WDTPS = PS4096    //Watchdog Timer Postscaler->1:4096
#pragma config FWPSA = PR32    //WDT Prescaler->Prescaler ratio of 1:32
#pragma config WINDIS = OFF    //Windowed WDT->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = ON    //Watchdog Timer->Watchdog Timer is enabled
#pragma config ICS = PGx2    //Emulator Pin Placement Select bits->Emulator functions are shared with PGEC2/PGED2
#pragma config GWRP = OFF    //General Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = OFF    //General Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF    //JTAG Port Enable->JTAG port is disabled

wt_hal_t * const wt_rx1400_hal_init(wt_rx1400_hal_t * const instance, wt_error_t * const error) {

  bool ok = true;

  enum {
    wt_rx1400_isr_priority_timer = 4,
    wt_rx1400_isr_priority_debug_uart_tx = 2,
    wt_rx1400_isr_priority_debug_uart_rx = 1,
    wt_rx1400_isr_priority_debug_uart_err = 1,
  };

  //======================================================================
  // PROCESSOR CLOCK
  //======================================================================

  // CPDIV 1:1; RCDIV FRC/1; DOZE 1:1; DOZEN disabled; ROI disabled; 
  CLKDIV = 0x00;
  // TUN Center frequency; 
  OSCTUN = 0x00;
  // ADC1MD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; SPI2MD enabled; SPI1MD enabled; T5MD enabled; I2C1MD enabled; 
  PMD1 = 0x00;
  // OC5MD enabled; OC6MD enabled; OC7MD enabled; OC8MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled; IC6MD enabled; IC7MD enabled; IC5MD enabled; IC8MD enabled; IC4MD enabled; IC3MD enabled; 
  PMD2 = 0x00;
  // I2C3MD enabled; PMPMD enabled; U3MD enabled; RTCCMD enabled; CMPMD enabled; CRCMD enabled; I2C2MD enabled; 
  PMD3 = 0x00;
  // U4MD enabled; UPWMMD enabled; USB1MD enabled; CTMUMD enabled; REFOMD enabled; LVDMD enabled; 
  PMD4 = 0x00;
  // IC9MD enabled; OC9MD enabled; 
  PMD5 = 0x00;
  // SPI3MD enabled; 
  PMD6 = 0x00;
  // CF no clock failure; NOSC PRIPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; 
  __builtin_write_OSCCONH((uint8_t) (0x03));
  __builtin_write_OSCCONL((uint8_t) (0x00));

  //======================================================================
  // PROGRAMMABLE PIN ASSIGNMENTS
  //======================================================================

  PPS_UNLOCK();
  
  // UART 2 - DEBUG
  _U2RXR = 23; // UART2:U2RX->RP23/RD2
  _RP22R = _RPOUT_U2TX; // RP22/RD3->UART2:U2TX

  PPS_LOCK();

  //======================================================================
  // HAL
  //======================================================================

  ok = !ok ? ok : (instance->hal.system = wt_rx14xx_system_init(&instance->system, error)) != NULL;

  ok = !ok ? ok : (instance->hal.t5_isr = wt_rx14xx_tmr5_init(&instance->t5_isr, wt_rx1400_isr_priority_timer, instance->t5_observers, sizeof (instance->t5_observers) / sizeof (instance->t5_observers[0]), error)) != NULL;
  ok = !ok ? ok : (instance->hal.timer5 = wt_rx14xx_timer5_init(&instance->timer5, error)) != NULL;
  ok = !ok ? ok : (instance->hal.counter = wthal_counter_init(&instance->counter, error)) != NULL;
  ok = !ok ? ok : (instance->hal.clock = wt_rx1400_clock_init(&instance->clock, instance->hal.timer5, instance->hal.t5_isr, instance->hal.counter, 10, error)) != NULL;

  ok = !ok ? ok : (instance->hal.led1 = wt_rx14xx_led1_init(&instance->led1, error)) != NULL;
  ok = !ok ? ok : (instance->hal.led2 = wt_rx14xx_led2_init(&instance->led2, error)) != NULL;
  ok = !ok ? ok : (instance->hal.ext_led = wt_rx14xx_ext_led_init(&instance->ext_led, error)) != NULL;

  ok = !ok ? ok : (wt_rx14xx_tp1_init(&tp1, error) != NULL);
  ok = !ok ? ok : (wt_rx14xx_tp2_init(&tp2, error) != NULL);
  ok = !ok ? ok : (wt_rx14xx_tp3_init(&tp3, error) != NULL);
  
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
    NULL,
    error
  )) != NULL;

  ok = !ok ? ok : wthal_gpio_weak_pull_up(&tp1.gpio, true, error);
  ok = !ok ? ok : wthal_gpio_set(&tp1.gpio, false, error);
  ok = !ok ? ok : wthal_gpio_weak_pull_up(&tp2.gpio, true, error);
  ok = !ok ? ok : wthal_gpio_set(&tp2.gpio, false, error);
  ok = !ok ? ok : wthal_gpio_weak_pull_up(&tp3.gpio, true, error);
  ok = !ok ? ok : wthal_gpio_set(&tp3.gpio, false, error);

  ok = !ok ? ok : wthal_uart_open(instance->hal.debug_uart, error);

  return ok ? &instance->hal : NULL;

}

static void device_reset(void * const context) {
  wthal_system_t * const system = context;
  (void) wthal_system_reset(system, NULL);
}

int main(void) {
  wt_error_t error;

#ifdef DEBUG_STACK
  fill_stack();
  check_stack();
  uint16_t LastUnusedStackBytes = UnusedStackBytes;
#endif

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
  wt_module_debug_set_modules(WT_RX14XX_BL_DEBUG_ALL);
//  wt_module_debug_set_modules(0);
  
  if (ok) {
    uint16_t status;
    ok = wthal_system_reset_status(hal->system, &status, false, &error);
    wt_debug_print(debug, "=========================== TEST APP ======================== (0x%04x)", status);
  }

#ifdef DEBUG_STACK
  wt_debug_print(debug, "Stack: %u at startup", UnusedStackBytes);
#endif

  wt_rx14xx_bootloader_task_t bootloader;
  ok = !ok ? ok : (wt_rx14xx_bootloader_task_init(&bootloader, hal->led1, 1000, hal->clock, debug, &error) != NULL);
  
  bool pending_reset = false;
  
  while (ok && wt_task_incomplete(&bootloader.task)) {
    ok = !ok ? ok : wthal_system_clear_watchdog_timer(hal->system, &error);

#ifdef DEBUG_STACK
    check_stack();
    if (UnusedStackBytes < LastUnusedStackBytes) {
      wt_debug_print(debug, "*** Stack Warning ***: %u bytes low-water", UnusedStackBytes);
      LastUnusedStackBytes = UnusedStackBytes;
    }
#endif
    
    ok = !ok ? ok : wt_task_spin(&bootloader.task, &error);
    
    if (ok) {
      if (!wt_task_incomplete(&bootloader.task)) {
        if (EZBL_IsAppPresent()) {
          wt_debug_print(debug, "Bootloader completed, handing over to app");
          ok = wthal_gpio_set(hal->led1, false, &error);
          
          EZBL_RAMSet((void*)&IEC0, 0x00, (unsigned int)&IPC0 - (unsigned int)&IEC0);   // Clear every bit in all IECx Interrupt Enable registers
          EZBL_ForwardAllIntToApp();                                                    // Forward all Interrupts to the Application
          EZBL_StartAppIfPresent();   // Sets EZBL_appIsRunning = 0xFFFF and temporarily disables IPL6 and lower interrupts before launching Application

        } else {
          wt_debug_print(debug, "**FATAL** Bootloader completed without app found");
        }
      }
    }
    if (!ok) {
      ok = true;
      if (!pending_reset) {
        wt_debug_print(debug, "**LAST ERROR**: %d file: %s, line: %d", error.error_code, error.file, error.line);
        ok = wthal_clock_set_alarm(hal->clock, device_reset, hal->system, 3000, &error);
        pending_reset = ok;
      }
    }
  }

  // allow for ISR's to complete
  for(uint32_t i=0 ; i < 100000 ; i++) {
    wthal_system_nop(hal->system, &error);
  }

  return error.error_code;

}
/**
 End of File
 */

