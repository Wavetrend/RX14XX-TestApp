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

#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr5.h"

/*
                         Main application
 */

typedef void (*isr_callback_t)(void * const context);
typedef bool (*isr_add_observer_t)(isr_callback_t const callback, void * const context);

typedef struct {
    isr_callback_t callback;
    void * context;
} isr_observer_t;

#define DEFINE_ISR(NAME, FLAG, MAX_OBS) \
    static isr_observer_t NAME ## _observers[MAX_OBS] = {}; \
    void __attribute__ ( ( interrupt, no_auto_psv ) ) NAME (void) { \
        for(size_t i=0 ; i < MAX_OBS ; i++) { \
            if (NAME ## _observers[i].callback != NULL) { \
                NAME ## _observers[i].callback(NAME ## _observers[i].context); \
            } \
        } \
        FLAG=0; \
    } \
    bool NAME ## _add_observer(isr_callback_t const callback, void * const context) { \
        bool ok = false; \
        for(size_t i=0 ; i < MAX_OBS ; i++) { \
            if (NAME ## _observers[i].callback == NULL) { \
                NAME ## _observers[i].callback = callback; \
                NAME ## _observers[i].context = context; \
                ok = true; \
                break; \
            } \
        } \
        return ok; \
    }

DEFINE_ISR(_T5Interrupt, _T5IF, 5);

#define DEFINE_TIMER(NAME, ISR, PERIOD_REG, PERIOD_VAL, TCON, TCKPS, TCKPS_VAL, TON, TMIE, TMIF) \
    typedef struct { \
        uint32_t volatile tick_count; \
        uint32_t volatile tick_limit; \
    } NAME ## _t; \
    static void NAME ## _handler(void * const context) { \
        NAME ## _t * const self = context; \
        self->tick_count++; \
    } \
    bool NAME ## _completed(NAME ## _t * const self) { \
        return self->tick_count >= self->tick_limit; \
    } \
    bool NAME ## _init(NAME ## _t * const self, uint32_t const ticks) { \
        self->tick_count = 0; \
        self->tick_limit = ticks; \
        PERIOD_REG = PERIOD_VAL; \
        TCON = 0; \
        TCKPS = TCKPS_VAL; \
        TMIF = 0; \
        TMIE = 1; \
        TON = 1; \
        return ISR ## _add_observer(NAME ## _handler, self); \
    }

DEFINE_TIMER(millisecond_timer, _T5Interrupt, PR5, 0x399, T5CON, T5CONbits.TCKPS, 3, T5CONbits.TON, _T5IE, _T5IF);

///////////////////////////////// LED ///////////////////////////////////////

typedef struct {
    bool (*on)(void * const context);
    bool (*off)(void * const context);
} wthal_led_impl_t;

typedef struct {
    
    wthal_led_impl_t * impl;
    void * const context;
    
} wthal_led_t;

wthal_led_t * const wthal_led_init(wthal_led_t * const self, wthal_led_impl_t * const impl, void * const context) {
    self->impl = impl;
    return self;
}

bool wthal_led_on(wthal_led_t * const self) {
    return self->impl->on(self->context);
}

bool wthal_led_off(wthal_led_t * const self) {
    return self->impl->off(self->context);
}

#define DEFINE_LED(NAME, TRIS, LAT, WPU, WPD, ODD) \
    typedef struct { \
        wthal_led_t led; \
    } NAME ## _t; \
    static bool NAME ## _on_impl(void * const context) { \
        LAT = 1; \
        return true; \
    } \
    static bool NAME ## _off_impl(void * const context) { \
        LAT = 0; \
        return true; \
    } \
    static wthal_led_impl_t NAME ## _impl = { \
        .on = NAME ## _on_impl, \
        .off = NAME ## _off_impl, \
    }; \
    wthal_led_t * const NAME ## _init(NAME ## _t * const self) { \
        TRIS = 0; \
        LAT = 0; \
        WPU = 0; \
        WPD = 0; \
        ODD = 0; \
        return wthal_led_init(&self->led, &NAME ## _impl, self); \
    }

DEFINE_LED(wt_rx14xx_led1, _TRISD7, _LATD7, _CN16PDE, _CN16PUE, _ODD7);
DEFINE_LED(wt_rx14xx_led2, _TRISD6, _LATD6, _CN15PDE, _CN15PUE, _ODD6);

int app_main(
    wthal_led_t * const startup_led,
    wthal_led_t * const activity_led
) {
        // initialize the device
    SYSTEM_Initialize();
    uint32_t count = 0;
        
    TMR5_SoftwareCounterClear();
    TMR5_Start();
    wthal_led_on(startup_led);
    while (TMR5_SoftwareCounterGet() < 5) {
        ClrWdt();
    }
    wthal_led_off(startup_led);
    TMR5_Stop();
    
    while (1)
    {
        // Add your application code
        wthal_led_on(activity_led);
        for (uint32_t i=0 ; i < 100000 ; i++) {
            Nop();
        }
        ClrWdt();
        wthal_led_off(activity_led);
        for (uint32_t i=0 ; i < 100000 ; i++) {
            Nop();
        }
        for (size_t i=0 ; i < 100 ; i++) {
            printf("\nHello World %ld", (uint32_t)count++);        
        }
    }

    return 1;

}

int main(void)
{
    wt_rx14xx_led1_t startup_led;
    wt_rx14xx_led2_t activity_led;
    
    return app_main(
        wt_rx14xx_led1_init(&startup_led),
        wt_rx14xx_led2_init(&activity_led)
    );

}
/**
 End of File
*/

