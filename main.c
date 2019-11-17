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

///////////////////////////////// GPIO ///////////////////////////////////////

typedef struct {

    bool (*set)(void * const, bool const high);
    bool (*toggle)(void * const);
    bool (*get)(void * const context);
    bool (*input)(void * const context, bool const input);
    bool (*analogue)(void * const context, bool const analogue);
    bool (*weak_pull_up)(void * const context, bool const enable);
    bool (*weak_pull_down)(void * const context, bool const enable);
    bool (*output_drain)(void * const context, bool const enable);
    
} wthal_gpio_impl_t;

typedef struct {
    
    wthal_gpio_impl_t * impl;
    void * context;
    
} wthal_gpio_t;

wthal_gpio_t * const wthal_gpio_init(wthal_gpio_t * const self, wthal_gpio_impl_t * const impl, void * const context) {
    self->impl = impl;
    self->context = context;
    return self;
}

bool wthal_gpio_set(wthal_gpio_t * const self, bool const high) {
    return self->impl->set(self->context, high);
}

bool wthal_gpio_toggle(wthal_gpio_t * const self) {
    return self->impl->toggle(self->context);
}

bool wthal_gpio_get(wthal_gpio_t * const self) {
    return self->impl->get(self->context);
}

bool wthal_gpio_input(wthal_gpio_t * const self, bool const input) {
    return self->impl->input(self->context, input);
}

bool wthal_gpio_analogue(wthal_gpio_t * const self, bool const analogue) {
    return self->impl->analogue(self->context, analogue);
}

bool wthal_gpio_weak_pull_up(wthal_gpio_t * const self, bool const enable) {
    return self->impl->weak_pull_up(self->context, enable);
}

bool wthal_gpio_weak_pull_down(wthal_gpio_t * const self, bool const enable) {
    return self->impl->weak_pull_down(self->context, enable);
}

bool wthal_gpio_output_drain(wthal_gpio_t * const self, bool const enable) {
    return self->impl->output_drain(self->context, enable);
}

#define DECLARE_GPIO(NAME) \
    typedef struct { \
        wthal_gpio_t gpio; \
    } NAME ## _t; \
    wthal_gpio_t * const NAME ## _init(NAME ## _t * const self);

#define DEFINE_GPIO(NAME, PORT, TRIS, LAT, ANS, WPU, WPD, ODRAIN) \
    DECLARE_GPIO(NAME) \
    static bool NAME ## _set_impl(void * const context, bool const high) { \
        LAT = high; \
        return true; \
    } \
    static bool NAME ## _toggle_impl(void * const context) { \
        return LAT ^= 1; \
    } \
    static bool NAME ## _get_impl(void * const context) { \
        return PORT; \
    } \
    static bool NAME ## _input_impl(void * const context, bool const input) { \
        TRIS = input; \
        return true; \
    } \
    static bool NAME ## _analogue_impl(void * const context, bool const analogue) { \
        ANS = analogue; \
        return true; \
    } \
    static bool NAME ## _weak_pull_up_impl(void * const context, bool const enabled) { \
        WPU = enabled; \
        return true; \
    } \
    static bool NAME ## _weak_pull_down_impl(void * const context, bool const enabled) { \
        WPD = enabled; \
        return true; \
    } \
    static bool NAME ## _output_drain_impl(void * const context, bool const enabled) { \
        ODRAIN = enabled; \
        return true; \
    } \
    static wthal_gpio_impl_t NAME ## _impl = { \
        .set = NAME ## _set_impl, \
        .toggle = NAME ## _toggle_impl, \
        .get = NAME ## _get_impl, \
        .input = NAME ## _input_impl, \
        .analogue = NAME ## _analogue_impl, \
        .weak_pull_up = NAME ## _weak_pull_up_impl, \
        .weak_pull_down = NAME ## _weak_pull_down_impl, \
        .output_drain = NAME ## _output_drain_impl, \
    }; \
    wthal_gpio_t * const NAME ## _init(NAME ## _t * const self) { \
        TRIS = 0; \
        LAT = 0; \
        ANS = 0; \
        WPU = 0; \
        WPD = 0; \
        ODRAIN = 0; \
        return wthal_gpio_init(&self->gpio, &NAME ## _impl, self); \
    }

DEFINE_GPIO(wt_rx14xx_led1, _RD7, _TRISD7, _LATD7, _ANSD7, _CN16PUE, _CN16PDE, _ODD7 )
DEFINE_GPIO(wt_rx14xx_led2, _RD6, _TRISD6, _LATD6, _ANSD6, _CN15PUE, _CN15PDE, _ODD6 )
DEFINE_GPIO(wt_rx14xx_xpc_reset, _RB2, _TRISB2, _LATB2, _ANSB2, _CN4PUE, _CN4PDE, _ODB2 )

int app_main(
    wthal_gpio_t * const startup_led,
    wthal_gpio_t * const activity_led,
    wthal_gpio_t * const xpc_reset
) {
        // initialize the device
    SYSTEM_Initialize();
    uint32_t count = 0;
    
    TMR5_SoftwareCounterClear();
    TMR5_Start();
    wthal_gpio_set(startup_led, true);
    while (TMR5_SoftwareCounterGet() < 5) {
        ClrWdt();
    }
    wthal_gpio_set(startup_led, false);
    TMR5_Stop();
    
    while (1)
    {
        // Add your application code
        wthal_gpio_set(activity_led, true);
        for (uint32_t i=0 ; i < 100000 ; i++) {
            Nop();
        }
        ClrWdt();
        wthal_gpio_set(activity_led, false);
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
    wt_rx14xx_xpc_reset_t xpc_reset;
    wthal_gpio_t * const p_startup_led = wt_rx14xx_led1_init(&startup_led);
    wthal_gpio_t * const p_activity_led = wt_rx14xx_led2_init(&activity_led);
    wthal_gpio_t * const p_xpc_reset = wt_rx14xx_xpc_reset_init(&xpc_reset);
    wthal_gpio_weak_pull_up(p_xpc_reset, true);
    
    return app_main(
        p_startup_led,
        p_activity_led,
        p_xpc_reset
    );

}
/**
 End of File
*/

