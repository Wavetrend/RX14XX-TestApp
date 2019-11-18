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
#include <p24FJ256GB206.h>

#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr5.h"

#include "utlist.h"

/*
                         Main application
 */

////////////////////////////////// TMR ///////////////////////////////////////

typedef struct {
    
    bool (*start)(void * const context, uint32_t const msecs);
    bool (*stop)(void * const context);
    
} wthal_timer_impl_t;

typedef struct {
    
    wthal_timer_impl_t * impl;
    void * context;
    
} wthal_timer_t;

wthal_timer_t * const wthal_timer_init(wthal_timer_t * const self, wthal_timer_impl_t * const impl, void * const context) {
    self->impl = impl;
    self->context = context;
    return self;
}

bool wthal_timer_start(wthal_timer_t * const self, uint32_t const msecs) {
    return self->impl->start(self->context, msecs);
}

bool wthal_timer_stop(wthal_timer_t * const self) {
    return self->impl->stop(self->context);
}

//---------------------------------
// XTAL 14745600

#define DEFINE_TIMER(NAME, XTAL, TIMER, PERIOD, TON, TCKPS, IF) \
    typedef struct { \
        wthal_timer_t timer; \
    } NAME ## _t; \
    static bool start_impl(void * const context, uint32_t const msecs) { \
        float prescale[4] = { 1, 8, 64, 256 }; \
        TON = 0; \
        TIMER = 0; \
        PERIOD = ((XTAL / prescale[TCKPS]) / 1000) * msecs; \
        IF = 0; \
        TON = 1; \
        return true; \
    } \
    static bool stop_impl(void * const context) { \
        TON = 0; \
        return true; \
    } \
    static wthal_timer_impl_t NAME ## _impl = { \
        .start = start_impl, \
        .stop = stop_impl, \
    }; \
    wthal_timer_t * const NAME ## _init(NAME ## _t * const self, uint8_t const prescale) { \
        TON = 0; \
        TIMER = 0; \
        TCKPS = prescale; \
        return wthal_timer_init(&self->timer, &NAME ## _impl, self); \
    }

DEFINE_TIMER(wt_rx14xx_timer5, 14745600, TMR5, PR5, T5CONbits.TON, T5CONbits.TCKPS, _T5IF)

//-------------------------

typedef struct {
    uint32_t count;
} wthal_counter_t;

wthal_counter_t * const wthal_counter_init(wthal_counter_t * const self) {
    self->count = 0;
    return self;
}

bool wthal_counter_reset(wthal_counter_t * const self) {
    self->count = 0;
    return true;
}

bool wthal_counter_increment(wthal_counter_t * const self) {
    self->count++;
    return true;
}

uint32_t wthal_counter_get(wthal_counter_t * const self) {
    return self->count;
}

void wthal_counter_isr(void * const context) {
    wthal_counter_t * const counter = context;
    wthal_counter_increment(counter);
}

//-----------------------------------

typedef struct wthal_observer_tag wthal_observer_t;

struct wthal_observer_tag {
    void (*callback)(void * const context);
    void * context;
    wthal_observer_t * next;
    wthal_observer_t * prev;
};

typedef struct {
    wthal_observer_t * head;
} wthal_observers_t;

void wthal_observers_dispatch(wthal_observers_t * const self) {
    wthal_observer_t * el;
    DL_FOREACH(self->head, el) {
        el->callback(el->context);
    }
}

wthal_observer_t * wthal_observers_head(wthal_observers_t * const self) {
    return self->head;
}

bool wthal_observers_add(wthal_observers_t * const self, wthal_observer_t * const elem) {
    DL_APPEND(self->head, elem);
    return true;
}

bool wthal_observers_delete(wthal_observers_t * const self, wthal_observer_t * const elem) {
    DL_DELETE(self->head, elem);
    return true;
}

typedef struct {
    
    bool (*enable)(void * const context, bool const enable);
    bool (*add_observer)(void * const context, void (*callback)(void * const context), void * const callback_context);
    
} wthal_isr_impl_t;

typedef struct {
    
    wthal_isr_impl_t * impl;
    void * context;
    
} wthal_isr_t;

wthal_isr_t * const wthal_isr_init(wthal_isr_t * const self, wthal_isr_impl_t * const impl, void * const context) {
    self->impl = impl;
    self->context = context;
    return self;
}

bool wthal_isr_enable(wthal_isr_t * const self, bool const enable) {
    return self->impl->enable(self->context, enable);
}

bool wthal_isr_add_observer(wthal_isr_t * const self, void (*callback)(void * const context), void * const context) {
    return self->impl->add_observer(self->context, callback, context);
}

#define DEFINE_ISR(NAME, ISR, IF, IE) \
    typedef struct { \
        wthal_isr_t isr; \
    } NAME ## _t; \
    static wthal_observers_t NAME ## _active = {}; \
    static wthal_observers_t NAME ## _inactive = {}; \
    void __attribute__ (( interrupt, no_auto_psv )) ISR ( void ) { \
        wthal_observers_dispatch(&NAME ## _active); \
        _T5IF = 0; \
    } \
    static bool NAME ## _enable_impl(void * const context, bool const enable) { \
        IE = enable; \
        return true; \
    } \
    static bool NAME ## _add_observer_impl(void * const context, void (*callback)(void * const context), void * const callback_context) { \
        wthal_observer_t * elem = wthal_observers_head(&NAME ## _inactive); \
        bool ok = true; \
        if (elem != NULL) { \
            wthal_observers_delete(&NAME ## _inactive, elem); \
            elem->callback = callback; \
            elem->context = callback_context; \
            wthal_observers_add(&NAME ## _active, elem); \
        } else { \
            ok = false; \
        } \
        return ok; \
    } \
    static wthal_isr_impl_t NAME ## _impl = { \
        .enable = NAME ## _enable_impl, \
        .add_observer = NAME ## _add_observer_impl, \
    }; \
    wthal_isr_t * const NAME ## _init(NAME ## _t * const self, wthal_observer_t * const observers, size_t const size) { \
        for (size_t i=0 ; i < size ; i++) { \
            wthal_observers_add(&NAME ## _inactive, &observers[i]); \
        } \
        return wthal_isr_init(&self->isr, &NAME ## _impl, self); \
    }

DEFINE_ISR(wt_rx14xx_tmr5, _T5Interrupt, _T5IF, _T5IE)

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
    wthal_gpio_t * const xpc_reset,
    wthal_counter_t * const counter
) {
        // initialize the device
    SYSTEM_Initialize();
    uint32_t count = 0;

    wthal_counter_reset(counter);
    wthal_gpio_set(startup_led, true);
    while (wthal_counter_get(counter) < 5) {
        ClrWdt();
        Nop();
    }
    wthal_gpio_set(startup_led, false);
    
    while (1)
    {
        // Add your application code
        wthal_gpio_set(activity_led, true);
        wthal_counter_reset(counter);
        while(wthal_counter_get(counter) < 1) {
            ClrWdt();
            Nop();
        }
        wthal_gpio_set(activity_led, false);
        wthal_counter_reset(counter);
        while(wthal_counter_get(counter) < 1) {
            ClrWdt();
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
    wthal_observer_t t5_observers[5];
    wt_rx14xx_tmr5_t t5isr;
    wthal_isr_t * p_t5isr = wt_rx14xx_tmr5_init(&t5isr, t5_observers, sizeof(t5_observers) / sizeof(t5_observers[0]));
    
    wt_rx14xx_led1_t startup_led;
    wt_rx14xx_led2_t activity_led;
    wt_rx14xx_xpc_reset_t xpc_reset;
    wthal_gpio_t * const p_startup_led = wt_rx14xx_led1_init(&startup_led);
    wthal_gpio_t * const p_activity_led = wt_rx14xx_led2_init(&activity_led);
    wthal_gpio_t * const p_xpc_reset = wt_rx14xx_xpc_reset_init(&xpc_reset);
    wthal_gpio_weak_pull_up(p_xpc_reset, true);
    
    wt_rx14xx_timer5_t timer5;
    wthal_timer_t * const p_timer = wt_rx14xx_timer5_init(&timer5, 3);

    wthal_counter_t t5counter;
    wthal_counter_init(&t5counter);
    wthal_isr_add_observer(p_t5isr, wthal_counter_isr, &t5counter);
    
    wthal_isr_enable(p_t5isr, true);
    wthal_timer_start(p_timer, 1000);
    
    return app_main(
        p_startup_led,
        p_activity_led,
        p_xpc_reset,
        &t5counter
    );

}
/**
 End of File
*/

