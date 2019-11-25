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

#include "utlist.h"

#include "wtapi_circular_buffer.h"
#include "wt_assert.h"
#include "wt_error.h"

/*
                         Main application
 */

WTAPI_DECLARE_CIRCULAR_BUFFER(uint8_t, wt_rx14xx_uint8_buffer);
WTAPI_DEFINE_CIRCULAR_BUFFER(uint8_t, wt_rx14xx_uint8_buffer);

////////////////////////////////// TMR ///////////////////////////////////////

typedef struct {
    
    bool (*start)(void * const context, uint32_t const msecs, wt_error_t * const error);
    bool (*stop)(void * const context, wt_error_t * const error);
    
} wthal_timer_impl_t;

typedef struct {
    
    wthal_timer_impl_t * impl;
    void * context;
    
} wthal_timer_t;

wthal_timer_t * const wthal_timer_init(wthal_timer_t * const instance, wthal_timer_impl_t * const impl, void * const context, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    ok = !ok ? ok : wt_assert_ptr(impl, error);
    
    if (ok) {
        instance->impl = impl;
        instance->context = context;
    }
    
    return ok ? instance : NULL;
}

bool wthal_timer_start(wthal_timer_t * const instance, uint32_t const msecs, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    ok = !ok ? ok : wt_assert_ptr(instance->impl->start, error);
    
    return !ok ? ok : instance->impl->start(instance->context, msecs, error);
}

bool wthal_timer_stop(wthal_timer_t * const instance, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    ok = !ok ? ok : wt_assert_ptr(instance->impl->start, error);

    return !ok ? ok : instance->impl->stop(instance->context, error);
}

//---------------------------------
// XTAL 14745600

typedef enum { 
    wt_rx14xx_timer_prescale_1 = 0,
    wt_rx14xx_timer_prescale_8 = 1,
    wt_rx14xx_timer_prescale_64 = 2,
    wt_rx14xx_timer_prescale_256 = 3,
} wt_rx14xx_timer_prescale_t;

#define WTHAL_TIMER_DECLARE(NAME) \
    typedef struct { \
        wthal_timer_t timer; \
    } NAME ## _t; \
    wthal_timer_t * const NAME ## _init(NAME ## _t * const instance, wt_rx14xx_timer_prescale_t const prescale, wt_error_t * const error);

#define WTHAL_TIMER_DEFINE(NAME, XTAL, TIMER, PERIOD, TON, TCKPS, IF) \
    static bool start_impl(void * const context, uint32_t const msecs, wt_error_t * const error) { \
        float prescale[4] = { 1, 8, 64, 256 }; \
        bool ok = false; \
        TON = 0; \
        for (size_t i=0 ; i < sizeof(prescale) / sizeof(float) ; i++) { \
            uint32_t period = (((XTAL / 2) / prescale[i]) / 1000) * msecs; \
            if (period <= UINT16_MAX) { \
                TCKPS = i; \
                PERIOD = (uint16_t)period; \
                TIMER = 0; \
                IF = 0; \
                TON = 1; \
                ok = true; \
                break; \
            } \
        } \
        ok = wt_assert(ok, WT_ERROR_EINVAL, error); \
        return ok; \
    } \
    static bool stop_impl(void * const context, wt_error_t * const error) { \
        TON = 0; \
        return true; \
    } \
    static wthal_timer_impl_t NAME ## _impl = { \
        .start = start_impl, \
        .stop = stop_impl, \
    }; \
    wthal_timer_t * const NAME ## _init(NAME ## _t * const instance, wt_rx14xx_timer_prescale_t const prescale, wt_error_t * const error) { \
        TON = 0; \
        TIMER = 0; \
        TCKPS = prescale; \
        return wthal_timer_init(&instance->timer, &NAME ## _impl, instance, error); \
    }

//-------------------------

typedef struct {
    uint32_t volatile count;
} wthal_counter_t;

wthal_counter_t * const wthal_counter_init(wthal_counter_t * const instance, wt_error_t * const error) {
    instance->count = 0;
    return instance;
}

bool wthal_counter_reset(wthal_counter_t * const instance, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    if (ok) {
        instance->count = 0;
    }
    return ok;
}

bool wthal_counter_increment(wthal_counter_t * const instance, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    if (ok) {
        instance->count++;
    }
    return ok;
}

uint32_t wthal_counter_get(wthal_counter_t * const instance, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    return ok ? instance->count : 0;
}

void wthal_counter_isr(void * const context, wt_error_t * const error) {
    wthal_counter_t * const counter = context;
    wthal_counter_increment(counter, error);
}

//-----------------------------------

typedef struct wthal_observer_tag wthal_observer_t;
typedef void (*wthal_observer_callback_t)(void * const context, wt_error_t * const error);

struct wthal_observer_tag {
    wthal_observer_callback_t callback;
    void * context;
    wt_error_t * error;
    wthal_observer_t * volatile next;
    wthal_observer_t * volatile prev;
};

typedef struct {
    wthal_observer_t * volatile head;
} wthal_observers_t;

void wthal_observers_dispatch(wthal_observers_t * const instance) {
    wthal_observer_t * el, * tmp;
    DL_FOREACH_SAFE(instance->head, el, tmp) {
        el->callback(el->context, el->error);
    }
}

wthal_observer_t * wthal_observers_head(wthal_observers_t * const instance) {
    return instance->head;
}

bool wthal_observers_add(wthal_observers_t * const instance, wthal_observer_t * const elem) {
    DL_APPEND(instance->head, elem);
    return true;
}

bool wthal_observers_delete(wthal_observers_t * const instance, wthal_observer_t * const elem) {
    DL_DELETE(instance->head, elem);
    return true;
}

typedef struct {
    
    bool (*enable)(void * const context, bool const enable, wt_error_t * const error);
    bool (*set_flag)(void * const context, bool const set, wt_error_t * const error);
    bool (*get_flag)(void * const context, wt_error_t * const error);
    bool (*set_priority)(void * const context, uint8_t const priority, wt_error_t * const error);
    uint8_t (*get_priority)(void * const context, wt_error_t * const error);
    bool (*add_observer)(void * const context, wthal_observer_callback_t callback, void * const callback_context, wt_error_t * const error);
    
} wthal_isr_impl_t;

typedef struct {
    
    wthal_isr_impl_t * impl;
    void * context;
    
} wthal_isr_t;

typedef enum {
    wthal_isr_priority_1 = 1,
    wthal_isr_priority_2 = 2,
    wthal_isr_priority_3 = 3,
    wthal_isr_priority_4 = 4,
    wthal_isr_priority_5 = 5,
    wthal_isr_priority_6 = 6,
    wthal_isr_priority_7 = 7,
} wthal_isr_priority_t;

wthal_isr_t * const wthal_isr_init(wthal_isr_t * const instance, wthal_isr_impl_t * const impl, void * const context, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    if (ok) {
        instance->impl = impl;
        instance->context = context;
    }
    return ok ? instance : NULL;
}

bool wthal_isr_enable(wthal_isr_t * const instance, bool const enable, wt_error_t * const error) {
    return instance->impl->enable(instance->context, enable, error);
}

bool wthal_isr_set_flag(wthal_isr_t * const instance, bool const set, wt_error_t * const error) {
    return instance->impl->set_flag(instance->context, set, error);
}

bool wthal_isr_get_flag(wthal_isr_t * const instance, wt_error_t * const error) {
    return instance->impl->get_flag(instance->context, error);
}

bool wthal_isr_set_priority(wthal_isr_t * const instance, uint8_t const priority, wt_error_t * const error) {
    return instance->impl->set_priority(instance->context, priority, error);
}

bool wthal_isr_get_priority(wthal_isr_t * const instance, wt_error_t * const error) {
    return instance->impl->get_priority(instance->context, error);
}

bool wthal_isr_add_observer(wthal_isr_t * const instance, wthal_observer_callback_t callback, void * const context, wt_error_t * const error) {
    return instance->impl->add_observer(instance->context, callback, context, error);
}

#define WTHAL_ISR_DECLARE(NAME) \
    typedef struct { \
        wthal_isr_t isr; \
    } NAME ## _t; \
    wthal_isr_t * const NAME ## _init(NAME ## _t * const instance, wthal_isr_priority_t const interrupt_priority, wthal_observer_t * const observers, size_t const size, wt_error_t * const error);

#define WTHAL_ISR_DEFINE(NAME, ISR, IF, IE, IP) \
    static wthal_observers_t NAME ## _active = {}; \
    static wthal_observers_t NAME ## _inactive = {}; \
    void __attribute__ (( interrupt, no_auto_psv )) ISR ( void ) { \
        wthal_observers_dispatch(&NAME ## _active); \
        IF = 0; \
    } \
    static bool NAME ## _enable_impl(void * const context, bool const enable, wt_error_t * const error) { \
        IE = enable; \
        return true; \
    } \
    static bool NAME ## _set_flag_impl(void * const context, bool const set, wt_error_t * const error) { \
        IF = set; \
        return true; \
    } \
    static bool NAME ## _get_flag_impl(void * const context, wt_error_t * const error) { \
        return IF; \
    } \
    static bool NAME ## _set_priority_impl(void * const context, uint8_t const priority, wt_error_t * const error) { \
        IP = priority; \
        return true; \
    } \
    static uint8_t NAME ## _get_priority_impl(void * const context, wt_error_t * const error) { \
        return IP; \
    } \
    static bool NAME ## _add_observer_impl(void * const context, wthal_observer_callback_t callback, void * const callback_context, wt_error_t * const error) { \
        wthal_observer_t * elem = wthal_observers_head(&NAME ## _inactive); \
        bool ok = true; \
        ok = !ok ? ok : wt_assert(elem != NULL, WT_ERROR_ENOSPC, error); \
        if (ok) { \
            wthal_observers_delete(&NAME ## _inactive, elem); \
            elem->callback = callback; \
            elem->context = callback_context; \
            elem->error = error; \
            wthal_observers_add(&NAME ## _active, elem); \
        } \
        return ok; \
    } \
    static wthal_isr_impl_t NAME ## _impl = { \
        .enable = NAME ## _enable_impl, \
        .add_observer = NAME ## _add_observer_impl, \
        .set_flag = NAME ## _set_flag_impl, \
        .get_flag = NAME ## _get_flag_impl, \
        .set_priority = NAME ## _set_priority_impl, \
        .get_priority = NAME ## _get_priority_impl, \
    }; \
    wthal_isr_t * const NAME ## _init(NAME ## _t * const instance, wthal_isr_priority_t const interrupt_priority, wthal_observer_t * const observers, size_t const size, wt_error_t * const error) { \
        IE = 0; \
        IF = 0; \
        IP = interrupt_priority; \
        for (size_t i=0 ; i < size ; i++) { \
            wthal_observers_add(&NAME ## _inactive, &observers[i]); \
        } \
        return wthal_isr_init(&instance->isr, &NAME ## _impl, instance, error); \
    }

///////////////////////////////// GPIO ///////////////////////////////////////

typedef struct {

    bool (*set)(void * const, bool const high, wt_error_t * const error);
    bool (*toggle)(void * const, wt_error_t * const error);
    bool (*get)(void * const context, wt_error_t * const error);
    bool (*input)(void * const context, bool const input, wt_error_t * const error);
    bool (*analogue)(void * const context, bool const analogue, wt_error_t * const error);
    bool (*weak_pull_up)(void * const context, bool const enable, wt_error_t * const error);
    bool (*weak_pull_down)(void * const context, bool const enable, wt_error_t * const error);
    bool (*output_drain)(void * const context, bool const enable, wt_error_t * const error);
    
} wthal_gpio_impl_t;

typedef struct {
    
    wthal_gpio_impl_t * impl;
    void * context;
    
} wthal_gpio_t;

wthal_gpio_t * const wthal_gpio_init(wthal_gpio_t * const instance, wthal_gpio_impl_t * const impl, void * const context, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    ok = !ok ? ok : wt_assert_ptr(impl, error);
    if (ok) {
        instance->impl = impl;
        instance->context = context;
    }
    return ok ? instance : NULL;
}

bool wthal_gpio_set(wthal_gpio_t * const instance, bool const high, wt_error_t * const error) {
    return instance->impl->set(instance->context, high, error);
}

bool wthal_gpio_toggle(wthal_gpio_t * const instance, wt_error_t * const error) {
    return instance->impl->toggle(instance->context, error);
}

bool wthal_gpio_get(wthal_gpio_t * const instance, wt_error_t * const error) {
    return instance->impl->get(instance->context, error);
}

bool wthal_gpio_input(wthal_gpio_t * const instance, bool const input, wt_error_t * const error) {
    return instance->impl->input(instance->context, input, error);
}

bool wthal_gpio_analogue(wthal_gpio_t * const instance, bool const analogue, wt_error_t * const error) {
    return instance->impl->analogue(instance->context, analogue, error);
}

bool wthal_gpio_weak_pull_up(wthal_gpio_t * const instance, bool const enable, wt_error_t * const error) {
    return instance->impl->weak_pull_up(instance->context, enable, error);
}

bool wthal_gpio_weak_pull_down(wthal_gpio_t * const instance, bool const enable, wt_error_t * const error) {
    return instance->impl->weak_pull_down(instance->context, enable, error);
}

bool wthal_gpio_output_drain(wthal_gpio_t * const instance, bool const enable, wt_error_t * const error) {
    return instance->impl->output_drain(instance->context, enable, error);
}

#define WTHAL_GPIO_DECLARE(NAME) \
    typedef struct { \
        wthal_gpio_t gpio; \
    } NAME ## _t; \
    wthal_gpio_t * const NAME ## _init(NAME ## _t * const instance, wt_error_t * const error);

#define WTHAL_GPIO_DEFINE(NAME, PORT, TRIS, LAT, ANS, WPU, WPD, ODRAIN) \
    static bool NAME ## _set_impl(void * const context, bool const high, wt_error_t * const error) { \
        LAT = high; \
        return true; \
    } \
    static bool NAME ## _toggle_impl(void * const context, wt_error_t * const error) { \
        return LAT ^= 1; \
    } \
    static bool NAME ## _get_impl(void * const context, wt_error_t * const error) { \
        return PORT; \
    } \
    static bool NAME ## _input_impl(void * const context, bool const input, wt_error_t * const error) { \
        TRIS = input; \
        return true; \
    } \
    static bool NAME ## _analogue_impl(void * const context, bool const analogue, wt_error_t * const error) { \
        ANS = analogue; \
        return true; \
    } \
    static bool NAME ## _weak_pull_up_impl(void * const context, bool const enabled, wt_error_t * const error) { \
        WPU = enabled; \
        return true; \
    } \
    static bool NAME ## _weak_pull_down_impl(void * const context, bool const enabled, wt_error_t * const error) { \
        WPD = enabled; \
        return true; \
    } \
    static bool NAME ## _output_drain_impl(void * const context, bool const enabled, wt_error_t * const error) { \
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
    wthal_gpio_t * const NAME ## _init(NAME ## _t * const instance, wt_error_t * const error) { \
        TRIS = 0; \
        LAT = 0; \
        ANS = 0; \
        WPU = 0; \
        WPD = 0; \
        ODRAIN = 0; \
        return wthal_gpio_init(&instance->gpio, &NAME ## _impl, instance, error); \
    }

///////////////////////////////// UART ///////////////////////////////////////

typedef struct {
    
    bool (*open)(void * const context, wt_error_t * const error);
    bool (*close)(void * const context, wt_error_t * const error);
    bool (*baudrate)(void * const context, uint32_t const baudrate, wt_error_t * const error);
    bool (*flowcontrol)(void * const context, bool const flowcontrol, wt_error_t * const error);
    size_t (*read)(void * const context, void * const data, size_t const size, wt_error_t * const error);
    size_t (*write)(void * const context, void const * const data, size_t const size, wt_error_t * const error);
    
} wthal_uart_impl_t;

typedef struct {
    
    wthal_uart_impl_t const * impl;
    void * context;
    
} wthal_uart_t;

wthal_uart_t * const wthal_uart_init(wthal_uart_t * const instance, wthal_uart_impl_t const * const impl, void * const context, wt_error_t * const error) {
    bool ok = true;
    ok = !ok ? ok : wt_assert_ptr(instance, error);
    if (ok) {
        instance->impl = impl;
        instance->context = context;
    }
    return ok ? instance : NULL;
}

bool wthal_uart_open(wthal_uart_t * const instance, wt_error_t * const error) {
    return instance->impl->open(instance->context, error);
}

bool wthal_uart_close(wthal_uart_t * const instance, wt_error_t * const error) {
    return instance->impl->close(instance->context, error);
}

bool wthal_uart_baudrate(wthal_uart_t * const instance, uint32_t const baudrate, wt_error_t * const error) {
    return instance->impl->baudrate(instance->context, baudrate, error);
}

bool wthal_uart_flowcontrol(wthal_uart_t * const instance, bool const flowcontrol, wt_error_t * const error) {
    return instance->impl->flowcontrol(instance->context, flowcontrol, error);
}

bool wthal_uart_read(wthal_uart_t * const instance, void * const data, size_t const size, wt_error_t * const error) {
    return instance->impl->read(instance->context, data, size, error);
}

bool wthal_uart_write(wthal_uart_t * const instance, void const * const data, size_t const size, wt_error_t * const error) {
    return instance->impl->write(instance->context, data, size, error);
}

#ifndef UART_TESTING
#define UART_TX_ISR(X)          (_U ## X ## TXInterrupt)
#define UART_RX_ISR(X)          (_U ## X ## RXInterrupt)
#define UART_ERR_ISR(X)         (_U ## X ## ErrInterrupt)
#define UART_TXIE(X)            (_U ## X ## TXIE)
#define UART_TXIF(X)            (_U ## X ## TXIF)
#define UART_TXIP(X)            (_U ## X ## TXIP)
#define UART_RXIE(X)            (_U ## X ## RXIE)
#define UART_RXIF(X)            (_U ## X ## RXIF)
#define UART_RXIP(X)            (_U ## X ## RXIP)
#define UART_ERIE(X)            (_U ## X ## ERIE)
#define UART_ERIF(X)            (_U ## X ## ERIF)
#define UART_ERIP(X)            (_U ## X ## ERIP)
#define UART_TXBF(X)            (U ## X ## STAbits.UTXBF)
#define UART_TXREG(X)           (U ## X ## TXREG)
#define UART_RXREG(X)           (U ## X ## RXREG)
#define UART_STA(X)             (U ## X ## STA)
#define UART_RXDA(X)            (U ## X ## STAbits.URXDA)
#define UART_OERR(X)            (U ## X ## STAbits.OERR)
#define UART_MODE(X)            (U ## X ## MODE)
#define UART_BRGH(X)            (U ## X ## MODEbits.BRGH)
#define UART_BRG(X)             (U ## X ## BRG)
#define UART_UARTEN(X)          (U ## X ## MODEbits.UARTEN)
#define UART_TXEN(X)            (U ## X ## STAbits.UTXEN)
#define UART_EN(X)              (U ## X ## MODEbits.UEN)
#endif /* UART_TESTING */

#define WTHAL_UART_DECLARE(NAME) \
    WTHAL_ISR_DECLARE(NAME ## _tx) \
    WTHAL_ISR_DECLARE(NAME ## _rx) \
    WTHAL_ISR_DECLARE(NAME ## _err) \
    typedef struct { \
        uint32_t baudrate; \
        bool flowcontrol; \
        wthal_uart_t uart; \
        wthal_isr_t * tx_isr; \
        wthal_isr_t * rx_isr; \
        wthal_isr_t * err_isr; \
        NAME ## _tx_t _tx_isr; \
        NAME ## _rx_t _rx_isr; \
        NAME ## _err_t _err_isr; \
        wthal_observer_t tx_observers[1]; \
        wthal_observer_t rx_observers[1]; \
        wthal_observer_t err_observers[1]; \
        wt_rx14xx_uint8_buffer_t tx_buffer; \
        wt_rx14xx_uint8_buffer_t rx_buffer; \
        uint8_t tx_storage[256]; \
        uint8_t rx_storage[256]; \
    } NAME ## _t; \
    wthal_uart_t * const NAME ## _init(NAME ## _t * const instance, uint32_t const baudrate, bool const flowcontrol, wthal_isr_priority_t const tx_priority, wthal_isr_priority_t const rx_priority, wthal_isr_priority_t const err_priority, wt_error_t * const error);

#define WTHAL_UART_DEFINE(NAME, UART, XTAL) \
    WTHAL_ISR_DEFINE(NAME ## _tx, UART_TX_ISR(UART), UART_TXIF(UART), UART_TXIE(UART), UART_TXIP(UART)); \
    WTHAL_ISR_DEFINE(NAME ## _rx, UART_RX_ISR(UART), UART_RXIF(UART), UART_RXIE(UART), UART_RXIP(UART)); \
    WTHAL_ISR_DEFINE(NAME ## _err, UART_ERR_ISR(UART), UART_ERIF(UART), UART_ERIE(UART), UART_ERIP(UART)); \
    static void NAME ## _tx_isr_impl(void * const context, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        while (! UART_TXBF(UART)) { \
            uint8_t data; \
            if (wt_rx14xx_uint8_buffer_read(&instance->tx_buffer, &data, sizeof(data), error)) { \
                UART_TXREG(UART) = data; \
            } else { \
                wthal_isr_enable(instance->tx_isr, false, error); \
                break; \
            } \
        } \
    } \
    static void NAME ## _rx_isr_impl(void * const context, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        bool ok = true; \
        while (ok && UART_RXDA(UART)) { \
            uint8_t data = UART_RXREG(UART); \
            ok = !ok ? ok : wt_rx14xx_uint8_buffer_write(&instance->rx_buffer, &data, sizeof(data), error); \
        } \
    } \
    static void NAME ## _err_isr_impl(void * const context, wt_error_t * const error) { \
        if (UART_OERR(UART) == 1) { \
            UART_OERR(UART) = 0; \
        } \
    } \
    static bool NAME ## _open_impl(void * const context, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        UART_MODE(UART) = 0x0000; \
        UART_BRGH(UART) = 1; \
        UART_STA(UART) = 0x0000; \
        wthal_uart_baudrate(&instance->uart, instance->baudrate, error); \
        wthal_uart_flowcontrol(&instance->uart, instance->flowcontrol, error); \
        wthal_isr_enable(instance->tx_isr, false, error); \
        wthal_isr_enable(instance->rx_isr, true, error); \
        wthal_isr_enable(instance->err_isr, true, error); \
        UART_UARTEN(UART) = 1; \
        UART_TXEN(UART) = 1; \
        return true; \
    } \
    static bool NAME ## _close_impl(void * const context, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        wthal_isr_enable(instance->tx_isr, false, error); \
        wthal_isr_enable(instance->rx_isr, false, error); \
        wthal_isr_enable(instance->err_isr, false, error); \
        UART_UARTEN(UART) = 0; \
        UART_TXEN(UART) = 0; \
        return true; \
    } \
    static bool NAME ## _baudrate_impl(void * const context, uint32_t const baudrate, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        instance->baudrate = baudrate; \
        UART_BRG(UART) = (XTAL / (4 * instance->baudrate)) - 1; \
        return true; \
    } \
    static bool NAME ## _flowcontrol_impl(void * const context, bool const flowcontrol, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        instance->flowcontrol = flowcontrol; \
        UART_EN(UART) = instance->flowcontrol ? 2 : 0; \
        return true; \
    } \
    static size_t NAME ## _read_impl(void * const context, void * const data, size_t const size, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        return wt_rx14xx_uint8_buffer_read(&instance->rx_buffer, data, size, error); \
    } \
    static size_t NAME ## _write_impl(void * const context, void const * const data, size_t const size, wt_error_t * const error) { \
        NAME ## _t * const instance = context; \
        bool ok = true; \
        size_t written = wt_rx14xx_uint8_buffer_write(&instance->tx_buffer, data, size, error); \
        if (written > 0) { \
            ok = !ok ? ok : wthal_isr_set_flag(instance->tx_isr, true, error); \
            ok = !ok ? ok : wthal_isr_enable(instance->tx_isr, true, error); \
        } \
        return written; \
    } \
    static wthal_uart_impl_t const NAME ## _impl = { \
        .open = NAME ## _open_impl, \
        .close = NAME ## _close_impl, \
        .baudrate = NAME ## _baudrate_impl, \
        .flowcontrol = NAME ## _flowcontrol_impl, \
        .read = NAME ## _read_impl, \
        .write = NAME ## _write_impl, \
    }; \
    wthal_uart_t * const NAME ## _init(NAME ## _t * const instance, uint32_t const baudrate, bool const flowcontrol, wthal_isr_priority_t const tx_priority, wthal_isr_priority_t const rx_priority, wthal_isr_priority_t const err_priority, wt_error_t * const error) { \
        bool ok = true; \
        instance->baudrate = baudrate; \
        instance->flowcontrol = flowcontrol; \
        ok = !ok ? ok : (instance->tx_isr = NAME ## _tx_init(&instance->_tx_isr, tx_priority, instance->tx_observers, 1, error)) != NULL; \
        ok = !ok ? ok : (instance->rx_isr = NAME ## _rx_init(&instance->_rx_isr, rx_priority, instance->rx_observers, 1, error)) != NULL; \
        ok = !ok ? ok : (instance->err_isr = NAME ## _err_init(&instance->_err_isr, err_priority, instance->err_observers, 1, error)) != NULL; \
        ok = !ok ? ok : wthal_isr_add_observer(instance->tx_isr, NAME ## _tx_isr_impl, instance, error); \
        ok = !ok ? ok : wthal_isr_add_observer(instance->rx_isr, NAME ## _rx_isr_impl, instance, error); \
        ok = !ok ? ok : wthal_isr_add_observer(instance->err_isr, NAME ## _err_isr_impl, instance, error); \
        ok = !ok ? ok : wt_rx14xx_uint8_buffer_init(&instance->tx_buffer, instance->tx_storage, 256, error); \
        ok = !ok ? ok : wt_rx14xx_uint8_buffer_init(&instance->rx_buffer, instance->rx_storage, 256, error); \
        return ok ? wthal_uart_init(&instance->uart, &NAME ## _impl, instance, error) : NULL; \
    }

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

WTHAL_UART_DECLARE(wt_rx14xx_xbee_uart);
WTHAL_UART_DEFINE(wt_rx14xx_xbee_uart, 1, XTAL);
WTHAL_UART_DECLARE(wt_rx14xx_debug_uart);
WTHAL_UART_DEFINE(wt_rx14xx_debug_uart, 2, XTAL);
WTHAL_UART_DECLARE(wt_rx14xx_primary_ethernet_uart);
WTHAL_UART_DEFINE(wt_rx14xx_primary_ethernet_uart, 3, XTAL);
WTHAL_UART_DECLARE(wt_rx14xx_secondary_ethernet_uart);
WTHAL_UART_DEFINE(wt_rx14xx_secondary_ethernet_uart, 4, XTAL);
WTHAL_GPIO_DECLARE(wt_rx14xx_led1);
WTHAL_GPIO_DEFINE(wt_rx14xx_led1, _RD7, _TRISD7, _LATD7, _ANSD7, _CN16PUE, _CN16PDE, _ODD7);
WTHAL_GPIO_DECLARE(wt_rx14xx_led2);
WTHAL_GPIO_DEFINE(wt_rx14xx_led2, _RD6, _TRISD6, _LATD6, _ANSD6, _CN15PUE, _CN15PDE, _ODD6);
WTHAL_GPIO_DECLARE(wt_rx1400_xpc_reset);
WTHAL_GPIO_DEFINE(wt_rx1400_xpc_reset, _RB2, _TRISB2, _LATB2, _ANSB2, _CN4PUE, _CN4PDE, _ODB2);
WTHAL_ISR_DECLARE(wt_rx14xx_tmr5);
WTHAL_ISR_DEFINE(wt_rx14xx_tmr5, _T5Interrupt, _T5IF, _T5IE, _T5IP);
WTHAL_TIMER_DECLARE(wt_rx14xx_timer5);
WTHAL_TIMER_DEFINE(wt_rx14xx_timer5, XTAL, TMR5, PR5, T5CONbits.TON, T5CONbits.TCKPS, _T5IF);

typedef struct {
    
    wthal_isr_t * t5_isr;
    wthal_gpio_t * startup_led;
    wthal_gpio_t * activity_led;
    wthal_gpio_t * xpc_reset;
    wthal_timer_t * timer5;
    wthal_counter_t * counter;
    wthal_uart_t * debug_uart;
    wthal_uart_t * xbee_uart;
    wthal_uart_t * primary_ethernet_uart;
    wthal_uart_t * secondary_ethernet_uart;
    
} wthal_t;

#define WT_RX1400_HAL_T5_OBSERVER_SIZE          (5)

typedef struct {
    
    wthal_t hal;
    
    wthal_observer_t t5_observers[WT_RX1400_HAL_T5_OBSERVER_SIZE];
    
    wt_rx14xx_tmr5_t t5_isr;
    wt_rx14xx_led1_t startup_led;
    wt_rx14xx_led2_t activity_led;
    wt_rx1400_xpc_reset_t xpc_reset;
    wt_rx14xx_timer5_t timer5;
    wthal_counter_t counter;
    wt_rx14xx_debug_uart_t debug_uart;
    wt_rx14xx_xbee_uart_t xbee_uart;
    wt_rx14xx_primary_ethernet_uart_t primary_ethernet_uart;
    wt_rx14xx_secondary_ethernet_uart_t secondary_ethernet_uart;
    
} wt_rx1400_hal_t;

wthal_t * const wt_rx1400_hal_init(wt_rx1400_hal_t * const instance, wt_error_t * const error) {
    
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
    
    ok = !ok ? ok : (instance->hal.t5_isr = wt_rx14xx_tmr5_init(&instance->t5_isr, wt_rx1400_isr_priority_timer, instance->t5_observers, WT_RX1400_HAL_T5_OBSERVER_SIZE, error)) != NULL;
    ok = !ok ? ok : (instance->hal.startup_led = wt_rx14xx_led1_init(&instance->startup_led, error)) != NULL;
    ok = !ok ? ok : (instance->hal.activity_led = wt_rx14xx_led2_init(&instance->activity_led, error)) != NULL;
    ok = !ok ? ok : (instance->hal.xpc_reset = wt_rx1400_xpc_reset_init(&instance->xpc_reset, error)) != NULL;
    ok = !ok ? ok : (instance->hal.timer5 = wt_rx14xx_timer5_init(&instance->timer5, wt_rx14xx_timer_prescale_1, error)) != NULL;
    ok = !ok ? ok : (instance->hal.counter = wthal_counter_init(&instance->counter, error)) != NULL;
    ok = !ok ? ok : (instance->hal.debug_uart = wt_rx14xx_debug_uart_init(&instance->debug_uart, 230400, true, wt_rx1400_isr_priority_debug_uart_tx, wt_rx1400_isr_priority_debug_uart_rx, wt_rx1400_isr_priority_debug_uart_err, error)) != NULL;
    ok = !ok ? ok : (instance->hal.xbee_uart = wt_rx14xx_xbee_uart_init(&instance->xbee_uart, 9600, true, wt_rx1400_isr_priority_xbee_uart_tx, wt_rx1400_isr_priority_xbee_uart_rx, wt_rx1400_isr_priority_xbee_uart_err, error)) != NULL;
    ok = !ok ? ok : (instance->hal.primary_ethernet_uart = wt_rx14xx_primary_ethernet_uart_init(&instance->primary_ethernet_uart, 115200, true, wt_rx1400_isr_priority_primary_ethernet_uart_tx, wt_rx1400_isr_priority_primary_ethernet_uart_rx, wt_rx1400_isr_priority_primary_ethernet_uart_err, error)) != NULL;
    ok = !ok ? ok : (instance->hal.secondary_ethernet_uart = wt_rx14xx_secondary_ethernet_uart_init(&instance->secondary_ethernet_uart, 115200, true, wt_rx1400_isr_priority_secondary_ethernet_uart_tx, wt_rx1400_isr_priority_secondary_ethernet_uart_rx, wt_rx1400_isr_priority_secondary_ethernet_uart_err, error)) != NULL;
    
    ok = !ok ? ok : wthal_gpio_weak_pull_up(instance->hal.xpc_reset, true, error);
    
    ok = !ok ? ok : wthal_isr_add_observer(instance->hal.t5_isr, wthal_counter_isr, instance->hal.counter, error);
    ok = !ok ? ok : wthal_isr_enable(instance->hal.t5_isr, true, error);
    ok = !ok ? ok : wthal_timer_start(instance->hal.timer5, 1, error);
    
    ok = !ok ? ok : wthal_uart_open(instance->hal.debug_uart, error);
    ok = !ok ? ok : wthal_uart_open(instance->hal.xbee_uart, error);
    ok = !ok ? ok : wthal_uart_open(instance->hal.primary_ethernet_uart, error);
    ok = !ok ? ok : wthal_uart_open(instance->hal.secondary_ethernet_uart, error);
    
    ok = !ok ? ok : wthal_set_stdout(instance->hal.debug_uart, error);
    
    return ok ? &instance->hal : NULL;
    
}

int app_main(wthal_t * const hal, wt_error_t * const error) {

    uint32_t count = 0;
    bool ok = true;
    
    ok = !ok ? ok : wthal_counter_reset(hal->counter, error);
    ok = !ok ? ok : wthal_gpio_set(hal->startup_led, true, error);
    while (ok && wthal_counter_get(hal->counter, error) < 3000) {
        ClrWdt();
        Nop();
    }
    ok = !ok ? ok : wthal_gpio_set(hal->startup_led, false, error);
    
    while (ok)
    {
        // Add your application code
        ok = !ok ? ok : wthal_gpio_set(hal->activity_led, true, error);
        ok = !ok ? ok : wthal_counter_reset(hal->counter, error);
        while(wthal_counter_get(hal->counter, error) < 1000) {
            ClrWdt();
            Nop();
        }
        ok = !ok ? ok : wthal_gpio_set(hal->activity_led, false, error);
        ok = !ok ? ok : wthal_counter_reset(hal->counter, error);
        while(wthal_counter_get(hal->counter, error) < 1000) {
            ClrWdt();
            Nop();
        }
        for (size_t i=0 ; i < 1 ; i++) {
            printf("\nHello World %ld", (uint32_t)count++);        
        }
    }

    return 1;

}

int main(void)
{
    wt_error_t error;
    
    // initialize the device - MUST COME FIRST OR WILL OVERRIDE HAL STYLE CONFIG
    SYSTEM_Initialize();

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
//    _U4CTSR = 12;               // UART3:U3CTS->RP12/RD11
//    _RP3R = _RPOUT_U4RTS;       // RP3/RD10->UART3:U3RTS

    PPS_LOCK();
    
    wt_rx1400_hal_t rx1400_hal;
    wthal_t * hal = wt_rx1400_hal_init(&rx1400_hal, &error);
 
    if (hal != NULL) { 
        return app_main(hal, &error);
    }
    
    return -1;

}
/**
 End of File
*/

