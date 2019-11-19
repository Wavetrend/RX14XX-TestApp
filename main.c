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

/*
                         Main application
 */

WTAPI_DECLARE_CIRCULAR_BUFFER(uint8_t, wt_rx14xx_uint8_buffer);
WTAPI_DEFINE_CIRCULAR_BUFFER(uint8_t, wt_rx14xx_uint8_buffer);

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

typedef enum { 
    wt_rx14xx_timer_prescale_1 = 0,
    wt_rx14xx_timer_prescale_8 = 1,
    wt_rx14xx_timer_prescale_64 = 2,
    wt_rx14xx_timer_prescale_256 = 3,
} wt_rx14xx_timer_prescale_t;

#define DEFINE_TIMER(NAME, XTAL, TIMER, PERIOD, TON, TCKPS, IF) \
    typedef struct { \
        wthal_timer_t timer; \
    } NAME ## _t; \
    static bool start_impl(void * const context, uint32_t const msecs) { \
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
        return ok; \
    } \
    static bool stop_impl(void * const context) { \
        TON = 0; \
        return true; \
    } \
    static wthal_timer_impl_t NAME ## _impl = { \
        .start = start_impl, \
        .stop = stop_impl, \
    }; \
    wthal_timer_t * const NAME ## _init(NAME ## _t * const self, wt_rx14xx_timer_prescale_t const prescale) { \
        TON = 0; \
        TIMER = 0; \
        TCKPS = prescale; \
        return wthal_timer_init(&self->timer, &NAME ## _impl, self); \
    }

//-------------------------

typedef struct {
    uint32_t volatile count;
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
    wthal_observer_t * volatile next;
    wthal_observer_t * volatile prev;
};

typedef struct {
    wthal_observer_t * volatile head;
} wthal_observers_t;

void wthal_observers_dispatch(wthal_observers_t * const self) {
    wthal_observer_t * el, * tmp;
    DL_FOREACH_SAFE(self->head, el, tmp) {
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
    bool (*set_flag)(void * const context, bool const set);
    bool (*get_flag)(void * const context);
    bool (*set_priority)(void * const context, uint8_t const priority);
    uint8_t (*get_priority)(void * const context);
    bool (*add_observer)(void * const context, void (*callback)(void * const context), void * const callback_context);
    
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

wthal_isr_t * const wthal_isr_init(wthal_isr_t * const self, wthal_isr_impl_t * const impl, void * const context) {
    self->impl = impl;
    self->context = context;
    return self;
}

bool wthal_isr_enable(wthal_isr_t * const self, bool const enable) {
    return self->impl->enable(self->context, enable);
}

bool wthal_isr_set_flag(wthal_isr_t * const self, bool const set) {
    return self->impl->set_flag(self->context, set);
}

bool wthal_isr_get_flag(wthal_isr_t * const self) {
    return self->impl->get_flag(self->context);
}

bool wthal_isr_set_priority(wthal_isr_t * const self, uint8_t const priority) {
    return self->impl->set_priority(self->context, priority);
}

bool wthal_isr_get_priority(wthal_isr_t * const self) {
    return self->impl->get_priority(self->context);
}

bool wthal_isr_add_observer(wthal_isr_t * const self, void (*callback)(void * const context), void * const context) {
    return self->impl->add_observer(self->context, callback, context);
}

#define DEFINE_ISR(NAME, ISR, IF, IE, IP) \
    typedef struct { \
        wthal_isr_t isr; \
    } NAME ## _t; \
    static wthal_observers_t NAME ## _active = {}; \
    static wthal_observers_t NAME ## _inactive = {}; \
    void __attribute__ (( interrupt, no_auto_psv )) ISR ( void ) { \
        wthal_observers_dispatch(&NAME ## _active); \
        IF = 0; \
    } \
    static bool NAME ## _enable_impl(void * const context, bool const enable) { \
        IE = enable; \
        return true; \
    } \
    static bool NAME ## _set_flag_impl(void * const context, bool const set) { \
        IF = set; \
        return true; \
    } \
    static bool NAME ## _get_flag_impl(void * const context) { \
        return IF; \
    } \
    static bool NAME ## _set_priority_impl(void * const context, uint8_t const priority) { \
        IP = priority; \
        return true; \
    } \
    static uint8_t NAME ## _get_priority_impl(void * const context) { \
        return IP; \
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
        .set_flag = NAME ## _set_flag_impl, \
        .get_flag = NAME ## _get_flag_impl, \
        .set_priority = NAME ## _set_priority_impl, \
        .get_priority = NAME ## _get_priority_impl, \
    }; \
    wthal_isr_t * const NAME ## _init(NAME ## _t * const self, wthal_isr_priority_t const interrupt_priority, wthal_observer_t * const observers, size_t const size) { \
        IE = 0; \
        IF = 0; \
        IP = interrupt_priority; \
        for (size_t i=0 ; i < size ; i++) { \
            wthal_observers_add(&NAME ## _inactive, &observers[i]); \
        } \
        return wthal_isr_init(&self->isr, &NAME ## _impl, self); \
    }

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

///////////////////////////////// UART ///////////////////////////////////////

typedef struct {
    
    bool (*open)(void * const context);
    bool (*close)(void * const context);
    bool (*baudrate)(void * const context, uint32_t const baudrate);
    bool (*flowcontrol)(void * const context, bool const flowcontrol);
    size_t (*read)(void * const context, void * const data, size_t const size);
    size_t (*write)(void * const context, void const * const data, size_t const size);
    
} wthal_uart_impl_t;

typedef struct {
    
    wthal_uart_impl_t const * impl;
    void * context;
    
} wthal_uart_t;

wthal_uart_t * const wthal_uart_init(wthal_uart_t * const self, wthal_uart_impl_t const * const impl, void * const context) {
    self->impl = impl;
    self->context = context;
    return self;
}

bool wthal_uart_open(wthal_uart_t * const self) {
    return self->impl->open(self->context);
}

bool wthal_uart_close(wthal_uart_t * const self) {
    return self->impl->close(self->context);
}

bool wthal_uart_baudrate(wthal_uart_t * const self, uint32_t const baudrate) {
    return self->impl->baudrate(self->context, baudrate);
}

bool wthal_uart_flowcontrol(wthal_uart_t * const self, bool const flowcontrol) {
    return self->impl->flowcontrol(self->context, flowcontrol);
}

bool wthal_uart_read(wthal_uart_t * const self, void * const data, size_t const size) {
    return self->impl->read(self->context, data, size);
}

bool wthal_uart_write(wthal_uart_t * const self, void const * const data, size_t const size) {
    return self->impl->write(self->context, data, size);
}

#define DEFINE_UART(NAME, UART, XTAL) \
    DEFINE_ISR(NAME ## _tx, _U ## UART ## TXInterrupt, _U ## UART ## TXIF, _U ## UART ## TXIE, _U ## UART ## TXIP); \
    DEFINE_ISR(NAME ## _rx, _U ## UART ## RXInterrupt, _U ## UART ## RXIF, _U ## UART ## RXIE, _U ## UART ## RXIP); \
    DEFINE_ISR(NAME ## _err, _U ## UART ## ErrInterrupt, _U ## UART ## ERIF, _U ## UART ## ERIE, _U ## UART ## ERIP); \
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
    static void NAME ## _tx_isr_impl(void * const context) { \
        NAME ## _t * const self = context; \
        while (! U ## UART ## STAbits.UTXBF) { \
            uint8_t data; \
            if (wt_rx14xx_uint8_buffer_read(&self->tx_buffer, &data, sizeof(data), NULL)) { \
                U ## UART ## TXREG = data; \
            } else { \
                wthal_isr_enable(self->tx_isr, false); \
                break; \
            } \
        } \
    } \
    static void NAME ## _rx_isr_impl(void * const context) { \
        NAME ## _t * const self = context; \
        bool ok = true; \
        while (ok && U ## UART ## STAbits.URXDA) { \
            uint8_t data = U ## UART ## RXREG; \
            ok = !ok ? ok : wt_rx14xx_uint8_buffer_write(&self->rx_buffer, &data, sizeof(data), NULL); \
        } \
    } \
    static void NAME ## _err_isr_impl(void * const context) { \
        if (U ## UART ## STAbits.OERR == 1) { \
            U ## UART ## STAbits.OERR = 0; \
        } \
    } \
    static bool NAME ## _open_impl(void * const context) { \
        NAME ## _t * const self = context; \
        U ## UART ## MODE = 0x0000; \
        U ## UART ## MODEbits.BRGH = 1; \
        U ## UART ## STA = 0x0000; \
        wthal_uart_baudrate(&self->uart, self->baudrate); \
        wthal_uart_flowcontrol(&self->uart, self->flowcontrol); \
        wthal_isr_enable(self->tx_isr, false); \
        wthal_isr_enable(self->rx_isr, true); \
        wthal_isr_enable(self->err_isr, true); \
        U ## UART ## MODEbits.UARTEN = 1; \
        U ## UART ## STAbits.UTXEN = 1; \
        return true; \
    } \
    static bool NAME ## _close_impl(void * const context) { \
        NAME ## _t * const self = context; \
        wthal_isr_enable(self->tx_isr, false); \
        wthal_isr_enable(self->rx_isr, false); \
        wthal_isr_enable(self->err_isr, false); \
        U ## UART ## MODEbits.UARTEN = 0; \
        U ## UART ## STAbits.UTXEN = 0; \
        return true; \
    } \
    static bool NAME ## _baudrate_impl(void * const context, uint32_t const baudrate) { \
        NAME ## _t * const self = context; \
        self->baudrate = baudrate; \
        U ## UART ## BRG = (XTAL / (4 * self->baudrate)) - 1; \
        return true; \
    } \
    static bool NAME ## _flowcontrol_impl(void * const context, bool const flowcontrol) { \
        NAME ## _t * const self = context; \
        self->flowcontrol = flowcontrol; \
        U ## UART ## MODEbits.UEN = self->flowcontrol ? 2 : 0; \
        return true; \
    } \
    static size_t NAME ## _read_impl(void * const context, void * const data, size_t const size) { \
        NAME ## _t * const self = context; \
        return wt_rx14xx_uint8_buffer_read(&self->rx_buffer, data, size, NULL); \
    } \
    static size_t NAME ## _write_impl(void * const context, void const * const data, size_t const size) { \
        NAME ## _t * const self = context; \
        size_t written = wt_rx14xx_uint8_buffer_write(&self->tx_buffer, data, size, NULL); \
        if (written > 0) { \
            wthal_isr_set_flag(self->tx_isr, true); \
            wthal_isr_enable(self->tx_isr, true); \
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
    wthal_uart_t * const NAME ## _init(NAME ## _t * const self, uint32_t const baudrate, bool const flowcontrol, wthal_isr_priority_t const tx_priority, wthal_isr_priority_t const rx_priority, wthal_isr_priority_t const err_priority) { \
        self->baudrate = baudrate; \
        self->flowcontrol = flowcontrol; \
        self->tx_isr = NAME ## _tx_init(&self->_tx_isr, tx_priority, self->tx_observers, 1); \
        self->rx_isr = NAME ## _rx_init(&self->_rx_isr, rx_priority, self->rx_observers, 1); \
        self->err_isr = NAME ## _err_init(&self->_err_isr, err_priority, self->err_observers, 1); \
        wthal_isr_add_observer(self->tx_isr, NAME ## _tx_isr_impl, self); \
        wthal_isr_add_observer(self->rx_isr, NAME ## _rx_isr_impl, self); \
        wthal_isr_add_observer(self->err_isr, NAME ## _err_isr_impl, self); \
        wt_rx14xx_uint8_buffer_init(&self->tx_buffer, self->tx_storage, 256, NULL); \
        wt_rx14xx_uint8_buffer_init(&self->rx_buffer, self->rx_storage, 256, NULL); \
        return wthal_uart_init(&self->uart, &NAME ## _impl, self); \
    }

//////////////////////////////// STDOUT ///////////////////////////////////////

static wthal_uart_t * wthal_stdout_uart = NULL;

int __attribute__ ( ( __section__(".libc.write" ) ) ) write(int handle, void * buffer, unsigned int len) {
    if (wthal_stdout_uart != NULL) {
        return (int) wthal_uart_write(wthal_stdout_uart, buffer, len);
    }
    return 0;
}

bool wthal_set_stdout(wthal_uart_t * const uart) {
    wthal_stdout_uart = uart;
    return true;
}

///////////////////////////////// MAIN ///////////////////////////////////////

#define PPS_UNLOCK()    __builtin_write_OSCCONL(OSCCON & 0xbf)
#define PPS_LOCK()      __builtin_write_OSCCONL(OSCCON | 0x40)

#define XTAL (14745600)

DEFINE_UART(wt_rx14xx_debug_uart, 2, XTAL);
DEFINE_GPIO(wt_rx14xx_led1, _RD7, _TRISD7, _LATD7, _ANSD7, _CN16PUE, _CN16PDE, _ODD7);
DEFINE_GPIO(wt_rx14xx_led2, _RD6, _TRISD6, _LATD6, _ANSD6, _CN15PUE, _CN15PDE, _ODD6);
DEFINE_GPIO(wt_rx1400_xpc_reset, _RB2, _TRISB2, _LATB2, _ANSB2, _CN4PUE, _CN4PDE, _ODB2);
DEFINE_ISR(wt_rx14xx_tmr5, _T5Interrupt, _T5IF, _T5IE, _T5IP);
DEFINE_TIMER(wt_rx14xx_timer5, XTAL, TMR5, PR5, T5CONbits.TON, T5CONbits.TCKPS, _T5IF);

typedef struct {
    
    wthal_isr_t * t5_isr;
    wthal_gpio_t * startup_led;
    wthal_gpio_t * activity_led;
    wthal_gpio_t * xpc_reset;
    wthal_timer_t * timer5;
    wthal_counter_t * counter;
    wthal_uart_t * debug_uart;
    
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
    
} wt_rx1400_hal_t;

wthal_t * const wt_rx1400_hal_init(wt_rx1400_hal_t * const self) {
    
    self->hal.t5_isr = wt_rx14xx_tmr5_init(&self->t5_isr, wthal_isr_priority_4, self->t5_observers, WT_RX1400_HAL_T5_OBSERVER_SIZE);
    self->hal.startup_led = wt_rx14xx_led1_init(&self->startup_led);
    self->hal.activity_led = wt_rx14xx_led2_init(&self->activity_led);
    self->hal.xpc_reset = wt_rx1400_xpc_reset_init(&self->xpc_reset);
    self->hal.timer5 = wt_rx14xx_timer5_init(&self->timer5, wt_rx14xx_timer_prescale_1);
    self->hal.counter = wthal_counter_init(&self->counter);
    self->hal.debug_uart = wt_rx14xx_debug_uart_init(&self->debug_uart, 230400, true, wthal_isr_priority_4, wthal_isr_priority_4, wthal_isr_priority_4);
    
    wthal_gpio_weak_pull_up(self->hal.xpc_reset, true);
    
    wthal_isr_add_observer(self->hal.t5_isr, wthal_counter_isr, self->hal.counter);
    wthal_isr_enable(self->hal.t5_isr, true);
    wthal_timer_start(self->hal.timer5, 1);
    
    wthal_uart_open(self->hal.debug_uart);
    wthal_set_stdout(self->hal.debug_uart);
    
    return &self->hal;
    
}

int app_main(wthal_t * const hal) {

    uint32_t count = 0;

    printf("\nPR5=%u, TCKPS=%u", PR5, T5CONbits.TCKPS);
    
    wthal_counter_reset(hal->counter);
    wthal_gpio_set(hal->startup_led, true);
    while (wthal_counter_get(hal->counter) < 3000) {
        ClrWdt();
        Nop();
    }
    wthal_gpio_set(hal->startup_led, false);
    
    while (1)
    {
        // Add your application code
        wthal_gpio_set(hal->activity_led, true);
        wthal_counter_reset(hal->counter);
        while(wthal_counter_get(hal->counter) < 1000) {
            ClrWdt();
            Nop();
        }
        wthal_gpio_set(hal->activity_led, false);
        wthal_counter_reset(hal->counter);
        while(wthal_counter_get(hal->counter) < 1000) {
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
    // initialize the device - MUST COME FIRST OR WILL OVERRIDE HAL STYLE CONFIG
    SYSTEM_Initialize();

    PPS_UNLOCK();

    _U2RXR = 23;                // UART2:U2RX->RP23/RD2
    _RP22R = _RPOUT_U2TX;       // RP22/RD3->UART2:U2TX

    PPS_LOCK();
    
    wt_rx1400_hal_t rx1400_hal;
    wthal_t * hal = wt_rx1400_hal_init(&rx1400_hal);
 
    return app_main(hal);

}
/**
 End of File
*/

