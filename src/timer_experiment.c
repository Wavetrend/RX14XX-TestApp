#ifdef INCLUDED

///////////////////////////// pic24f_isr //////////////////////////////

//typedef void (*pic24f_isr_handler_t)(void * const context);
//
//typedef struct {
//    pic24f_isr_handler_t handler;
//    void * context;
//} pic24f_isr_t;
//
//typedef enum {
//    PIC24F_ISR_INDEX_TMR1,
//    PIC24F_ISR_INDEX_TMR2,
//    PIC24F_ISR_INDEX_TMR3,
//    PIC24F_ISR_INDEX_TMR4,
//    PIC24F_ISR_INDEX_TMR5,
//    PIC24F_ISR_INDEX_MAX,
//} pic24f_isr_index_t;
//
//static pic24f_isr_t pic24_isrs[PIC24F_ISR_INDEX_MAX] = {};
//
//void pic24f_isr_dispatch(pic24f_isr_index_t const index) {
//    
//    if (pic24_isrs[index].handler != NULL) {
//        pic24_isrs[index].handler(pic24_isrs[index].context);
//    }
//}
//
//bool pic24f_isr_register(pic24f_isr_index_t const index, pic24f_isr_handler_t handler, void * const context) {
//    pic24_isrs[index].context = context;
//    pic24_isrs[index].handler = handler;
//    return true;
//}
//
//bool pic24f_isr_deregister(pic24f_isr_index_t const index) {
//    pic24_isrs[index].handler = NULL;
//    pic24_isrs[index].context = NULL;
//    return true;
//}
//
//#define DEFINE_ISR(index, symbol) 
//    void __attribute__ ( ( interrupt, no_auto_psv ) ) symbol ( void ) { 
//        pic24f_isr_dispatch(index); 
//    }
//
//DEFINE_ISR(PIC24F_ISR_INDEX_TMR1, _T1Interrupt);
//DEFINE_ISR(PIC24F_ISR_INDEX_TMR2, _T2Interrupt);
//DEFINE_ISR(PIC24F_ISR_INDEX_TMR3, _T3Interrupt);
//DEFINE_ISR(PIC24F_ISR_INDEX_TMR4, _T4Interrupt);
//DEFINE_ISR(PIC24F_ISR_INDEX_TMR5, _T5Interrupt);

/////////////////////////// wthal_timer ////////////////////////////


typedef struct {
    bool (*start)(void * const context);
    bool (*stop)(void * const context);
    bool (*set_handler)(void * const context, void (*handler)(void * const context));
} wthal_timer_impl_t;

typedef struct {
    wthal_timer_impl_t * impl;
    void * context;
} wthal_timer_t;

wthal_timer_t * const wthal_timer_init(
    wthal_timer_t * const instance,
    wthal_timer_impl_t * const impl,
    void * const context
) {
    instance->impl = impl;
    instance->context = context;
    return instance;
}

bool wthal_timer_start(wthal_timer_t * const instance) {
    return instance->impl->start(instance->context);
}

bool wthal_timer_stop(wthal_timer_t * const instance) {
    return instance->impl->stop(instance->context);
}

bool wthal_timer_set_handler(wthal_timer_t * const instance, void (*handler)(void * const context)) {
    return instance->impl->set_handler(instance->context, handler);
}

//////////////////////// TIMER //////////////////////////

#define DECLARE_TIMER(name) \
    typedef enum { \
        name ## _source_internal, \
        name ## _source_external, \
    } name ## _source_t; \
    typedef enum { \
        name ## _prescale_1, \
        name ## _prescale_8, \
        name ## _prescale_64, \
        name ## _prescale_256, \
    } name ## _prescale_t; \
    typedef struct { \
        wthal_timer_t timer; \
        wthal_timer_impl_t impl; \
    } name ## _t; \
    wthal_timer_t * const name ## _init( name ## _t * const instance, name ## _source_t const source, name ## _prescale_t const prescale, uint32_t const msecs ); \
    bool name ## _set_interrupt_handler( void (*handler)(void * const), void * const context); \

#define DEFINE_TIMER(name, TMR, PR, TCON, IF, IE, ISR) \
    static void (*_ ## name ## _callback)(void * const context) = NULL; \
    static void *_ ## name ## _context = NULL; \
    void __attribute__ ( ( interrupt, no_auto_psv ) ) ISR ( void ) { \
        if (_ ## name ## _callback != NULL) _ ## name ## _callback(_ ## name ## _context); \
        IF = 0; \
    } \
    void __attribute__ ( ( weak ) ) name ## _callback ( void * const context ) { \
    } \
    static bool name ## _start_impl ( void * const context ) { \
        TCON ## bits.TON = 1; \
        return true; \
    } \
    static bool name ## _stop_impl ( void * const context ) { \
        TCON ## bits.TON = 0; \
        return true; \
    } \
    static bool name ## _set_handler_impl( void * const context, void (*handler)(void * const context)) { \
        _ ## name ## _context = context; \
        _ ## name ## _callback = handler; \
        return true; \
    } \
    wthal_timer_t * const name ## _init( name ## _t * const instance, name ## _source_t const source, name ## _prescale_t const prescale, uint32_t const msecs ) { \
        instance->impl.start = name ## _start_impl; \
        instance->impl.stop = name ## _stop_impl; \
        instance->impl.set_handler = name ## _set_handler_impl; \
        TCON ## bits.TCS = source == name ## _source_internal ? 0 : 1; \
        TCON ## bits.TCKPS = prescale; \
        PR = msecs; \
        if (wthal_timer_init(&instance->timer, &instance->impl, instance) != NULL) { \
            wthal_timer_set_handler(&instance->timer, name ## _callback); \
        } \
        return &instance->timer; \
    }

/////////////////////////////////////////////////////////////

DECLARE_TIMER(timer5);
DEFINE_TIMER(timer5, TMR5, PR5, T5CON, _T5IF, _T5IE, _T5Interrupt);

int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
    timer5_t timer5;
    wthal_timer_t * const timer = timer5_init(&timer5, timer5_source_internal, timer5_prescale_8, 0x399 /* 1 msec */);
    // wthal_clock would register a tick counter
    wthal_timer_start(timer);
    
    printf("\nStartup...");
    
    uint32_t ticks = 0;
    uint32_t counter = 0;
    
    do {
        if (_T5IF) {
            ticks++;
            _T5IF = 0;
        }
        counter++;
    } while (ticks < 1000);

    printf("\nticks=%lu, counter=%lu", ticks, counter);
    
    wthal_timer_stop(timer);
    
    return 1;
}

#endif