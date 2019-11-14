/**
  @Generated PIC24 / dsPIC33 / PIC32MM MCUs Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.145.0
        Device            :  PIC24FJ256GB206
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36b
        MPLAB             :  MPLAB X v5.25
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


// Configuration bits: selected in the GUI

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

#include "mcc.h"
#include "clock.h"


void OSCILLATOR_Initialize(void)
{
    CLOCK_Initialize();
}

/**
 End of File
*/