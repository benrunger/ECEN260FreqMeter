/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 Timer_A - Continuous Overflow Interrupt
 *
 *  Description: Toggle P1.0 using software and the Timer0_A overflow ISR.
 *  In this example an ISR triggers when TA overflows. Inside the ISR P1.0
 *  is toggled. Toggle rate is exactly 0.5Hz. 
 *
 *  ACLK = TACLK = 32768Hz, MCLK = SMCLK = DCO = 3MHz
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P1.0  |---> P1.0 LED
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 ******************************************************************************/
/* Standard Includes */
#include "msp.h"
#include <stdint.h>
#define LOW 0
#define HIGH 1

/* Statics */
unsigned int result = 0;
    unsigned int period = 0;
    unsigned int prev_time = 0;
    unsigned int cur_time = 0;
    int prev_val = HIGH;

#define CE  0x01    /* P6.0 chip select */
#define RESET 0x40  /* P6.6 reset */
#define DC 0x80     /* P6.7 register select */

/* define the pixel size of display */
#define GLCD_WIDTH  84
#define GLCD_HEIGHT 48

void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_init(void);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_putchar(int c);
void SPI_init(void);
void SPI_write(unsigned char data);

/* sample font table */
const char font_table[][6] = {
                              {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00},  /* 0 */
                              {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},  /* 1 */
                              {0x42, 0x61, 0x51, 0x49, 0x46, 0x00},  /* 2 */
                              {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00},  /* 3 */
                              {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00},  /* 4 */
                              {0x27, 0x45, 0x45, 0x45, 0x39, 0x00},  /* 5 */
                              {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00},  /* 6 */
                              {0x01, 0x71, 0x09, 0x05, 0x03, 0x00},  /* 7 */
                              {0x36, 0x49, 0x49, 0x49, 0x36, 0x00},  /* 8 */
                              {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00},  /* 9 */
                              {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  /*   */
                              {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00},  /* H */
                              {0x61, 0x51, 0x49, 0x45, 0x43, 0x00},  /* Z */
};

///**
// * main.c
// */
//int main(void)
//{
//    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
//    GLCD_init();    /* initialize the GLCD controller */
//    GLCD_clear();   /* clear display and  home the cursor */
//
//    GLCD_putchar('H'-0x40);    // display letter H
//    GLCD_putchar('E'-0x40);    // display letter E
//    GLCD_putchar('L'-0x40);    // display letter L
//    GLCD_putchar('L'-0x40);    // display letter L
//    GLCD_putchar('O'-0x40);    // display letter L
//    GLCD_putchar(0);           // display ' '
//    GLCD_putchar('W'-0x40);    // display letter W
//    GLCD_putchar('O'-0x40);    // display letter O
//    GLCD_putchar('R'-0x40);    // display letter R
//    GLCD_putchar('L'-0x40);    // display letter L
//    GLCD_putchar('D'-0x40);    // display letter D
//    GLCD_putchar(27);          // display letter !
//    GLCD_putchar(28);          // display FIRST PART OF :)
//    GLCD_putchar(29);          // display SECOND PART OF :)
//    GLCD_
//
//
//
//    while(1) {}
//}
int j=0;
void GLCD_putchar(int c)
{
    int i;
    for(i = 0; i < 6; i++)
        GLCD_data_write(font_table[c][i]);
}

void GLCD_setCursor(unsigned char x, unsigned char y)
{
    GLCD_command_write(0x80 | x); /* column */
    GLCD_command_write(0x40 | y); /* bank (8 rows per bank) */
}

/* clears the GLCD by writing zeros to the entire screen */
void GLCD_clear(void)
{
    int32_t index;
    for(index = 0; index < (GLCD_WIDTH * GLCD_HEIGHT / 8); index++)
        GLCD_data_write(0x00);
    GLCD_setCursor(0, 0); /* return to the home position */
}

/* send the initialization commands to PCD8544 GLCD controller */
void GLCD_init(void)
{
    SPI_init();
    /* hardware reset of GLCD controller */
    P6->OUT |= RESET;   /* deasssert reset */

    GLCD_command_write(0x21);   /* set extended command mode */
    GLCD_command_write(0xB8);   /* set LCD Vop for contrast */
    GLCD_command_write(0x04);   /* set temp coefficient */
    GLCD_command_write(0x14);   /* set LCD bias mode 1:48 */
    GLCD_command_write(0x20);   /* set normal command mode */
    GLCD_command_write(0x0C);   /* set display normal mode */
}

/* write to GLCD controller data register */
void GLCD_data_write(unsigned char data)
{
    P6->OUT |= DC;              /* select data register */
    SPI_write(data);            /* send data via SPI */
}

/* write to GLCD controller command register */
void GLCD_command_write(unsigned char data)
{
    P6->OUT &= ~DC;             /* select command register */
    SPI_write(data);            /* send data via SPI */
}

void SPI_init(void)
{
    EUSCI_B0->CTLW0 = 0x0001;   /* put UCB0 in reset mode */
    EUSCI_B0->CTLW0 = 0x69C1;   /* PH=0, PL=1, MSB first, Master, SPI, SMCLK */
    EUSCI_B0->BRW = 3;          /* 3 MHz / 3 = 1MHz */
    EUSCI_B0->CTLW0 &= ~0x001;   /* enable UCB0 after config */

    P1->SEL0 |= 0x60;           /* P1.5, P1.6 for UCB0 */
    P1->SEL1 &= ~0x60;

    P6->DIR |= (CE | RESET | DC); /* P6.7, P6.6, P6.0 set as output */
    P6->OUT |= CE;              /* CE idle high */
    P6->OUT &= ~RESET;          /* assert reset */
}

void SPI_write(unsigned char data)
{
    P6->OUT &= ~CE;             /* assert /CE */
    EUSCI_B0->TXBUF = data;     /* write data */
    while(EUSCI_B0->STATW & 0x01);/* wait for transmit done */
    P6->OUT |= CE;              /* deassert /CE */
}

void display(int frequency)
{
    GLCD_clear();   /* clear display and  home the cursor */

    int digit0 = frequency % 10;
    frequency = frequency - digit0;

    int digit1 = (frequency % 100) / 10;
    frequency = frequency - (digit1 * 10);

    int digit2 = (frequency % 1000) / 100;
    frequency = frequency - (digit2 * 100);

    int digit3 = (frequency % 10000) / 1000;
    frequency = frequency - (digit3 * 100);

    int digit4 = (frequency % 100000) / 10000;

    GLCD_putchar(digit4);
    GLCD_putchar(digit3);
    GLCD_putchar(digit2);
    GLCD_putchar(digit1);
    GLCD_putchar(digit0);
    GLCD_putchar(10);
    GLCD_putchar(11);
    GLCD_putchar(12);
}

int main(void)
{
    volatile unsigned int i;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
            WDT_A_CTL_HOLD;

    P5->SEL1 |= BIT4;                       // Configure P5.4 for ADC A1
    P5->SEL0 |= BIT4;

    // Enable global interrupt
    __enable_irq();

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);  // Enable ADC interrupt in NVIC module

    // Configure internal reference
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);// If ref generator busy, WAIT
    REF_A->CTL0 |= REF_A_CTL0_VSEL_3 |      // Select internal ref = 2.5V
            REF_A_CTL0_ON;                  // Internal Reference ON
    for (i = 75; i > 0; i--);               // Delay (~75us) for Ref to settle

    // Configure ADC14
    // tsample = 16ADC14CLK cycles, tconvert = 16 ADC12CLK cycles
    // software trigger for SOC, MODOSC, single ch-single conversion,
    // tsample controlled by SHT0x settings
    // Channel 1, reference = internal, enable window comparator
    // Set thresholds for ADC14 interrupts
    // Enable Interrupts
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON | ADC14_CTL0_SHS_0 | ADC14_CTL0_SSEL_0 | ADC14_CTL0_CONSEQ_0;
    ADC14->CTL1 =  ADC14_CTL1_RES_3;

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1 | ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_WINC;
    ADC14->HI0 = 0x3333;
    ADC14->LO0 = 0x1999;
    ADC14->IER1 |= ADC14_IER1_HIIE | ADC14_IER1_LOIE | ADC14_IER1_INIE;
    //Configure Timer32
    TIMER32_1->CONTROL = 0b11000010;

    // Wake up on exit from ISR
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();
    /* initialize the GLCD controller */
    GLCD_init();

    while (1)
    {
        for (i = 20000; i > 0; i--); // Delay
        // Start sampling/conversion
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
        __sleep();

        display(period);

        __no_operation(); // For debugger
    }
}

/* ADC Interrupt Handler. This handler is called whenever there is a conversion
 * that is finished for ADC_MEM0.
 */
void ADC14_IRQHandler(void) {
    j++;
    if (ADC14->IFGR1 & ADC14_IFGR1_HIIFG)
    {
        ADC14->CLRIFGR1 |= ADC14_CLRIFGR1_CLRHIIFG;
        if(prev_val==LOW){
                            cur_time = TIMER32_1->VALUE;
                            period = prev_time-cur_time;
                            prev_val = HIGH;
                            prev_time = cur_time;
                        }

    }
    if (ADC14->IFGR1 & ADC14_IFGR1_LOIFG)
    {
        // Clear interrupt flag
        ADC14->CLRIFGR1 |= ADC14_CLRIFGR1_CLRLOIFG;

        prev_val = LOW;

    }
}
