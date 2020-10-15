/* --COPYRIGHT--,BSD
 * Copyright (c) 2020, Texas Instruments Incorporated
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
//******************************************************************************
//  MSP430FR2433 Demo - ADC Report on Threshold
//
//  Description: This example sets up the ADC to monitor an analog input
//  continuously while in LPM0, and to set a GPIO and start transmitting ADC
//  conversion data over UART when a high threshold is reached by the ADC. UART
//  communication also allows the user to change the threshold
//  value.
//  ACLK = default REFO ~32768Hz
//  SMCLK = MCLK = DCO + FLL + 32KHz REFO REF = 1MHz
//
//           MSP430FR2433
//         ---------------
//     /|\|               |
//      | |               |
//      --|RST            |
//        |          P1.4 |<--- UART TX
//        |               |
//        |          P1.5 |---> UART RX
//        |               |
//        |          P1.3 |<--- A3
//        |               |
//        |          P1.0 |---> GPIO
//
//   Xiaodong Li
//   Texas Instruments Inc.
//   September 2020
//   Built with Code Composer Studio v9.2 and IAR 7.20
//******************************************************************************
#include <msp430.h> 
                                           // Software Trim to get the best DCOFTRIM value
#define GPIO_ALL        BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7
#define MCLK_FREQ_MHZ 1                                          // MCLK = 1MHz

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                     // Stop watchdog timer

    // Initialize Clock System
    // SMCLK = MCLK = DCO + FLL + 32KHz REFO REF = 1MHz
    // refer to msp430fr243x_CS_04 code example on https://www.ti.com/lit/zip/slac700
    CSCTL3 |= SELREF__REFOCLK;                                    // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;      // DCOFTRIM=3, DCO Range = 1MHz
    CSCTL2 = FLLD_0 + 30;                                         // DCODIV = 1MHz

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;                    // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                                  // default DCODIV as MCLK and SMCLK source


    // Initialize ports
    P1OUT = 0;                                                    // Set all pins low
    P1DIR = GPIO_ALL;                                             // Set all pins as outputs

    P2OUT = 0;                                                    // Set all pins low
    P2DIR = GPIO_ALL;                                             // Set all pins as outputs

    P3OUT = 0;                                                    // Set all pins low
    P3DIR = GPIO_ALL;                                             // Set all pins as outputs

    // Initialize GPIOs
    P1SEL0 |= BIT3;                                               // Set A3 for ADC
    P1SEL1 |= BIT3;

    P1SEL0 |= BIT4 | BIT5;                                        // Configure P1.4 and P1.5 for UART
    // Disable the GPIO power-on default high-impedance
    // mode to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure UART 9600 baud
    UCA0CTLW0 |= UCSWRST;                                         // eUSCI_A logic held in reset state.
    UCA0CTLW0 |= UCSSEL__SMCLK;                                   // One stop bit, no parity, LSB first, 8-bit data as the default setting
    // Baud Rate calculation
    // Refer to 22.3.10 Setting a Baud Rate of SLAU445: https://www.ti.com/lit/ug/slau445i/slau445i.pdf
    // N = 1000000/9600 = 104.17; N>16; oversampling mode is selected; UCOS16 = 1
    // UCBRx = INT(N/16) = 6; UCBRFx = INT([(N/16) � INT(N/16)] � 16) = 8
    // UCBRSx = 0x20 at Fractional Portion is 0.1667 according to the table 22-4 on SLAU445
    UCA0BR0 = 6;                                                  // 1000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x2000 | UCOS16 | UCBRF_8;
    UCA0CTLW0 &= ~UCSWRST;                                        // eUSCI_A reset released for operation
    UCA0IE = UCRXIE;                                              // Enable USCI_A0 RX interrupt

    // Initialize Timer_A
    TA0CTL |= TASSEL__ACLK;                                       // Use ACLK as source
    TA0CCR0 = 32768;                                              // 1 sec for ACLK = 32768 Hz
    TA0CCTL0 |= CCIE;                                             // Enable capture and compare interrupts*/

    // Initialize ADC
    ADCCTL0 |= ADCMSC | ADCON;                                    // Conversions performed automatically
                                                                  // ADC turned on
    ADCCTL1 |= ADCSHP | ADCSSEL_1 | ADCCONSEQ_2;                  // Use input signal
                                                                  // ADC clock source is ACLK
                                                                  // ADC conversion mode = repeat-single-channel

    ADCMCTL0 |= ADCINCH_3;                                        // Input channel set to A3
 //   ADCHI = highThreshold;                                        // Set the high threshold
    ADCIE |= ADCHIIE;                                             // Above upper threshold interrupt enabled

    ADCCTL0 |= ADCSC | ADCENC;                                    // Enable and start conversion

}
