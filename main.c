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
#include <stdint.h>
void Software_Trim();                                            // Software Trim to get the best DCOFTRIM value
#define GPIO_ALL        BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7
#define MCLK_FREQ_MHZ 1                                          // MCLK = 1MHz

// Declare global variables
volatile uint8_t uartByteNum;           // 0 for high, 1 for low
#pragma PERSISTENT(highThreshold)       // High threshold for ADC
 uint16_t highThreshold = 0x01AA;

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                                     // Stop watchdog timer

    __bis_SR_register(SCG0);                                      // disable FLL

    // Initialize Clock System
    // SMCLK = MCLK = DCO + FLL + 32KHz REFO REF = 1MHz
    // refer to msp430fr243x_CS_04 code example on https://www.ti.com/lit/zip/slac700
    CSCTL3 |= SELREF__REFOCLK;                                    // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_0;      // DCOFTRIM=3, DCO Range = 1MHz
    CSCTL2 = FLLD_0 + 30;                                         // DCODIV = 1MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                                      // enable FLL
    Software_Trim();                                              // Software Trim to get the best DCOFTRIM value

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;                    // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                                  // default DCODIV as MCLK and SMCLK source
    // Initialize globals
    uartByteNum = 0;

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
    // UCBRx = INT(N/16) = 6; UCBRFx = INT([(N/16) – INT(N/16)] × 16) = 8
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
    ADCHI = highThreshold;                                        // Set the high threshold
    ADCIE |= ADCHIIE;                                             // Above upper threshold interrupt enabled

    ADCCTL0 |= ADCSC | ADCENC;                                    // Enable and start conversion

    __bis_SR_register(LPM0_bits | GIE);                           // Go to LPM0 with interrupts
    __no_operation();
}

/**
* ADC interrupt service routine
*/

#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    ADCIFG &= ~ADCHIIFG;                                          // Clear interrupt flag
    P1OUT |= BIT0;                                                // Set P1.0
    ADCIE &= ~ADCHIIE;                                            // Disable interrupts
    TA0CTL |= MC__UP;                                             // Start TIMER_A0
}

/**
* Timer_A interrupt service routine
*/

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER_A_ISR(void)
{
    TA0CTL &= ~CCIFG;                                             // CLear interrupt flag
    // Send ADC conversion result over UART
    UCA0TXBUF = ADCMEM0_H;                                        // Send high byte
    while(!(UCA0IFG&UCTXIFG));
    UCA0TXBUF = ADCMEM0_L;                                        // Send low byte
}

/**
* eUSCI interrupt service routine
*/

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_UART_UCRXIFG:
            SYSCFG0 = FRWPPW;                                     // FRAM write enable
            if(!uartByteNum)                                      // High UART byte
            {
                ADCCTL0 &= ~ADCSC & ~ADCENC;                      // Stop ADC until low byte received
                highThreshold = UCA0RXBUF << 8;                   // Store received byte
                uartByteNum = 1;                                  // Prepare to receive low byte
            }
            else                                                  // Low UART byte
            {
                highThreshold += UCA0RXBUF;                       // Store received byte
                ADCCTL0 |= ADCSC | ADCENC;                        // Resume ADC operation
                uartByteNum = 0;                                  // Prepare to receive high byte
            }
            SYSCFG0 = FRWPPW | PFWP;                              // FRAM write disable
            break;
        default: break;
    }
}

// refer to msp430fr243x_CS_04 code example on https://www.ti.com/lit/zip/slac700
// refer to 3.2.11.2 DCO Software Trim of SLAU445: https://www.ti.com/lit/ug/slau445i/slau445i.pdf
void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                                           // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                                    // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);                                 // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);       // Wait FLL lock status (FLLUNLOCK) to be stable
                                                                  // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                                      // Read CSCTL0
        csCtl1Read = CSCTL1;                                      // Read CSCTL1

        oldDcoTap = newDcoTap;                                    // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;                          // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;                   // Get DCOFTRIM value

        if(newDcoTap < 256)                                       // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;                        // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256))       // DCOTAP cross 256
                endLoop = 1;                                      // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~(DCOFTRIM0+DCOFTRIM1+DCOFTRIM2))) | (dcoFreqTrim<<4);
            }
        }
        else                                                      // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;                        // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                                   // DCOTAP cross 256
                endLoop = 1;                                      // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~(DCOFTRIM0+DCOFTRIM1+DCOFTRIM2))) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)                            // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                                         // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                                          // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                                          // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));                    // Poll until FLL is locked
}
