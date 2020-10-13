MSP430 Housekeeping Example - ADC Wake and Transmit 

## Example Summary
This example sets up the ADC to monitor an analog input continuously while in LPM0. 
Then it sets a GPIO (LED) and start transmitting ADC conversion data periodically
over UART when a high threshold is reached by the ADC. 
UART communication also allows the user to change the threshold value.

## Peripherals & Pin Assignments
* P1.0 --> GPIO (LED)
* P1.3 <-- A3 ADC Input 
* P1.4 --> eUSCI_A0 UART TX 
* P1.5 <-- eUSCI_A0 UART RX
* All other pins are initialized as output low to reduce power consumption.

## Clock configuration
* ACLK = default, REFO ~32768Hz， 
* SMCLK = DCO + FLL + 32KHz REFO reference = 1MHz
* MCLK = DCO + FLL + 32KHz REFO reference = 1MHz

## BoosterPacks, Board Resources & Jumper Settings
This example was developed and tested on MSP-EXP430FR2433.
* Pin J1.9: P1.3/A3, analog input driven externally
* Jumper J10: ON, enables LED1 driven by P1.0
* Jumper J101-GND: ON, Connects the debugger circuitry's GND to Launchpad
* Jumper J101-3V3: ON, Powers Launchpad through USB 
* Jumper J101-RXD: ON, enables communication to back-channel UART
* Jumper J101-TXD: ON, enables communication to back-channel UART

## Example Usage
- Connect an analog signal to J1-9 (P1.3/A3).
- Open a serial terminal (PuTTY, Teraterm, etc) with the following settings:
    Baud-rate:       9600
    Data bits:          8
    Stop bits:          1
    Parity:          None
    Flow Control:    None
- If the ADC input is lower than the threshold, the device will be idle just
  continue monitoring input. 
  If the ADC input is higher than the threshold, the device will start sending 
  data (in binary) to the terminal.
  The default treshold is set to 0x1AA (426), which is 426/1024 = ~41.6% of ADC
  reference. 
- The threshold can be updated via UART. The first byte received will be the 
  most significant byte, while the second byte will be the least significant byte.
  The data is received in binary (i.e. typing '0' will send 0x30).
- The threshold will be stored in non-volatile FRAM and will be retained after
  a reset.


