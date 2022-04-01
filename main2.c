//******************************************************************************
//  MSP430i20xx Demo - SD24, Single Conversion on a Group of 3 Channels
//
//  Description: This program uses the SD24 module to perform a single
//  conversion on a group of channels (0, 1 and 2). A SD24 interrupt occurs
//  when the conversions have completed.
//
//  Test by applying voltages to the 3 input channels and setting a breakpoint
//  at the indicated line. Run program until it reaches the breakpoint, then use
//  the debugger's watch window to view the conversion results.
//
//  Results (upper 16 bits only) are stored in the array "results"
//
//  ACLK = 32kHz, MCLK = SMCLK = Calibrated DCO = 16.384MHz, SD_CLK = 1.024MHz
//  * Ensure low_level_init.c is included when building/running this example *
//
//  Notes: For minimum Vcc required for SD24 module - see datasheet
//         100nF cap btw Vref and AVss is recommended when using 1.2V ref
//
//               MSP430i20xx
//             -----------------
//         /|\|                |
//          | |                |
//          --|RST             |
//            |                |
//   Vin1+ -->|A0.0+      VREF |---+
//   Vin1- -->|A0.0-           |   |
//   Vin2+ -->|A1.0+           |  -+- 100nF
//   Vin2- -->|A1.0-           |  -+-
//   Vin3+ -->|A2.0+           |   |
//   Vin3- -->|A2.0-      AVss |---+
//
//
//******************************************************************************
#include "msp430.h"

/* Array to store SD24 conversion results */
int results[3];
char mode=0,modes=0;
float resf[3],resf_0[3]={0,0,0},dif_f[3]={0,0,0};
unsigned int absr[3];
int resi[3];
int ss=0;
int trig=1,trig0=-1;
int i=0,imax=1900,j=0,jmax=0,LED0=0;
void send(void);


void main(void) {
   unsigned long *jtagPwd = (unsigned long *)JTAG_DIS_PWD1;

    /* Feed the watchdog timer */
    WDTCTL = WDTPW | WDTCNTCL;
 

    /* Check JTAG password locations and disable JTAG if passwords don't match.
     * Else the JTAG will be enabled in the 64th cycle after reset.
     */
    if ((*jtagPwd != 0x00000000) && (*jtagPwd != 0xFFFFFFFF))
    {
        /* Disable JTAG */
        SYSJTAGDIS = JTAGDISKEY;
    }

    /* Calibration section
     * Check for the BORIFG flag in IFG1. Execute calibration if this was a BORIFG.
     * Else skip calibration
     */
    if (IFG1 & BORIFG)
    {
        /* Perform 2's complement checksum on 62 bytes of TLV data */
        unsigned int checksum = 0;
        unsigned char *TLV_address_for_parse = ((unsigned char *)TLV_START);
        unsigned int *TLV_address_for_checksum = ((unsigned int *)TLV_START + 1);

        do
        {
            checksum ^= *TLV_address_for_checksum++;
        } while (TLV_address_for_checksum <= (unsigned int *)TLV_END);

        checksum ^= 0xFFFF;
        checksum++;

        /* If check sum is not correct go to LPM4 */
        if (*((unsigned int *)TLV_START) != checksum)
        {
            /* Enter LPM4 if checksum failed */
            __bis_SR_register(LPM4_bits);
        }

        /* Check sum matched, now set calibration values */

        /* Calibrate REF */
        REFCAL1 = *(TLV_address_for_parse + TLV_CAL_REFCAL1);
        REFCAL0 = *(TLV_address_for_parse + TLV_CAL_REFCAL0);

        /* Calibrate DCO */
        CSIRFCAL = *(TLV_address_for_parse + TLV_CAL_CSIRFCAL);
        CSIRTCAL = *(TLV_address_for_parse + TLV_CAL_CSIRTCAL);
        CSERFCAL = *(TLV_address_for_parse + TLV_CAL_CSERFCAL);
        CSERTCAL = *(TLV_address_for_parse + TLV_CAL_CSERTCAL);

        /* Calibrate SD24 */
        SD24TRIM = *(TLV_address_for_parse + TLV_CAL_SD24TRIM);

        /* Clear BORIFG */
        IFG1 &= ~(BORIFG);
    }
    WDTCTL = WDTPW | WDTHOLD;                   // Stop WDT
    SD24CTL = SD24REFS;                         // Internal ref
    for(i=0;i<2500;i++) ;
    P1SEL0 |=   BIT2 | BIT3;                // P1.2/3 eUSCI_A Function
    P1SEL1 &= ~(BIT2 | BIT3);
    
    P1DIR |= BIT0;                      // Set P1.4 as output
    TA0CCTL0 = CCIE;                    // CCR0 Interrupt Enabled
    TA0CCR0 = 8160;
    TA0CTL = TASSEL_2 | MC_1 | ID_3;    // SMCLK/8, Cont. Mode
    

    UCA0CTL1 |= UCSWRST;                // Hold eUSCI in reset
    UCA0CTL1 |= UCSSEL_2;               // SMCLK
    UCA0BR0   = 142;                    // 115200 baud
    UCA0BR1   = 0;
    UCA0MCTLW = 0x2200;                 // 16.384MHz/115200 = 142.22 (See UG)
    
   // UCA0BR0   = 0xAA;                       // 9600 baud
   // UCA0BR1   = 0x06;
   // UCA0MCTLW = 0xD600;                     // 16.384MHz/9600 = 1706.6667 (See UG)
    UCA0CTL1 &= ~UCSWRST;               // Release from reset
    UCA0IE   |= UCRXIE;                     // Enable RX interrupt

   // SD24CTL = SD24REFS;                         // Internal ref
    SD24CCTL0  |= SD24SNGL | SD24GRP | SD24DF | SD24OSR_256 ; //| SD24OSR_64 ;//| SD24LSBACC;  // Group with CH1
    SD24CCTL1  |= SD24SNGL | SD24GRP | SD24DF| SD24OSR_256;// | SD24OSR_64 ;//| SD24LSBACC;  // Group with CH2
    SD24CCTL2  |= SD24SNGL | SD24IE | SD24DF | SD24OSR_256;//| SD24OSR_64 ;//| SD24LSBACC;   // Enable interrupt
  //  SD24INCTL0=SD24GAIN_16;                              
   // SD24INCTL1=SD24GAIN_16;
   // SD24INCTL2=SD24GAIN_16;

    __delay_cycles(13000);                       // Delay ~200us for 1.2V ref to settle

    resf[0]=0.0;
    resf[1]=0.0;
    resf[2]=0.0;
    while(1) {
       __bis_SR_register(GIE);                   // Set bit to start conversion              
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SD24_VECTOR
__interrupt void SD24_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SD24_VECTOR))) SD24_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(SD24IV,SD24IV_SD24MEM3)) {
        case SD24IV_NONE: break;
        case SD24IV_SD24OVIFG: break;
        case SD24IV_SD24MEM0: break;
        case SD24IV_SD24MEM1: break;
        case SD24IV_SD24MEM2:
                   results[0] = SD24MEM0;       // Save CH0 results (clears IFG)
                   results[1] = SD24MEM1;       // Save CH1 results (clears IFG)
                   results[2] = SD24MEM2;       // Save CH2 results (clears IFG)
                   __bic_SR_register_on_exit(LPM0_bits); // Wake up
                   break;
        case SD24IV_SD24MEM3: break;
        default: break;
    }
}

// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    __bic_SR_register(LPM3_bits|GIE); 
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
          if(UCA0RXBUF == 10) modes = 0;
          if(UCA0RXBUF == 11) modes = 1;
          if(UCA0RXBUF == 12) modes = 2;
          if(UCA0RXBUF == 13) jmax++;
          if(UCA0RXBUF == 14) jmax--;
          if(UCA0RXBUF == 15) modes = 5;
          if(UCA0RXBUF == 16) modes = 6;
          if(UCA0RXBUF > 16) {
            modes = 7;
            i=0;
            imax = UCA0RXBUF * 64;
          }               
       break;
          
        case USCI_UART_UCTXIFG:  break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

void send(void){
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = 0xA;
  
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = resi[0] & 0xff;
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = (resi[0] >> 8) & 0xff;
  /*
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = resi[1] & 0xff;
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = (resi[1] >> 8) & 0xff;
  
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = resi[2] & 0xff;
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = (resi[2] >> 8) & 0xff;
*/
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TA0_ISR (void)
#else
#error Compiler not supported!
#endif
{
   LED0++;
   if(LED0 > 1000) {
     P1OUT ^= BIT0;                      // Toggle LED
     LED0=0;
   }
         resf[0]=(resf[0]*0.98+results[0]*0.02);
    //     resf[1]=(resf[1]*0.98+results[1]*0.02);
      //   resf[2]=(resf[2]*0.98+results[2]*0.02);
       
         
         resi[0]=(int)resf[0];
   //      resi[1]=(int)resf[1];
     //    resi[2]=(int)resf[2];
         send();        
          SD24CCTL2 |= SD24SC;    
   // TA0CCR0 += 5000;                   // Add offset to CCR0
}

