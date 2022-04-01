# UART-Live-graph

An msp430i2040 code provided to send data 
 or just send an 0xA before transmiting any number like
//********************************************************************
while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = 0xA;
  
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = tada & 0xff;  //here the "tada" is a uint16_t data
  while(!(UCA0IFG & UCTXIFG)) ;
  UCA0TXBUF = (tada >> 8) & 0xff;
  //********************************************************-----------ikaya
