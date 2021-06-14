#include <p18cxxx.h>
#include "nrf24l01.h"

// Configuration bits are described in section 24.1
// Note: For a complete list of the available config pragmas and their values, in MPLAB
// click Help|Topics then select "PIC18 Config Settings" in the Language Tools section.
#pragma config FOSC=INTIO67		// Internal oscillator block, port function on RA6 and RA7 
#pragma config PLLCFG=OFF		// Oscillator PLL set in software for HFINTOSC  
#pragma config PRICLKEN=OFF		// Primary clock disabled, using INTOSC  
#pragma config FCMEN=OFF		// Fail-Safe Clock Monitor disabled  
#pragma config IESO=OFF			// Oscillator Switchover mode disabled  
#pragma config PWRTEN=OFF		// Power up timer disabled  
#pragma config BOREN=OFF		// Brown-out Reset enabled in hardware only (SBOREN is disabled) 
#pragma config BORV=285			// VBOR set to 2.85 V nominal  
#pragma config WDTEN=OFF		// Watch dog timer is always disabled. SWDTEN has no effect
#pragma config WDTPS=1			// Watchdog timer prescalar 1:1  
#pragma config CCP2MX=PORTC1	// CCP2 input/output is multiplexed with RC1  
#pragma config PBADEN=OFF		// PORTB<5:0> pins are configured as digital I/O on Reset  
#pragma config CCP3MX=PORTC6	// P3A/CCP3 input/output is mulitplexed with RC6  
#pragma config HFOFST=OFF		// HFINTOSC output and ready status are delayed by the oscillator stable status  
#pragma config T3CMX=PORTC0		// T3CKI is on RC0  
#pragma config P2BMX=PORTC0		// P2B is on RC0  
#pragma config MCLRE=EXTMCLR	// MCLR pin enabled, RE3 input pin disabled  
#pragma config STVREN=OFF		// Stack full/underflow will not cause Reset
#pragma config LVP=OFF			// Single-Supply ICSP disabled  
#pragma config XINST=OFF		// Instruction set extension and Indexed Addressing mode disabled (Legacy mode)  
#pragma config CP0=OFF			// Block 0 (000800-003fffh) not code-protected  
#pragma config CP1=OFF			// Block 1 (004000-007fffh) not code-protected  
#pragma config CP2=OFF			// Block 2 (008000-00bfffh) not code-protected  
#pragma config CP3=OFF			// Block 3 (00c000-00ffffh) not code-protected  
#pragma config CPB=OFF			// Boot block (000000-0007ffh) not code-protected  
#pragma config CPD=OFF			// Data EEPROM not code-protected  
#pragma config WRT0=OFF			// Block 0 (000800-003fffh) not write-protected  
#pragma config WRT1=OFF			// Block 1 (004000-007fffh) not write-protected  
#pragma config WRT2=OFF			// Block 2 (008000-00bfffh) not write-protected  
#pragma config WRT3=OFF			// Block 3 (00c000-00ffffh) not write-protected  
#pragma config WRTC=OFF			// Configuration registers (300000-3000ffh) not write-protected  
#pragma config WRTB=OFF			// Boot Block (000000-0007ffh) not write-protected  
#pragma config WRTD=OFF			// Data EEPROM not write-protected  
#pragma config EBTR0=OFF		// Block 0 (000800-003fffh) not protected from table reads executed in other blocks  
#pragma config EBTR1=OFF		// Block 1 (004000-007fffh) not protected from table reads executed in other blocks  
#pragma config EBTR2=OFF		// Block 2 (008000-00bfffh) not protected from table reads executed in other blocks  
#pragma config EBTR3=OFF		// Block 3 (00c000-00ffffh) not protected from table reads executed in other blocks  
#pragma config EBTRB=OFF		// Boot Block (000000-0007ffh) not protected from table reads executed in other blocks  

void low_isr(void);
unsigned char send_button_press(unsigned char data);

volatile unsigned char buttons;
volatile unsigned int c; 

//initialize routine
void initialize(void) 
{ 
	c=0;
	buttons=0;
	
	// set all pins as digital i/o
	ANSELA=0;
	ANSELB=0;
	ANSELC=0;
	
	// set all unused pins as output and high
	TRISA=0;
	LATA=0xff;
	TRISCbits.TRISC0=0;
	LATCbits.LATC0=1;
	TRISCbits.TRISC6=0;
	LATCbits.LATC6=1;
	TRISCbits.TRISC7=0;
	LATCbits.LATC7=1;	
	TRISBbits.TRISB1=0;
	LATBbits.LATB1=1;	
	TRISBbits.TRISB2=0;
	LATBbits.LATB2=1;
	TRISBbits.TRISB3=0;
	LATBbits.LATB3=1;
	TRISBbits.TRISB6=0;
	LATBbits.LATB6=1;
	TRISBbits.TRISB7=0;
	LATBbits.LATB7=1;

	// set state of miso pin used during sleep
	LATCbits.LATC4=1;
	
	// IRQ=B0
	TRISBbits.TRISB0=1; 		// set irq pin as input
	
	// CSN=C2
	TRISCbits.TRISC2=0;			// set csn pin as output
	
	// CE=C1
	TRISCbits.TRISC1=0;			// set ce pin as output

	// buttons
	TRISBbits.TRISB4=1;			// set button pins as inputs
	TRISBbits.TRISB5=1;
	WPUBbits.WPUB4=1;			// pull ups enabled on button pins
	WPUBbits.WPUB5=1;
	INTCON2bits.RBPU=0;			// enable all port b pull ups
	IOCBbits.IOCB4=1;			// interrupt on change enabled on button pins
	IOCBbits.IOCB5=1;
	INTCONbits.RBIF=0;			// clear port b change interrupt

	// button debounce and short/long press timer
	T2CONbits.TMR2ON=0;			// set timer off, section 13.1
	T2CONbits.T2CKPS=3;         // set prescalar to 16, section 13.1
	T2CONbits.T2OUTPS=0x0f;     // set postscalar to 16, section 13.1
	PR2=0xff;                   // set timer period, section 13.1
	TMR2=0;                     // set timer initial value, section 13.1
	PIR1bits.TMR2IF=0;          // clear interrupt flag, section 9.5
	PIE1bits.TMR2IE=0;          // enable timer interrupt, section 9.6
	T2CONbits.TMR2ON=1;         // set timer on, section 13.1
			
	LATCbits.LATC2=1; 			// set CSN bit
	
	// init the spi port
	SSP1CON1bits.SSPEN=0;		// disable spi1
	
	// SDO1=C5
	TRISCbits.TRISC5=0;			// set mosi pin as output
	
	// SDI1=C4
	TRISCbits.TRISC4=1;      	// set miso pin as input
	
	// SCK1=C3
	TRISCbits.TRISC3=0;      	// set clk pin as output
	
	// setup spi parameters
	SSP1CON1bits.SSPM=0;
	SSP1CON1bits.CKP=0;	
	SSP1STATbits.SMP=0;	
	SSP1STATbits.CKE=1;	
	
	SSP1CON1bits.SSPEN=1; 		// enable spi1
	
	// init the rf chip as transmitter, no autoack, no retry
	nrf24l01_initialize(nrf24l01_CONFIG_DEFAULT_VAL,
						true,
						nrf24l01_EN_AA_ENAA_NONE,
						nrf24l01_EN_RXADDR_DEFAULT_VAL,
						nrf24l01_SETUP_AW_DEFAULT_VAL,
						nrf24l01_SETUP_RETR_ARC_0,
						nrf24l01_RF_CH_DEFAULT_VAL,
						nrf24l01_RF_SETUP_DEFAULT_VAL,
						NULL,
						NULL,
						nrf24l01_RX_ADDR_P2_DEFAULT_VAL,
						nrf24l01_RX_ADDR_P3_DEFAULT_VAL,
						nrf24l01_RX_ADDR_P4_DEFAULT_VAL,
						nrf24l01_RX_ADDR_P5_DEFAULT_VAL,
						NULL,
						1U,
						nrf24l01_RX_PW_P1_DEFAULT_VAL,
						nrf24l01_RX_PW_P2_DEFAULT_VAL,
						nrf24l01_RX_PW_P3_DEFAULT_VAL,
						nrf24l01_RX_PW_P4_DEFAULT_VAL,
						nrf24l01_RX_PW_P5_DEFAULT_VAL);
	
	// turn on interrupts
	INTCONbits.RBIE=1;			// enable port b change interrupts
	INTCONbits.PEIE=1;          // enable all unmasked peripheral interrupts
	INTCONbits.GIE=1;           // globally enable interrupts
} 

void main (void) 
{ 
	unsigned char data; 
	unsigned char i=0;
	unsigned char j;
	unsigned int count;
	
	OSCCONbits.IRCF=5;			// internal oscillator 4MHz, section 2.2.2
	OSCCONbits.IDLEN=0;			// SLEEP enters sleep mode, section 2.2.4
	OSCCONbits.SCS=0;			// system clock determined by config bits, section 2.3
	OSCTUNEbits.PLLEN=1;
	
	//initialize IO, SPI, set up nRF24L01 as transmitter
	initialize();  

	while(1)
	{	
		if(buttons>0)
		{
			PIE1bits.TMR2IE=0;				// kill timer
			INTCONbits.RBIE=1;				// enable port b interrupts again			
			send_button_press(buttons);
			buttons=0;
		}	

		if(!PIE1bits.TMR2IE)
		{
			// not powering down increases standby current from 10 uA to 25uA
			// if this is removed leave in power up
			nrf24l01_power_down();		// put rf module into power down mode
			TRISCbits.TRISC4=0;			// set miso pin as output
			Sleep();
			TRISCbits.TRISC4=1; 		// set miso as input again
			nrf24l01_power_up(false);	// power up rf module, receiver off
		}
	}		
}	

unsigned char send_button_press(unsigned char data)
{
	unsigned int tries;
	unsigned char success=0;
	
	nrf24l01_write_tx_payload(&data, 1, true); // transmit received char over rf 
	
	for(tries=0; tries<5000; tries++)
	{
		// check to see if the data has been sent
		if(nrf24l01_irq_pin_active() && nrf24l01_irq_tx_ds_active())
		{
			// data sent, no more tries
			success=1;
			break;
		}
	}	
	
	nrf24l01_flush_tx();					// make sure everything is sent by rf module
	nrf24l01_irq_clear_all(); 				// clear rf module interrupts 

	return success;
}

#pragma code low_vector=0x08
void low_interrupt(void)
{
	_asm GOTO low_isr _endasm
}
	
#pragma code

#pragma interruptlow low_isr 
void low_isr(void)
{
	static unsigned char temp_buttons;

	if(INTCONbits.RBIE && INTCONbits.RBIF)
	{
		// if falling edge and not currently debouncing...
		if((!PORTBbits.RB5 || !PORTBbits.RB4) && !PIE1bits.TMR2IE)		
		{
			TMR2=0;
			PIE1bits.TMR2IE=1;				// enable timer interrupt
			INTCONbits.RBIE=0;				// disable further port b interrupts in the meantime	
			c=0;							// clear timer count
		}
					
		INTCONbits.RBIF=0;		
	}	

	// check tick timer
	if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
	{
		PIR1bits.TMR2IF=0;
		c++;								// increment count of number of times timer interrupt has fired
		if(c==6)					
		{
			if(!PORTBbits.RB5)				// is input pin still low?
			{
				temp_buttons=7;
			}	
			else if(!PORTBbits.RB4)			// is input pin still low?
			{
				temp_buttons=5;
			}
			else
			{
				// got a bounce or very short press which is ignored
				PIE1bits.TMR2IE=0;			// kill timer interrupts and go back to sleep in main loop
				INTCONbits.RBIE=1;			// enable port b interrupts again
			}			

		}		
		else if(c>6 && c<30)
		{
			if(PORTBbits.RB5 && PORTBbits.RB4)
			{
				// got a short press
				c=100;						// go into inter button press period
			}	
		}	
		else if(c>30 && c<100)
		{
			// got a long press
			temp_buttons+=1;
			c=100;							// go into inter button press period	
		}	
		else if (c>110)
		{
			buttons=temp_buttons;			// signal main loop that a button press has happened
		}
	}
}
