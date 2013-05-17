/*************************************************************************
**  STK500 Protocol v2 Bootloader/monitor for ATmega128,AT90CAN128
**  require AVR Studio 4.11 build 401 or later
**  (c) Milosz Klosowicz, MikloBit
**
**  Released under GNU GENERAL PUBLIC LICENSE
**  See gpl.txt
**
**  contact: 	support@miklobit.com
**  homepage:	www.miklobit.com
**
** 
**  
**  based partly on oryginal code: 
**  (c) Michael Wolf, webmaster@mictronics.de 
**  (c) Erik Lins, chip45.com      
**	(c) Jason P. Kyle, avr1.org
**
**************************************************************************/
#include <ctype.h>
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <util/delay.h>

#include "command_v2.h"
//#include "xtea.h"

#define XTEA_ENCODE_ON
#define BOOTUART_SET_ATmega128_UART0_ONLY	
/* monitor functions will only be compiled when using ATmega64/128/CAN128, 
 * due to bootblock size constraints 
*/
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
//#define MONITOR
//#include "rtc72423.c"
#endif

#define BOOTLOADER_BASE (0x1F000UL) /*(0x1E000UL)*/

#define XTEA_KEY_ADDRESS (0x1FFC0UL)

static void XTEA_decode_special(unsigned long* data, unsigned char dataLength);
static void XTEA_encode_special_(unsigned long* data, unsigned char dataLength);

#define app_start()	 asm volatile("jmp 0x0000");	      

#define	STATE_READY		      0x00
#define STATE_START           0x01
#define STATE_GET_SEQ_NUM     0x02
#define STATE_GET_MSG_SIZE1   0x03
#define STATE_GET_MSG_SIZE2   0x04
#define STATE_GET_TOKEN       0x05
#define STATE_GET_DATA        0x06
#define STATE_GET_CS          0x07

/* Set values as required supress popup messages in AVR-Studio */
#define HARDWARE_VERSION      0x02
#define SOFTWARE_MAJOR        0x02
//#define SOFTWARE_MINOR        0x01
#define SOFTWARE_MINOR        0x0A

/* values for possible topcard */
#define STK501		0xAA
#define STK502		0x55
#define STK503		0xFA
#define STK504		0xEE
#define STK505		0xE4
#define STK520		0xDD		

/* value for vtarget: always return 5.0V */
#define VTARGET		0x32
/* value for vadjust: always return 5.0V */
#define VADJUST		0x32
/* prescalled clock frequesncy equal to system clock */
#define PSCALE_FSYS 0x01
#define CMATCH_DEF	0x01
#define SCK_DURATION_DEF 0x01

#define BAUD_RATE		115200

#define MAX_BUF_SIZE	300

/* Adjust to suit whatever pin your hardware uses to enter the bootloader */

/* ATmega128 has two UARTS so two pins are used to enter bootloader and select UART */
/* BL0... means UART0, BL1... means UART1 */
#ifdef __AVR_ATmega128__
#define BL_DDR  DDRF
#define BL_PORT PORTF
#define BL_PIN  PINF
#define BL0     PINF7
#define BL1     PINF6
#elif defined __AVR_AT90CAN128__
#define BL_DDR  DDRF
#define BL_PORT PORTF
#define BL_PIN  PINF
#define BL0     PINF7
#define BL1     PINF6
#else
/* other ATmegas have only one UART, so only one pin is defined to enter bootloader */
#define BL_DDR  DDRD
#define BL_PORT PORTD
#define BL_PIN  PIND
#define BL      PIND6
#endif



#define SIG1	0x1E	// Always Atmel
#if defined __AVR_ATmega128__
#define SIG2	0x97
#define SIG3	0x02
#define UART0
#elif defined __AVR_AT90CAN128__
#define SIG2	0x97
#define SIG3	0x81
#define UART0
#elif defined __AVR_ATmega64__
#define SIG2	0x96
#define SIG3	0x02
#define UART0
#elif defined __AVR_ATmega32__
#define SIG2	0x95
#define SIG3	0x02
#elif defined __AVR_ATmega16__
#define SIG2	0x94
#define SIG3	0x03
#elif defined __AVR_ATmega8__
#define SIG2	0x93
#define SIG3	0x07
#elif defined __AVR_ATmega162__
#define SIG2	0x94
#define SIG3	0x04
#define UART0
#elif defined __AVR_ATmega163__
#define SIG2	0x94
#define SIG3	0x02
#elif defined __AVR_ATmega169__
#define SIG2	0x94
#define SIG3	0x05
#elif defined __AVR_ATmega8515__
#define SIG2	0x93
#define SIG3	0x06
#elif defined __AVR_ATmega8535__
#define SIG2	0x93
#define SIG3	0x08
#endif



static void putch(char);
static char getch(void);
static void initPorts(void); // init port pins for bootloader start
static void initUart(void);  // check uart selected and init
static inline void bootCheck(void); // check bootloader/application start
void handleMessage(void) ; 
static void sendResponse(void);
static inline void cmdSignOn(void);
static inline void cmdReadSignatureIsp(void);
static inline void cmdSetParameter(void);
static inline void cmdEnterProgmodeIsp(void);
static inline void cmdLeaveProgmodeIsp(void);
static inline void cmdChipEraseIsp(void);
static inline void cmdLoadAddress(void);
static inline void cmdGetParameter(void);
static inline void cmdProgramFlashIsp(void);
static inline void cmdReadFlashIsp(void);
static inline void cmdProgramEepromIsp(void);
static inline void cmdReadEepromIsp(void);
static inline void cmdReadFuseLockIsp(void);
static inline void cmdError(void);
static void eeprom_wb(unsigned int uiAddress, unsigned char ucData);
static unsigned char eeprom_rb(unsigned int uiAddress);
static unsigned char readBits( unsigned int address ); // read lock/fuse bits

#ifdef MONITOR	
#define MONITOR_FLAG '!' 
#define MONITOR_PROMPT ':'
#define MONITOR_BUFFER_SIZE 256 
#define MONITOR_DISPLAY_LINES 8

#define MONITOR_CMD_TIME  'T'
#define MONITOR_CMD_DATE  'D'
#define MONITOR_CMD_FLASH 'F'
#define MONITOR_CMD_RAM   'X'
#define MONITOR_CMD_EEPROM  'E'
#define MONITOR_CMD_QUIT 'Q'
#define MONITOR_CMD_HELP '?'

#define MONITOR_MEM_FLASH  0
#define MONITOR_MEM_RAM    1
#define MONITOR_MEM_EEPROM 2

	void monitorMain(void); 
	void monitorInit(void);
	uint32_t monitorDump( uint32_t address, uint8_t lineNum, uint8_t memType );
	void monitorChange( uint32_t address, uint8_t value, uint8_t memType );
	void print( char *s );
	void print_P( uint32_t address );
	char  *monitorReadLine( void );
	char *getValue( char *src,  uint32_t *value, uint8_t len );
	uint8_t htoi( uint8_t val );

static uint8_t monitorWelcome[] PROGMEM = "\nMonitor\n" ;	
static uint8_t monitorQuit[] PROGMEM = "Quit...\n" ;	
static uint8_t monitorError[] PROGMEM = "Error\n" ;	
char monitorBuf[MONITOR_BUFFER_SIZE];
unsigned char monitor_cnt = 0; // check if we enter monitor
#endif


#if defined(BOOTUART_SET_ATmega128_UART0_ONLY)
#else
unsigned char bootuart0=0,bootuart1=0;
#endif //BOOTUART_SET_ATmega128_UART0_ONLY
unsigned char rx_buffer[MAX_BUF_SIZE];  // RX buffer
unsigned char tx_buffer[MAX_BUF_SIZE];  // TX buffer
unsigned char *rx_pntr = &rx_buffer[0];  // pointer to rx buffer
unsigned char *tx_pntr = &tx_buffer[0];  // pointer to rx buffer
unsigned char *size_pntr = &tx_buffer[2];    // pointer to msg size buffer field
unsigned char *answer_id_pntr = &tx_buffer[5];    // pointer to answer id buffer field
unsigned int msg_size=0x00;  // Rx message size
unsigned char msg_cs=0x00;  // calculated message checksum
unsigned long address_flash = 0L; // actual address in flash memory
unsigned int address_eeprom = 0x00; // actual address in eeprom memory
unsigned int n_bytes = 0x00;  // number of databytes in message
unsigned char n_pages = 0x00;  // number of page to program
unsigned char sequence_number=0x00;  // sequence number from host
unsigned char answer_id=0x00;  // answer cmd id
unsigned int i=0, j=0;  // for loop variables
#ifdef MONITOR
unsigned char echo = 0;  // rs232 terminal echo 
#endif



static unsigned long key[4]={1,2,3,4};
static inline void key_load(void);

// pins for MEMEX4 board
#define PROG_PORT  PORTE
#define PROG_DDR   DDRE
#define PROG_IN    PINE
#define PROG_PIN   PINE2

#define DC_OZU_DDR  DDRD
#define DC_OZU_PORT PORTD
#define DC_OZU_BIT  5

#define REG574 (*(volatile unsigned char*)0x4000)
#define SHDN_BIT  3

#define I82c54_CMD (*(volatile unsigned char*)0x3003)
#define I82c54_DAT (*(volatile unsigned char*)0x3000)

#define I82c54_WD_CLR	10000UL

#define MC146818_REG_A (*(volatile unsigned char*)0x200A)
#define MC146818_REG_B (*(volatile unsigned char*)0x200B)

//#define MC146818_REG_A_VALUE   0x24
#define MC146818_REG_A_VALUE   0x21 // 32768 - 32768
//#define MC146818_REG_B_VALUE   0x0A
#define MC146818_REG_B_MASK   0x04  // SQWE only


static void FLOUTEK_WatchDog(void){
    DC_OZU_PORT|= 1<<DC_OZU_BIT;//ddr for reg_in_use
      /* инициализация счетчика 0 ВИ54 для работы сторожа */
	I82c54_CMD = 0x30;  /* 00110000 - регистр управления ВИ54 */
	I82c54_DAT = (unsigned char)I82c54_WD_CLR;    //Lo
	I82c54_DAT = (unsigned char)(I82c54_WD_CLR>>8); //Hi
    REG574 = (1<<SHDN_BIT);//ADM242 Tx_On
}

//static 
void LCD_Init(void);
const char str1[]="STK500_2";
static void LCD_Send(const char * msg);
int main(void)
{
	unsigned char rx_data;  // received USART data byte
	unsigned char state = STATE_READY;  // actual state
 
	initPorts();
	if((PROG_IN & (1<<PROG_PIN))){
	  app_start();
	}

#if defined(BOOTUART_SET_ATmega128_UART0_ONLY)
#else	
	bootCheck();
#endif //BOOTUART_SET_ATmega128_UART0_ONLY
	initUart();

	//FLOUTEK board!!!!
	MCUCR = (1 << SRE) | (1 << SRW10);
    XMCRA = (1 << SRW01) | (1 << SRW00) | (1<< SRW11);
    XMCRB = (1 << XMBK);//|(1<<XMM2)|(1<<XMM1)|(1<<XMM0);
	DC_OZU_DDR |= 1<<DC_OZU_BIT;//reg_in_use
	DC_OZU_PORT|= 1<<DC_OZU_BIT;//ddr for reg_in_use

	REG574 = (1<<SHDN_BIT);//ADM242 Tx_On

    MC146818_REG_A = MC146818_REG_A_VALUE;
    //MC146818_REG_B = MC146818_REG_B_VALUE;
    rx_data = MC146818_REG_B;
    MC146818_REG_B = rx_data | MC146818_REG_B_MASK;
    rx_data = 0;


    FLOUTEK_WatchDog();

	LCD_Init();
    LCD_Send(str1);
	//=====	  
	
	key_load();



	for(;;)
	{
		rx_data=getch();  // get one byte from USART
		msg_cs ^= rx_data;


		switch( state )  {
			// start processing when byte received
			case STATE_READY :	
				if(rx_data == MESSAGE_START)  // wait for message start
	      		{
			        rx_pntr = &rx_buffer[0];  // reset pointer 
			        msg_size = 0x00;  // reset message size
			        state = STATE_GET_SEQ_NUM;  // get sequence number next
	      		}
	      		else {
	      			msg_cs = 0x00;  // clear old checksum
#ifdef MONITOR	      		
	      			if ( rx_data == MONITOR_FLAG )  {
	      				monitor_cnt ++;
	      				if( monitor_cnt == 3 ) {
	      					monitorMain();	
	      				}
	      			}
	      			else {
	      				monitor_cnt = 0;	
	      			}
#endif	      			
	      		}
	        	break;		
	        	
	        // Get Sequence Number	
			case STATE_GET_SEQ_NUM :
				sequence_number = rx_data;  // store sequence number
				state = STATE_GET_MSG_SIZE1;  // get message size LSB next			
				break; 	 
				
			// Get Message Size 1	
			case STATE_GET_MSG_SIZE1:	
				msg_size = (((unsigned int)rx_data)<<8);  // store message size MSB
				state = STATE_GET_MSG_SIZE2;  // get message size LSB next			
				break;
			
			// Get Message Size 2	
			case STATE_GET_MSG_SIZE2:
				msg_size += (unsigned int) rx_data; // make message size from MSB and LSB
		                                            // +1 because we check if >0
				state = STATE_GET_TOKEN;  // get token next			
				break;
				
			// Get Token		
			case STATE_GET_TOKEN:
				if(rx_data == TOKEN)  // check if token is correct
					state = STATE_GET_DATA;  // get data next
				else
					state = STATE_READY; // wait for new message			
				break;  
				
			// Get Data	
			case STATE_GET_DATA:
				*rx_pntr++ = rx_data;
				--msg_size;
				
				if(msg_size == 0)
				{
					state = STATE_GET_CS;
				}			
				break;	
		
			// Get Checksum and handle message if correct
			case STATE_GET_CS:
				if( msg_cs == 0x00) { // check for valid checksum
					handleMessage();
				} 			
				state = STATE_READY;
				break; 					       		
		}
    
	}  // end for endless loop

	return 0; // main return value

} // end of main
 
/*----------------------------------------------------------------------------*/
/* Handle message                                                             */
/*----------------------------------------------------------------------------*/ 
void handleMessage(void) {
	 	
// new code begin  	
 		tx_pntr = &tx_buffer[0];
 		*(tx_pntr++) = MESSAGE_START ;
 		*(tx_pntr++) = sequence_number ; 
 		tx_pntr += 2;  // space reserved for message size
 		*(tx_pntr++) = TOKEN ;		
        rx_pntr = &rx_buffer[0];  // reset pointer
        
        switch( *rx_pntr ) {
        	case CMD_SIGN_ON:
				cmdSignOn();
        		break;	
        	case CMD_GET_PARAMETER:
        		cmdGetParameter();
        		break;
        	case CMD_SET_PARAMETER:
        		cmdSetParameter();
        		break;
        	case CMD_ENTER_PROGMODE_ISP:
        		cmdEnterProgmodeIsp();
        		break;
        	case CMD_LEAVE_PROGMODE_ISP:
        		cmdLeaveProgmodeIsp();
        		break;
        	case CMD_CHIP_ERASE_ISP:
        		cmdChipEraseIsp();
        		break;
        	case CMD_READ_SIGNATURE_ISP:
        		cmdReadSignatureIsp();
        		break;
        	case CMD_LOAD_ADDRESS:
        		cmdLoadAddress();
        		break;
        	case CMD_PROGRAM_FLASH_ISP:
        		cmdProgramFlashIsp();
        		break;
        	case CMD_READ_FLASH_ISP:
				cmdReadFlashIsp();        	
        		break;	
        	case CMD_PROGRAM_EEPROM_ISP:
        		cmdProgramEepromIsp();
        		break;
        	case CMD_READ_EEPROM_ISP:
        		cmdReadEepromIsp();
        		break;      
        	case CMD_READ_FUSE_ISP:
        	case CMD_READ_LOCK_ISP:        	
        		cmdReadFuseLockIsp();
        		break;         		        		  			
        	default:
				cmdError();
				break;	
        }
        
        sendResponse();
         
        // leave bootloader at programming end
/*
        if( *answer_id_pntr == CMD_LEAVE_PROGMODE_ISP)
        {
        	if(pgm_read_byte_near(0x0000) != 0xFF) {	
          		app_start();
        	}	
        }
*/        
       	
} // end handleMessage

static inline void cmdError(void)  {
	msg_size = 2;  			// set message length		
	*(tx_pntr++) = *rx_pntr;
	*(tx_pntr++) = STATUS_CMD_FAILED;
} 

 

/*----------------------------------------------------------------------------*/ 
/* send response message                                                      */
/*----------------------------------------------------------------------------*/
static void sendResponse(void) {
	unsigned int msg_len, idx;

    size_pntr[0] = (unsigned char) (msg_size >> 8);
    size_pntr[1]=  (unsigned char) (msg_size & 0xFF);
		
	msg_len = tx_pntr - tx_buffer;
	msg_cs = 0;
	for( idx = 0; idx < msg_len; idx++ )  {
		putch( tx_buffer[idx] );
		msg_cs ^= tx_buffer[idx];		
	}

	putch(msg_cs);	
	msg_cs = 0x00;  // reset checksum
}
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_SIGN_ON                                               */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdSignOn(void)  {

	msg_size = 11;  		// set message length
	*(tx_pntr++) = CMD_SIGN_ON;
	*(tx_pntr++) = STATUS_CMD_OK;
	
	*(tx_pntr++) = 0x08;  	// send signature length
	*(tx_pntr++) = 'S';		// send identifier
	*(tx_pntr++) = 'T';
	*(tx_pntr++) = 'K';
	*(tx_pntr++) = '5';
	*(tx_pntr++) = '0';
	*(tx_pntr++) = '0';
	*(tx_pntr++) = '_';
	*(tx_pntr++) = '2';
	
} 
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_READ_SIGNATURE_ISP                                    */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdReadSignatureIsp(void)  {

	msg_size = 4;  		// set message length
	*(tx_pntr++) = CMD_READ_SIGNATURE_ISP;
	*(tx_pntr++) = STATUS_CMD_OK;

	if( *(rx_pntr+4) == 0 )  {  // 1st sig byte
		*(tx_pntr++) = SIG1;	
	}    
	else if ( *(rx_pntr+4) == 1 ) { // 2nd sig byte
		*(tx_pntr++) = SIG2;								
	} 
	else if ( *(rx_pntr+4) == 2 ) { // 3rd sig byte
		*(tx_pntr++) = SIG3;								
	} 
	else {
		*(tx_pntr++) = 0x00;   // error 	
	}
    *(tx_pntr++) = STATUS_CMD_OK;	
	
}  
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_SET_PARAMETER                                         */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdSetParameter(void)  {


	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_SET_PARAMETER;
	*(tx_pntr++) = STATUS_CMD_OK; 
} 

/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_ENTER_PROGMODE_ISP                                    */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdEnterProgmodeIsp(void)  {


	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_ENTER_PROGMODE_ISP;
	*(tx_pntr++) = STATUS_CMD_OK; 
} 

/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_LEAVE_PROGMODE_ISP                                    */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdLeaveProgmodeIsp(void)  {


	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_LEAVE_PROGMODE_ISP;
	*(tx_pntr++) = STATUS_CMD_OK; 
} 
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_CHIP_ERASE_ISP                                        */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdChipEraseIsp(void)  {

	// clear only first page becuase of timeout in AvrStudio
	boot_page_erase((unsigned long)(0L));  	// clear 1st page
	while(boot_spm_busy());	        				
    boot_rww_enable();  			// enable Read-While-Write section
	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_CHIP_ERASE_ISP;
	*(tx_pntr++) = STATUS_CMD_OK; 
} 
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_LOAD_ADDRESS                                          */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdLoadAddress(void)  {

    address_eeprom = ((*(rx_pntr+3)*256)+*(rx_pntr+4));
    address_flash  = ((*(rx_pntr+3)*256)+*(rx_pntr+4))*2;

	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_LOAD_ADDRESS;
	*(tx_pntr++) = STATUS_CMD_OK; 
}  
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_GET_PARAMETER                                         */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdGetParameter(void)  {


	msg_size = 3;  			// set message length		
	*(tx_pntr++) = CMD_GET_PARAMETER;
	*(tx_pntr++) = STATUS_CMD_OK;

	switch( *(rx_pntr+1) )  {
		default: 	
            *(tx_pntr++) = 0x00; // send dummy value for not supported parameters   
        return; //break;
		case PARAM_HW_VER: 
            *(tx_pntr++) = HARDWARE_VERSION;  // send hardware version          				
			return; //break;
		case PARAM_SW_MAJOR: 
            *(tx_pntr++) = SOFTWARE_MAJOR; // send software major version         				
			return; //break;
		case PARAM_SW_MINOR: 
            *(tx_pntr++) = SOFTWARE_MINOR;  // send software minor version          				
			return; //break;			
		case PARAM_VTARGET: 
            *(tx_pntr++) = VTARGET; // target supply voltage         				
			return; //break;        			
		case PARAM_VADJUST: 
            *(tx_pntr++) = VADJUST; // target VREF voltage          				
			return; //break;  
		case PARAM_OSC_PSCALE: 
            *(tx_pntr++) = PSCALE_FSYS; // oscilator prescaler value         				
			return; //break;
		case PARAM_OSC_CMATCH: 
            *(tx_pntr++) = CMATCH_DEF; // oscilator compare value         				
			return; //break;			
		case PARAM_SCK_DURATION: 
            *(tx_pntr++) = SCK_DURATION_DEF; // oscilator compare value         				
        	return; //break;
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
		case PARAM_TOPCARD_DETECT: 
            *(tx_pntr++) =  STK501; // STK501 is expected          				
			return; //break;
#endif 
/*@Vit       			
		default: 	
            *(tx_pntr++) = 0x00; // send dummy value for not supported parameters   
        return; //break;
*/
	}

	
	
				
} 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_PROGRAM_FLASH_ISP                                     */
/*----------------------------------------------------------------------------*/ 

static inline void cmdProgramFlashIsp(void) {
	
unsigned char byte1,byte2;
unsigned long address, address_page1,address_page2,offset;
	
	n_bytes = ((*(rx_pntr+1)*256)+*(rx_pntr+2));  	// number of databytes to flash
	rx_pntr += 10;  								// set pointer to flash data
	//XTEA_encode_special_((unsigned long*) rx_pntr, (unsigned char)(n_bytes>>2));	
	XTEA_decode_special((unsigned long*) rx_pntr, (unsigned char)(n_bytes>>2));	
	address_page1 = address_flash - ( address_flash % SPM_PAGESIZE ); // addres of 1st page to program
	address_page2 = ( address_flash + n_bytes ) - 
	                ( ( address_flash + n_bytes ) % SPM_PAGESIZE ); // addres of last page to program
	n_pages = (( address_page2 - address_page1) / SPM_PAGESIZE ) + 1;  	// number of pages to flash	  
	offset = address_flash - address_page1 ;              		
	for( j = 0; j < n_pages; j++ )  { 					// do for all pages in message
        for(i=0; i < SPM_PAGESIZE; i += 2)  		// load old content of flash page
        {
        	address = address_page1 + i;
        	if( (address < address_flash  ) ||
        	    (address > ( address_flash + n_bytes) ) )  {  // copy to buffer old data
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)		
	    		byte1 = pgm_read_byte_far(address);
	    		byte2 = pgm_read_byte_far(address + 1);
#else
	    		byte1 = pgm_read_byte(address_page1 + i );
	    		byte2 = pgm_read_byte(address_page1 + i + 1);    		
#endif          
			}
			else  {			// copy to buffer new data
				byte1 = *rx_pntr;
				byte2 = *(rx_pntr+1);
				rx_pntr += 2;								
			}	
			boot_page_fill(i,(unsigned int)(byte1 |(byte2 << 8)));
        }																			
		boot_page_erase((unsigned long)address_page1);  	// clear page
        while(boot_spm_busy());	        				
        boot_page_write((unsigned long)address_page1);  	// write page to flash
        while(boot_spm_busy());
        boot_rww_enable();  								// enable Read-While-Write section
		address_page1 += SPM_PAGESIZE ;
	}	
	address_flash += n_bytes ;
	for(i = 0; i < MAX_BUF_SIZE; i++) { 
		rx_buffer[i] = 0xFF;  // clear data buffer
	}
		
	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_PROGRAM_FLASH_ISP;
	*(tx_pntr++) = STATUS_CMD_OK; 	
	
} // end of programFlashIsp
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_READ_FLASH_ISP                                        */
/*----------------------------------------------------------------------------*/ 


static inline void cmdReadFlashIsp(void) {
    unsigned char * ptr;
	*(tx_pntr++) = CMD_READ_FLASH_ISP;
	*(tx_pntr++) = STATUS_CMD_OK;
	//ptr = tx_pntr+1;
	ptr = tx_pntr;//Test
	n_bytes=((*(rx_pntr+1)*256)+*(rx_pntr+2));  // number of databytes to read from flash 
	//@Vit
#ifdef XTEA_ENCODE_ON	
	//n_bytes &= ~(7U);
#endif // XTEA_ENCODE_ON		
	//====
	for(i=0;i < n_bytes; i++) { // fill data buffer with n_bytes
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
    	*(tx_pntr++) = (address_flash<BOOTLOADER_BASE)?(pgm_read_byte_far(address_flash++)):(0xFF);
#else
    	*(tx_pntr++) = pgm_read_byte(address_flash++);
#endif    	
	}
	//@Vit	
#ifdef XTEA_ENCODE_ON		
	XTEA_encode_special_((unsigned long*)ptr, (unsigned char)(n_bytes>>2));
#endif // XTEA_ENCODE_ON		
	//====	
	msg_size = n_bytes + 3;  // set message length
	*(tx_pntr++) = STATUS_CMD_OK;	
	
}   // end of readFlashIsp
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_PROGRAM_EEPROM_ISP                                     */
/*----------------------------------------------------------------------------*/ 

static inline void cmdProgramEepromIsp(void) {
	
	n_bytes = ((*(rx_pntr+1)*256)+*(rx_pntr+2));  	// number of databytes to flash
	rx_pntr += 10;  								// set pointer to flash data
	for( j = 0; j < n_bytes; j++ )  {
	    eeprom_wb(address_eeprom++,*(rx_pntr++));		
	}

	for(i = 0; i < MAX_BUF_SIZE; i++) { 
		rx_buffer[i] = 0xFF;  // clear data buffer
	}
		
	msg_size = 2;  			// set message length		
	*(tx_pntr++) = CMD_PROGRAM_EEPROM_ISP;
	*(tx_pntr++) = STATUS_CMD_OK; 	
	
} // end of programEepromIsp 
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_READ_EEPROM_ISP                                        */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdReadEepromIsp(void) {

	*(tx_pntr++) = CMD_READ_EEPROM_ISP;
	*(tx_pntr++) = STATUS_CMD_OK;
	
	n_bytes=((*(rx_pntr+1)*256)+*(rx_pntr+2));  // number of databytes to read from flash 
	for(i=0;i < n_bytes; i++) { // fill data buffer with n_bytes
    	*(tx_pntr++) = eeprom_rb( address_eeprom++ );
	}
	msg_size = n_bytes + 3;  // set message length
	*(tx_pntr++) = STATUS_CMD_OK;	
	
}   // end of readEepromIsp 
 
/*----------------------------------------------------------------------------*/ 
/* execute command  CMD_READ_FUSE_ISP,CMD_READ_LOCK_ISP                       */
/*----------------------------------------------------------------------------*/ 
 
static inline void cmdReadFuseLockIsp(void)  {
	unsigned int command,address;

	msg_size = 4;  		// set message length
	*(tx_pntr++) = *rx_pntr;
	*(tx_pntr++) = STATUS_CMD_OK;
	
	command = *(rx_pntr+2) * 256 + *(rx_pntr+3);

	switch( command ) {
		case 0x5800: address = 0x0001; break; // lock bits		
		case 0x5000: address = 0x0000; break; // fuse low	
		case 0x5808: address = 0x0003; break; // fuse hi	
		case 0x5008: address = 0x0002; break; // fuse ext
		default: address = 0x0000;			
	}
	
	*(tx_pntr++) = readBits(address);	 
    *(tx_pntr++) = STATUS_CMD_OK;	
	
}   
 
/*----------------------------------------------------------------------------*/ 
/* set pin direction for bootloader pin and enable pullup                     */
/*----------------------------------------------------------------------------*/
static void initPorts(void)  {
#ifdef NAFIG_080922
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
  	BL_DDR &= ~(1 << BL0);
  	BL_DDR &= ~(1 << BL1);
  	BL_PORT |= (1 << BL0);
  	BL_PORT |= (1 << BL1);
  	
#else
	BL_DDR &= ~(1 << BL);
	BL_PORT |= (1 << BL);
#endif

#ifdef MONITOR

#endif
#endif // NAFIG_080922
} 
 
/*----------------------------------------------------------------------------*/ 
/* initialize UART(s) depending on CPU defined                                */
/*----------------------------------------------------------------------------*/ 
static void initUart(void)  {
#if defined(BOOTUART_SET_ATmega128_UART0_ONLY)
  UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
  UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
  UCSR0A = 0x00;
  UCSR0C = 0x06;
  UCSR0B = _BV(TXEN0)|_BV(RXEN0);
#else
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
 	if(bootuart0) {
	    UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	    UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	    UCSR0A = 0x00;
	    UCSR0C = 0x06;
	    UCSR0B = _BV(TXEN0)|_BV(RXEN0);
	}
	if(bootuart1) {
	    UBRR1L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	    UBRR1H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	    UCSR1A = 0x00;
	    UCSR1C = 0x06;
	    UCSR1B = _BV(TXEN1)|_BV(RXEN1);
	}
#elif defined __AVR_ATmega163__
	UBRR = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRHI = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRB = _BV(TXEN)|_BV(RXEN);	
#else
  /* m8,m16,m32,m169,m8515,m8535 */
	UBRRL = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRRH = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
	UCSRA = 0x00;
	UCSRC = 0x86;
	UCSRB = _BV(TXEN)|_BV(RXEN);
#endif	
#endif //BOOTUART_SET_ATmega128_UART0_ONLY
} // end of initUart
 

/*----------------------------------------------------------------------------*/
/* check if flash is programmed already, if not start bootloader anyway       */
/*----------------------------------------------------------------------------*/
static inline void bootCheck(void)  {
#ifdef NAFIG_080917_Vit
	if(pgm_read_byte_near(0x0000) != 0xFF) {

#ifdef __AVR_ATmega128__
	    /* check which UART should be used for booting */
	    if(bit_is_clear(BL_PIN,BL0)) {
	      bootuart0 = 1;
	    }
	    else if(bit_is_clear(BL_PIN,BL1)) {
	      bootuart1 = 1;
	    } else {
	      /* if no UART is being selected, start application */

	      app_start();
	    }
#elif defined __AVR_AT90CAN128__
	    /* check which UART should be used for booting */
	    if(bit_is_clear(BL_PIN,BL0)) {
	      bootuart1 = 1;   
	    }
	    else if(bit_is_clear(BL_PIN,BL1)) {
	      bootuart0 = 1;
	    } else {
	      /* if no UART is being selected, start application */
	      app_start();
	    }
#else
	    /* check if bootloader pin is set low */
	    if(bit_is_set(BL_PIN,BL)) app_start();
#endif
  }
#endif // NAFIG_080917_Vit

#if defined(BOOTUART_SET_ATmega128_UART0_ONLY)
#else
#ifdef __AVR_ATmega128__
  /* if no UART is being selected, default is RS232 */
  /* @Vit
  if((bootuart0 == 0) && (bootuart1 == 0)) {
    bootuart0 = 1;
  }
  */
  if(bootuart1 == 0) {
    bootuart0 = 1;
  }
#elif defined __AVR_AT90CAN128__
  /* if no UART is being selected, default is USB */
  if((bootuart0 == 0) && (bootuart1 == 0)) {
    bootuart1 = 1;
  }  
#endif
#endif	//BOOTUART_SET_ATmega128_UART0_ONLY
} // end of bootCheck 
 
static void putch(char ch)
{
#if defined(BOOTUART_SET_ATmega128_UART0_ONLY)
  while (!(UCSR0A & _BV(UDRE0))){FLOUTEK_WatchDog();};
  UDR0 = ch;
#else
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
  if(bootuart0) {
    while (!(UCSR0A & _BV(UDRE0))){FLOUTEK_WatchDog();};
    	UDR0 = ch;
  }
  else if (bootuart1) {
    while (!(UCSR1A & _BV(UDRE1)));
    	UDR1 = ch;
  }
#else
  /* m8,16,32,169,8515,8535,163 */
  while (!(UCSRA & _BV(UDRE)));
	UDR = ch;
#endif
#endif //BOOTUART_SET_ATmega128_UART0_ONLY
}  // end of putch

static char getch(void)
{
	char tmp;
#if defined(BOOTUART_SET_ATmega128_UART0_ONLY)	
  while(!(UCSR0A & _BV(RXC0))){
    FLOUTEK_WatchDog();
  };
  tmp = UDR0;
#else
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__)
  if(bootuart0) {
    while(!(UCSR0A & _BV(RXC0))){
      FLOUTEK_WatchDog();
    };
    tmp = UDR0;
  }
  else if(bootuart1) {
    while(!(UCSR1A & _BV(RXC1))){
      FLOUTEK_WatchDog();
    };
     tmp = UDR1;
  }
#else 
  /* m8,16,32,169,8515,8535,163 */
  while(!(UCSRA & _BV(RXC)));
   tmp = UDR ;
#endif
#endif //BOOTUART_SET_ATmega128_UART0_ONLY
/* @Vit 090908*/
#ifdef MONITOR
  if( echo ) {
     putch( tmp ); 
  } 
#endif

    
  return( tmp );

}  // end of getch

static void eeprom_wb(unsigned int uiAddress, unsigned char ucData)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEWE))
;
/* Set up address and data registers */
EEAR = uiAddress;
EEDR = ucData;
/* Write logical one to EEMWE */
EECR |= (1<<EEMWE);
/* Start eeprom write by setting EEWE */
EECR |= (1<<EEWE);
}

static unsigned char eeprom_rb(unsigned int uiAddress)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEWE))
;
/* Set up address register */
EEAR = uiAddress;
/* Start eeprom read by writing EERE */
EECR |= (1<<EERE);
/* Return data from data register */
return EEDR;
} 

/* read lock/fuse bits */
static unsigned char readBits( unsigned int address ) {	
#ifdef NAFIG_090908_Vit
	asm volatile(
				"mov	r31,r25 \n\t"	
		       	"mov	r30,r24 \n\t"	
				"lds	r24,%0 \n\t"		       	
				"ori	r24,0x09 \n\t"
				"sts	%0,r24 \n\t"									
		       	"lpm	\n\t" 
		       	"mov	r24,r0  \n\t" 
		       	: "=m" (SPMCSR)
				);     
#else
    return 0;
#endif // NAFIG_090908_Vit
				
}

#ifdef MONITOR

void monitorMain( void )  {
	
    uint8_t *buf ;
    uint8_t convBuf[20];
	uint8_t quit = 0;
	uint32_t address ;
	uint8_t  value;
	uint8_t  cmd;
	uint8_t  lastCmd = 0;
	struct time t;
	struct date d;

	monitorInit();
   	print_P( (uint32_t)monitorWelcome ); 
   	echo = 1;
    while( ! quit ) {
   		putch( MONITOR_PROMPT );    	
    	buf = monitorReadLine();
    	while( isblank( *buf ) ) {
    		buf ++ ;	
    	}	
    	cmd = toupper(*(buf++));
    	switch( cmd ) {
    		case MONITOR_CMD_QUIT:
    			echo = 0;
    			quit = 1;
    			monitor_cnt = 0; 
    			print_P( (uint32_t)monitorQuit );
    			break;
    		case MONITOR_CMD_TIME:	
    		 	buf = getValue( buf, &(t.hour10), 1 );
    		 	if( buf ) {
    		 		buf = getValue( buf, &(t.hour), 1 );
    		 		buf = getValue( buf, &(t.min10), 1 );
    		 		buf = getValue( buf, &(t.min), 1 );
    		 		buf = getValue( buf, &(t.sec10), 1 );
    		 		buf = getValue( buf, &(t.sec), 1 );
    		 		set_clock( 0, &t );	
    		 	}			 			    							
				print( time2str(get_time(&t),convBuf) );
				putch('\n');
    			lastCmd = cmd;
    			break;    			
    		case MONITOR_CMD_DATE:	
    		 	buf = getValue( buf, &(d.year10), 1 );
    		 	if( buf ) {
    		 		buf = getValue( buf, &(d.year), 1 );
    		 		buf = getValue( buf, &(d.month10), 1 );
    		 		buf = getValue( buf, &(d.month), 1 );
    		 		buf = getValue( buf, &(d.day10), 1 );
    		 		buf = getValue( buf, &(d.day), 1 );
    		 		set_clock( &d, 0 );	
    		 	}			 			    							
				print( date2str(get_date(&d),convBuf) );
				putch('\n');
    			lastCmd = cmd;
    			break;    			    			
    		case MONITOR_CMD_FLASH:
    			buf = getValue( buf, &address, 5 );
    			if( buf )  {
    				address = monitorDump( address,MONITOR_DISPLAY_LINES, MONITOR_MEM_FLASH );	
    			}
    			lastCmd = cmd;
    			break;
    		
    		case MONITOR_CMD_RAM:
    			buf = getValue( buf, &address, 4 );
    			if( buf )  {
    				buf = getValue( buf, &value, 2 );
    				if( buf )  {
    					monitorChange( address, value, MONITOR_MEM_RAM );	
    					address = monitorDump( address,1, MONITOR_MEM_RAM );
    				}
    				else {    				
    					address = monitorDump( address,MONITOR_DISPLAY_LINES, MONITOR_MEM_RAM );	
    				}	
    			}
    			lastCmd = cmd;    			
    			break;   		
    			
    		case MONITOR_CMD_EEPROM:
    			buf = getValue( buf, &address, 3 );
    			if( buf )  {
    				buf = getValue( buf, &value, 2 );
    				if( buf )  {
    					monitorChange( address, value, MONITOR_MEM_EEPROM );
    					address = monitorDump( address,1, MONITOR_MEM_EEPROM );	
    				}
    				else {
    					address = monitorDump( address,MONITOR_DISPLAY_LINES, MONITOR_MEM_EEPROM );
    				}		
    			} 
    			lastCmd = cmd;    			
    			break;	
    		case MONITOR_CMD_HELP:
    			print_P( (uint32_t) PSTR("\nF AAAA    - dump flash\n") );
    			print_P( (uint32_t) PSTR("X AAAA XX - dump/modify ram\n") );
    			print_P( (uint32_t) PSTR("E AAAA XX - dump/modify eeprom\n") ); 
    			print_P( (uint32_t) PSTR("T HHMMSS - display/set time\n") ); 
    			print_P( (uint32_t) PSTR("D YYMMDD - display/set date\n") );    			   			  
				print_P( (uint32_t) PSTR("Q         - quit monitor\n") ); 			    			
    			break;    			
    		case '\0':
    		case '\n':
    		case '\r':
    			switch( lastCmd ) {
    				case MONITOR_CMD_TIME:
    					print( time2str(get_time(&t),convBuf) );
    					putch('\n');
    					break;  
    				case MONITOR_CMD_DATE:
    					print( date2str(get_date(&d),convBuf) );
    					putch('\n');
    					break;     			    					   				
 		    		case MONITOR_CMD_FLASH:
    					address = monitorDump( address,MONITOR_DISPLAY_LINES, MONITOR_MEM_FLASH );	
    					break;   
 		    		case MONITOR_CMD_RAM:
    					address = monitorDump( address,MONITOR_DISPLAY_LINES, MONITOR_MEM_RAM );	
    					break; 
 		    		case MONITOR_CMD_EEPROM:
    					address = monitorDump( address,MONITOR_DISPLAY_LINES, MONITOR_MEM_EEPROM );	
    					break;     					    									
    			} 
    			break;
    		
    		default: 
    			print_P( (uint32_t) monitorError );
    			break;
    	}

    }
}

void monitorInit( void )  {
	XMCRB = 0x00 ; // 60kb RAM available	
#if defined(__AVR_ATmega128__) 	
	XDIV  = 0x00; //xtal divider disabled
	XMCRA = 0x00; //ext ram one sector 1100-FFFF
	MCUCR = ( 1 << SRE ); // ext ram enable	
#elif defined(__AVR_AT90CAN128__)
	CLKPR = 0x00; // clock divider disabled 
	XMCRA = ( 1 << SRE ) ; // ext ram enable, one segment	
#endif	
}

uint32_t monitorDump( uint32_t address, uint8_t lineNum, uint8_t memType ) {
	uint8_t charCnt ;
	uint8_t lineCnt ;
	uint8_t memByte ;
	uint8_t bufHex[10];
	uint8_t bufAsc[20];
	
	putch('\n');
	for( lineCnt = 0; lineCnt < lineNum; lineCnt++ ) {
		print( ultoa(address,bufHex,16) );
		putch( ' ');
		for( charCnt = 0; charCnt < 16; charCnt++ ) {
			switch( memType ) {
				case MONITOR_MEM_FLASH: 
					memByte = pgm_read_byte_far( address++ );
					break;
				case MONITOR_MEM_RAM: 
					memByte = *((uint8_t *)address++);
					break;			
				case MONITOR_MEM_EEPROM: 
					memByte = eeprom_rb( address++ );
			}
			if( memByte < 0x10 )  {
				putch('0');	
			}
			print( ultoa(memByte,bufHex,16) );
			putch(' ');
			bufAsc[charCnt] = isprint(memByte) ? memByte : '.';	
		}
		bufAsc[16] = '\0';
		print( bufAsc );
		putch('\n');	
	}
	return address ;
	
}

void monitorChange( uint32_t address, uint8_t value, uint8_t memType ) {
	switch( memType ) {
		case MONITOR_MEM_RAM: 
			*((uint8_t *)address) = value;
			break;			
		case MONITOR_MEM_EEPROM: 
			eeprom_wb( address, value );
			break;		
	}	
}



char  *monitorReadLine( void ) {
	unsigned int cnt;
	char key;
	
	cnt = 0;
	while( ((key = getch()) != '\n') && cnt < ( MONITOR_BUFFER_SIZE - 1) )  {
		monitorBuf[ cnt++ ] = key ; 
	}
	monitorBuf[ cnt ] = '\0';
	return monitorBuf ; 	
}

void print( char *s )  {
   while( *s ) {
   		putch( *s );
   		s++ ;
   }	
}

void print_P( uint32_t address ) {
   uint8_t c;
   while ((c = pgm_read_byte_far(address++))) {
		putch(c);
   }				
}


uint8_t htoi( uint8_t val ) {
	if( val >= '0' && val <= '9' ) {
		return (val - '0');
	}
	else {
	    return ( toupper( val ) - 'A' + 10 );
	}
}

char *getValue( char *src,  uint32_t *value, uint8_t len )  {
	char buf[10];
	
	*buf = '\0';
	*value = 0;
    while( isblank( *src ) && ( *src != '\0' ) ) {
    	src ++ ;	
    }	
    if ( *src == '\0' || !isxdigit(*src) )  {
    	return 0; 	
    }
    else {
    	while( isxdigit( *src ) && len ) {
    		*value = (*value << 4) | htoi( *src );  
    		len-- ;	
    		src++ ;
    	}  	
 	    return src ;				
    }   
}







#endif
//const unsigned long DELTA = 0x9E3779B9;
//enum { DELTA = 0x9E3779B9};
#define DELTA 0x9E3779B9
#define NUMROUNDS  32ul;
//#define NUMROUNDS  4ul;
//const unsigned long key[4]={1,2,3,4};
/*
void XTEA_encode(unsigned long* data, unsigned char dataLength)
{
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	unsigned char iterationCount;
	dataLength &= ~(1U);	
	while(dataLength)
	{
		sum = 0;
		x1=*data;
		x2=*(data+1);
		iterationCount = NUMROUNDS;

		while(iterationCount > 0)
		{
			x1 += ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
			sum+=DELTA;
			x2 += ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));

			iterationCount--;
		}
		*(data++)=x1;
		*(data++)=x2;
		dataLength-=2;
	}	
}
*/
static void XTEA_encode_special_(unsigned long* data, unsigned char dataLength)
{
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	unsigned char iterationCount;
	//dataLength &= ~(1U);	
	while(dataLength)
	{
		if((*data==0xFFFFFFFF)&&(*(data+1)==0xFFFFFFFF)){
		  dataLength-=2;
		  continue;
		}
		sum = 0;
		x1=*data;
		x2=*(data+1);
		iterationCount = NUMROUNDS;

		while(iterationCount > 0)
		{
			x1 += ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
			sum+=DELTA;
			x2 += ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));

			iterationCount--;
		}
		*data++ = x1;
		*data++ = x2;
		dataLength-=2;
	}	
}

/*
void XTEA_decode(unsigned long* data, unsigned char dataLength) 
{
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	dataLength &= ~(1U);
	while(dataLength)
	{
		sum = DELTA*NUMROUNDS; // 32 итерации
		x1=*data;
		x2=*(data+1);

		while(sum != 0)
		{
			x2 -= ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));
			sum-=DELTA;
			x1 -= ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
		}
		*(data++)=x1;
		*(data++)=x2;
		dataLength-=2;
	}	
}
*/
static void XTEA_decode_special(unsigned long* data, unsigned char dataLength){
	unsigned long x1;
	unsigned long x2;
	unsigned long sum;
	//dataLength &= ~(1U);	
	while(dataLength)
	{
        if((*data==0xFFFFFFFF)&&(*(data+1)==0xFFFFFFFF)){
		  dataLength-=2;
		  continue;
		}
		sum = DELTA*NUMROUNDS; // 32 итерации
		x1=*data;
		x2=*(data+1);

		while(sum != 0)
		{
			x2 -= ((x1<<4 ^ x1>>5) + x1) ^ (sum + *(key+(sum>>11&0x03)));
			sum-=DELTA;
			x1 -= ((x2<<4 ^ x2>>5) + x2) ^ (sum + *(key+(sum&0x03)));
		}
		*data++ =x1;
		*data++ =x2;
		dataLength-=2;	
	}
}
/**/
static inline void key_load(void){
  unsigned char zzz = 16;
  unsigned char tmp = 0xFF;
  uint32_t address_flash = XTEA_KEY_ADDRESS;

  do{   
    tmp &= pgm_read_byte_far(address_flash++);	
  }while(--zzz);

  if(tmp!=0xFF){
	address_flash = XTEA_KEY_ADDRESS;
    for(zzz=0; zzz<4; zzz++){
	  key[zzz] = pgm_read_dword_far(address_flash);address_flash+=4;
	}
  }else{//none - default value is set now
    ;
  }
}


#define DELAY_5ms()  _delay_loop_2(4609*2) //4609 for XTAL 3.6864 - FLOUTEK has XTAL (3.6864*2)MHz
#define DELAY_50ms() do{_delay_loop_2(46090UL);_delay_loop_2(46090UL);}while(0) 

#define LCD_ADDR_C (*(volatile unsigned char*)0x8000)
#define LCD_ADDR_D (*(volatile unsigned char*)0x8001)
#define LCD_CMD_WR(X)  do{                                  \
                         /*DC_OZU_PORT|= 1<<DC_OZU_BIT;*/   \
                         LCD_ADDR_C = (X);                  \
						 /*DC_OZU_PORT$= ~(1<<DC_OZU_BIT);*/\
                       }while(0)
#define LCD_DATA_WR(X) do{                                  \
                         /*DC_OZU_PORT|= 1<<DC_OZU_BIT;*/   \
                         LCD_ADDR_D = (X);                  \
						 /*DC_OZU_PORT$= ~(1<<DC_OZU_BIT);*/\
                       }while(0)

#define LCD_FIRST_LINE_FIRST_FIELD 0x80
void LCD_Init(void)
{
  //DC_OZU_PORT|= 1<<DC_OZU_BIT;//ddr for reg_in_use
  DELAY_50ms();
  LCD_CMD_WR(0x38);
  DELAY_5ms();
  LCD_CMD_WR(0x38);
  DELAY_5ms();
  LCD_CMD_WR(0x38); // 8-bit, 2-line, 5-10
  DELAY_5ms();
  LCD_CMD_WR(0x0C); // Display ON, Coursor OFF, Blink OFF 
  DELAY_5ms();
  LCD_CMD_WR(0x01); // Display CLEAR;
  DELAY_5ms();
  LCD_CMD_WR(0x06); // Autoincrement, Shift
  DELAY_5ms();
  LCD_CMD_WR(0x80); // set position to first field of first line (??)
  DELAY_5ms();
}

static void LCD_Send(const char * msg)
{
  //LCD_CMD_WR(LCD_FIRST_LINE_FIRST_FIELD);
  LCD_CMD_WR(0x85);
  DELAY_50ms();
  //i = 0; 
  while(*msg){
    LCD_DATA_WR(*(msg++));
    DELAY_5ms();		
  };
}
