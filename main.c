/********************************************************************
 *
 *                			SHT15 Test
 *
 ********************************************************************
 *
 * Author               Date        Comment
 ********************************************************************
 * Phillip Pickett     10.31.09		Initial Release		
 * Phillip Pickett     11.4.09		Added in dew point	
 ********************************************************************/

#include <p18cxxx.h>
#include <delays.h>
#include "TypeDefs.h"
#include <usart.h>
#include <string.h>
#include <math.h>


enum {TEMP,HUMI};

#define _SHT15_DAT (TRISBbits.TRISB5) //Rouge
#define DATA (PORTBbits.RB5)
#define _SHT15_SCK (LATBbits.LATB4) //Noir
#define SET_SHT15_SCK (TRISBbits.TRISB4) 

#define noACK 0
#define ACK 1

#define STATUS_REG_W 	0x06 //000 0110 0
#define STATUS_REG_R 	0x07 //000 0111 1
#define MEASURE_TEMP 	0x03 //000 0011 1
#define MEASURE_HUMI 	0x05 //000 0101 1
#define RESET 			0x1E //000 1111 0

void s_transstart (void);
void s_connectionreset(void);
char s_softreset(void);
char s_write_byte(unsigned char value);
char s_read_byte(unsigned char ack);
char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode);
unsigned char Lecture_Sht15 (void);
void Init_Sht15 (void);
char s_write_statusreg(unsigned char *p_value);
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum); 
void usartInit(void);
void delay(int);
void calc_sth11(float *p_humidity ,float *p_temperature);
float calc_dewpoint(float h,float t);
void calc_sth11(float *p_humidity ,float *p_temperature);
void ftoa (char *buf, double fval, int cField, int cPlaces);



void ftoa (char *buf, double fval, int cField, int cPlaces) {
	long lTemp;
	long scale;
	int i;
	char dbuf[8];			// Temporary - used and overwritten in routine
	char pbuf[8];			// Temporary - used and overwritten in routine
	char fbuf[16];			// The output string is built here
														//  and copied into the *buf buffer at the end.
	char cDigits;
	//---------------------------------------------------------------------------
	//size_t szTemp;
	int szTemp;
	if (cPlaces)
	{
	  cDigits = (cField - cPlaces - 1);	//FP formatting.  Includes '.' and decimal part
	}
	else
	{
	  cDigits = cField;		//Special case for integers
	}
																			//EXAMPLE USED: fval=123.456789, fp(5,5) has been set
	memset (fbuf, 0, sizeof (fbuf));		//Clear the buffer
	scale = 1;													//Set default scale value
	lTemp = (long) fval;			//convert double to long	- ltemp=123
	ltoa (lTemp, dbuf);			//convert long to ascii in dbuf
	szTemp = strlen (dbuf);		//How many digits does the number use?
																			//---------------------------------------	
	if ((char) szTemp < cDigits)		//Do we need leading whitespaces?
	{
		for (i=0; i < (cDigits - (int) szTemp +0); i++)
		{
		 pbuf[i] = ' ';		//Insert the reqd number of spaces into the buffer 'pbuf'
		}
		if ((fval < 0) && (fval > -1) )      //Special case for -0.1 (etc) where 'digit' is = 0, but fval is -ve.
		{
		 pbuf[i-1] = '-';		//Add -ve symbol to the main buffer
		}
		pbuf[i] = 0;											                                                               //Null terminate the spaces so strcat can find it
																			//No need to add '1' - the for loop does this for us!
		strcat(fbuf, pbuf);		//Add the spaces (and sign?) into the main buffer
	}
	else
	{
		if ((fval < 0) && (fval > -1) )  	//Special case for -0.1 (etc) where 'digit' is = 0, but fval is -ve.
		{
			fbuf[0] = '-';	//Add -ve symbol to the main buffer
			fbuf[1] = 0;	//Null terminate
		}
	}
	strcat(fbuf, dbuf);			//Add the numerical part to the main buffer (after '-' and spaces if reqd)
	if (cPlaces)				//Do we need the decimal part?
	{															
		strcatpgm2ram (fbuf, ".");	//add '.' to end of main buffer
		switch (cPlaces)		// Convert fractional	
		{
			case 1:	scale = 10; break;
			case 2:	scale = 100; break;
			case 3:     scale = 1000;  break;
			case 4:	scale = 10000; break;
			case 5:     scale = 100000; break;
		}
		fval = (fval - (int) fval);               //Remove integer part... fval=0.456789

		if (fval >= 0)		// Do not lose "0" in fractional part (i.e. 0.0456)
			fval += 1;
		else
			fval -= 1;
									
		fval *= scale;
		
		if (fval >= 0)										
		 fval += 0.5f;		//  For positive round number up
		else
		 fval -= 0.5f;		// For negative round number down
										
		lTemp = (long) fval;		//convert to long integer - ltemp=45678
		if (lTemp < 0) lTemp =- lTemp;	//No negative fractions!!! (prevents -1.-234 etc)
		ltoa (lTemp, dbuf);		//copy into dbuf - dbuf='45678'
		strcat(fbuf, &dbuf[1]);		//add the fractional part to the main buffer
	}				// without "1" in dbuf[0]
	strncpy (buf, fbuf, (size_t) cField);	//copy the final result to the output buffer
	buf[cField] = 0;			//Null terminate buf
									
}



void delay(int x){
	int i;
	for(i = 0; i < x; i++){
		Delay100TCYx(120); //wait 1ms
	}
}


void usartInit(void) {
	OpenUSART( USART_TX_INT_OFF &
		USART_RX_INT_OFF &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_LOW,
		77);
}


void Init_Sht15 (void) {
	DATA = 0;
	_SHT15_DAT = 1;		// data en enter
	SET_SHT15_SCK = 0; 	// clock en sortie
	_SHT15_SCK = 0; 	// clock = low

	s_softreset();
	s_connectionreset();
}


//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
void s_transstart (void) {
	_SHT15_DAT = 1;
	_SHT15_SCK = 0; //Initial state
	Nop();
	_SHT15_SCK = 1;
	Nop();
	_SHT15_DAT = 0;
	Nop();
	_SHT15_SCK = 0;
	Nop();Nop();Nop();// .5uS
	_SHT15_SCK = 1;
	Nop();
	_SHT15_DAT = 1;
	Nop();
	_SHT15_SCK = 0;
}


//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
void s_connectionreset(void) {
	unsigned char i;
	
	_SHT15_DAT = 1;
	_SHT15_SCK = 0; //Initial state

	for(i = 0; i < 15; i++) {
		_SHT15_SCK = 1;
		_SHT15_SCK = 0;
	}
	
	s_transstart(); //transmission start
}


char s_write_byte(unsigned char value) {
	unsigned char i;
	unsigned char error = 0;
	
	for(i = 0x80; i > 0; i/=2) { 
		if (i & value) {
			_SHT15_DAT=1;
		}
		else {
			_SHT15_DAT=0;
		}

		_SHT15_SCK = 1; 		//clk for SENSI-BUS
		Nop();Nop();Nop();		// .5uS //pulswith approx. .5 us
		_SHT15_SCK = 0;
	}

	_SHT15_DAT = 1; 			//release DATA-line
	_SHT15_SCK = 1; 			//clk #9 for ack
	error = DATA;				//check ack (DATA will be pulled down by SHT11)
	Nop();
	_SHT15_SCK = 0;

	return (error); 			//error=1 in case of no acknowledge
}


char s_read_byte(unsigned char ack) {
	unsigned char i;
	unsigned char val = 0;
	int counter = 1;

	_SHT15_DAT = 1; 			//release DATA-line

	for(i = 0x80; i > 0; i/=2) { 
		_SHT15_SCK = 1; 		//clk for SENSI-BUS

		delay(1);

		if(DATA) {
			//putrsUSART("data high");
			val = (val | i);
		}

		_SHT15_SCK = 0;
	}
	
	_SHT15_DAT =! ack; 			//in case of "ack==1" pull down DATA-Line
	_SHT15_SCK = 1; 			//clk #9 for ack
	Nop();Nop();Nop();			// .5uS //pulswith approx. .5 us
	_SHT15_SCK = 0;
	_SHT15_DAT = 1;				//release DATA-line

	return val;
}


char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum) {
	unsigned char error = 0;

	s_transstart(); 						//transmission start
	error = s_write_byte(STATUS_REG_R); 	//send command to sensor
	*p_value = s_read_byte(ACK); 			//read status register (8-bit)
	*p_checksum = s_read_byte(noACK); 		//read checksum (8-bit)

	return error; 							//error=1 in case of no response form the sensor
}


char s_write_statusreg(unsigned char *p_value) {
	unsigned char error = 0;

	s_transstart(); 						//transmission start
	error += s_write_byte(STATUS_REG_W);	//send command to sensor
	error += s_write_byte(*p_value); 		//send value of status register

	return error; 							//error>=1 in case of no response form the sensor
}


char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode) {
	unsigned char error = 0;
	unsigned long i;
	
	s_transstart(); 											//transmission start

	switch(mode){ 												//send command to sensor
		case TEMP : error += s_write_byte(MEASURE_TEMP); break;
		case HUMI : error += s_write_byte(MEASURE_HUMI); break;
		default : break;
	}
	
	for(i = 0; i < 0xFFFFFF; i++) {
		if(DATA == 0) {
			break;					 		//wait until sensor has finished the measurement
		}
	}
	
	if(DATA) {
		error += 1; 						// or timeout (~2 sec.) is reached
	}

	*(p_value+1) = s_read_byte(ACK); 		//read the first byte (MSB)
	*(p_value) = s_read_byte(ACK); 			//read the second byte (LSB)
	*p_checksum = s_read_byte(noACK); 		//read checksum

	return error;
}


char s_softreset(void) {
	unsigned char error=0;

	s_connectionreset(); 					//reset communication
	error += s_write_byte(RESET);			//send RESET-command to sensor
	Delay10KTCYx(200);						//200mS

	return error; 							//error=1 in case of no response form the sensor
} 



void calc_sth11(float *p_humidity ,float *p_temperature) {
	const float C1=-4.0;              		// for 12 Bit
	const float C2=+0.0405;           		// for 12 Bit
	const float C3=-0.0000028;        		// for 12 Bit
	const float T1=+0.01;             		// for 14 Bit @ 5V
	const float T2=+0.00008;           		// for 14 Bit @ 5V	
	
	float rh=*p_humidity;             		// rh:      Humidity [Ticks] 12 Bit 
	float t=*p_temperature;           		// t:       Temperature [Ticks] 14 Bit
	float rh_lin;                     		// rh_lin:  Humidity linear
	float rh_true;                    		// rh_true: Temperature compensated humidity
	float t_C;                       		// t_C   :  Temperature [°C]
	
	t_C=t*0.01 - 40;                  		//calc. temperature from ticks to [°C]
	//t_C=t*0.018 - 40;                  	//calc. temperature from ticks to [°F]
	rh_lin=C3*rh*rh + C2*rh + C1;     		//calc. humidity from ticks to [%RH]
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;   	//calc. temperature compensated humidity [%RH]

	if(rh_true>100) {
		rh_true=100;       					//cut if the value is outside of
	}

	if(rh_true<0.1) {
		rh_true=0.1;       					//the physical possible range
	}
	
	*p_temperature=t_C;               		//return temperature [°C]
	*p_humidity=rh_true;              		//return humidity[%RH]
}



//Def: The dew point is the temperature to which a given parcel of air must be cooled, at constant barometric pressure, for water vapor to condense into water.
float calc_dewpoint(float h,float t) {
	float logEx,dew_point;
	logEx=0.66077+7.5*t/(237.3+t)+(log10(h)-2);
	dew_point = (logEx - 0.66077)*237.3/(0.66077+7.5-logEx);
	return dew_point;
}


void main() {
	unsigned char error,checksum;
	unsigned int temp_reading;
	unsigned int humi_reading;
	float dew_point;

	usartInit();
	Init_Sht15();
	s_connectionreset();

	putrsUSART("Starting...");

	while(1) {

		error+=s_measure((unsigned char*) &humi_reading,&checksum,HUMI);  //measure humi
		error=0;
		error+=s_measure((unsigned char*) &temp_reading,&checksum,TEMP);  //measure temperature

		if(error!=0) {
			putrsUSART("error");
			s_connectionreset();                 //in case of an error: connection reset
		}
		else {
			char buffera[8];
			char bufferb[8];
			char bufferc[8];

			float humi_val = (float)humi_reading;                   	//converts integer to float
			float temp_val_c = (float)temp_reading;                   	//converts integer to float
			float temp_val_f;
			calc_sth11(&humi_val,&temp_val_c);           				//calculate humidity, temperature

			temp_val_f = ((1.8*temp_val_c) + 32);						//convert C to F
			dew_point = calc_dewpoint(humi_val,temp_val_c); 			//calculate dew point

			//Print out temp in deg F
			ftoa (buffera, temp_val_f, 4, 1);
			putsUSART(buffera);
		    putcUSART(0xF8);		//degree symbol
			putrsUSART("F");
			putrsUSART("\n\r");

			//Print out humidity in %
			ftoa (bufferb, humi_val, 4, 1);
			putsUSART(bufferb);
			putrsUSART("%");
			putrsUSART("\n\r");

			//Print out the dew point in deg F
			dew_point = ((1.8*dew_point) + 32);  
			ftoa (bufferc, dew_point, 4, 1);
			putsUSART(bufferc);
		    putcUSART(0xF8);		//degree symbol			
			putrsUSART("F");
			putrsUSART("\n\r");

			putrsUSART("\n\r");
		}
		
		delay(2000);                  
	}
} 