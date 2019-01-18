/***  NEED to use a mosfet/bjt on pin 10 to pull down the KLIN
	Pin 7 needs to be high for lkas_active to go 1.  might need a pull down resistor
***/

#include <Arduino.h>
#include <CustomSoftwareSerial.h>

// #define Serialwrite Serial.write
#define Serialwrite customSerial->write  //on change fix setup
#define DEBUG 1

CustomSoftwareSerial* customSerial;

uint8_t lkas_off_array[][4] =  {{0x20,0x80,0xc0,0xa0},{0x00,0x80,0xc0,0xc0}};
uint8_t counterbit = 0;
uint8_t buffi = 0;
bool lkas_active = false;
uint8_t errorcount = 0;
volatile uint8_t  sendArrayFlag = 0;
int16_t rotaryCounter = 0;  // min is 0 max is 906 -- default sent to center (actual 450)
unsigned long lastLightChange = 0;
uint8_t lightOn = 0;

void putByteInNextBuff(uint8_t *msgnum, uint8_t *temp);
void sendArray(uint8_t*);
void sendLKASOffArray();
void sendLKASOnArray();
void createSerialMsg(uint8_t*, uint8_t*);
uint8_t chksm(uint16_t*);
void timeToSendSerial();
void handleRotary();
void setupTimersOld();
void checkAndRunSendArrayFlag();
void checkForRightCounterInPreCreatedMsg();



void setup() {
	Serial.begin(9600,SERIAL_8E1);
	customSerial = new CustomSoftwareSerial(9, 10, true); // rx, tx, inverse logic (default false)
	customSerial->begin(9600, CSERIAL_8E1);         // Baud rate: 9600, configuration: CSERIAL_8N1

	cli();  // no interrupts
	// setupTimer1();
	setupTimersOld();
	sei();//allow interrupts
	pinMode(7, INPUT);
	pinMode(13,OUTPUT);
	digitalWrite(13,LOW);
}

// OP to LIN2LIN data structure (it still sends 4 bytes, but the last 2 are 0x00)
// b01A0####    ####is big_steer   A = Active   first 2 bits is the byte counter
// b10A#####    ##### is little steer
void loop() {
	lkas_active = digitalRead(7);
	handleRotary();
	checkAndRunSendArrayFlag();
	if(lkas_active){
		if((millis() - lastLightChange) > 300) {
				digitalWrite(13,lightOn);
				lastLightChange = millis();
				lightOn = lightOn > 0 ? LOW : HIGH;
		}
	} else {
		digitalWrite(13,LOW);
	}

}  // end of loop

void checkAndRunSendArrayFlag(){

	if(sendArrayFlag){
		if(lkas_active){ //if lkas_active need to send the live data, if not, send
			sendLKASOnArray();
		}else {
			sendLKASOffArray();
		}
		sendArrayFlag = 0;
	}
}

void sendLKASOffArray(){
	Serialwrite(lkas_off_array[counterbit][0]);
	Serialwrite(lkas_off_array[counterbit][1]);
	Serialwrite(lkas_off_array[counterbit][2]);
	Serialwrite(lkas_off_array[counterbit][3]);
	counterbit = counterbit > 0 ? 0x00 : 0x01;
}

void sendLKASOnArray(){
	uint8_t data[4];

	data[0] = (counterbit << 5) | ((rotaryCounter >> 11) & 0x10) | ((rotaryCounter >> 5) & 0xF);
	data[1] = 0xA0 | (rotaryCounter & 0x1F);
	data[2] =  0x80;
	Serialwrite(data[0]);
	Serialwrite(data[1]);
	uint16_t total = data[0] + data[1] + data[2];
	Serialwrite(data[2]);
 	data[3] = chksm(&total);
	Serialwrite(data[3]);
	counterbit = counterbit > 0 ? 0x00 : 0x01;
}


void handleRotary(){  // min is 0 max is 906 . center is ~450
	rotaryCounter = (analogRead(A5) / 2) - 225;  //new center is 225   //new center is 0. neg is left, pos is right
}


/*** CHECKSUMS ***/

uint8_t chksm(uint16_t *input){
	uint16_t local = *input % 512;
	local = 512 - local;
	return (uint8_t)(local % 256);
}


/*** TIMERS AND INTERRUPT FUNCTS ***/

ISR(TIMER2_COMPA_vect) {sendArrayFlag = 1;}//
ISR(TIMER1_COMPA_vect){sendArrayFlag = 1;}//
ISR(TIMER0_COMPA_vect){sendArrayFlag = 1;}//

void setupTimersOld(){
	  TCCR2A = 0;// set entire TCCR2A register to 0
	  TCCR2B = 0;// same for TCCR2B
	  TCNT2  = 0;//initialize counter value to 0
	  // set compare match register for 8khz increments
	  OCR2A = 229;// = (16*10^6) / (87.4hz*1024) - 1 (must be <256)  177== 11.4ms apart 87.2 hz
	  // turn on CTC mode
	  TCCR2A |= 0b10; //(1 << WGM21);
	  // Set CS21 bit for 1024 prescaler
	  TCCR2B |= 0b111; //(1 << CS12) | (1 << CS10;
	  // enable timer compare interrupt
	  TIMSK2 |= (1 << OCIE2A);
}

void setupTimer1WebSite(){
	TCCR2A = 0; // set entire TCCR2A register to 0
	TCCR2B = 0; // same for TCCR2B
	TCNT2  = 0; // initialize counter value to 0
	// set compare match register for undefined Hz increments
	OCR2A = 0x80 ;
	// turn on CTC mode
	TCCR2B |= (1 << WGM21);
	// Set CS22, CS21 and CS20 bits for 1 prescaler
	TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

}

void setupTimer1(){
	TIMSK1 |= (1 << OCIE1A);

	  TCCR1A = 0;// set entire TCCR2A register to 0
	  TCCR1B = 0;// same for TCCR2B
	  TCNT1  = 0;//initialize counter value to 0
	  // set compare match register for 8khz increments
	  OCR1A = 48000 ;// = (16*10^6) / (87.4hz*1024) - 1 (must be <256) // mod from 3690 to half, then 66%
	  // turn on CTC mode
	  TCCR1A |= 0b10; //(1 << WGM21);
	  // Set CS21 bit for 1024 prescaler
	  TCCR1B |= (1 << CS11) | (1 << CS10);

	  // enable timer compare interrupt
}

void setupTimer2(){

	  TCCR2A = 0;// set entire TCCR2A register to 0
	  TCCR2B = 0;
	  TCNT2  = 0;//initialize counter value to 0
	  // set compare match register for 8khz increments
	  OCR2A = 236;// = (16*10^6) / (87.4hz*1024) - 1 (must be <256)
		OCR2B = 236;
		// turn on CTC mode
	  TCCR2A |= 0x2;
	  // Set CS21 bit for 1024 prescaler
	  TCCR2B |= 0b111; //(1 << CS11) | (1 << CS10);

	  // enable timer compare interrupt
	  TIMSK2 |= 0x80;
}
