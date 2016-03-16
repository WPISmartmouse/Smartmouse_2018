#include "Mouse16.h"
#include "Arduino.h"
Mouse16::Mouse16(){

}

void Mouse16::init(){


	digitalWrite(LEDR,LOW);
	digitalWrite(LEDG,LOW);
	digitalWrite(LEDB,LOW);
	digitalWrite(LEDGO,LOW);	
	digitalWrite(SDCS,LOW);
	digitalWrite(BUZZER,LOW);

	digitalWrite(IREMITTER1,LOW);
	digitalWrite(IREMITTER2,LOW);
	digitalWrite(IREMITTER3,LOW);
	digitalWrite(IREMITTER4,LOW);
	digitalWrite(IREMITTER5,LOW);
	digitalWrite(IREMITTER6,LOW);
	
	digitalWrite(VL6180EN1,LOW);
	digitalWrite(VL6180EN2,LOW);
	digitalWrite(VL6180EN3,LOW);
	digitalWrite(VL6180EN4,LOW);
	digitalWrite(VL6180EN5,LOW);
	digitalWrite(VL6180EN6,LOW);

	digitalWrite(MOTOR1A,LOW);
	digitalWrite(MOTOR1B,LOW);
	digitalWrite(MOTOR2A,LOW);
	digitalWrite(MOTOR2B,LOW);
	digitalWrite(MOTORDIR1,LOW);
	digitalWrite(MOTORDIR2,LOW);		

	pinMode(IREMITTER1,OUTPUT);
	pinMode(IREMITTER2,OUTPUT);
	pinMode(IREMITTER3,OUTPUT);
	pinMode(IREMITTER4,OUTPUT);
	pinMode(IREMITTER5,OUTPUT);
	pinMode(IREMITTER6,OUTPUT);
	pinMode(IRRECEIVER1,INPUT);
	pinMode(IRRECEIVER2,INPUT);
	pinMode(IRRECEIVER3,INPUT);
	pinMode(IRRECEIVER4,INPUT);
	pinMode(IRRECEIVER5,INPUT);
	pinMode(IRRECEIVER6,INPUT);

	pinMode(BATTERYSENSE,INPUT);

	pinMode(VL6180EN1,OUTPUT);
	pinMode(VL6180EN2,OUTPUT);
	pinMode(VL6180EN3,OUTPUT);
	pinMode(VL6180EN4,OUTPUT);
	pinMode(VL6180EN5,OUTPUT);
	pinMode(VL6180EN6,OUTPUT);	

	pinMode(ENCODER1A,INPUT);
	pinMode(ENCODER1B,INPUT);
	pinMode(ENCODER2A,INPUT);
	pinMode(ENCODER2B,INPUT);

	pinMode(MOTOR1A,OUTPUT);
	pinMode(MOTOR1B,OUTPUT);
	pinMode(MOTOR2A,OUTPUT);
	pinMode(MOTOR2B,OUTPUT);		

	pinMode(MOTORDIR1,OUTPUT);
	pinMode(MOTORDIR2,OUTPUT);	

	pinMode(LEDR,OUTPUT);
	pinMode(LEDG,OUTPUT);
	pinMode(LEDB,OUTPUT);
	pinMode(LEDGO,OUTPUT);
	
	pinMode(BUTTONGO,INPUT_PULLUP);
	pinMode(BUTTON1,INPUT_PULLUP);
	pinMode(BUTTON2,INPUT_PULLUP);
	pinMode(SDCARDDETECT,INPUT_PULLUP);

	pinMode(SDCS,OUTPUT);
	pinMode(BUZZER,OUTPUT);


	digitalWrite(LEDR,LOW);
	digitalWrite(LEDG,LOW);
	digitalWrite(LEDB,LOW);
	digitalWrite(LEDGO,LOW);	
	digitalWrite(SDCS,LOW);
	digitalWrite(BUZZER,LOW);

	digitalWrite(IREMITTER1,LOW);
	digitalWrite(IREMITTER2,LOW);
	digitalWrite(IREMITTER3,LOW);
	digitalWrite(IREMITTER4,LOW);
	digitalWrite(IREMITTER5,LOW);
	digitalWrite(IREMITTER6,LOW);
	
	digitalWrite(VL6180EN1,LOW);
	digitalWrite(VL6180EN2,LOW);
	digitalWrite(VL6180EN3,LOW);
	digitalWrite(VL6180EN4,LOW);
	digitalWrite(VL6180EN5,LOW);
	digitalWrite(VL6180EN6,LOW);

	digitalWrite(MOTOR1A,LOW);
	digitalWrite(MOTOR1B,LOW);
	digitalWrite(MOTOR2A,LOW);
	digitalWrite(MOTOR2B,LOW);
	digitalWrite(MOTORDIR1,LOW);
	digitalWrite(MOTORDIR2,LOW);		

}

