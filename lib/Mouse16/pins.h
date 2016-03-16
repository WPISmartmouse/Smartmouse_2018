#ifndef PINS_H
#define PINS_H

#define IREMITTER1 A1
#define IREMITTER2 A4
#define IREMITTER3 A6
#define IREMITTER4 A9
#define IREMITTER5 A11
#define IREMITTER6 CANRX

#define IRRECEIVER1 A2
#define IRRECEIVER2 A5
#define IRRECEIVER3 A7
#define IRRECEIVER4 A10
#define IRRECEIVER5 DAC1
#define IRRECEIVER6 A3 // wired to non adc pin. this does not work.

#define BATTERYSENSE A0

#define VL6180EN1 22
#define VL6180EN2 23
#define VL6180EN3 24
#define VL6180EN4 25
#define VL6180EN5 26
#define VL6180EN6 27

#define ENCODER1A 3
#define ENCODER1B 29
#define ENCODER2A 2
#define ENCODER2B 28

#define MOTOR1A 9 //PWML4 PC21
#define MOTOR1B 11 //
#define MOTOR2A 10
#define MOTOR2B 12

#define MOTORDIR1 30
#define MOTORDIR2 31

#define LEDGO 13
#define LEDR 52
#define LEDG 41
#define LEDB 51
#define BUZZER 46

#define BUTTONGO 44
#define BUTTON1 48
#define BUTTON2 50

#define SDCS 4
#define SDCARDDETECT 42

#endif
