#include <Arduino.h>
#include <Wire.h>
#include "ServoEasing.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define __DELAY_BACKWARD_COMPATIBLE__

//#define F_CPU 16000000UL
//#define __AVR_ATmega328P__


const char SERVO1_PIN = 9;
const char SERVO2_PIN = 10;
const char SERVO3_PIN = 11;
bool SERVOS_ATTACHED = true;
unsigned long SERVOS_TIMESTAMP_LAST_ACTIVITY = 0;
unsigned long SERVOS_ATTACH_DELAY = 10000;

const char SERVO_RAW_STEP = 12;
const char SERVO_DEGREE_PER_SECOND = 33;

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;


const char BLOWER_PIN_CONTROL = 12;
const char BLOWER_PIN_ZERO = 2;
const char BLOWER_DELAY_FASTEST = 300;
const char BLOWER_DELAY_SLOWEST = 600;
const char BLOWER_DELAY_CHANGE_PER_SECOND = 2;
const char BLOWER_RAW_STEP = 22;

bool BLOWER_ON = false;
short BLOWER_DELAY_CURRENT = 0;
short BLOWER_DELAY_TARGET = 0;

byte BLOWER_COUNTER_ZERO = 0;

const char I2C_DRIVER_ADDRESS = 0x12;

byte x = 0;


bool experiment = true;


void delayed( int j ) {
  for( int i = 0; i<j ; ++i) {
    _delay_ms(0.01);
  }
}

void turnOnTriac() {
      _delay_ms(0.4);
      PORTB |= (1 << PORTB5);
      PORTB |= (1 << PORTB4);
      _delay_ms(3);
      PORTB &= (0 << PORTB5);
      PORTB &= (0 << PORTB4);
}

ISR( INT0_vect ) {
//  if( BLOWER_COUNTER_ZERO++ == 100 ) {
//    BLOWER_COUNTER_ZERO = 0;
//    
//    if( BLOWER_DELAY_CURRENT != BLOWER_DELAY_TARGET ) {
//      if( BLOWER_DELAY_CURRENT < BLOWER_DELAY_TARGET ) {
//        if( BLOWER_DELAY_CURRENT == 0 ) {
//          BLOWER_DELAY_CURRENT = BLOWER_DELAY_SLOWEST;
//        } else {
//          BLOWER_DELAY_CURRENT += BLOWER_DELAY_CHANGE_PER_SECOND;  
//        }
//      } else {
//        if( BLOWER_DELAY_CURRENT <= BLOWER_DELAY_SLOWEST ) {
//          BLOWER_DELAY_CURRENT = 0;
//        } else {
//          BLOWER_DELAY_CURRENT -= BLOWER_DELAY_CHANGE_PER_SECOND;  
//        }
//      }
//    }
//  }
//  
//  if( BLOWER_DELAY_CURRENT >= BLOWER_DELAY_SLOWEST ) {
//    delayed(BLOWER_DELAY_CURRENT);
//    turnOnTriac();
//  }
}


ISR (TIMER2_COMPA_vect)
{
  if(experiment) {
    PORTB |= (1 << PORTB5);
    experiment = false;
  } else {
    PORTB &= (0 << PORTB5);
    experiment = true;
  }
    // action to be done every 250 usec
}

void servosAttach( bool attach ) {
  if( attach ) {
    Servo1.attach(SERVO1_PIN);
    Servo2.attach(SERVO2_PIN);
    Servo3.attach(SERVO3_PIN);
    SERVOS_ATTACHED = true;
    SERVOS_TIMESTAMP_LAST_ACTIVITY = millis();
  } else {
    Servo1.detach();
    Servo2.detach();
    Servo3.detach();
    SERVOS_ATTACHED = false;
  }
}

void servosUpdateLastActivity() {
  SERVOS_TIMESTAMP_LAST_ACTIVITY = millis();
}

void servosCheckAttachDelay() {
  if( SERVOS_ATTACHED && ( millis() > SERVOS_TIMESTAMP_LAST_ACTIVITY + SERVOS_ATTACH_DELAY ) ) {
    servosAttach(false);
  }
}

void setup() {

    DDRB |= (1 << DDB5);
    PORTB &= (0 << PORTB5);
    DDRB |= (1 << DDB4);
    PORTB &= (0 << PORTB4);

    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin

    PORTD |= (1 << PORTD2);    // turn On the Pull-up
 
    EICRA |= (1 << ISC00);    
    EICRA |= (1 << ISC01);    // set external INT0 interrupt to catch only rising edge
    
    EIMSK |= (1 << INT0);     // Turns on INT0



    //timer2 experiment start
    OCR2A = 62;

    TCCR2A |= (1 << WGM21);
    // Set to CTC Mode

    TIMSK2 |= (1 << OCIE2A);
    //Set interrupt on compare match

    TCCR2B |= (1 << CS21);
    // set prescaler to 64 and starts PWM

    //timer2 experiment stop

    sei();                    // turn on interrupts
    
  
  Wire.begin(I2C_DRIVER_ADDRESS);
  Wire.onReceive(receiveEvent);
  
  Serial.begin(115200);
  
  Servo1.attach(SERVO1_PIN);
  Servo1.setEasingType(EASE_LINEAR);
  Servo2.attach(SERVO2_PIN);
  Servo2.setEasingType(EASE_LINEAR);
  Servo3.attach(SERVO3_PIN);
  Servo3.setEasingType(EASE_LINEAR);
  
  Servo1.startEaseTo(0, SERVO_DEGREE_PER_SECOND);
  Servo2.startEaseTo(0, SERVO_DEGREE_PER_SECOND);
  Servo3.startEaseTo(0, SERVO_DEGREE_PER_SECOND);
  delay(2000);
//  Servo1.write(180);
//  Servo2.write(180);
//  Servo3.write(180);
//  delay(1700);
//  digitalWrite(BLOWER_PIN_CONTROL, HIGH);
//  delay(3000);
//  digitalWrite(BLOWER_PIN_CONTROL, LOW);
//  delay(5000);
//  Servo1.write(0);
//  Servo2.write(0);
//  Servo3.write(0);
//  delay(2000);
  servosAttach(false);
}

void loop() {
  servosCheckAttachDelay();
}

void receiveEvent(int howMany) {
  x = Wire.read();
  Serial.println(x, DEC);
  Serial.println(x, BIN);

  byte reserved =     0b00000000;
  byte device_id =    0b00000000;
  byte device_value = 0b00000000;

  reserved = x & 0b11000000;
  reserved = x >> 6;
  device_id = x & 0b00110000;
  device_id = x >> 4;
  device_value = x & 0b00001111;

  Serial.print("reserved: ");
  Serial.println(reserved, BIN);
  Serial.print("device_id: ");
  Serial.println(device_id, BIN);
  Serial.println(device_id, DEC);
  Serial.print("device_value: ");
  Serial.println(device_value, BIN);


  if( device_id == 0b00 ) {
    // blower
    if( device_value == 0 ) {
      BLOWER_DELAY_TARGET = 0;
    } else {
      BLOWER_DELAY_TARGET = BLOWER_DELAY_SLOWEST - ((device_value-1) * BLOWER_RAW_STEP );
    }
    
  } else {
    // servos
    servosAttach(true);
    if( device_id == 0b01 ) {
      Servo1.startEaseTo(device_value * SERVO_RAW_STEP, SERVO_DEGREE_PER_SECOND);
    }
    if( device_id == 0b10 ) {
      Servo2.startEaseTo(device_value * SERVO_RAW_STEP, SERVO_DEGREE_PER_SECOND);
    }
    if( device_id == 0b11 ) {
      Servo3.startEaseTo(device_value * SERVO_RAW_STEP, SERVO_DEGREE_PER_SECOND);
    }
  }
}
