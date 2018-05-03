#include <PinChangeInt.h>
#include <PID_v1.h>

int encoder_pin=2;
int in3=3;
int in4=4;

double Kp, Ki, Kd;
double input=0, output=0, setpoint=0;

unsigned int rpm = 0;

float velocity = 0;

volatile byte pulses = 0; //kod njega stoji voltaile long encoderPos=0, pa sam pretpostavila da su to ovdje impulsi

unsigned long timeold = 0;  
unsigned int pulsesperturn = 8; 

const int wheel_diameter = 31;
static volatile unsigned long debounce = 0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(encoder_pin, INPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  attachInterrupt(0, counter, RISING);               // azuriranje pozicije enkodera, u ovom slučaju senzora
                                                     // pa sam mislila da odgovara pozivanje na counter koju si ti definisao
  pulses = 0;
  rpm = 0;
  timeold = 0;

  TCCR1B = TCCR1B & 0b11111000 | 1;                   // postaviti 31KHz PWM kako bi se sprijecio sum motora
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Serial.begin (115200);                              // za uklanjanje greške :/
}

void loop() {
    if (millis() - timeold >= 1000) {

    noInterrupts(); 

    setpoint = analogRead(0) * 5;                       // ovo dio ne kontam
    input = pulses ;                                // podatak koji se očitava sa senzora
  
    rpm = (60 * 1000 / pulsesperturn ) / (millis() - timeold) * pulses;
    velocity = rpm * 3.1416 * wheel_diameter * 60 / 1000000; 
    timeold = millis();
  
    Serial.print(millis() / 1000); Serial.print("       ");
    Serial.print(rpm, DEC); Serial.print("   ");
    Serial.print(pulses, DEC); Serial.print("     ");
    Serial.println(velocity, 2);
  
  myPID.Compute(); 
  
  interrupts();
   }
}

void counter() {
  if (  digitalRead (encoder_pin) && (micros() - debounce > 500) && digitalRead (encoder_pin) ) {
    debounce = micros();
    pulses++;
  }
  else ;
}


