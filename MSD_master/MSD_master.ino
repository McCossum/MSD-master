#include <Wire.h>
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *motor = AFMS.getStepper(200, 1);
#define switch1 35
#define switch2 37
#define switch3 39
#define greenLight 29
#define yellowLight 27
#define redLight 25
#define agitatorMotor 31
#define lightSensor1 A14
#define lightSensor2 A13
#define lightSensor3 A12
#define lightSensor4 A11
#define levelSensor A10
float rawRange = 1024; // 3.3v
float logRange = 5.0; // 3.3v = 10^5 lux
int swValue = 0;
int ltValue = 100;
int ltValueLast = 100;
int i = 0;
int n = 0;

void setup() {

  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(switch3, INPUT);
  pinMode(lightSensor1, INPUT);
  pinMode(lightSensor2, INPUT);
  pinMode(lightSensor3, INPUT);
  pinMode(lightSensor4, INPUT);
  pinMode(levelSensor, INPUT);
  
  pinMode(greenLight, OUTPUT);
  pinMode(yellowLight, OUTPUT);
  pinMode(redLight, OUTPUT);
  pinMode(agitatorMotor, OUTPUT);
  
  AFMS.begin(); //Create with the default frequency 1.6KHz
  motor->setSpeed(200); //RPM
  digitalWrite(greenLight, HIGH);
  Serial.begin(9600);
}

void loop() {
  swValue = digitalRead(switch1); 
  Serial.print("Switch one = ");
  Serial.println(swValue);
  if (swValue == 1) {
    digitalWrite(greenLight, HIGH);
    digitalWrite(yellowLight, LOW);
    motor->step(1, BACKWARD, DOUBLE);
    ltValue = analogRead(lightSensor2);     
    Serial.print("Raw = ");
    Serial.println(ltValue);
    if (ltValue >= 350 && ltValueLast >= 350) {
      digitalWrite(agitatorMotor, HIGH);
      delay(500);  // waits for 0.25 second
      if (i >= 4) {
        if (n == 0) {
          digitalWrite(agitatorMotor, LOW);
          delay(500);  // waits for 0.25 second
          n = 1;
        } else {
          n = 0;
        }
      }
      if (i >= 20) {
        digitalWrite(redLight, HIGH);
        digitalWrite(greenLight, LOW);
        digitalWrite(agitatorMotor, LOW);
        motor->release();
        break
      }
      i += 1;
    } else {
      digitalWrite(agitatorMotor, LOW);
    }
    ltValueLast = ltValue;
  } else {
    digitalWrite(agitatorMotor, LOW);
    motor->release();
    digitalWrite(greenLight, LOW);
    digitalWrite(yellowLight, HIGH);
  }
}

