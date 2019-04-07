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
#define levelSensor 41
float rawRange = 1024; // 3.3v
float logRange = 5.0; // 3.3v = 10^5 lux
int swValue = 0;
int rodValue = 0;
int diamValue = 0;
int lvValue = 0;
int ltValue = 1000;
int ltValueLast = 1000;
int i = 0;
int n = 0;
int t = 0;
int GO = 1;

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

  Serial.begin(250000);

  Serial.println("Waiting 5 sec for rod size and material switches");
  digitalWrite(yellowLight, HIGH);
  delay(5000);  // waits for 5 second

  rodValue = digitalRead(switch2); 
  Serial.print("rodValue = ");
  Serial.println(rodValue);
  diamValue = digitalRead(switch3); 
  Serial.print("diamValue = ");
  Serial.println(diamValue);
  if (rodValue == 1) {
    if (diamValue == 1) {
      motor->setSpeed(75); //RPM
    } else {
      motor->setSpeed(150); //RPM
    }
  } else {
    if (diamValue == 1) {
      motor->setSpeed(120); //RPM
    } else {
      motor->setSpeed(200); //RPM
    }
  }

  digitalWrite(yellowLight, LOW);
  
  AFMS.begin(); //Create with the default frequency 1.6KHz
  digitalWrite(greenLight, HIGH);
}

void loop() {
  if(GO == 1) {
    swValue = digitalRead(switch1); 
    Serial.print("Switch one = ");
    Serial.println(swValue);
    if (swValue == 1) {
      digitalWrite(greenLight, HIGH);
      digitalWrite(yellowLight, LOW);
      digitalWrite(redLight, LOW);
      lvValue = digitalRead(levelSensor); 
      Serial.print("Level sensor = ");
      Serial.println(lvValue);
      if (lvValue == 1) {
        motor->step(1, BACKWARD, DOUBLE);
        ltValue = analogRead(lightSensor2);     
        Serial.print("Light = ");
        Serial.println(ltValue);
        if (ltValue >= 50 && ltValueLast >= 50) {
          if (i >= 20) { 
            digitalWrite(agitatorMotor, LOW);
            motor->release();
            digitalWrite(greenLight, LOW);
            digitalWrite(yellowLight, LOW);
            digitalWrite(redLight, HIGH);
            Serial.println("                    Stopped (agitator running too long)");
            GO = 0;
          } else {
            if (i >= 4) {
              Serial.println("                    Freakout (agititor running a while)");
              if (n == 0) {
                digitalWrite(agitatorMotor, LOW);
                delay(500);  // waits for 0.5 second
                n = 1;
              } else {
                n = 0;
              }
            }
            digitalWrite(agitatorMotor, HIGH);
            delay(500);  // waits for 0.5 second
          }
          i += 1;
          Serial.print("                    Time agititor running:");
          Serial.println(t);
        } else {
          digitalWrite(agitatorMotor, LOW);
        }
        if (t >= 2000) {
          GO = 0; //                      Hash out for test code
        } else {
          t += 1;
        }
        Serial.print("                    Time motor running:");
        Serial.println(t);
        ltValueLast = ltValue;
      } else {
        motor->release();
        digitalWrite(agitatorMotor, LOW);
      }
    } else {
      digitalWrite(agitatorMotor, LOW);
      motor->release();
      digitalWrite(greenLight, LOW);
      //digitalWrite(yellowLight, HIGH);
      digitalWrite(redLight, LOW);
      Serial.println("                    Stalled");
    }
  } else {
    digitalWrite(agitatorMotor, LOW);
    motor->release();
    digitalWrite(greenLight, LOW);
    digitalWrite(yellowLight, LOW);
    digitalWrite(redLight, HIGH);
    Serial.println("                      Stopped (motor running too long)");
  }
}

