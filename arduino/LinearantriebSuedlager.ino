#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers

#define OUTER_END_POTI  (950)
#define INNER_END_POTI  (300)

const int guiding_input_north = 6;
const int guiding_input_south = 7;

const int dir_pin = 2;
const int step_pin = 9;
const int enable_pin = 3;

const int m1_pin = 10;
const int m2_pin = 11;
const int m3_pin = 12;

const int guiding_speed = 500/10*8;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);

  pinMode(guiding_input_north, INPUT_PULLUP);
  pinMode(guiding_input_south, INPUT_PULLUP);

  //Pin hack as GND and VCC
  pinMode(A4, OUTPUT); 
  digitalWrite(A4, LOW);
  pinMode(A3, OUTPUT); 
  digitalWrite(A3, HIGH);
  
  pinMode(m1_pin, OUTPUT); 
  pinMode(m2_pin, OUTPUT); 
  pinMode(m3_pin, OUTPUT); 

  digitalWrite(m1_pin, HIGH);
  digitalWrite(m2_pin, LOW);
  digitalWrite(m3_pin, LOW);

  engine.init();
  stepper = engine.stepperConnectToPin(step_pin);
  if (stepper) {
    stepper->setDirectionPin(dir_pin);
    stepper->setEnablePin(enable_pin);
    stepper->setAutoEnable(true);
    //stepper->enableOutputs();

    stepper->setSpeedInHz(150);       // 500 steps/s
    stepper->setAcceleration(2000);    // 100 steps/s²
    //stepper->move(1000);
  }
  //while(1);
}

unsigned long currentTime = 0, previousTime = 0;
void loop() {
  int manual_pot = analogRead(A1);
  int length_pot = analogRead(A0);

  //Serial.print(manual_pot);
  //Serial.print(",");
  Serial.println(length_pot);

  currentTime = millis();

  //use this range to start star speed controlled movment of dc motor
  if(manual_pot > (1023 - 255) )
  {
    //if( (currentTime - previousTime) >= 500)
    if(length_pot < OUTER_END_POTI)
    {
      //half step
      digitalWrite(m1_pin, HIGH);
      digitalWrite(m2_pin, LOW);
      digitalWrite(m3_pin, LOW);

      stepper->setAcceleration(2000);    // 100 steps/s²
      stepper->setSpeedInHz( (manual_pot - (1023 - 255)) * 4);
      stepper->applySpeedAcceleration();
      stepper->runForward();

      previousTime = currentTime;
    }
    else
    {
      if(length_pot > (OUTER_END_POTI + 10))
        stepper->stopMove();
    }
  }
  //move plattform back in variable speed
  else if(manual_pot < (255))
  {
    //if( (currentTime - previousTime) >= 500)
    if(length_pot > INNER_END_POTI)
    {
      //half step
      digitalWrite(m1_pin, HIGH);
      digitalWrite(m2_pin, LOW);
      digitalWrite(m3_pin, LOW);
      stepper->setAcceleration(2000);    // 100 steps/s²
      stepper->setSpeedInHz( (255 - manual_pot) * 4);
      stepper->applySpeedAcceleration();
      stepper->runBackward();

      previousTime = currentTime;
    }
    else
    {
      if(length_pot < (INNER_END_POTI - 10))
        stepper->stopMove();
    }
  }
  else
  {
    if(digitalRead(guiding_input_north) == LOW && digitalRead(guiding_input_south) == HIGH && length_pot > INNER_END_POTI)
    {
      //sixtheenth step - lower vibration
      digitalWrite(m1_pin, HIGH);
      digitalWrite(m2_pin, HIGH);
      digitalWrite(m3_pin, HIGH);
      stepper->setAcceleration(4000);
      stepper->setSpeedInHz(guiding_speed);
      stepper->applySpeedAcceleration();
      stepper->runBackward();
    }
    else if(digitalRead(guiding_input_south) == LOW && digitalRead(guiding_input_north) == HIGH && length_pot < OUTER_END_POTI)
    {
      //sixtheenth step - lower vibration
      digitalWrite(m1_pin, HIGH);
      digitalWrite(m2_pin, HIGH);
      digitalWrite(m3_pin, HIGH);
      stepper->setAcceleration(4000);
      stepper->setSpeedInHz(guiding_speed);
      stepper->applySpeedAcceleration();
      stepper->runForward();
    }
    else
    {
      stepper->stopMove();
    }
    
  }
}
