//Inkluderer nødvendige biblioteker for å styre Zumo
#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors; //Deklarerer motorer
Zumo32U4LineSensors lineSensors; //Deklarerer linjesensorer
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA; //Deklarerer knapp A
Zumo32U4LCD display; //Deklarerer skjerm

#define numberOfSensors 5 //Definerer antallet sensorer
unsigned int lineSensorValues[numberOfSensors];  //Lager en liste for sensorverdier

int wanted_lineSensor_val = 1500;
int max_speed = 300;
int error;
int last_error;
int time_since_on_track;
int left_speed;
int right_speed;

void calibrateSensors() { //Oppretter funksjon for å kalibrere sensorer
  int now = millis(); //Bruker funksjonen millis() til å lagre tiden
  motors.setSpeeds(-200, 200); //Roterer bilen om egen akse
  while (millis() - now < 400) { //I et sekund...
    lineSensors.calibrate(); //...oppdaterer sensorene mens bilen roterer
  }
  delay(300); //Vent 200ms for at motorene ikke skal snu retning momentant
  now = millis(); //Tilbakestiller tiden
  motors.setSpeeds(200, -200); //Roterer bilen motsatt vei...
  while (millis() - now < 900) { //...dobbelt så lenge...
    lineSensors.calibrate(); //...mens sensorene oppdateres
  }
  delay(300); //Vent 200ms for at motorene ikke skal snu retning momentant
  now = millis();//Tilbakestiller tiden
  motors.setSpeeds(-200, 200); //Roterer bilen...
  while (millis() - now < 400) { //...tilbake til startposisjon...
    lineSensors.calibrate(); //...men sensorene oppdateres
  }
  motors.setSpeeds(0, 0); //Skru av motorene
}

void no_tape()
{
  lineSensors.readLine(lineSensorValues);

  while(lineSensorValues[0] < 150 && lineSensorValues[1] < 150
    && lineSensorValues[2] < 150 && lineSensorValues[3] < 150)
  { 
    motors.setSpeeds(max_speed, max_speed);
    
    if(millis() - time_since_on_track > 600)
    {
      motors.setSpeeds(0,0);
      delay(200);
      
      while(lineSensorValues[0] < 600 && lineSensorValues[3] < 600)
      {
        motors.setSpeeds(-max_speed, -max_speed);
        lineSensors.readLine(lineSensorValues);
      }
      
      motors.setSpeeds(0,0);
      delay(250);

      if(lineSensorValues[0] > 400)
      {
        motors.setSpeeds(-100, 300);
        delay(400);
      }
        
      else if(lineSensorValues[3] > 400)
      {
        motors.setSpeeds(300, -100);
        delay(400);
      }

      motors.setSpeeds(0,0);
      lineSensors.readLine(lineSensorValues);
      break;
    }
    
    lineSensors.readLine(lineSensorValues);
  }
}

void line_follower()
{
  int read_value = lineSensors.readLine(lineSensorValues);

  error = read_value - wanted_lineSensor_val;

  int speed_difference = (error * 1) + 0.5 * (error - last_error);

  left_speed = max_speed + speed_difference;
  right_speed = max_speed - speed_difference;

  left_speed = constrain(left_speed, 0, max_speed);
  right_speed = constrain(right_speed, 0, max_speed);
}

void left_turn()
{
  lineSensors.readLine(lineSensorValues);

  motors.setSpeeds(max_speed, max_speed);
    delay(100);
    motors.setSpeeds(0,0);
    delay(100);
    lineSensors.readLine(lineSensorValues);

    if(lineSensorValues[0] < 150 && lineSensorValues[1] < 150
    && lineSensorValues[2] < 150 && lineSensorValues[3] < 150)
    {
      while(lineSensorValues[2] < 800)
      {
        motors.setSpeeds(-250,250);
        lineSensors.readLine(lineSensorValues);
      }
    }

    else
    {
      motors.setSpeeds(max_speed, max_speed);
      delay(10);
    }
}

void right_turn()
{
  motors.setSpeeds(max_speed, max_speed);
    delay(100);
    motors.setSpeeds(0,0);
    delay(100);
    lineSensors.readLine(lineSensorValues);
    
    if(lineSensorValues[0] < 150 && lineSensorValues[1] < 150
    && lineSensorValues[2] < 150 && lineSensorValues[3] < 150)
    {
      while(lineSensorValues[1] < 800)
      {
        motors.setSpeeds(250,-250);
        lineSensors.readLine(lineSensorValues);
      }
    }

    else
    {
      motors.setSpeeds(max_speed, max_speed);
      delay(10);
    }
}

void right_prox_stop()
{
  proxSensors.read();

  //if(proxSensors.countsRightWithRightLeds() >= 6)
  //{
    while(proxSensors.countsRightWithRightLeds() >= 6)
    {
      motors.setSpeeds(0, 0);
      proxSensors.read();
    }
  //}
}

void setup() {

  Serial1.begin(9600);

  uint8_t lineSensorPins[] = { SENSOR_DOWN1, SENSOR_DOWN2, SENSOR_DOWN3, SENSOR_DOWN5 };
  lineSensors.init(lineSensorPins, sizeof(lineSensorPins));
  uint8_t proxSensorPins[] = { SENSOR_FRONT, SENSOR_RIGHT };
  proxSensors.init(proxSensorPins, sizeof(proxSensorPins));

  //Printer melding på skjerm:
  display.print("Press A");
  display.gotoXY(0, 1);
  display.print("To calib.");

  buttonA.waitForButton(); //Vent på knappetrykk

  calibrateSensors(); //Kjør funksjonen for å kalibrere sensorer

  //Printer ny melding på skjerm:
  display.clear();
  display.print("Ready!");
  display.gotoXY(0, 1);
  display.print("Press A");

  buttonA.waitForButton(); //Vent på knappetrykk

}

void loop() {

  line_follower();

  if(lineSensorValues[0] < 150 && lineSensorValues[1] < 150
    && lineSensorValues[2] < 150 && lineSensorValues[3] < 150)
  {
    no_tape();
  }

  else if(lineSensorValues[0] > 600 && lineSensorValues[1] > 600
  && lineSensorValues[2] > 600 && lineSensorValues[3] > 600)
  {
    motors.setSpeeds(max_speed, max_speed);
  }
  
  else if(lineSensorValues[0] > 700)
  {
    left_turn();
  }

  else if(lineSensorValues[3] > 700)
  {
    right_turn();
  }

  else
  {
    motors.setSpeeds(left_speed, right_speed);
  }

  last_error = error;
  time_since_on_track = millis();

  right_prox_stop();

}