//Inkluderer nødvendige biblioteker for å styre Zumo
#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors; //Deklarerer motorer
Zumo32U4LineSensors lineSensors; //Deklarerer linjesensorer
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA; //Deklarerer knapp A
Zumo32U4LCD display; //Deklarerer skjerm
Zumo32U4Encoders encoders;

#define numberOfSensors 5 //Definerer antallet sensorer
unsigned int lineSensorValues[numberOfSensors];  //Lager en liste for sensorverdier

//Variabler linjefølger:
int wanted_lineSensor_val = 1500;
int max_speed = 300;
int error;
int last_error;
int time_since_on_track;
int left_speed;
int right_speed;

//Variabler speedometer:
float total_distance;
unsigned long last_speed_update;
unsigned long last_minute_update;
float prev_minute_distance;
float max_speedometer;
float timeOverSeventy;

//Variabler SW-batteri:
float max_battery_capacity = 0.5; //mAh
float current_battery_capacity = 0.5;

void calibrateSensors() { //Oppretter funksjon for å kalibrere sensorer
  int now = millis(); //Bruker funksjonen millis() til å lagre tiden
  motors.setSpeeds(-200, 200); //Roterer bilen om egen akse
  while (millis() - now < 400) { //I et sekund...
    lineSensors.calibrate(); //...oppdaterer sensorene mens bilen roterer
  }
  delay(300); //Vent 200ms for at motorene ikke skal snu retning momentant
  now = millis(); //Tilbakestiller tiden
  motors.setSpeeds(200, -200); //Roterer bilen motsatt vei...
  while (millis() - now < 800) { //...dobbelt så lenge...
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

  while(lineSensorValues[0] < 100 && lineSensorValues[1] < 100
    && lineSensorValues[2] < 100 && lineSensorValues[3] < 100)
  { 
    motors.setSpeeds(max_speed, max_speed);
    
    if(millis() - time_since_on_track > 500)
    {
      motors.setSpeeds(0,0);
      delay(200);
      
      while(lineSensorValues[0] < 800 && lineSensorValues[3] < 800)
      {
        motors.setSpeeds(-max_speed, -max_speed);
        lineSensors.readLine(lineSensorValues);
      }
      
      motors.setSpeeds(0,0);
      delay(250);

      if(lineSensorValues[0] > 500)
      {
        motors.setSpeeds(-100, 300);
        delay(400);
      }
        
      else if(lineSensorValues[3] > 500)
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

  int speed_difference = (error * 0.5) + 0.5 * (error - last_error);

  left_speed = max_speed + speed_difference;
  right_speed = max_speed - speed_difference;

  left_speed = constrain(left_speed, 0, max_speed);
  right_speed = constrain(right_speed, 0, max_speed);

  motors.setSpeeds(left_speed, right_speed);
}

void left_turn()
{
  lineSensors.readLine(lineSensorValues);

  motors.setSpeeds(max_speed, max_speed);
    delay(50);
    motors.setSpeeds(0,0);
    delay(50);
    lineSensors.readLine(lineSensorValues);

    if(lineSensorValues[0] < 100 && lineSensorValues[1] < 100
    && lineSensorValues[2] < 100 && lineSensorValues[3] < 100)
    {
      while(lineSensorValues[2] < 800)
      {
        motors.setSpeeds(-250,250);
        lineSensors.readLine(lineSensorValues);
      }
    }
}

void right_turn()
{
  motors.setSpeeds(max_speed, max_speed);
    delay(50);
    motors.setSpeeds(0,0);
    delay(50);
    lineSensors.readLine(lineSensorValues);
    
    if(lineSensorValues[0] < 100 && lineSensorValues[1] < 100
    && lineSensorValues[2] < 100 && lineSensorValues[3] < 100)
    {
      while(lineSensorValues[1] < 800)
      {
        motors.setSpeeds(250,-250);
        lineSensors.readLine(lineSensorValues);
      }
    }
}

void right_prox_stop()
{
  proxSensors.read();

  while(proxSensors.countsRightWithRightLeds() >= 6)
  {
    motors.setSpeeds(0, 0);
    proxSensors.read();
  }
}

void speedometer()
{
  if (millis() - last_speed_update > 100)
  {
    float leftCounts = encoders.getCountsAndResetLeft();
    float rightCounts = encoders.getCountsAndResetRight();
    float distance = (leftCounts + rightCounts) / (2 * 77); //cm
    total_distance += distance;
    float current_speed = distance / 0.1;

    float power_consumption = (2.0 * current_speed + distance) * (1.0 / 36000.0); //mAh
    current_battery_capacity -= power_consumption;

    if(current_battery_capacity < 10)
    {
      //Varsling
    }

    if(current_battery_capacity <= 0)
    {
      while (current_battery_capacity <= 0)
      {
        motors.setSpeeds(0, 0);
      }
      
    }

    if (current_speed > max_speedometer)
    {
      max_speedometer = current_speed;
    }

    if (current_speed > 36)
    {
      timeOverSeventy += 0.1;
    }

    display.clear();
    display.print(round(current_speed));
    display.print(" cm/s");
    display.gotoXY(0, 1);
    display.print(current_battery_capacity);
    //display.print(total_distance);
    display.print("cm");

    last_speed_update = millis();

  }

  if (millis() - last_minute_update > 60000)
  {
    float minute_distance = total_distance - prev_minute_distance;
    float avr_speed = minute_distance / 60;

    max_speedometer = 0;
    timeOverSeventy = 0;
    last_minute_update = millis();
    prev_minute_distance = total_distance;
  }
}

transmit_to_esp()
{
  Serial1.println("current_battery_capacity ");
  Serial1.print(current_battery_capacity);
  Serial1.println(" ");
  Serial1.println("total_distance ");
  Serial1.print(total_distance);
  Serial1.println(" ");
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

  if(lineSensorValues[0] < 100 && lineSensorValues[1] < 100
    && lineSensorValues[2] < 100 && lineSensorValues[3] < 100)
  {
    no_tape();
  }

  else if(lineSensorValues[0] > 800 && lineSensorValues[1] > 800
  && lineSensorValues[2] > 800 && lineSensorValues[3] > 800)
  {
    motors.setSpeeds(max_speed, max_speed);
  }
  
  else if(lineSensorValues[0] > 900)
  {
    left_turn();
  }

  else if(lineSensorValues[3] > 900)
  {
    right_turn();
  }

  /*else
  {
    motors.setSpeeds(left_speed, right_speed);
  }*/

  last_error = error;
  time_since_on_track = millis();

  right_prox_stop();

  speedometer();

  transmit_to_esp();

}
