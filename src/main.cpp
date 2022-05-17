#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <Wire.h>
#include <sdpsensor.h>

#define switch1 4
#define switch2 5
#define btn1 6
#define btn2 7

#define daci2cAdress 0x60
#define sfm3300i2cAdress 0x40

#define airFlowLimit1 30
#define airFlowLimit2 90
#define airFlowLimit3 190
#define numPLateauDataPoints 20

const float measurementPoints[] = {airFlowLimit1, airFlowLimit2, airFlowLimit3};

typedef enum
{
  INIT = 0,
  IDLE,
  FAN_STARTUP,
  MEASURE,
  MEASURMENT_POINT,
} HEAD_STATE;

static HEAD_STATE head_state = INIT;
Adafruit_MCP4725 dac;
SDP8XXSensor pressureSensor;

float currAirflow = 0;
float airflowLimit = 0;
float pressure = 0;

int currentVoltage = 0;
int startVoltage = 900;
int maxVoltage = 4095;
int numMeasurementpoints = sizeof(measurementPoints)/sizeof(measurementPoints[0]);
int currMeasurementPoint = 0;
int numCurrPlateauPoints = 0;


float readPressureSensor();
float readAirflowSensor();
void resetMeasurement();
void checkLimit();
void serialOutput();

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  while(!Serial){}  
  Serial.println("Serial Connection established!");

   Wire.beginTransmission(sfm3300i2cAdress);  
   Wire.write(0x10); // start continuous measurement
   Wire.write(0x00); // command 0x1000
   Wire.endTransmission();

    do {  
     int ret = pressureSensor.init();  
     if (ret == 0) {
       Serial.print("init(): success\n");
       break;
     } else {
       Serial.print("init(): failed, ret = ");
       Serial.println(ret);
       delay(1000);
     }
   } while(true);

  dac.begin(daci2cAdress);

  pinMode(switch1,INPUT);
  pinMode(switch2,INPUT);
  pinMode(btn1,INPUT);
  pinMode(btn2,INPUT);  


  dac.setVoltage(currentVoltage, false);  
  head_state=IDLE; 
  delay(200);
  Serial.println("Device ready!");
}

void loop() {
  switch (head_state)
  {
  case IDLE:  
    checkLimit();
    if (digitalRead(btn1) == HIGH)
      {         
        currentVoltage = startVoltage;
        dac.setVoltage(currentVoltage, false); 
        head_state = FAN_STARTUP;
      }
  break;
  case FAN_STARTUP:
      if(readAirflowSensor()>5){          
          Serial.println("################# STARTING MEASUREMENT #################");  
          head_state = MEASURE;
      }else{
        currentVoltage = currentVoltage+5;
        dac.setVoltage(currentVoltage, false); 
      }
  break;
  case MEASURE:
    if(digitalRead(btn1) == HIGH){
        Serial.println("!!!!!!!!  Measurment aborted !!!!!!!!!!");
        resetMeasurement();
        break;
    }
    if(currentVoltage>maxVoltage){
      Serial.println("!!!!! REQUESTED AIRFLOW COULDNT BE REACHED !!!!!!!");
      resetMeasurement();
      break;
    }  
    if(currMeasurementPoint<(numMeasurementpoints-1)){
      if(currAirflow>measurementPoints[currMeasurementPoint]){
        head_state = MEASURMENT_POINT;         
        currentVoltage = currentVoltage-25;
        dac.setVoltage(min(currentVoltage,maxVoltage), false);
        break;
      }
    }
    if(currAirflow>airflowLimit){
      Serial.println("################# MEASUREMENT COMPLETE #################");
      resetMeasurement();
      break;
    }
    serialOutput();
    currentVoltage = currentVoltage+25;
    dac.setVoltage(min(currentVoltage,maxVoltage), false);
  break;
  case MEASURMENT_POINT:
    if(numCurrPlateauPoints<numPLateauDataPoints){
      serialOutput();
      numCurrPlateauPoints++;
    }else{
      numCurrPlateauPoints = 0;
      currMeasurementPoint++;
      head_state = MEASURE;
    }
  break;
  default:
  break;
  }

  delay(200);
}

float readAirflowSensor()
{
  float flow = 0;
  if (2 == Wire.requestFrom(sfm3300i2cAdress, 2))
  {                           // just keep reading SLM (Standard Liter per Minute)
    uint16_t a = Wire.read(); // only two bytes need to be read
    uint8_t b = Wire.read();  // if we don't care about CRC
    a = (a << 8) | b;
    flow = ((float)a - 32768) / 120;
  }
  currAirflow = flow;
  return flow;
}

float readPressureSensor(){
  int ret = pressureSensor.readSample();
  if (ret == 0) {
   pressure = pressureSensor.getDifferentialPressure(); 
   return pressure;
  } 
}

void resetMeasurement(){
      currMeasurementPoint = 0;
      head_state = IDLE;
      currentVoltage = 0;
      currAirflow = 0;
      dac.setVoltage(currentVoltage, false); 
}

void checkLimit(){
  bool s1 = digitalRead(switch1);
  bool s2 = digitalRead(switch2);
  airflowLimit = airFlowLimit2;
  if(s1 && s2){
     airflowLimit = airFlowLimit3;
  }
  if(!s1&&!s2){
    airflowLimit = airFlowLimit1;
  }
}

void serialOutput(){
     Serial.print(readAirflowSensor());   
    Serial.print("\t\t");
    Serial.println(readPressureSensor());   
}


