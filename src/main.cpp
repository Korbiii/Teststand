#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <sdpsensor.h>

#define switch1 4
#define switch2 5
#define btn1 6
#define btn2 7

#define daci2cAdress 0x60
#define sfm3300i2cAdress 0x40

#define airFlowLimit1 30
#define airFlowLimit2 95
#define airFlowLimit3 160
#define numPLateauDataPoints 20

const float measurementPoints[] = {airFlowLimit1, airFlowLimit2, airFlowLimit3};

typedef enum {
    INIT = 0,
    IDLE,
    FAN_STARTUP,
    NEXT_POINT,
    MEASURMENT,
    LINEAR_MEASUREMENT
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
int numMeasurementpoints = sizeof(measurementPoints) / sizeof(measurementPoints[0]);
int currMeasurementPoint = 0;
bool linear_measurment = true;
bool b_btn1, b_btn2, b_switch1, b_switch2;

long timestamp, timestampInterval;

//PID stuff
double setPoint, input, Output;

double factorSq = 0.28, factorX = -13.4, factorConst = 1113;
double Kp = 2, Ki = 0, Kd = 0.1;
PID myPID(&input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);

float readPressureSensor();
float readAirflowSensor();
void resetMeasurement();
void checkLimit();
void serialOutput();
void checkButtons();

void setup() {
    // put your setup code here, to run once:
    Wire.begin();
    Serial.begin(115200);
    while (!Serial) {
    }
    Serial.println("Serial Connection established!");

    Wire.beginTransmission(sfm3300i2cAdress);
    Wire.write(0x10);  // start continuous measurement
    Wire.write(0x00);  // command 0x1000
    Wire.endTransmission();
    Serial.print("Airflow Sensor initialized\n");
    do {
        int ret = pressureSensor.init();
        if (ret == 0) {
            Serial.print("Pressure Sensor initialized\n");
            break;
        } else {
            Serial.print("Pressure Sensor failed initializing");
            Serial.println(ret);
            delay(1000);
        }
    } while (true);

    dac.begin(daci2cAdress);

    pinMode(switch1, INPUT);
    pinMode(switch2, INPUT);
    pinMode(btn1, INPUT);
    pinMode(btn2, INPUT);

    dac.setVoltage(currentVoltage, false);
    head_state = IDLE;
    delay(200);
    timestamp = millis();
    myPID.SetMode(AUTOMATIC);
    Serial.println("READY FOR MEASUREMENT");
}

void loop() {
    checkButtons();
    switch (head_state) {
        case IDLE:
            checkLimit();
            if (b_btn1) {
                linear_measurment = false;
            }
            if (b_btn2) {
                linear_measurment = true;
            }
            if (b_btn1 || b_btn2) {
                currentVoltage = startVoltage;
                dac.setVoltage(currentVoltage, false);
                timestampInterval = millis();
                setPoint = 25;
                head_state = FAN_STARTUP;
                Serial.println("################# FAN STARTING UP #################");
            }

            break;
        case FAN_STARTUP:
            if (readAirflowSensor() > 5) {
                Serial.println("################# STARTING MEASUREMENT #################");
                if (linear_measurment) {
                    head_state = LINEAR_MEASUREMENT;
                } else {
                    head_state = NEXT_POINT;
                }
            } else {
                currentVoltage = currentVoltage + 5;
                dac.setVoltage(currentVoltage, false);
            }
            break;
        case LINEAR_MEASUREMENT:
            serialOutput();
            if (setPoint > airflowLimit) {
                Serial.println("################# MEASURMENT COMPLETE #################");
                resetMeasurement();
                head_state = IDLE;
                break;
            }
            if ((millis() - timestampInterval) > 1000) {
                setPoint += 5;
                if (setPoint < 60) {
                    currentVoltage = (setPoint * setPoint * factorSq) + (setPoint * factorX) + factorConst;
                } else {
                    currentVoltage = (16.1 * setPoint) + 311;
                }
                dac.setVoltage(min(currentVoltage, maxVoltage), false);
                timestampInterval = millis();
            }

            break;
        case NEXT_POINT:
            if (currentVoltage > maxVoltage) {
                Serial.println("!!!!! REQUESTED AIRFLOW COULDNT BE REACHED !!!!!!!");
                resetMeasurement();
                break;
            }
            setPoint = measurementPoints[currMeasurementPoint];
            if (setPoint < 60) {
                currentVoltage = (setPoint * setPoint * factorSq) + (setPoint * factorX) + factorConst;
            } else {
                currentVoltage = (16.1 * setPoint) + 311;
            }
            dac.setVoltage(min(currentVoltage, maxVoltage), false);
            if (abs(setPoint - readAirflowSensor()) < 5) {
                timestamp = millis();
                timestampInterval = timestamp;
                head_state = MEASURMENT;
            }
            break;
        case MEASURMENT:
            if (millis() - timestamp > 5000) {
                currMeasurementPoint++;
                if (currMeasurementPoint == numMeasurementpoints) {
                    Serial.println("################# ALL POINTS MEASURED #################");
                    resetMeasurement();
                    head_state = IDLE;
                    break;
                }
                head_state = NEXT_POINT;
            } else {
                if (millis() - timestampInterval > 500) {
                    input = readAirflowSensor();
                    if (input < setPoint) {
                        currentVoltage += 1;
                    } else if (input > setPoint) {
                        currentVoltage -= 1;
                    }
                    dac.setVoltage(min(currentVoltage, maxVoltage), false);
                    timestampInterval = millis();
                }
                serialOutput();
            }
            break;
        default:
            break;
    }

    delay(200);
}

float readAirflowSensor() {
    float flow = 0;
    if (2 == Wire.requestFrom(sfm3300i2cAdress, 2)) {  // just keep reading SLM (Standard Liter per Minute)
        uint16_t a = Wire.read();                      // only two bytes need to be read
        uint8_t b = Wire.read();                       // if we don't care about CRC
        a = (a << 8) | b;
        flow = abs(((float)a - 32768) / 120);
    }
    currAirflow = flow;
    return flow;
}

float readPressureSensor() {
    int ret = pressureSensor.readSample();
    if (ret == 0) {
        pressure = pressureSensor.getDifferentialPressure();
        return pressure;
    }
}

void resetMeasurement() {
    currMeasurementPoint = 0;
    head_state = IDLE;
    currentVoltage = 0;
    currAirflow = 0;
    dac.setVoltage(currentVoltage, false);
}

void checkLimit() {
    airflowLimit = airFlowLimit2;
    numMeasurementpoints = 2;
    if (b_switch1 && b_switch2) {
        airflowLimit = airFlowLimit3;
        numMeasurementpoints = 3;
    }
    if (!b_switch1 && !b_switch2) {
        airflowLimit = airFlowLimit1;
        numMeasurementpoints = 1;
    }
}

void serialOutput() {
    Serial.print("\t\t");
    Serial.print(readAirflowSensor());
    Serial.print("\t\t");
    Serial.println(readPressureSensor());
}

void checkButtons() {
    b_btn1 = digitalRead(btn1);
    b_btn2 = digitalRead(btn2);
    b_switch1 = digitalRead(switch1);
    b_switch2 = digitalRead(switch2);
    if (head_state != IDLE) {
        if (b_btn1 || b_btn2) {
            resetMeasurement();
            Serial.println("!!!!!!!!!!!!!!  ABORTED  !!!!!!!!!!!!!!");
            b_btn1 = false;
            b_btn2 = false;
        }
    }
}
