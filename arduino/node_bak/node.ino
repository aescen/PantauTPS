/********************************************************************************************Includes**/
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <SPI.h>
#include <MQUnifiedsensor.h>
/************************Hardware Related Macros************************************/
#define         Board                     ("Arduino UNO")
#define         PinMQ4                    (A0)  //Analog input 0 of your arduino
#define         PinMQ135CO2               (A1)  //Analog input 1 of your arduino
#define         PinMQ135NH4               (A2)  //Analog input 1 of your arduino
/***********************Software Related Macros************************************/
#define         TypeMQ4                   ("MQ-4") //MQ4
#define         TypeMQ135                 ("MQ-135") //MQ135
#define         Voltage_Resolution        (5)
#define         ADC_Bit_Resolution        (10)  // For arduino UNO/MEGA/NANO
#define         RatioMQ4CleanAir          (4.4) // RS / R0 = 60 ppm
#define         RatioMQ135CleanAir        (3.6) // RS / R0 = 3.6 ppm  
MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ4, TypeMQ4);
MQUnifiedsensor MQ135CO2(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ135CO2, TypeMQ135);
MQUnifiedsensor MQ135NH4(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ135NH4, TypeMQ135);
/**/
/********************************************************************************************Radio setup**/
RF24 radio(7, 8); //CE,CSN
RF24Network network(radio);
RF24Mesh mesh(radio, network);
#define nodeId 1 // set node address(1-255)
/**/
/********************************************************************************************Variables**/
float CH4, newCH4, CO2, newCO2, NH4, newNH4;
unsigned long displayTimer = 0L;
unsigned long tStart = 0L; //For interval
unsigned long tInterval = 2000L; //For interval
bool mqxTimer = false;
bool nrfTimer = false;
struct Payload_out {
  float CH4;
  float CO2;
  float NH4;
  float systemNodeID;
};

/**/
/********************************************************************************************Functions goes below**/
//Check timer
bool checkTimer(unsigned long t = 0L); //default argument for t is 0L
bool checkTimer(unsigned long t) {
  unsigned long now = millis(); //get timer value
  if (t > 0) tInterval = t;
  if ( now - tStart >= tInterval  ) { //check to see if it is time to transmit based on set interval
    tStart = now; //reset start time of timer
    return true;
  }
  else return false;
}

//Send payload
void sendPayload( float CH4, float CO2, float NH4 ) {
  float systemNodeID = (float)nodeId;
  RF24NetworkHeader header;
  Payload_out payload = { CH4, CO2, NH4, systemNodeID };
  // Send an 'M' type message containing the current millis()
  if (!mesh.write(&payload, 'M', sizeof(payload))) {
    // If a write fails, check connectivity to the mesh network
    if ( !mesh.checkConnection() ) {
      //refresh the network address
      Serial.println(F("|#Renewing Address"));
      mesh.renewAddress(10000);
      if (!mesh.write(&payload, 'M', sizeof(payload))) {
        // If a write STILL fails, mesh network is error
        Serial.println(F("|#Send fail, mesh network is error"));
      }
    } else {
      Serial.println(F("|#Send fail, test OK"));
    }
  } else {
    Serial.print(F("|#Send OK|Assigned node:"));
    Serial.println(mesh.mesh_address);
  }
}
/**/
/********************************************************************************************Arduino Setup**/
void setup() {
  Serial.begin(115200);
  SPI.begin();

  /* Set math model to calculate the PPM concentration and the value of constants
     Exponential regression for MQ4:
      Gas    | a      | b
      LPG    | 3811.9 | -3.113
      CH4    | 1012.7 | -2.786
      CO     | 200000000000000 | -19.05
      Alcohol| 60000000000 | -14.01
      smoke  | 30000000 | -8.308
     Exponential regression for MQ135:
      GAS      | a      | b
      CO       | 605.18 | -3.937
      Alcohol  | 77.255 | -3.18
      CO2      | 110.47 | -2.862
      Tolueno  | 44.947 | -3.445
      NH4      | 102.2  | -2.473
      Acetona  | 34.668 | -3.369
  */
  MQ4.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ4.setA(1012.7); MQ4.setB(-2.786); // Configurate the ecuation values to get CH4 concentration

  MQ135CO2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135CO2.setA(110.47); MQ135CO2.setB(-2.862); // Configurate the ecuation values to get CO2 concentration

  MQ135NH4.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135NH4.setA(102.2); MQ135NH4.setB(-2.473); // Configurate the ecuation values to get NH4 concentration

  //Initialize MQx sensors
  MQ4.init();
  MQ135CO2.init();
  MQ135NH4.init();
  /*
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ4.setRL(10);
    MQ135CO2.setRL(10);
    MQ135NH4.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************
    Explanation:
    In this routine the sensor will measure the resistance of the sensor supposing before was pre-heated
    and now is on clean air (Calibration conditions), and it will setup R0 value.
    We recomend execute this routine only on setup or on the laboratory and save on the eeprom of your arduino
    This routine not need to execute to every restart, you can load your R0 if you know the value
    Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  */
  Serial.print("Calibrating MQ4 please wait.");
  const String infStr = F("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply");
  const String shortStr = F("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++) {
    MQ4.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ4.calibrate(RatioMQ4CleanAir);
    Serial.print(".");
  }
  MQ4.setR0(calcR0 / 10);
  Serial.println(" done!.");

  if (isinf(calcR0)) Serial.println(infStr); while (1);
  if (calcR0 == 0) Serial.println(shortStr); while (1);

  Serial.print("Calibrating MQ135-CO2 please wait.");
  calcR0 = 0;
  for (int i = 1; i <= 10; i ++) {
    MQ135CO2.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135CO2.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135CO2.setR0(calcR0 / 10);
  Serial.println(" done!.");

  if (isinf(calcR0)) Serial.println(infStr); while (1);
  if (calcR0 == 0) Serial.println(shortStr); while (1);

  Serial.print("Calibrating MQ135-NH4 please wait.");
  calcR0 = 0;
  for (int i = 1; i <= 10; i ++) {
    MQ135NH4.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135NH4.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135NH4.setR0(calcR0 / 10);
  Serial.println(" done!.");

  if (isinf(calcR0)) Serial.println(infStr); while (1);
  if (calcR0 == 0) Serial.println(shortStr); while (1);
  /*****************************  MQ CAlibration ********************************************/

  //Radio
  mesh.setNodeID(nodeId);
  //Connect to the mesh
  Serial.println(F("Mesh begin"));
  mesh.begin(97, RF24_1MBPS, 5000);
  radio.setPALevel(RF24_PA_MAX);
  Serial.println(F("Connected to the mesh network..."));
  CH4 = MQ4.readSensor();
  CO2 = MQ135CO2.readSensor();
  NH4 = MQ135NH4.readSensor();
  sendPayload(CH4, CO2, NH4);
}
/**/
/********************************************************************************************Loop start**/
void loop() {
  if (mqxTimer) {
    MQ4.update(); // Update data, the arduino will be read the voltage on the analog pin
    MQ135CO2.update(); // Update data, the arduino will be read the voltage on the analog pin
    MQ135NH4.update(); // Update data, the arduino will be read the voltage on the analog pin

    newCH4 = MQ4.readSensor(); // Sensor will read PPM concentration using MQ4 a and b model.
    newCO2 = MQ135CO2.readSensor(); // Sensor will read PPM concentration using MQ135 CO2 a and b model.
    newNH4 = MQ135NH4.readSensor(); // Sensor will read PPM concentration using MQ135 NH4 a and b model.
  }

  mesh.update();
  if (nrfTimer) {
    if (newCH4 != CH4 || newCO2 != CO2 || newNH4 != NH4) {
      CH4 = newCH4;
      CO2 = newCO2;
      NH4 = newNH4;
      Serial.print(F("CH4\t\tCO2\t\tNH4"));
      Serial.print(CH4);
      Serial.print(F(" ppm\t"));
      Serial.print(CO2);
      Serial.print(F(" ppm\t"));
      Serial.print(NH4);
      Serial.println(F(" ppm\t"));
      sendPayload(CH4, CO2, NH4);
    }
  }

  mqxTimer = checkTimer(500);
  nrfTimer = checkTimer();

}
/**/
