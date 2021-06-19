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
//#define         PinMQ135NH3               (A2)  //Analog input 2 of your arduino
/***********************Software Related Macros************************************/
#define         TypeMQ4                   ("MQ-4")   //MQ4
#define         TypeMQ135                 ("MQ-135") //MQ135
#define         Voltage_Resolution        (5)        // 5 volt
#define         ADC_Bit_Resolution        (10)       // For arduino UNO/MEGA/NANO
#define         RatioMQ4CleanAir          (4.4)      // RS / R0 = 60 ppm
#define         RatioMQ135CleanAir        (3.6)      // RS / R0 = 3.6 ppm  
MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ4, TypeMQ4);
MQUnifiedsensor MQ135CO2(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ135CO2, TypeMQ135);
//MQUnifiedsensor MQ135NH3(Board, Voltage_Resolution, ADC_Bit_Resolution, PinMQ135NH3, TypeMQ135);
/**/
/********************************************************************************************Radio setup**/
RF24 radio(9, 10); //CE,CSN
RF24Network network(radio);
RF24Mesh mesh(radio, network);
#define nodeId 1 // set node address(1-255)
/**/
/********************************************************************************************Variables**/
float CH4, newCH4, CO2, newCO2, NH3, newNH3;
uint32_t tStart = 0; //For interval
uint16_t tInterval = 2000; //For interval
bool mqxTimer = false;
bool nrfTimer = false;
struct Payload_out {
  float CH4;
  float CO2;
  float NH3;
  float systemNodeID;
};

/**/
/********************************************************************************************Functions goes below**/
//Check timer
bool checkTimer(uint32_t t = 0L); //default argument for t is 0L
bool checkTimer(uint32_t t) {
  uint32_t now = millis(); //get timer value
  uint16_t interval = tInterval;
  if (t > 0) interval = t;
  if ( now - tStart >= interval  ) { //check to see if it is time to transmit based on set interval
    tStart = now; //reset start time of timer
    return true;
  }
  else return false;
}

//Send payload
void sendPayload( float CH4, float CO2, float NH3 ) {
  float systemNodeID = (float)nodeId;
  RF24NetworkHeader header;
  Payload_out payload = { CH4, CO2, NH3, systemNodeID };
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

/* Source: https://circuitdigest.com/microcontroller-projects/arduino-mq137-ammonia-sensor
  log(y) = m*log(x) + b
  m = (log(y2) - log(y1)) / (log(x2) - log(x1))
  b = log(y) - m*log(x)
  PPM = 10 ^ {[log(ratio) - b] / m}
  where,
  y = ratio (Rs/Ro)
  x = PPM
  m = slope of the line
  b = intersection point
  x1 = Left Rs/Ro point*
  y1 = Left ppm point*
  x2 = Right Rs/Ro point*
  y2 = Right ppm point*
    from Rs/Ro vs PPM chart
    using (40,1) for x1,y1 point
    using (100,0.8) for x2,y2 point
*/
#define RL 47  //The value of resistor RL is 47K
#define m -0.263 //Enter calculated Slope 
#define b 0.42 //Enter calculated intercept
#define Ro 20 //Enter found Ro value
#define MQ_sensor A2 //Sensor is connected to A2
float getNH3() {
  float VRL; //Voltage drop across the MQ sensor
  float Rs; //Sensor resistance at gas concentration
  float ratio; //Define variable for ratio
  float analog_value;

  for (int test_cycle = 1 ; test_cycle <= 200 ; test_cycle++) { //Read the analog output of the sensor for 200 times
    analog_value += analogRead(MQ_sensor); //add the values for 200
  }
  analog_value = analog_value / 200.0;
  VRL = analog_value * (5.0 / 1023.0); //Measure the voltage drop and convert to 0-5V
  Rs = ((5.0 * RL) / VRL) - RL; //Use formula to get Rs value
  ratio = Rs / Ro; // find ratio Rs/Ro
  float ppm = pow(10, ((log10(ratio) - b) / m)); //use formula to calculate ppm
  
  return ppm;
}
/**/
/********************************************************************************************Arduino Setup**/
void setup() {
  Serial.begin(115200);
  SPI.begin();
  delay(2000);

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
      NH3      | 102.2  | -2.473
      Acetona  | 34.668 | -3.369
  */
  MQ4.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ4.setA(1012.7); MQ4.setB(-2.786); // Configurate the ecuation values to get CH4 concentration

  MQ135CO2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135CO2.setA(110.47); MQ135CO2.setB(-2.862); // Configurate the ecuation values to get CO2 concentration

  //MQ135NH3.setRegressionMethod(1); //_PPM =  a*ratio^b
  //MQ135NH3.setA(102.2); MQ135NH3.setB(-2.473); // Configurate the ecuation values to get NH3 concentration

  //Initialize MQx sensors
  MQ4.init();
  MQ135CO2.init();
  //MQ135NH3.init();
  /*
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ4.setRL(10);
    MQ135CO2.setRL(10);
    MQ135NH3.setRL(10);
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
  const String infStr = F("Warning: Connection issue founded, R0 is infinite (Open circuit detected) please check your wiring and supply");
  const String shortStr = F("Warning: Connection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
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

  /*Serial.print("Calibrating MQ135-NH3 please wait.");
    calcR0 = 0;
    for (int i = 1; i <= 10; i ++) {
    MQ135NH3.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ135NH3.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
    }
    MQ135NH3.setR0(calcR0 / 10);
    Serial.println(" done!.");

    if (isinf(calcR0)) Serial.println(infStr); while (1);
    if (calcR0 == 0) Serial.println(shortStr); while (1);
  */
  /*****************************  MQ CAlibration End ********************************************/

  //Radio
  mesh.setNodeID(nodeId);
  //Connect to the mesh
  Serial.println(F("Mesh begin"));
  mesh.begin(97, RF24_1MBPS, 5000);
  radio.setPALevel(RF24_PA_MAX);
  Serial.println(F("Connected to the mesh network..."));
  CH4 = MQ4.readSensor();
  CO2 = MQ135CO2.readSensor();
  //NH3 = MQ135NH3.readSensor();
  NH3 = getNH3();
  sendPayload(CH4, CO2, NH3);
}
/**/
/********************************************************************************************Loop start**/
void loop() {
  if (mqxTimer) {
    MQ4.update(); // Update data, the arduino will be read the voltage on the analog pin
    MQ135CO2.update(); // Update data, the arduino will be read the voltage on the analog pin
    //MQ135NH3.update(); // Update data, the arduino will be read the voltage on the analog pin

    newCH4 = MQ4.readSensor(); // Sensor will read PPM concentration using MQ4 a and b model.
    newCO2 = MQ135CO2.readSensor(); // Sensor will read PPM concentration using MQ135 CO2 a and b model.
    //newNH3 = MQ135NH3.readSensor(); // Sensor will read PPM concentration using MQ135 NH3 a and b model.
    newNH3 = getNH3();
  }

  mesh.update();
  if (nrfTimer) {
    if (newCH4 != CH4 || newCO2 != CO2 || newNH3 != NH3) {
      CH4 = newCH4;
      CO2 = newCO2;
      NH3 = newNH3;
      Serial.print(F("CH4\t\tCO2\t\tNH3"));
      Serial.print(CH4);
      Serial.print(F(" ppm\t"));
      Serial.print(CO2);
      Serial.print(F(" ppm\t"));
      Serial.print(NH3);
      Serial.println(F(" ppm\t"));
      sendPayload(CH4, CO2, NH3);
    }
  }

  mqxTimer = checkTimer(500);
  nrfTimer = checkTimer();
}
/**/
