
#include "control_relay_function.h"
#include <ESP8266WiFi.h> // esp 8266
#include <PCF8574.h>
#include "Wire.h"
#include "I2C_checking_function.h"
#include <OneWire.h>  // Include the OneWire library for communication with the DS18B20 sensor
#include <DallasTemperature.h>  // Include the DallasTemperature library for temperature readings
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <ESP8266WiFi.h> // esp 8266 wifi
#include <PubSubClient.h>
#include <QuickStats.h>
QuickStats stats; //initialize an instance of this class

//--------------------------------------------------------------------------------------------------------------------------
// define parameter for netpie
WiFiClient espClient;
PubSubClient client(espClient);

const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;

// wifi parameter
// define wifi parameter
const char* ssid = "True_IoT_Pocket_WiFi_P1_95148"; //"EETAR";// "plant";
const char* password = "39695148";//"EETARNET"; // "plantkingdom";

// main
const char* mqtt_Client = "5c140cb3-26e5-4e7c-a1f7-1b70a46b6499";//"18de8c49-83ff-4229-8da9-dbbf1f52b038";
const char* mqtt_username = "bMDLzVBocboU8PmkHdsLNL2KD6cKLtR2";//"5LRDoYQWwuC7MQWhxrFiqSCBzZkm15B7";
const char* mqtt_password = "ifAQAaUkVUcCFdYJVKQLVxqfNXAE5DuS";//"83uGSMt5csI8$XfRJwtN94F)V-#h91fd";
int clientCount = 1;
// End define parameter for netpie
//--------------------------------------------------------------------------------------------------------------------------


LiquidCrystal_I2C lcd(0x27, 16, 2);

//--------------------------------------------------------------------------------------------------------------------------
// pH parameter
#define PCF8591 (0x48)
const int address = 0x48;  // Address of the PCF8591 module

const byte numReadings = 20;     //the number of sample times
byte PHsensorPin = 1;  //EC Meter analog output,pin on analog 0
unsigned int AnalogSampleInterval=25,printInterval=1000,tempSampleInterval=850;  //analog sample interval;serial print interval;temperature sample interval
int pH_readings[numReadings];      // the readings from the analog input
byte Index = 0;                  // the index of the current reading
unsigned long pH_AnalogValueTotal = 0;                  // the running total
unsigned int pH_AnalogAverage = 0;          // the average
float pH_averageVoltage=0; 
unsigned long pH_AnalogSampleTime,pH_printTime,tempSampleTime;
float PHcurrent; 

// calibrate parameter
const float C = 20.3;//21.34; //Constant of straight line (Y = mx + C)
const float m = -5.07;//-5.70; // Slope of straight line (Y = mx + C)

// End pH parameter
//--------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------
// EC parameter
byte ECsensorPin = A0;  //EC Meter analog output,pin on analog 0
byte DS18B20_Pin = 2; //DS18B20 signal, pin on digital 2
const byte num_analogReadings = 5;     //the number of sample times
float EC_readings[numReadings];      // the readings from the analog input
float analogReadings[num_analogReadings];
byte analogIndex = 0;           // the index of the analog reading
unsigned long EC_AnalogValueTotal = 0;                  // the running total
float EC_AnalogAverage = 0;
float EC_TrialAnalogAverage = 0;
float EC_averageVoltage=0;                // the average
unsigned long EC_AnalogSampleTime, EC_printTime, EC_tempSampleTime;
float temperature,ECcurrent,TempCoefficient,CoefficientVolatge; 
float shift_Analog = 0;

// for temperature
// Short circuit with 5V pin and Data pin 
#define ONE_WIRE_BUS D7  // Pin for the DS18B20 data line

OneWire oneWire(ONE_WIRE_BUS);  // Create an instance of the OneWire class with the data line pin
DallasTemperature sensors(&oneWire);  // Create an instance of the DallasTemperature class, passing the OneWire object

// End EC parameter
//--------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------
// modela parameter
PCF8574 pcf8574(0x39);
// define pin button 
const int button1 = P2;
const int button2 = P3;
const int button3 = D3;

// define pin relay
const int relay1 = P0; // A pump
const int relay2 = P1; // B pump
const int stir_pump = D5; // stir pump
//const int water_pump = D6;

// define vocab on off
bool ON = LOW;
bool OFF = HIGH;

// define state relay
bool state1 = HIGH;
bool state2 = HIGH;

// define pump control state
bool control_state = OFF;

// define led wifi
int wifiLed = D4;

const int act_Times = 250;
long act_currentTimes, act_lastTimes;

// End modela parameter
//--------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------
// Control parameter
int maxEC_control = 1100;
int minEC_control = 1000;
int maxPH_control = 8;
int minPH_control = 6;
int parameter_control = 0;
const int change_EC_value = 100;
int onPeriod = 3*1000;
const int waitPeriod = 20*1000;

bool Atest_state = OFF;
bool Btest_state = OFF;
bool pumpA_state = OFF;
bool pumpB_state = OFF;

// End Control parameter
//--------------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------------
// partial paramerter

// wifi parameter
const int wifi_Times = 20*60*1000; // check wifi every 1 min

// pump mode parameter
bool EC_mode = 0;
bool pH_mode = 0;

// parameter for datetime
int Year, Month, Day, Hour, Minute, Second;

// Volume per day parameter
int dayVolume = 0; 
int count_pump = 0;
// End partial paramerter
//--------------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------------
// client function

// client function
char msg[100];
unsigned long client_lastTimes = 0;
// check client conneted
void client_isConnected(){
  long client_currrentTimes = millis();
  //client_lastTimes = set_lastTimes(client_currrentTimes, client_lastTimes);
  if(client_currrentTimes - client_lastTimes < 0){
    client_lastTimes = 0;
  }
  if(client_currrentTimes - client_lastTimes > 30*60*1000){
    client_lastTimes = client_currrentTimes;
    Serial.println("Check Client");
    if(!client.connected() && EC_mode == 0 && pH_mode == 0 ){
      Serial.println("Client not Connected");
      reconnect();
    }
    else{
      Serial.println("Client Connected");
    }
  }
}

// reconnect to client
void reconnect() {
  int count = 0;
  int delay_time = 5000;
  int times = 10*1000/delay_time;
  while (!client.connected() && count < times) { // 1 min
    Serial.print("Attempting MQTT connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("@msg/#");  
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
      count += 1;
    }
  }
}

// run this function when recieve a message
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  String tpc;
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println(message);
  getMsg(topic, message);
}

// action when recieve a message
void getMsg(String topic_, String message_) {     
  if ( topic_ == "@msg/maxEC" ){
    maxEC_control = message_.toInt();
    Serial.println("Get Max");
    public_EC_control();
  }
  else if ( topic_ == "@msg/minEC" ){
    minEC_control = message_.toInt();
    Serial.println("Get Min");
    public_EC_control();
  }
  else if ( topic_ == "@msg/automode" ){
    if(message_ == "ON"){
      control_state = ON;
    }
    else{
      control_state = OFF;
    }
    Serial.println("Control State");
    public_EC_control();
  }
}

void public_EC_control(){
  String payload = "{\"data\":{";
  payload.concat("\"maxEC\":" + String(maxEC_control));
  payload.concat(", ");  // separator between data
  payload.concat("\"minEC\":" + String(minEC_control));
  payload.concat(", ");  // separator between data
  payload.concat("\"automode\":" + String(control_state));
  payload.concat("}}");  
  payload.toCharArray(msg, (payload.length()+1));
  Serial.println(msg);
  client.publish("@shadow/data/update", msg);
  Serial.println("Message is send");
}

long public_lastTimes = 0;
void publicMessage(){
  
  String payload = "{\"data\":{";
  payload.concat("\"ECcurrent\":" + String(ECcurrent));
  payload.concat(", ");  // separator between data
  payload.concat("\"water_volume\":" + String(dayVolume));
  payload.concat(", ");  // separator between data
  payload.concat("\"temperature\":" + String(temperature));
  payload.concat("}}");  
  payload.toCharArray(msg, (payload.length()+1));
  Serial.println(msg);
  client.publish("@shadow/data/update", msg);
  Serial.println("Message is send");

}

// recieve message from netpie every 5 secs
long empty_lastTimes = 0;
void empty_publicMessage(){
  if(client.connected()){
    String payload = "{\"data\":{}}";
    payload.toCharArray(msg, (payload.length()+1));
    client.publish("@shadow/data/update", msg);
    Serial.println("Message is send");
  }
  else{
    client.connect(mqtt_Client, mqtt_username, mqtt_password);
    Serial.println("Attemp tp connect");
    client.subscribe("@msg/#");  
  }
  
}

void publicAll(){

  long public_currentTimes = millis();
  public_lastTimes = set_lastTimes(public_currentTimes, public_lastTimes);
  if(public_currentTimes - public_lastTimes > 1*60*1000){ // every 60 secs
    public_lastTimes = public_currentTimes;
    if(client.connected()){
      client.loop();
      publicMessage();
    }
    else{
      Serial.println("Fail to send message");
    }
  }
  long empty_currentTimes = millis();
  empty_lastTimes = set_lastTimes(empty_currentTimes, empty_lastTimes);

  // check client is not contected more 3 send empty message
  int emptyTimes;
  if(clientCount > 3){
    emptyTimes = 60*1000;
  }
  else{
    emptyTimes = 5*1000;
  }

  if(empty_currentTimes - empty_lastTimes > emptyTimes){  // nomally every 5 secs
    empty_lastTimes = empty_currentTimes;
    if(client.connected()){
      client.loop();
      empty_publicMessage();
      clientCount = 0;
    }
    else{
      if(WiFi.status() == WL_CONNECTED) {
        Serial.println("Client is not connected but Wifi is connected");
        client.disconnect();
        client.connect(mqtt_Client, mqtt_username, mqtt_password);
        client.subscribe("@msg/#");
        clientCount += 1;
      } 
      else {
        WiFi.disconnect();
        WiFi.begin(ssid, password);
      }
    }
  }
}

// End client function
//--------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------
// declare function

bool toggle(bool state);
bool check_lastTimes(long currentTimes , long lastTimes);
long set_lastTimes(long currentTimes , long lastTimes);
void runProgram();
void wifi_isConnected();
void connectWifi();
void action_modela();
bool toggleState(bool state);
void read_pH();
void pH_avgAnalog();
void convert2pH();
void increaseEC(int EC_mode);
void decreaseEC(int EC_mode);
void EC_pump_control();
void DateTimes();
int calculateInterval();
void wifiLed_state(int led);
void testPump();
void water_DateTimes();
void volume_Display();


// End declare function
//--------------------------------------------------------------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // lcd setup
  lcd.begin();

  // set wifi
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting Wifi");
  connectWifi();

  // set up modela board
  pcf8574.pinMode(button1, INPUT);
  pcf8574.pinMode(button2, INPUT);
  pcf8574.pinMode(relay1, OUTPUT);
  pcf8574.pinMode(relay2, OUTPUT);
  pcf8574.begin();

  pinMode(button3, INPUT);
  pinMode(stir_pump, OUTPUT);
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, LOW);
  //pinMode(water_pump, OUTPUT);

  // setup Times for multitask caculate timing 
  setTime(0, 0, 0, 1, 1, 2023); // Set an initial time (optional)
  DateTimes();
  water_DateTimes();

  // set up wire
  Wire.begin();

  // EC setup
  sensors.begin();

  // set up client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.connect(mqtt_Client, mqtt_username, mqtt_password);
  client.subscribe("@msg/#");
  lcd.setCursor(0,1);
  lcd.print("Connecting MQTT");
  delay(1000);
  //reconnect();

  Serial.println("");
  Serial.println("String Program......");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Start program");
  delay(1000);

  start_avgEC();
  Serial.print("Analog: ");
  Serial.println(EC_AnalogAverage);
}

void loop() {
  // put your main code here, to run repeatedly:
  runProgram();
  //I2C_check();
}

void runProgram(){
  wifi_isConnected(); // check wifi is connected?
  Serial_Display();
  action_modela(); // relay and button funtion in modela 
  
  read_EC(); // get EC value
  LCD_Display(); // Display EC value in LCD

  if(control_state == ON){
    EC_pump_control_v2(); // On Off Fertilizer pump
  }
  else{
    digitalWrite(stir_pump, HIGH); // off stir pump from EC mode = 1
    testPump(); // test pump A B
  }
  
  volume_Display(); // Display Fertilizer volume per day
  client_isConnected(); // check netpie is connected?
  publicAll(); // public data to netpie
  wifiLed_state(wifiLed); // check wifi connnect
  
}

//-----------------------------------------------------------------------
// modela control function

void action_modela(){
  act_currentTimes = millis();
  act_lastTimes = set_lastTimes(act_currentTimes , act_lastTimes);

  if(act_currentTimes - act_lastTimes > act_Times){
    act_lastTimes = act_currentTimes;
    if(pcf8574.digitalRead(button1)==ON){
      state1 = toggleState(state1); // toggle state
      Serial.println("button1");
      increase_parameterControl_v2(parameter_control);
      set_parameter();
      public_EC_control();
      
    }
    else if(pcf8574.digitalRead(button2)==ON){
      state2 = toggleState(state2); // toggle state
      Serial.println("button2");
      decrease_parameterControl_v2(parameter_control);
      set_parameter();
      public_EC_control();
    }
    else if(digitalRead(button3)==ON){
      Serial.println("button3");
      parameter_control += 1;
      set_parameter();
    }
    //set_parameter();
  }
}

// Not using code
/*  
void setRelay(){
  if(state1 == ON){pcf8574.digitalWrite(relay1, ON);}
  else{pcf8574.digitalWrite(relay1, OFF);}
  if(state2 == ON){pcf8574.digitalWrite(relay2, ON);}
  else{pcf8574.digitalWrite(relay2, OFF);}
}
*/

void increase_parameterControl(int parameter){
  if(parameter == 0){
    maxEC_control += 100;
  }
  else if(parameter == 1){
    minEC_control += 100;
  }
  else if(parameter == 2){
    //maxPH_control += 1;
    control_state = ON; 
  }
  else if(parameter == 3){
    onPeriod += 1000;
  }
}

void increase_parameterControl_v2(int parameter){
  switch(parameter){
    case 0:
      maxEC_control += 100;
      break;
    case 1:
      minEC_control += 100;
      break;
    case 2:
      control_state = ON; 
      break;
    case 3:
      onPeriod += 1000;
      break;
    case 4:
      pumpA_state = ON;
    case 5:
      pumpB_state = ON;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
}

void decrease_parameterControl(int parameter){
  if(parameter == 0){
    maxEC_control -= 100;
  }
  else if(parameter == 1){
    minEC_control -= 100;
  }
  else if(parameter == 2){
    //maxPH_control -= 1;
    control_state = OFF;
  }
  else if(parameter == 3){
    minPH_control -= 1;
  }
  else if(parameter == 4){
    onPeriod -= 1000;
  }
}

void decrease_parameterControl_v2(int parameter){
  switch(parameter){
    case 0:
      maxEC_control -= 100;
      break;
    case 1:
      minEC_control -= 100;
      break;
    case 2:
      control_state = OFF; 
      break;
    case 3:
      onPeriod -= 1000;
      break;
    case 4:
      pumpA_state = OFF;
    case 5:
      pumpB_state = OFF;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
}


void set_parameter(){
  maxEC_control = set_EC_parameter(maxEC_control);
  minEC_control = set_EC_parameter(minEC_control);
  maxPH_control = set_pH_parameter(maxPH_control);
  minPH_control = set_pH_parameter(minPH_control);
  parameter_control = set_parameter_control(parameter_control);
}

int set_EC_parameter(int EC_parameter){
  if( EC_parameter > 5000){
    return 0;
  }
  else if( EC_parameter < 0){
    return 5000;
  }
  else{
    return EC_parameter;
  }
}

int set_pH_parameter(int pH_parameter){
  if( pH_parameter > 14){
    return 0;
  }
  else if( pH_parameter < 0){
    return 14;
  }
  else{
    return pH_parameter;
  }
}

int set_parameter_control(int parameter_control){
  // set A B pump test
  if(parameter_control == 4){
    Atest_state = ON;
    Serial.println("A test ON");
  }
  else{
    Atest_state = OFF;
    pumpA_state = OFF;
  }
  if(parameter_control == 5){
    Btest_state = ON;
    
  }
  else{
    Btest_state = OFF;
    pumpB_state = OFF;
  }

  // reset value
  if(parameter_control > 5){
    return 0;
  }
  else if(parameter_control < 0){
    return 5;
  }
  else{
    return parameter_control;
  }

}

// End modela control function
//-----------------------------------------------------------------------



//-----------------------------------------------------------------------
// Serial Display



long serial_lastTimes;
void Serial_Display(){
  long serial_currentTimes = millis();
  serial_lastTimes = set_lastTimes(serial_currentTimes, serial_lastTimes);
  if(millis() - serial_lastTimes > 1000){
    serial_lastTimes = millis();
    Serial.print("Period: ");
    Serial.print(calculateInterval());
    Serial.print("\tVolume: ");
    Serial.print(dayVolume);
    Serial.print("\tShift Analog: ");
    Serial.print(shift_Analog);
    switch(parameter_control){
      case 0:
        Serial.print("\tMax EC: ");
        Serial.println(maxEC_control);
        break;
      case 1:
        Serial.print("\tMin EC: ");
        Serial.println(minEC_control);
        break;
      case 2:
        Serial.print("\tControl State: ");
        Serial.println(control_state);
        break;
      case 3:
        Serial.print("\tOn Period: ");
        Serial.println(onPeriod);
        break;
      case 4:
        Serial.print("\tA pump state: ");
        Serial.println(Atest_state);
        break;
      case 5:
        Serial.print("\tB pump state: ");
        Serial.println(Btest_state);
        break;
      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }
}

// End Serial Display
//-----------------------------------------------------------------------




//-----------------------------------------------------------------------
// wifi function

long wifi_lastTimes;
void wifi_isConnected() {
  long wifi_currrentTimes = millis();
  wifi_lastTimes = set_lastTimes(wifi_currrentTimes , wifi_lastTimes);
  // main function
  if(wifi_currrentTimes - wifi_lastTimes > wifi_Times){
    wifi_lastTimes = wifi_currrentTimes;
    if(WiFi.status() != WL_CONNECTED){
      if(EC_mode == 0 && pH_mode == 0){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Re-connecting");
        delay(2000);
        connectWifi();
      }
    }
    else{
      Serial.println("WiFi is connected");
      if(EC_mode == 0 && pH_mode == 0){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Wifi connected");
        delay(2000);
      }
    }

  }
}

// connect wifi function
void connectWifi(){
  WiFi.begin(ssid, password);
  delay(1000);
}

// End wifi function
//-----------------------------------------------------------------------



//-----------------------------------------------------------------------
// pH function

void read_pH() {
  pH_avgAnalog();
  convert2pH();
}


void pH_avgAnalog(){
  long analog_currentTimes = millis();
  pH_AnalogSampleTime = set_lastTimes(analog_currentTimes, pH_AnalogSampleTime);
  if(analog_currentTimes-pH_AnalogSampleTime>=200)  
  {
    pH_AnalogSampleTime = analog_currentTimes;
     // subtract the last reading:
    pH_AnalogValueTotal = pH_AnalogValueTotal - pH_readings[Index];
    // read from the sensor:
    pH_readings[Index] = readAnalog(PHsensorPin);  // analogRead(ECsensorPin);
    Serial.print("pH Analog: ");
    Serial.println(pH_AnalogAverage);
    // add the reading to the total:
    pH_AnalogValueTotal = pH_AnalogValueTotal + pH_readings[Index];
    // advance to the next position in the array:
    Index = Index + 1;
    // if we're at the end of the array...
    if (Index >= numReadings){
      // ...wrap around to the beginning:
      int size = sizeof(pH_readings) / sizeof(pH_readings[0]);

      //bubbleSort(pH_readings, size);
      pH_AnalogAverage = pH_readings[(size+1)/2];
      Index = 0;
    }
    
  }
}

// edit later
void convert2pH(){
  if(millis()-pH_printTime>=printInterval)
  { 
    pH_printTime=millis();
    pH_averageVoltage = pH_AnalogAverage*(float)5/256;
    // m = -5.7 , C = 20.64
    PHcurrent = m*pH_averageVoltage + C;
  }
}

int readAnalog(int chanel){
  int adcvalue0, adcvalue1, adcvalue2, adcvalue3; 
  Wire.beginTransmission(PCF8591);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(PCF8591, 5);
  adcvalue0=Wire.read();
  adcvalue0=Wire.read();
  adcvalue1=Wire.read();
  adcvalue2=Wire.read();
  adcvalue3=Wire.read();
  Wire.beginTransmission(PCF8591);
  Wire.write(0x40);
  Wire.write(adcvalue0);
  Wire.endTransmission();

  if(chanel == 0){
    return adcvalue0;
  }
  else if(chanel == 1){
    return adcvalue1;
  }
  else if(chanel == 2){
    return adcvalue2;
  }
  else if(chanel == 3){
    return adcvalue3;
  }
  else{
    return -1;
  }
}

// End pH function
//-----------------------------------------------------------------------



//-----------------------------------------------------------------------
// EC function

void read_EC()
{
  avgAnalog();
  /*
   Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
  recieveTemp();
   /*
   Every once in a while,print the information on the serial monitor.
  */
  convert2EC();
}

void start_avgEC(){
  int size = sizeof(EC_readings) / sizeof(EC_readings[0]);
  for(int index = 0; index < size; index++){
    // subtract the last reading:
    EC_AnalogValueTotal = EC_AnalogValueTotal - EC_readings[index];
    // read from the sensor:
    EC_readings[index] = readAnalog(ECsensorPin);
    Serial.println(EC_readings[index]);
    // add the reading to the total:
    EC_AnalogValueTotal = EC_AnalogValueTotal + EC_readings[index];
    // advance to the next position in the array:
    // if we're at the end of the array...
    if (index == numReadings-1){
      bubbleSort(EC_readings, size);
      EC_AnalogAverage = EC_readings[(size+1)/2];
      Serial.println("Finish");
    }
    delay(25);
  }
}

void avgAnalog(){
  if(millis()-EC_AnalogSampleTime>=AnalogSampleInterval)  // 25 ms
  {
    EC_AnalogSampleTime=millis();
     // subtract the last reading:
    EC_AnalogValueTotal = EC_AnalogValueTotal - EC_readings[Index];
    // read from the sensor:
    EC_readings[Index] = analogRead(ECsensorPin);  // analogRead(ECsensorPin);
    // add the reading to the total:
    EC_AnalogValueTotal = EC_AnalogValueTotal + EC_readings[Index];
    // advance to the next position in the array:
    Index = Index + 1;
    // if we're at the end of the array...
    if (Index >= numReadings){ // 20 n, 500 ms
      // ...wrap around to the beginning:
      int size = sizeof(EC_readings) / sizeof(EC_readings[0]);
      float multi = 1.0;
      bubbleSort(EC_readings, size); // sort EC readding
      for(int i=0;i<size;i++){
        Serial.print(EC_readings[i]);
        Serial.print(" ");
      }
      Serial.println();
      analogReadings[analogIndex] = stats.g_average(EC_readings,numReadings); //
      analogIndex = analogIndex + 1;
      if(analogIndex >= num_analogReadings){ // 2500 ms
        for(int i=0;i<num_analogReadings;i++){
          Serial.print(analogReadings[i]);
          Serial.println(" ");
        }
        EC_TrialAnalogAverage = stats.average(analogReadings,num_analogReadings);/*
        if(EC_TrialAnalogAverage < 10){ // check at nomal water
          shift_Analog = EC_TrialAnalogAverage - 3.04;
          lcd.setCursor(7, 1);
          lcd.print(int(shift_Analog));

        }*/
        EC_AnalogAverage = stats.average(analogReadings,num_analogReadings) - shift_Analog; // - ...
        Serial.println("Get New");
        analogIndex = 0;
      } 
      Index = 0;
    }
  }
}

// edit later
void convert2EC(){
  if(millis()-EC_printTime>=printInterval)
  {
    EC_printTime = millis();
    EC_averageVoltage = EC_AnalogAverage*(float)5000/256;
    Serial.print("EC Analog: ");
    Serial.print(EC_AnalogAverage);
    Serial.print("\tVoltage: ");
    Serial.print(EC_averageVoltage);  //millivolt average,from 0mv to 4995mV
    Serial.print("mV\tTemperature: ");
    Serial.print(temperature);    //current temperature
    Serial.print("degC\tEC: ");
      
    //TempCoefficient= 47.579/(1.05*temperature+20.3);//1.0+0.0185*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    TempCoefficient = (1.05*27.097+20.3)/(1.05*temperature+20.3);
    CoefficientVolatge=(float)EC_averageVoltage/TempCoefficient;   
    if(EC_AnalogAverage<10){
      //Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
      ECcurrent = 0;//81.269*EC_AnalogAverage - 16.37*temperature + 124.82;
    } 
    else if(EC_AnalogAverage>250){ 
      //Serial.println("Out of the range!");  //>20ms/cm,out of the range
      ECcurrent = 5000;
    }
    else
    { 
      //ECcurrent = 42.3*EC_AnalogAverage*TempCoefficient - 22.2;
      ECcurrent = 43.4*EC_AnalogAverage*TempCoefficient - 91;
    }
    if(ECcurrent < 0){
      ECcurrent = 0;
    }
    Serial.print(ECcurrent,2);  //two decimal
    Serial.println("us/cm");
  }
}

// sort array
void bubbleSort(float arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // Swap arr[j] and arr[j + 1]
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

void recieveTemp(){
  long temp_currentTimes = millis();
  tempSampleTime = set_lastTimes(temp_currentTimes, tempSampleTime); 
  if(temp_currentTimes-tempSampleTime>=tempSampleInterval) 
  {
    tempSampleTime = temp_currentTimes;
    temperature = getTemp();  // read the current temperature from the  DS18B20
  }
}

float getTemp() {
  sensors.requestTemperatures();  // Request temperature reading from the sensor
  float temperatureC = sensors.getTempCByIndex(0);  // Get the temperature in Celsius from the first sensor found
  
  if (temperatureC != DEVICE_DISCONNECTED_C) {  // Check if the reading is valid
    return temperatureC;
  } 
  else {
    Serial.println("Error: Unable to read temperature");
    return 25.0;
  }
}



// End EC function
//-----------------------------------------------------------------------


//-----------------------------------------------------------------------
// lcd display function

long lcd_lastTimes;
int lcd_times = 1000;
void LCD_Display(){
  long lcd_currentTimes = millis();
  lcd_lastTimes = set_lastTimes(lcd_currentTimes, lcd_lastTimes);
  if(lcd_currentTimes - lcd_lastTimes > lcd_times){
    lcd_lastTimes = lcd_currentTimes;
    lcd.clear();
    lcd.setCursor(0,0);
    switch (parameter_control) {
    case 0:
      lcd.print("Max EC: ");
      lcd.print(maxEC_control);
      break;
    case 1:
      lcd.print("Min EC: ");
      lcd.print(minEC_control);
      break;
    case 2:
      lcd.print("Ctrl state: ");
      if(control_state == ON){
        lcd.print("ON");  
      }
      else{
        lcd.print("OFF");  
      }
      
      break;
    case 3:
      lcd.print("On Periods: ");
      lcd.print(onPeriod/1000);
      break;
    case 4:
      lcd.print("A pump:");
      if(pumpA_state == ON){
        lcd.print("ON");
      }
      else{
        lcd.print("OFF");
      }
      break;
    case 5:
      lcd.print("B pump:");
      if(pumpB_state == ON){
        lcd.print("ON");
      }
      else{
        lcd.print("OFF");
      }
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
    lcd.setCursor(0,1);
    lcd.print("EC: ");
    int EC = ECcurrent;
    lcd.print(EC);
    if(EC == 0){
      shift_Analog = EC_TrialAnalogAverage - 3.04;
      lcd.setCursor(7, 1);
      lcd.print(int(shift_Analog));
    }
    lcd.setCursor(11,1);
    lcd.print(temperature);
    /*
    if(WiFi.status() == WL_CONNECTED){
      lcd.print("Yes");
    }
    else{
      lcd.print("No");
    }
    */
  }
}



// lcd display function
//-----------------------------------------------------------------------



//-----------------------------------------------------------------------
// pump control function

// relay1 = A pump
// relay2 = B pump


// EC pump control
long ECpump_lastTimes;
const int check_period = 10*60*1000;

void EC_pump_control(){
  //float EC = ECcurrent;
  if(ECcurrent < minEC_control && pH_mode == 0){
    EC_mode = 1; // 1 mean on & 0 mean off
    while(ECcurrent < maxEC_control){
      pcf8574.digitalWrite(relay1, ON); // on A pump
      //Serial.println("A pump ON");
      delay(onPeriod);
      pcf8574.digitalWrite(relay1, OFF); // off A pump
      delay(waitPeriod);
      pcf8574.digitalWrite(relay2, ON); // on B pump
      //Serial.println("B pump ON");
      delay(onPeriod);
      pcf8574.digitalWrite(relay2, OFF); // off B pump
      delay(waitPeriod);
      start_avgEC();
      read_EC();
    }
    EC_mode = 0;
  } 
}

void EC_pump_control_v2(){
  if(EC_mode == 0 && pH_mode == 0){
    DateTimes();
  }
  if(ECcurrent < minEC_control && pH_mode == 0){
    EC_mode = 1;
  }
  // ON OFF stir pump
  if(EC_mode == 1){
    ABpump_function();
    digitalWrite(stir_pump, LOW);
  }
  else{
    digitalWrite(stir_pump, HIGH);
  }
}

void ABpump_function(){
  int times = calculateInterval();
  int endTimes = (onPeriod+waitPeriod+onPeriod+waitPeriod)/1000 + 5;
  // on A pump
  if(times < onPeriod/1000){
    pcf8574.digitalWrite(relay1, ON); // on A pump
  }
  // off A pump
  else if(times < (onPeriod+waitPeriod)/1000 ){
    pcf8574.digitalWrite(relay1, OFF); // off A pump
  }
  // on B pump
  else if(times < (onPeriod+waitPeriod+onPeriod)/1000){
    pcf8574.digitalWrite(relay2, ON); // on B pump
  }
  // off B pump
  else if(times < (onPeriod+waitPeriod+onPeriod+waitPeriod)/1000){
    pcf8574.digitalWrite(relay2, OFF); // off B pump
  }
  else if(times > endTimes && ECcurrent < maxEC_control){
    DateTimes();
    Serial.println("Complete 1");
    count_pump += 1;
  }
  else if(times > endTimes && ECcurrent > maxEC_control){
    EC_mode = 0;
    DateTimes(); // update times for using with times parameter
    Serial.println("Complete 2");
    count_pump += 1;
  }
}

void ABpump_function_V2(){
  int times = calculateInterval(); // count times in multi task 
  // on A pump
  if(times < onPeriod/1000){ // 10 < times 
    pcf8574.digitalWrite(relay1, ON); // on A pump
  }
  // off A pump , on stir pump
  else if(times < (onPeriod+waitPeriod)/1000 && times > onPeriod/1000){ // 10 < times < 20
    pcf8574.digitalWrite(relay1, OFF); // off A pump
    digitalWrite(stir_pump, HIGH);
  }
  // on B pump , off stir pump 
  else if( (onPeriod+waitPeriod)/1000 < times && times < (2*onPeriod+waitPeriod)/1000 ){ // 20 < times < 30
    pcf8574.digitalWrite(relay2, ON); // off A pump
    digitalWrite(stir_pump, LOW);
  }
  // off B pump , on stir pump 
  else if( (2*onPeriod+waitPeriod)/1000 < times && times < (2*onPeriod+2*waitPeriod)/1000 ){ // 30 < times < 40
    pcf8574.digitalWrite(relay2, OFF); // on B pump
    digitalWrite(stir_pump, HIGH);
  }
  // off stir pump
  else if(times > 2*onPeriod+2*waitPeriod && ECcurrent < maxEC_control){
    digitalWrite(stir_pump, LOW);
    DateTimes();
    Serial.println("Complete 1");
    count_pump += 1;
  }
  else if(times > 2*onPeriod+2*waitPeriod && ECcurrent > maxEC_control){
    digitalWrite(stir_pump, LOW);
    EC_mode = 0;
    DateTimes();
    Serial.println("Complete 2");
    count_pump += 1;
  }
}

// test pump function
void testPump(){
  // A pump
  if(Atest_state == ON && pumpA_state == ON){
    pcf8574.digitalWrite(relay1, ON);
  }
  else{
    pcf8574.digitalWrite(relay1, OFF);
  }
  // B pump
  if(Btest_state == ON && pumpB_state == ON){
    pcf8574.digitalWrite(relay2, ON);
  }
  else{
    pcf8574.digitalWrite(relay2, OFF);
  }
}


void PH_pump_control(){
  float pH = PHcurrent;
  if(pH < minPH_control && EC_mode == 0){
    pH_mode = 1;
    while(pH < maxPH_control){
      //digitalWrite(PH_pump, ON);
      delay(onPeriod);
    }
    pH_mode = 0;
  }
}

// End pump control function
//-----------------------------------------------------------------------


//-----------------------------------------------------------------------
// Timing function

// give period between datetime parameter and present time 
int calculateInterval(){
  time_t t = now();
  int seconds = 0;
  seconds += (year(t) - Year)*365*24*60*60;
  seconds += (month(t) - Month)*30*24*60*60;
  seconds += (day(t) - Day)*24*60*60;
  seconds += (hour(t) - Hour)*60*60;
  seconds += (minute(t) - Minute)*60;
  seconds += second(t)  - Second;
  return seconds;
}

// update datetime parameter
void DateTimes(){
  // Get the current time
  time_t t = now(); // Use the now() function to obtain the current time

  // Extract individual components of the time
  Year = year(t);
  Month = month(t);
  Day = day(t);
  Hour = hour(t);
  Minute = minute(t);
  Second = second(t);
}

// End Timing function
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// water function


// parameter for water_datetime
int water_Year, water_Month, water_Day, water_Hour, water_Minute, water_Second;
// update datetime parameter
long volume_lastTimes = 0;
void volume_Display(){
  dayVolume = count_pump*1.0*onPeriod/1000;
  long volume_currentTimes = millis();
  volume_lastTimes = set_lastTimes(volume_currentTimes, volume_lastTimes);
  if(volume_currentTimes - volume_lastTimes > 60*1000 && EC_mode == 0 && pH_mode == 0){
    volume_lastTimes = volume_currentTimes;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Volume: ");
    lcd.print(dayVolume);
    lcd.print(" ml");
    lcd.setCursor(0,1);
    lcd.print("On Times: ");
    lcd.print(count_pump);
    delay(3000);
  }
  int secs = water_calculateInterval(); // count time to reset water volume
  if(secs > 24*60*60*1000){
    water_DateTimes();
    count_pump = 0;
  }
}

// give period between datetime parameter and present time 
int water_calculateInterval(){
  time_t t = now();
  int seconds = 0;
  seconds += (year(t) - water_Year)*365*24*60*60;
  seconds += (month(t) - water_Month)*30*24*60*60;
  seconds += (day(t) - water_Day)*24*60*60;
  seconds += (hour(t) - water_Hour)*60*60;
  seconds += (minute(t) - water_Minute)*60;
  seconds += second(t)  - water_Second;
  return seconds;
}

void water_DateTimes(){
  // Get the current time
  time_t t = now(); // Use the now() function to obtain the current time

  // Extract individual components of the time
  water_Year = year(t);
  water_Month = month(t);
  water_Day = day(t);
  water_Hour = hour(t);
  water_Minute = minute(t);
  water_Second = second(t);
}

// End water function
//-----------------------------------------------------------------------


void wifiLed_state(int led){
  if(WiFi.status() == WL_CONNECTED){
    digitalWrite(led, ON); // led on
  }
  else{
    digitalWrite(led, OFF); // led off
  }
}