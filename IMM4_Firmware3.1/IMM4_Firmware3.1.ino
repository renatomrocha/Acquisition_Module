#include "N_MAX11632.h"
#include "N_MAX31856.h"
#include <TimerOne.h>
#include <EEPROM.h>
#include <ArduinoJson.h>


MAX11632 MAX_ADC = MAX11632();

N_MAX31856 ThermoCouples[5] = {N_MAX31856(1) , N_MAX31856(2) , N_MAX31856(3) ,N_MAX31856(4),N_MAX31856(5)};

bool State;
      
bool TempRead = false;
bool ConfigRcvd = false;

//Flag for interrupt pin 
volatile bool interruptTriggered = false;
volatile unsigned long last_micros;
unsigned long time_now = 0;
volatile long initTime = 0;

bool active = false;
bool unactive = false;
bool acq_performed = false;


char pString[255];

//String to hold incoming data
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
//For JSON DESERIALIZATION
char inputJSON [400];
StaticJsonDocument<500> doc;
//FOR JSON SERIALIZATION
const uint8_t capacity = JSON_OBJECT_SIZE(30);
DynamicJsonDocument data(100);

typedef struct{
char* value[16]; //Type of sensor/Thermocouple
uint8_t cycles[16];  //Number of idle cycles
bool states[16]; //Slot state
byte counts[16]; //Number of occurred idle cycles
} Params;
 
Params T1;


void setup() {
  Serial.begin(115200);

  State = false;
  //Inicializar Termopares
  for(uint8_t k = 0; k<6;k++){
    ThermoCouples[k].begin();
    }
  //Inicializar variável com counts
  for(uint8_t q = 0; q<15; q++){
    T1.counts[q] = 0;
    }
    
  //Iniciar interface comunicação MAX11632  
  MAX_ADC.begin();
  Timer1.initialize(1);
 Timer1.attachInterrupt(callback,1000000); // ISR for analog Sensor Reading delay in microseconds
  //eeprom_Read();
  Serial.print("Restarted... ");
  Serial.print("Read eeprom and first value is: ");
  
  //attachInterrupt(digitalPinToInterrupt(2),callback3,FALLING);
  pinMode(13,OUTPUT);
}

void loop() {
  //Thermocouple acquisition routine
  
  while(Serial.available() == 0){
     if(digitalRead(2)){
      if(!unactive && acq_performed){
      Serial.println("stop");
        unactive = true;
        active = false;
        State = false;
        acq_performed = false;
        Timer1.detachInterrupt();
        digitalWrite(13,LOW);
      }
      }
      else{
        if(!active){
        Serial.println("start");
        acq_performed = true;
        active = true;
        unactive = false;
        State = true;
        Timer1.attachInterrupt(callback,100000);
        digitalWrite(13,HIGH);
        }
        }
    }
  
  ConfigRcvd = true;
  Timer1.detachInterrupt();
  serialEvent();
  ConfigRcvd = false;
  Timer1.attachInterrupt(callback,100000);
  active = false; 
  unactive = false;

}

//////////Auxiliary functions
//Saves Sensor type in address 0 - 9
void eeprom_Save(){
   byte addr;
   byte addr2;
   uint8_t rate;
   uint8_t value;
   
  for (addr = 0; addr<15;addr++){
    value = atoi(T1.value[addr]);
    EEPROM.write(addr,value);
    rate = T1.cycles[addr];
    addr2 = addr+15;
    EEPROM.write(addr2,rate);
    
  }
 
}

/*
void eeprom_Read(){
  for (uint8_t addr = 0; addr<15;addr++){
    
    uint8_t val = EEPROM.read(addr);
    char* cval;
    cval = &val;
    uint8_t val2 = EEPROM.read(addr+15);
    
    T1.value[addr] = cval;
    T1.cycles[addr] = val2;
  }
    e
}
*/

void serialEvent(){
  int i=0;
  //eeprom_Read();
 // Serial.print("Previous slot 1: ");
  //Serial.println(T1.value[0]);
  //memset(inputString,0,500);
  while(Serial.available()){
    char inChar = (char)Serial.read();
   
    inputJSON[i] = inChar;
    i++;
    if (inChar=='\n'){
      stringComplete = true;
      //Serial.println("Will start processing JSON!");
      DeserializationError error = deserializeJson(doc, inputJSON);

        // Test if parsing succeeds.
        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
        return;
        }
        //Thermocouple types
        T1.value[0] = doc["1"]["t"];
        T1.value[1] = doc["2"]["t"];
        T1.value[2] = doc["3"]["t"];
        T1.value[3] = doc["4"]["t"];
        T1.value[4] = doc["5"]["t"];
        
        for (int j = 5; j<15; j++){
          T1.value[j] = "0";
        }
        //Sensors Rates
        T1.cycles[0] = doc["1"]["r"];
        T1.cycles[1] = doc["2"]["r"];
        T1.cycles[2] = doc["3"]["r"];
        T1.cycles[3] = doc["4"]["r"];
        T1.cycles[4] = doc["5"]["r"];
        T1.cycles[5] = doc["6"]["r"];
        T1.cycles[6] = doc["7"]["r"];
        T1.cycles[7] = doc["8"]["r"];
        T1.cycles[8] = doc["9"]["r"];
        T1.cycles[9] = doc["10"]["r"];
        T1.cycles[10] = doc["11"]["r"];
        T1.cycles[11] = doc["12"]["r"];
        T1.cycles[12] = doc["13"]["r"];
        T1.cycles[13] = doc["14"]["r"];
        T1.cycles[14] = doc["15"]["r"];
        //////////////STATES
        T1.states[0] = doc["1"]["s"];
        T1.states[1] = doc["2"]["s"];
        T1.states[2] = doc["3"]["s"];
        T1.states[3] = doc["4"]["s"];
        T1.states[4] = doc["5"]["s"];
        T1.states[5] = doc["6"]["s"];
        T1.states[6] = doc["7"]["s"];
        T1.states[7] = doc["8"]["s"];
        T1.states[8] = doc["9"]["s"];
        T1.states[9] = doc["10"]["s"];
        T1.states[10] = doc["11"]["s"];
        T1.states[11] = doc["12"]["s"];
        T1.states[12] = doc["13"]["s"];
        T1.states[13] = doc["14"]["s"];
        T1.states[14] = doc["15"]["s"];
      
      }
    }
  
}



void callback(void){
  
data.clear();

if(State == true){

if(ConfigRcvd == false){
//Serial.println("Started cycle");

 for(int i = 0; i<15 ;i++){
    //Ler thermocouples
    if(i<5){
       if(T1.states[i] == true){
        if(T1.counts[i]==T1.cycles[i]-1){  
          //Serial.print("T");
          //Serial.print(i+1);
        switch(i+1){
        case 1:
            data["1"].set(ThermoCouples[i].readThermocoupleTemperature());
            break;
        case 2:
            data["2"].set(ThermoCouples[i].readThermocoupleTemperature());
            break;
        case 3:
            data["3"].set(ThermoCouples[i].readThermocoupleTemperature());
            break;
        case 4:
            data["4"].set(ThermoCouples[i].readThermocoupleTemperature());
            break;
        case 5:
            data["5"].set(ThermoCouples[i].readThermocoupleTemperature());
            break;
        
        }

          T1.counts[i] = 0;
       }
       else{
       T1.counts[i]+=1;
       }
       }
    }
    else{
      if(T1.states[i] == true ){
      if(T1.counts[i]==T1.cycles[i]-1){    

      switch(i+1){
        case 6:
            data["6"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 7:
            data["7"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 8:
            data["8"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 9:
            data["9"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 10:
            data["10"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 11:
            data["11"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 12:
            data["12"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 13:
            data["13"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 14:
            data["14"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        case 15:
            data["15"].set(MAX_ADC.readSingleChannel(i-5));
            break;
        }
           
          
          T1.counts[i]=0;
      }
      else{
        T1.counts[i]+=1;
        }
      }
      }
      
      
      }
    
    }
char* output;
//int len = measureJson(data);
//serializeJson(data,output, len);
String out_to_serial = data.as<String>();
Serial.print(out_to_serial);

//Serial.println((String)data);
//Serial.println("{'1':29.38281h'4':28.05469,'5':31.17188,'7':207,'2':28.86719,'3':26.03125,'6':23,'12':2341}");
//delay(5);

Serial.flush();


  //uint8_t fault = max1.readFault();
/*  
  if (fault) {
    if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
    if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
    if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
    if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
    if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
    if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
    if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
    if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
  }
*/
}
}

void callback2(void){

  detachInterrupt(digitalPinToInterrupt(2));  
  if(!State){
    Serial.println("start");
    Timer1.attachInterrupt(callback,100000);
     State=!State;
  digitalWrite(13,State);
  attachInterrupt(digitalPinToInterrupt(2),callback2,FALLING);
  }
  else{
    Timer1.detachInterrupt();
    //time_now = millis(); // Get initial time... millis does not increment inside ISR

   
    Serial.println("stop");   
     State=!State;
    digitalWrite(13,State);
    attachInterrupt(digitalPinToInterrupt(2),callback2,FALLING);
  }
 
  
  
 
}


  
