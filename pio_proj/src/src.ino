#include <Wire.h>
#include <openag_am2315.h>
#include <openag_mhz16.h>
#include <openag_ds18b20.h>
#include <openag_pwm_actuator.h>

/***************************************************
  This is an example for the AM2315 Humidity + Temp sensor

  Designed specifically to work with the Adafruit BMP085 Breakout
  ----> https://www.adafruit.com/products/1293

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect RED of the AM2315 sensor to 5.0V
// Connect BLACK to Ground
// Connect WHITE to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect YELLOW to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4

// Sensor Instances
Am2315 am2315_1;
MHZ16 mhz16_1(77);
Ds18b20 ds18b20_1(5);

// Actuator Instances
PwmActuator led_blue_1(40, true, 0);
PwmActuator led_white_1(41, true, 0);
PwmActuator led_red_1(42, true, 0);

// Message string
String message = "";
bool stringComplete = false;
const int COMMAND_LENGTH = 4;

void split(String messages, String* splitMessages,  char delimiter=',');

void setup() {
  Serial.begin(9600);
  while(!Serial){
    // wait for serial port to connect, needed for USB
  }
  message.reserve(200);

  beginModule(am2315_1, "AM2315 #1");
  beginModule(mhz16_1, "MHZ16 #1");
  beginModule(ds18b20_1, "DS18B20 #1");
}

void loop() {

  if(stringComplete){
    String splitMessages[COMMAND_LENGTH];
    split(message, splitMessages);

    // status, blue, white, red
    if(splitMessages[0] != "0"){
      return;
    }
    led_blue_1.set_cmd((splitMessages[1]).toFloat());
    led_white_1.set_cmd((splitMessages[2]).toFloat());
    led_red_1.set_cmd((splitMessages[3]).toFloat());

    message = "";
    stringComplete = false;
  }

  bool allSuccess = true;

  allSuccess = allSuccess && updateModule(am2315_1, "AM2315 #1");
  allSuccess = allSuccess && updateModule(mhz16_1, "MHZ16 #1");
  allSuccess = allSuccess && updateModule(ds18b20_1, "DS18B20 #1");

  if(!allSuccess){
    return;
  }

  // Prints the data in CSV format via serial.
  // Columns: status,hum,temp,co2
  Serial.print(OK);                               Serial.print(',');
  Serial.print(am2315_1.get_air_humidity());      Serial.print(',');
  Serial.print(am2315_1.get_air_temperature());   Serial.print(',');
  Serial.print(mhz16_1.get_air_carbon_dioxide()); Serial.print(',');
  Serial.print(ds18b20_1.get_temperature());      Serial.print('\n');
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    message += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      return;
    }
  }
}

// C is disgusting and I hate it deeply...
void split(String messages, String* splitMessages,  char delimiter){
  int indexOfComma = 0;
  for(int i = 0; messages.indexOf(delimiter, indexOfComma) > 0; i++){
    int nextIndex = messages.indexOf(delimiter, indexOfComma+1);
    String nextMessage;

    // The first message doesn't have an initial comma, so account for that.
    if(indexOfComma == 0){
      indexOfComma = -1;
    }
    if(nextIndex == -1){
      nextMessage = messages.substring(indexOfComma+1);
    }else{
      nextMessage = messages.substring(indexOfComma+1, nextIndex);
    }
    splitMessages[i] = nextMessage;
    indexOfComma = nextIndex;
  }
}

bool beginModule(Module &module, String name){
  bool status = module.begin() == OK;
  if(!status){
    Serial.print(module.status_level); Serial.print(',');
    Serial.print(name);  Serial.print(',');
    Serial.print(module.status_code);  Serial.print(',');
    Serial.print(module.status_msg);   Serial.print('\n');
  }
  return status;
}

bool updateModule(Module &module, String name){
  bool status = module.update() == OK;
  if(!status){
    Serial.print(module.status_level); Serial.print(',');
    Serial.print(name);  Serial.print(',');
    Serial.print(module.status_code);  Serial.print(',');
    Serial.print(module.status_msg);   Serial.print('\n');
  }
  return status;
}

bool any(bool *all){
  int length = sizeof(all)/sizeof(all[0]);
  for(int i=0; i < length; i++){
    if(all[i]){
      return true;
    }
  }
  return false;
}
