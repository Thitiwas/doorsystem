#include <ESP8266WiFi.h>
#include <Wire.h> // Use the Wire library
#include <ArduinoJson.h>

#define I2C_SCL_PIN      (5)      // D1 pin (SCL / GPIO-5)
#define I2C_SDA_PIN      (4)      // D2 pin (SDA / GPIO-4)
#define DEBUG

const uint8_t I2C_ADDR = 0x20;    // <--- set the I2C address of the PCF8574(A) chip 
char sbuf[64];

const char* MY_SSID = "Bigcamp-Fttx";
const char* MY_PWD =  "bc654321";
const char* server = "fitmcoworkingspace.me";
const char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
String typeItem = "largeRoom";
String nameTypeItem = "largeRoom1";
String tempPassword="";

void i2c_scan() {
  int count = 0;
  Serial.println( "Scanning I2C slave devices..." );
  for( uint8_t addr=0x01; addr <= 0x7f; addr++ ) {
     Wire.beginTransmission( addr ); 
     if ( Wire.endTransmission() == 0 ) {
       sprintf( sbuf, "I2C device found at 0x%02X.", addr );
       Serial.println( sbuf );
       count++;
    }
  }
  if ( count > 0 ) {
    sprintf( sbuf, "Found %d I2C device(s).", count );
  } else {
    sprintf( sbuf, "No I2C device found." );
  }
  Serial.println( sbuf );
}

inline void i2c_write_byte( uint8_t i2c_addr, uint8_t data ) {
  Wire.beginTransmission( i2c_addr );
  Wire.write( data );
  Wire.endTransmission();
}

inline uint8_t i2c_read_byte( uint8_t i2c_addr  ) {
  Wire.beginTransmission( i2c_addr );
  Wire.endTransmission();
  Wire.requestFrom( i2c_addr, (uint8_t) 1 ); // request only one byte
  while ( Wire.available() == 0 ) ; // waiting 
  uint8_t data = Wire.read();
  Wire.endTransmission();  
  return data;
}

// PCF8574: P0..P3 output, P4..P7 input (with 4x 10k pull-up resistors)
char getKey() { // blocking function call
  byte value;
  char key = '\0';
  for ( uint8_t row=0; row < 4; row++ ) { 
     // Send 0xFE for Row 0, 0xFD for Row 1, 0xFB for Row 2, and 0xF7 for Row 3
     value = 0xF0 | ~(1 << row); 
     i2c_write_byte( I2C_ADDR, value ); // send a data byte to PCF8754A
     value = i2c_read_byte( I2C_ADDR ); // read a data byte from PCF8574A
     for ( uint8_t col=0; col < 4; col++ ) {
        if ( (~value >> (col+4)) & 1 ) {
          key = keys[row][col];
#ifdef DEBUG
          sprintf( sbuf, "R%dC%d", row, col );
          Serial.println( sbuf );
#endif
          break;
        }
     }
     if ( key != '\0' ) {
        while ( value == i2c_read_byte( I2C_ADDR ) ) ; // blocking
        return key; 
     } 
  }
  return key; // no key press
}

void setup() {
  Serial.begin( 115200 );
  pinMode(D7, OUTPUT);
  digitalWrite(D7, LOW);
  Serial.print("Connecting to "+*MY_SSID);
  WiFi.begin(MY_SSID, MY_PWD);
  Serial.println("going into wl connect");

  while (WiFi.status() != WL_CONNECTED){
      delay(1000);
      Serial.print(".");
  }
  Serial.println("wl connected");
  Serial.println("");
  Serial.println("WIFI connected\n ");
  Serial.println("");

  Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN ); 
  Wire.setClock( 100000 ); // set clock speed
  i2c_scan(); // scan I2C devices
}

void loop() {
  WiFiClient client;
  char key = getKey();
  if ( key != '\0' ) {
    if (key == '0' || key == '1' || key == '2' || key == '3' ||
        key == '4' || key == '5' || key == '6' || key == '7' ||
        key == '8' || key == '9' ){
          tempPassword += key;
          Serial.println(key);
          Serial.println(tempPassword);
        }
        else if (key == 'A'){
          if (tempPassword != "") {
            String data;
            String res = "";
            StaticJsonBuffer<300> jsonBuffer;
            JsonObject& PostData = jsonBuffer.createObject();
            PostData["typeItem"] = typeItem;
            PostData["nameTypeItem"] = nameTypeItem;
            PostData["password"] = tempPassword;
            PostData.printTo(data);  
            
            Serial.println(data);
            client.connect(server, 5000);
            client.println("POST /checkRoomPassword HTTP/1.1");
            client.println("Host: fitmcoworkingspace.me");
            client.println("Content-Type: application/json");
            client.println("Connection: close");
            client.print("Content-Length: ");
            client.println(data.length());
            client.println();
            client.print(data);
            client.println();           
            delay(500);
            long interval = 2000;
            unsigned long currentMillis = millis(), previousMillis = millis();
            while(!client.available()){           
              if( (currentMillis - previousMillis) > interval ){            
                Serial.println("Timeout");
                client.stop();     
                return;
              }
              currentMillis = millis();
            }
            while (client.connected()) {
              if(client.available()){
                res = client.readStringUntil('\n');
                  delay(1);
               }
            }
            client.stop();
            Serial.println(res);
            Serial.println("closing connection");
            if(res == "accept"){
                Serial.println("delay");
                digitalWrite(D7, HIGH);
                delay(5000);
                digitalWrite(D7, LOW);
                tempPassword = "";
              }
            }
         }
     else if (key == 'C'){  //Check for password
      tempPassword = "";
      Serial.println( "clear");
    }
  }
}

//////////////////////////////////////////////////////////////////
