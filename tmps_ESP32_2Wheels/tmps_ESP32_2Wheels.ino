/* TMPS 2 Wheels by JS 9/2025

based on 
https://www.youtube.com/upir_upir
YouTube Video: https://youtu.be/P85tkCbQGo8
Source Files: https://github.com/upiir/arduino_tpms_tire_pressure

* Only 2 sensors for motor bike
* if sensor were detected the display part of the sensor will be inverted
  after timeout (e.g. 30 sec) the part will be normal
* pressure in bar (see comments), without 1 bar normal air pressure!
* voltage of the sensors 

Connections
OLED        XIAO      ESP32C3 supermini
GND       2. vo.Re      2. vo.Re
Vcc 3,3V  3. vo.Re      3. vo.Re
SCL       2. vu.Li      4. vu.li  IP 9
SDA       3. vu.Li      5. vu.Li  IO 8
*/


#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <U8g2lib.h>


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R3, /* reset=*/ U8X8_PIN_NONE); // initialization for the used OLED display

int scanTime = 5;  //In seconds
BLEScan *pBLEScan;

int offset=64;  // screen start for second sensor 

#define zero_logo_width 64
#define zero_logo_height 64

// Zero Motorcycle logo

static const unsigned char zero_logo_bits[] U8X8_PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x03, 0x00, 0x00,
   0x00, 0x00, 0xfe, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff,
   0xff, 0xff, 0x01, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00,
   0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0xf8, 0xff, 0xff,
   0xff, 0xff, 0x1f, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00,
   0x00, 0xfe, 0xff, 0x05, 0xf0, 0xff, 0x7f, 0x00, 0x00, 0xff, 0x0f, 0x00,
   0xe0, 0xff, 0xfb, 0x00, 0x80, 0xff, 0x03, 0x00, 0xc0, 0xff, 0xfb, 0x00,
   0x80, 0xff, 0x00, 0x00, 0xe0, 0xff, 0xfc, 0x01, 0xc0, 0x7f, 0x00, 0x00,
   0xe0, 0xff, 0xfc, 0x01, 0xc0, 0x3f, 0x00, 0x00, 0xf0, 0x7f, 0xfe, 0x03,
   0xc0, 0x3f, 0x00, 0x00, 0xf0, 0x3f, 0xfc, 0x03, 0xe0, 0x1f, 0x00, 0x00,
   0xf8, 0x1f, 0xfc, 0x03, 0xe0, 0x1f, 0x00, 0x00, 0xfc, 0x0f, 0xf8, 0x07,
   0xe0, 0x1f, 0x00, 0x00, 0xfc, 0x07, 0xf8, 0x07, 0xe0, 0x0f, 0x00, 0x00,
   0xfe, 0x07, 0xf8, 0x07, 0xf0, 0x0f, 0x00, 0x00, 0xff, 0x03, 0xf0, 0x07,
   0xe0, 0x0f, 0x00, 0x80, 0xff, 0x01, 0xf0, 0x07, 0xf0, 0x0f, 0x00, 0xc0,
   0xff, 0x00, 0xf8, 0x07, 0xf0, 0x0f, 0x00, 0xc0, 0x7f, 0x00, 0xf0, 0x07,
   0xf0, 0x0f, 0x00, 0xf0, 0x3f, 0x00, 0xf0, 0x07, 0xf0, 0x0f, 0x00, 0xf0,
   0x3f, 0x00, 0xf0, 0x07, 0xf0, 0x0f, 0x00, 0xf8, 0x0f, 0x00, 0xf0, 0x07,
   0xf0, 0x0f, 0x00, 0xfc, 0x0f, 0x00, 0xf0, 0x07, 0xf0, 0x0f, 0x00, 0xfe,
   0x07, 0x00, 0xf0, 0x07, 0xf0, 0x0f, 0x00, 0xfe, 0x03, 0x00, 0xf0, 0x07,
   0xe0, 0x0f, 0x00, 0xff, 0x01, 0x00, 0xf0, 0x07, 0xf0, 0x0f, 0x80, 0xff,
   0x00, 0x00, 0xf0, 0x07, 0xe0, 0x0f, 0xc0, 0xff, 0x00, 0x00, 0xf8, 0x07,
   0xe0, 0x0f, 0xe0, 0x3f, 0x00, 0x00, 0xf8, 0x07, 0xe0, 0x1f, 0xf0, 0x3f,
   0x00, 0x00, 0xf8, 0x07, 0xe0, 0x1f, 0xf0, 0x1f, 0x00, 0x00, 0xf8, 0x03,
   0xc0, 0x1f, 0xfc, 0x0f, 0x00, 0x00, 0xfc, 0x03, 0xc0, 0x3f, 0xfc, 0x0f,
   0x00, 0x00, 0xfc, 0x03, 0xc0, 0x3f, 0xfe, 0x07, 0x00, 0x00, 0xfe, 0x03,
   0x80, 0x3f, 0xff, 0x07, 0x00, 0x00, 0xff, 0x01, 0x80, 0x9f, 0xff, 0x03,
   0x00, 0x80, 0xff, 0x01, 0x00, 0xdf, 0xff, 0x03, 0x00, 0xe0, 0xff, 0x00,
   0x00, 0xdf, 0xff, 0x07, 0x00, 0xfc, 0xff, 0x00, 0x00, 0xfe, 0xff, 0xbf,
   0xfa, 0xff, 0x7f, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00,
   0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xf0, 0xff, 0xff,
   0xff, 0xff, 0x0f, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00,
   0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff,
   0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x80, 0xfe, 0x7f, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


int v_updated, h_updated;

unsigned long v_currentMillis = 0;    // for reset the inverted screen 30*1000 sec
unsigned long h_currentMillis = 0;

float v_voltage, v_voltage_old;       // sensor battery voltage in V 
float h_voltage, h_voltage_old; 

int v_temperature, v_temperature_old; // sensor temperature in °C
int h_temperature, h_temperature_old; 

uint16_t v_pressure_psi_x10;          // sensor pressure psi * 10
uint16_t h_pressure_psi_x10; 

float v_pressure, v_pressure_old; // sensor pressure PSI
float h_pressure_psi, h_pressure_psi_old; 


class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {

    v_updated = 0;
    h_updated = 0;

    if (advertisedDevice.haveName()) { // does the advertised device have a name?
      if (advertisedDevice.getName() == "BR") { // if the name is "BR", it´s out pressure sensor
        //Serial.printf("Found BR Device: %s \n", advertisedDevice.toString().c_str()); // print all the received data to serial

        // get the address, and if it´s a known address, set which sensor should be updated see upirs explanation
        if (advertisedDevice.getAddress().toString() == "3b:5c:00:00:83:42") { // vorne
          v_updated = 1;
        }
        if (advertisedDevice.getAddress().toString() == "3b:7d:00:00:47:2a") {
          h_updated = 1;
        }
    
        if (advertisedDevice.haveManufacturerData() == true) { // does the advertised device send manufacturer data? that data has the pressure and temperature values
          String strManufacturerData = advertisedDevice.getManufacturerData(); // get the advertised data as String

          byte cManufacturerData[100]; // helper variable to convert String (with capital S) to C-style string
          memcpy(cManufacturerData, strManufacturerData.c_str(), strManufacturerData.length()); // convert String to c-style string

          //for (int i = 0; i < strManufacturerData.length(); i++) { // print individual bytes of the manufacturer data
          //  Serial.printf("[%02X]", cManufacturerData[i]); // %02 means pad the value with zeros to have at least two digits, X is the byte
          //}
          //Serial.printf("\n"); // print new line after the manufacturers data

          // found a device that has the BR name

          // Received Manufacturers Data Example
          // [80] [1f] [1a] [00] [92] [14] [74]
          //
          // meaning:
          // 80 - status - ignore for now
          // 1f - battery voltage - 0x1f hex = 31 = 3.1V
          // 1a - temperature 0x1a hex = 26°C
          // 0092 - pressure 0x0092 hex = 146 = 14.6 psi
          // 14 74 - checksum - ignore for now  

          if (v_updated == 1) {   // v = vorne = front
            v_voltage = (float)cManufacturerData[1] / 10.0;  // get battery voltage
            v_temperature = cManufacturerData[2];  // get temperature in °C
            v_pressure_psi_x10 = (uint16_t)cManufacturerData[3] << 8 | cManufacturerData[4]; // use two bytes for out 16bit value, psi x10
            //v_pressure = (float)v_pressure_psi_x10 / 10.0;          // pressure in psi
            v_pressure = (float)v_pressure_psi_x10 / 10.0 / 14.50377; // pressure in bar minus normal air pressure
          }
          
          if (h_updated == 1) {   // h = hinten = rear
            h_voltage = (float)cManufacturerData[1] / 10.0;  // get battery voltage
            h_temperature = cManufacturerData[2];  // get temperature in °C
            h_pressure_psi_x10 = (uint16_t)cManufacturerData[3] << 8 | cManufacturerData[4]; // use two bytes for out 16bit value, psi x10
            //h_pressure_psi = (float)h_pressure_psi_x10 / 10.0;          // pressure in psi
            h_pressure_psi = ((float)h_pressure_psi_x10 / 10.0 - 14.50377) / 14.50377; // pressure in bar
          }         
        } 
        // Serial.println(advertisedDevice.getAddress().toString()); // get the device address and print it to serial
      }  
    }
  }
};

void setup(void) {
  Serial.begin(115200);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();  
  
  u8g2.setFlipMode(1);
  u8g2.setFont(u8g2_font_helvB12_tf);
  u8g2.setCursor(10,16);
  u8g2.print("Norm."); 
  u8g2.setCursor(7,34);
  u8g2.print("V 2.2 b"); 
  u8g2.setCursor(7,50);
  u8g2.print("H 2.3 b"); 


  u8g2.drawXBMP(
    0, // x-Position
    64, // y-Position
    zero_logo_width,
    zero_logo_height,
    zero_logo_bits
  );
  u8g2.sendBuffer();
  
  Serial.println("Scanning ...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

}

void loop(void) {
  
  BLEScanResults *foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("\nDevices found: ");
  Serial.println(foundDevices->getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory

  if (v_updated == 1) {
    Serial.println("\nVorne updated");
    Serial.print("Pressure (b/psi): ");
    Serial.println(String(v_pressure,1)); 
    Serial.print("Temperature (°C): ");    
    Serial.println(v_temperature);  
    Serial.print("Voltage (V)     : ");
    Serial.println(v_voltage);
    draw_values( 0, 1, String(v_pressure,1), String(v_temperature), String(v_voltage));
    v_currentMillis = millis();
    v_pressure_old = v_pressure;
    v_temperature_old = v_temperature;
    v_voltage_old = v_voltage;
  }
  if (h_updated == 1) {
    Serial.println();
    Serial.println("Hinten updated");
    Serial.print("Pressure(b/psi) : ");
    Serial.println(String(h_pressure_psi,1)); 
    Serial.print("Temperature (°C): ");    
    Serial.println(h_temperature);  
    Serial.print("Voltage (V)     : ");
    Serial.println(h_voltage);
    draw_values( 63, 1, String(h_pressure_psi,1), String(h_temperature), String(h_voltage));
    h_currentMillis = millis();    // z.B. 12345
    h_pressure_psi_old = h_pressure_psi;
    h_temperature_old = h_temperature;
    h_voltage_old = h_voltage;
  }
  
  if ((v_currentMillis != 0) && (millis() - v_currentMillis >= 10*1000 )) { // 10 sekunden
    draw_values( 0, 0, String(v_pressure_old,1), String(v_temperature_old), String(v_voltage_old));
    Serial.println("V resetted");
    v_currentMillis=0;
  }
  if ((h_currentMillis != 0 ) && (millis() - h_currentMillis >= 10*1000 )) { // 10 sekunden 
    draw_values( 63, 0, String(h_pressure_psi_old,1), String(h_temperature_old), String(h_voltage_old));
    Serial.println("H resetted");
    h_currentMillis=0;
  }

  delay(100);
}

void draw_values(int offset, int invertiert, String druck, String temp, String volt) {
  u8g2.setFontMode(1);  /* activate transparent font mode */

  if ( invertiert == 1 ) {
    u8g2.setDrawColor(1); /* color 1 for the box */
    u8g2.drawRBox(0, 1+offset, 63, 61, 4);
    u8g2.setDrawColor(0);
  } else {
    u8g2.setDrawColor(0); /* color 1 for the box */
    u8g2.drawRBox(0, 1+offset, 63, 61, 4);
    u8g2.setDrawColor(1);
  }
 
  u8g2.setFont(u8g2_font_helvB18_tf);
  u8g2.setCursor(5,24+offset);
  u8g2.print(druck + " b");
  u8g2.setFont(u8g2_font_helvB14_tf);
  u8g2.setCursor(10,43+offset);
  u8g2.print(temp + " °C"); 
  u8g2.setFont(u8g2_font_helvB08_tf);
  u8g2.setCursor(15,57+offset);
  u8g2.print( volt +" V");
  u8g2.sendBuffer();
}


