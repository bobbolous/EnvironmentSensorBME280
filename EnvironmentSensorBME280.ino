/*  Author: Jan Schöfer
    Changelog:

    Version 0.21a: 26.04.2018
      -minor changes

    Version 0.20a: 12.02.2018
      - added Display of Up Time
      - changed display behaviour 

    Version 0.11: 11.02.2018
      - moved timer resets for better accuracy 
      - increased baud from 9600 to 115200

    Version 0.1: 29.01.2018
      - minor improvements

    Version 0.0a: 11.11.2016
      - initial Version

    Setup:
    Arduino UNO
    LCD Keypad Shield and 
    BME280 connected to A4(SDA), A5 (SCL)
*/


#define VERSION "0.21a"

#define SERIAL_INTERVAL 1000 //interval for sending serial [ms]
#define LCD_INTERVAL 1000 //interval for refreshing LCD [ms] 
#define BME_INTERVAL 2000 //interval for reading BME280 (may collide with sensor standby)
#define BLINK_INTERVAL 1000 //interval for blinking (LED, LCD heart)

//for Sensor BME280
#include <Wire.h> //I2C library
#include <Adafruit_Sensor.h> //not really needed here (Adafruit_BME280.h needs this)
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float humidity = 0.0;
float temperature = 0.0;
float pressure = 0.0;

//for LCD Keypad Shield
#include <Wire.h> //I2C library
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // pins for LCD panel

int btnID     = 0;
int btnAnalogValue  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//for time handling
long t_lastDisplay = 0;
long t_lastSerial = 0;
long t_lastSensor = 0;
long t_lastBlink = 0;

bool blinkState = false;

//additional LCD characters
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0}; //heart symbol
uint8_t degree[8] = {0x2, 0x5, 0x2, 0x0, 0x0, 0x0, 0x0}; //degree symbol
uint8_t arrowup[8] = {0x4, 0xe, 0x15, 0x4, 0x4, 0x4, 0x4}; //arrow-up symbol

// read buttons
int read_buttons()
{
  btnAnalogValue = analogRead(0);      // read the value from analog button pin
  // analog button values (right:0; up:132; down:306; left:480; select:721; none:1023)

  if (btnAnalogValue > 1000) return btnNONE;
  if (btnAnalogValue < 50)   return btnRIGHT;
  if (btnAnalogValue < 250)  return btnUP;
  if (btnAnalogValue < 400)  return btnDOWN;
  if (btnAnalogValue < 650)  return btnLEFT;
  if (btnAnalogValue < 850)  return btnSELECT;

  return btnNONE;  // when all others fail, return this...
}

void displayHandler()
{
  if (t_lastDisplay < (millis() - LCD_INTERVAL)) {
    t_lastDisplay = millis();
    btnID = read_buttons();         // read the buttons
    switch (btnID) {               // depending on which button was pushed, we perform an action
        
      case btnUP:
        {
        //show up time
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Up-Time:");
        lcd.setCursor(0, 1);
        lcd.print(String(millis()/1000) + "s ~ ");
        lcd.print(String(float(millis())/3600000.0, 3));
        lcd.print("h");
        break;
      }
      case btnRIGHT:
      {
        //break;
      }
      case btnLEFT:
      {
        //break;
      }
      case btnDOWN:
      {
        //break;
      }
      case btnSELECT:
      {
        //break;
      }
      case btnNONE:
      {
        // standard state
        lcd.clear();
        lcd.setCursor(0, 0);

        if (!blinkState) {
          lcd.write(byte(0)); //heart
          //lcd.print("X ");
        } else {
          lcd.print("  "); //delete heart
        }
        lcd.setCursor(1, 0);
        lcd.print(" " + String(temperature));
        lcd.write(byte(1)); //degree symbol
        lcd.print("C " + String(humidity) + "%");

        // lcd.setCursor(row, line);
        lcd.setCursor(0, 1); // move to the begining of the second LCD line
        lcd.print(String(btnID) + " " + String(pressure) + "hPa" + "                ");
      }
      break;
    }
  }
}

void serialWriteHandler()
{
  if (t_lastSerial < (millis() - SERIAL_INTERVAL)) {
    t_lastSerial = millis();
    Serial.println(String(millis()/1000) + "; " + String(temperature) + "; " + String(humidity) + "; " + String(pressure) + "; ");
  }
}

void serialReadHandler()
{
  if (Serial.available() > 0) {
    String tmp = Serial.readString();
  }
}

void setup()
{
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  lcd.clear();
  lcd.createChar(0, heart);
  lcd.createChar(1, degree);
  lcd.createChar(2, arrowup);
  lcd.home();

  pinMode(LED_BUILTIN, OUTPUT);

  lcd.begin(16, 2);              // initialise LCD
  lcd.setCursor(0, 0);           // set cursor to beginning
  lcd.print("Welcome!");         // print a simple message
  lcd.setCursor(0, 1);           // cursor to second line
  lcd.print("EnviroSens ");      // print Version
  lcd.print(VERSION);

  delay(2000); // wait so someone can read the display

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //lcd.clear;
    lcd.setCursor(0, 1);
    lcd.print("No BME280 found...");
    while (1);
  }
// TODO: does not work for now
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF,  // oversampling (average value)   
                    Adafruit_BME280::STANDBY_MS_0_5); // standby between readings in ms (possible values: 0.5, 10, 20, 62.5, 125, 250, 500, 1000)           
}

void loop()
{
  //blink LED and Display
  if (t_lastBlink < (millis() - BLINK_INTERVAL)) {
    t_lastBlink = millis();
    if (!blinkState) {
      digitalWrite(LED_BUILTIN, HIGH);
      blinkState = true;
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      blinkState = false;
    }
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // read BME280
  if (t_lastSensor < (millis() - BME_INTERVAL)) {
    t_lastSensor = millis();
    // only in forced mode (otherwise only old data is read)
    bme.takeForcedMeasurement();
    //TODO: read sensor as one (not possible with current library)
    humidity = bme.readHumidity();
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
  }

  // serialReadHandler();
  serialWriteHandler();
  displayHandler();
  


/*
    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {
      case btnRIGHT:
        {
          lcd.print("RIGHT ");
          break;
        }
      case btnLEFT:
        {
          lcd.print("LEFT   ");
          break;
        }
      case btnUP:
        {
          lcd.print("UP    ");
          break;
        }
      case btnDOWN:
        {
          lcd.print("DOWN  ");
          break;
        }
      case btnSELECT:
        {
          lcd.print("SELECT");
          break;
        }
      case btnNONE:
        {
          lcd.print("NONE  ");
          break;
        }
    }
    */
}
