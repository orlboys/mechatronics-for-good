#include <stdint.h>
#include <stdio.h>
#include <Wire.h>

// Sensor and Actuator Libraries

#include <MQ135.h>
#include "DFRobot_AirQualitySensor.h"
#include <LiquidCrystal_I2C.h>

// Defining Pins and IIC Addresses

#define PIN_MQ135 A2
#define PM_ADDRESS 0x19
#define LCD_ADDRESS 0x20
#define RELAY_PIN 12

// Creating Objects

DFRobot_AirQualitySensor particle(&Wire, PM_ADDRESS);
MQ135 mq135_sensor(PIN_MQ135);
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// Constants

float temperature = 21.0; // Assumed Temperature
float humidity = 25.0; // Assumed Humidity

/** 
* AQI Thresholds and Breakpoints
*
* example_breakpoints[i] is the breakpoint which makes example_aqi_values[i] the current AQI value
**/

const float pm2_5_breakpoints[] = {12.0, 35.0, 55.0, 150.0, 250.0, 500.0};
const int pm2_5_aqi_values[] = {50, 100, 150, 200, 300, 500};

const float mq_breakpoints[] = {400.0, 800.0, 1200.0, 1600.0, 2000.0, 5000.0};
const int mq_aqi_values[] = {50, 100, 150, 200, 300, 500};

// Global variables

float mq_ppm; // ppm given from MQ135
float correctedPPM; // corrected ppm given from MQ135
uint16_t pm; // pm2.5 um/l Using uint16_t, more efficient on our low-memory arduino
int aqi; // the AQI value calculated based off of pm and ppm
bool fan;

// SETUP AND LOOP FUNCTIONS


void setup() 
{
  Serial.begin(115200);
  Serial.println("Beginning Setup...");
  // Setup functions for each component
  pinMode(12, OUTPUT);
  lcd.init();
  lcd.backlight();
  pm_setup();

  Serial.println("Setup function complete.");
}

void loop() 
{
  Serial.println("In loop function.");
  delay(1000);
  pm = pm_collect_data();
  mq_ppm, correctedPPM = mq_collect_data();
  aqi = calculate_aqi();
  lcd_display(1);
  auto_purify();
}

// Purification / Relay Functions

void auto_purify()
{
  if (aqi > 200)
  {
    fan_on();
    Serial.println("AQI above limit");
    fan = true;
  }
  else
  {
    fan_off();
    Serial.println("AQI below limit");
    fan = false;
  }
}

// Calculation / AQI functions

int calculate_aqi()
{
  bool max_val_bool = pm < correctedPPM; // Is true if the MQ135 is greater than the PM2.5 sensor --> Since we want to take the highest possible value
  float prev = 0;
  if (max_val_bool) // If the MQ is greater than the PM2.5 Sensor
  {
    for (int i = 0; i <= 5; i++) 
    {
      if ((prev < correctedPPM) && (correctedPPM <= mq_breakpoints[i]))
      {
        return mq_aqi_values[i];
      }
      else
      {
        prev = mq_breakpoints[i];
      }
    }
  }
  else
  {
    for (int i = 0; i <= 5; i++) 
    {
      if ((prev < pm) && (pm <= pm2_5_breakpoints[i]))
      {
        return pm2_5_aqi_values[i];
      }
      else
      {
        prev = pm2_5_breakpoints[i];
      }
    }
  }
  return 0;
}


// MQ135-DEDICATED FUNCTIONS

int mq_collect_data() 
{

  /**
  *@brief get the gas PPM reading from the MQ135 sensor
  *@param temperature a constant temperature to get corrected PPM
  *@param humidity a constant humidity to get corrected PPM
  **/

  int ppm = mq135_sensor.getPPM();
  int c_ppm = mq135_sensor.getCorrectedPPM(temperature, humidity);

  // Printing to Serial Monitor

  Serial.print("PPM: ");
  Serial.print(ppm);
  Serial.print("\t Corrected PPM: ");
  Serial.print(c_ppm);
  Serial.println("ppm");

  return ppm, c_ppm;

}

// PM2.5-DEDICATED FUNCTIONS

void pm_setup()
{
  while(!particle.begin()){
    Serial.println("NO Devices !");
    delay(1000);
  }
  Serial.println("sensor begin success!");
}

int pm_collect_data()
{
  /**
  *@brief Gets the number of 0.3um PM in 0.1L of air
  *@param PARTICLENUM_0_3_UM_EVERY0_1L_AIR returns number of particles of size 0.3um per 0.1L
  *
  * also can be used for various sizes of particles:
  *
  *@n PARTICLENUM_0_5_UM_EVERY0_1L_AIR 
  *@n PARTICLENUM_1_0_UM_EVERY0_1L_AIR 
  *@n PARTICLENUM_2_5_UM_EVERY0_1L_AIR 
  *@n PARTICLENUM_5_0_UM_EVERY0_1L_AIR 
  *@n PARTICLENUM_10_UM_EVERY0_1L_AIR
  *
  *@return int --> 0.3um PM in 0.1L air
  */

  int part_count = particle.gainParticleNum_Every0_1L(PARTICLENUM_0_3_UM_EVERY0_1L_AIR);
  Serial.print("PM2.5 ");
  Serial.println(part_count);
  delay(1000);
  return part_count;
}

// LCD-related Functions

void lcd_display(int type)
{
  /**
  *@brief displays information on the LCD1602 Module
  *@param type - determines what information will be displayed
  *         1: AQI Info
  *         default: Raw Data (from PM2.5 and MQ135 sensor)
  *@note the LCD's blink turns off before displaying info.
  **/
  lcd.blink_off();
  switch(type)
  {
    case 1:
      display_aqi();
      break;
    default:
      display_data();
      break;
  }
}

void display_data() 
{
  lcd.home();
  lcd.clear();
  lcd.print(pm);
  lcd.setCursor(0,1);
  lcd.print(mq_ppm);
}

void display_aqi()
{
  lcd.home();
  lcd.clear();
  lcd.print("Overall AQI: ");
  lcd.setCursor(0,1);
  lcd.print(aqi);

  lcd.setCursor(7,1);

  if (fan == true)
  {
    lcd.print("Filtering");
  }
    else
  {
    lcd.print("         "); // Print spaces to clear the text
  }
}

// Relay / Fan functions

void fan_on()
{
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Fan On ");
}

void fan_off()
{
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Fan Off ");
}

// I2C Bus Functions

void reset_I2C_Bus()
{
  Wire.end();
  delay(100);
  Wire.begin();
}