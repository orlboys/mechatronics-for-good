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

const float pm2_5_breakpoints[] = {0, 360, 1050, 1650, 4500, 7500, 15000};
const int pm2_5_aqi_values[] = {0, 50, 100, 150, 200, 300, 500};

const float mq_breakpoints[] = {0, 400.0, 1000.0, 2000.0, 5000.0, 10000.0, 40000.0};
const int mq_aqi_values[] = {0, 50, 100, 150, 200, 300, 500};

// Data struct
struct Data 
{
  /**
  * @brief A contained structure holding all data from sensors.
  * @n This simplifies function signatures as a single object, making the code more modular.
  * @n It also increases scalability in the case of extra sensors being added.
  */
  float mq_ppm; // ppm given from MQ135
  float mq_corrected_ppm; // corrected ppm given from MQ135
  uint16_t pm; // pm2.5 um/l Using uint16_t, more efficient on our low-memory arduino
  int aqi; // the AQI value calculated based off of pm and ppm
  bool fan;
};

Data SensorData; // Struct containing all sensor data

// SETUP AND LOOP FUNCTIONS

void setup() // Runs once at the start of the program
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

void loop() // Continuously runs throughout the program
{
  Serial.println("In loop function.");
  delay(1000);
  collect_data();
  SensorData.aqi = calculate_aqi();
  lcd_display(1);
  auto_purify();
}

// Data Collection

void collect_data()
{
  /** @brief An overarching data collection function **/
  SensorData.pm = pm_collect_data();
  mq_collect_data();
}

void mq_collect_data() 
{

  /**
  *@brief get the gas PPM reading from the MQ135 sensor
  *@param temperature a constant temperature to get corrected PPM
  *@param humidity a constant humidity to get corrected PPM
  **/

  SensorData.mq_ppm = mq135_sensor.getPPM(); // We have to change this directly here --> C++ doesn't allow us to return multiple values from one function
  SensorData.mq_corrected_ppm = mq135_sensor.getCorrectedPPM(temperature, humidity);

  // Printing to Serial Monitor

  Serial.print("PPM: ");
  Serial.print(SensorData.mq_ppm);
  Serial.print("\t Corrected PPM: ");
  Serial.print(SensorData.mq_corrected_ppm);
  Serial.println("ppm");
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

// Purification / Relay Functions

void auto_purify()
{
  /**
  * @brief Determines if the AQI is at a point where the purification system must be turned on.
  * @n This limit is 150, calculated in calculate_aqi based on the higher of CO2 and PM2.5 levels.
  **/

  if (SensorData.aqi >= 100)
  {
    fan_on();
    Serial.println("AQI above limit");
    SensorData.fan = true;
  }
  else
  {
    fan_off();
    Serial.println("AQI below limit");
    SensorData.fan = false;
  }
}

// Calculation / AQI functions

int calculate_aqi()
{
  /**
  * @brief Calculates the AQI given the larger of CO2 or PM2.5 material.
  * @details This function compares the CO2 Value (from the MQ135 sensor) and the PM2.5 readings, selects the
  * higher value, and calculates the corresponding AQI. It uses predefined breakpoints and values for both
  * pollutants (defined in mq/pm2_5_breakpoints and mq/pm2_5_values constants).
  * @return int - the calculated AQI range.
  **/

  Serial.print("PM2.5 Reading: ");
  Serial.println(SensorData.pm);
  Serial.print("MQ135 PPM: ");
  Serial.println(SensorData.mq_ppm);
  Serial.print("MQ135 Corrected PPM: ");
  Serial.println(SensorData.mq_corrected_ppm);
  int pm = SensorData.pm;
  int correctedPPM = SensorData.mq_corrected_ppm;

  int mq_aqi = 0;
  int pm_aqi = 0;

  for (int i = 0; i < 6; i++)
  {
    if (correctedPPM <= mq_breakpoints[i]) // If the PPM is less than or equal to the breakpoint above it
    {
      mq_aqi = mq_aqi_values[i]; // Returns the highest breakpoint / aqi value to which the correctedPPM is below it
      break;
    }
  }
  if (correctedPPM > mq_breakpoints[6]) 
  {  // Handle value greater than the highest breakpoint (if you are larger than every breakpoint in the array)
    mq_aqi = mq_aqi_values[6];
  }
  for (int i = 1; i < 7; i++)
  {
    if (pm <= pm2_5_breakpoints[i])
    {
      pm_aqi = pm2_5_aqi_values[i];
      break;
    }
  }

  int final_aqi = max(mq_aqi, pm_aqi); // gets the highest out of the two AQI's
  Serial.print("Final AQI: ");
  Serial.println(final_aqi);

  return final_aqi;
}

// PM2.5-DEDICATED FUNCTIONS

void pm_setup()
{
  /**
  * @brief Ensures the PM2.5 Sensor is connected before beginning the program.
  **/

  while(!particle.begin()){
    Serial.println("NO Devices !");
    delay(1000);
  }
  Serial.println("sensor begin success!");
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
  lcd.print(SensorData.pm);
  lcd.setCursor(0,1);
  lcd.print(SensorData.mq_corrected_ppm);
}

void display_aqi()
{
  lcd.home();
  lcd.clear();
  lcd.print("Overall AQI: ");
  lcd.setCursor(0,1);
  lcd.print(SensorData.aqi);

  lcd.setCursor(7,1);

  if (SensorData.fan == true)
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
  digitalWrite(RELAY_PIN, LOW); // Note it is switched --> Low = On, High = Off
  Serial.println("Fan On ");
}

void fan_off()
{
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Fan Off ");
}

// I2C Bus Functions

void reset_I2C_Bus() // for debugging
{
  Wire.end();
  delay(100);
  Wire.begin();
}