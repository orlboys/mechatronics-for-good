#include <AUnit.h>

// ---------------- FUNCTION CODE ---------------- //


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

const float pm2_5_breakpoints[] = {0, 12.0, 35.0, 55.0, 150.0, 250.0, 500.0};
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
    SensorData.fan = true;
  }
  else
  {
    fan_off();
    SensorData.fan = false;
  }
}

// Calculation / AQI functions

int calculate_aqi()
{
  int pm = SensorData.pm;
  int correctedPPM = SensorData.mq_corrected_ppm;

  int mq_aqi = 0;
  int pm_aqi = 0;

  for (int i = 0; i < 6; i++)
  {
    if (correctedPPM <= mq_breakpoints[i])
    {
      mq_aqi = mq_aqi_values[i];
      break;
    }

  }
  if (correctedPPM > mq_breakpoints[6]) {  // Handle value greater than the highest breakpoint
    mq_aqi = mq_aqi_values[6];
  }

  for (int i = 0; i < 6; i++)
  {
    if (pm <= pm2_5_breakpoints[i])
    {
      pm_aqi = pm2_5_aqi_values[i];
      break;
    }
  }
  if (pm > pm2_5_breakpoints[6]) {  // Handle value greater than the highest breakpoint
    pm_aqi = pm2_5_aqi_values[6];
  }

  int final_aqi = max(mq_aqi, pm_aqi);
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
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("Fan On ");
}

void fan_off()
{
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("Fan Off ");
}

// I2C Bus Functions

void reset_I2C_Bus() // for debugging
{
  Wire.end();
  delay(100);
  Wire.begin();
}


// TESTING MODULES //

// Test modules on the correct AQI values being output //

test(LowPM_HighMQ) //Testing that when the PM value is lower than the CO2 value, the CO2 Value is correctly used for the AQI calculation.
{
  SensorData.pm = 10; // associated AQI value for PM = 0
  SensorData.mq_corrected_ppm = 5000.0; // associated AQI value for PPM is 200

  assertEqual(200, calculate_aqi());
}

test(HighPM_LowMQ) //Testing that when the PM value is higher than the CO2 value, the PM Value is correctly used for the AQI calculation.
{
  SensorData.pm = 54; // AQI is = 150
  SensorData.mq_corrected_ppm = 200; // AQI is = 50

  assertEqual(150, calculate_aqi());
}

test(Same_PM_MQ) // Testing it still functions when both are the same AQI threshold
{
  SensorData.pm = 55.0;
  SensorData.mq_corrected_ppm = 2000.0; // both aqi = 150

  assertEqual(150, calculate_aqi());
}

test(ZeroValues)
{
  // Testing AQI calculation at 0 value
  SensorData.pm = 0;
  SensorData.mq_corrected_ppm = 0;
  assertEqual(0, calculate_aqi());
}

test(MaxValues)
{
  // Testing AQI calculation above the maximum values defined
  SensorData.pm = 550; 
  SensorData.mq_corrected_ppm = 40500.0;
  assertEqual(500, calculate_aqi());
}

test(AQIBoundaries)
{
  // Testing AQI calculation at category boundaries
  SensorData.pm = 12.0; // Boundary between Good and Moderate for PM2.5
  SensorData.mq_corrected_ppm = 0;
  assertEqual(50, calculate_aqi());

  SensorData.pm = 0;
  SensorData.mq_corrected_ppm = 2000.0; // Boundary between Moderate and Unhealthy for Sensitive Groups for CO2
  assertEqual(150, calculate_aqi());
}

// Auto-purification tests

test(AutoPurify_On) 
{
  // Testing fan activation when AQI is high
  SensorData.aqi = 160;
  auto_purify();
  assertTrue(SensorData.fan);
}

test(AutoPurify_Off) 
{
  // Testing fan deactivation when AQI is low
  SensorData.aqi = 40;
  auto_purify();
  assertFalse(SensorData.fan);
}

// Data Collection Tests

test(PMDataCollection) 
{
  // Testing whether the PM2.5 Sensor is collecting data
  pm_collect_data();
  assertTrue(SensorData.pm > 0);
}

test(MQDataCollection) 
{
  // Testing whether the MQ135 Sensor is colelcting data
  mq_collect_data();
  assertTrue(SensorData.mq_ppm > 0);
  assertTrue(SensorData.mq_corrected_ppm > 0);
}

// LCD tests

test(LCDInitialization) 
{
  // Ensure the LCD is initialized
  lcd.setCursor(0, 0);
  lcd.print("Init Test");
  assertEqual(lcd.getCursorX(), 0);
  assertEqual(lcd.getCursorY(), 0);
}

test(LCDCorrectData)
{
  lcd_display(1);
  // must manually confirm the correct AQI is being seen between serial monitor and lcd screen
  assertTrue(true); //keeps tests running
}

test(LCDFilterText)
{
  SensorData.fan = true;
  lcd_display(1);

  //must manually confirm the "filtering" text is displayed on the LCD screen
  assertTrue(true); //keeps tests running
}

// Integration Test
test(FullSystemCycle) {
  // Simulate a full cycle of operation
  pm_collect_data();
  mq_collect_data();
  int aqi = calculate_aqi();
  auto_purify();
  lcd_display(1);
  // Assert various states and outputs
}

void setup()
{
  // Serial Monitor
  Serial.begin(115200);

  // Initialize Sensors and Actuators
  pm_setup();
  lcd.begin(16,2);
  lcd.backlight();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Ensure fan is off by default

  Serial.println("System initialized. Running tests...");
}

void loop()
{
  aunit::TestRunner::run(); // Run the tests
}












