#include <AUnit.h>
#include "mech_for_good.ino"

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

// Test modules on the correct AQI values being output //

class AQIFixture : public aunit::TestOnce
{
  /**
  * @brief defining the common setup variables going to be used in all AQI testing-modules.
  * @n This is the PM value and the Correct CO2 PPM value input.
  */

  protected:
    void setup() override
    {
      SensorData.pm = 0;
      SensorData.mq_corrected_ppm = 0;
    }
}

testF(AQIFixture, LowPM_HighMQ) //Testing that when the PM value is lower than the CO2 value, the CO2 Value is correctly used for the AQI calculation.
{
  SensorData.pm = 10; // associated AQI value for PM = 0
  SensorData.mq_corrected_ppm = 5000.0 // associated AQI value for PPM is 200

  assertEqual(200, calculate_aqi())
}