#ifndef MECH_FOR_GOOD_H
#define MECH_FOR_GOOD_H

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

// Function declarations
void collect_data();
void mq_collect_data();
void pm_collect_data();
int calculate_aqi();
void auto_purify();
void pm_setup();
void lcd_display();
void display_data();
void display_aqi();
void fan_on();
void fan_off();

#endif