#include <kalman_filter.h>

struct kalman_filter sensor_filter;

void setup()
{
	Serial.begin(115200);

	/*
	 * A larger measurement noise value smooths noisy readings more.
	 * A larger process noise value follows real changes faster.
	 */
	if (kalman_filter_init_with_noise(&sensor_filter, 0.05f, 4.0f) !=
	    KALMAN_FILTER_OK) {
		Serial.println("Invalid filter settings");
		while (true)
			delay(1000);
	}
}

void loop()
{
	float filtered_value;
	float raw_value = analogRead(34);

	if (kalman_filter_update(&sensor_filter, raw_value,
				 &filtered_value) == KALMAN_FILTER_OK)
		Serial.printf("%.2f,%.2f\n", raw_value, filtered_value);

	delay(20);
}
