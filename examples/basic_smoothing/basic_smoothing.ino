#include <kalman_filter.h>

struct kalman_filter sensor_filter;

void setup()
{
	Serial.begin(115200);
	kalman_filter_init(&sensor_filter);
}

void loop()
{
	float raw_value = analogRead(34);
	float filtered_value;

	if (kalman_filter_update(&sensor_filter, raw_value,
				 &filtered_value) != KALMAN_FILTER_OK) {
		Serial.println("Filter update failed");
		return;
	}

	Serial.print(raw_value);
	Serial.print(",");
	Serial.println(filtered_value);
	delay(20);
}
