#include <particle_filter.h>

static const size_t particle_count = 100;

static struct particle_filter filter;
static float particles[particle_count];
static float weights[particle_count];
static float scratch[particle_count];

static float read_position_sensor()
{
	/*
	 * Replace this placeholder with a position reading from your sensor.
	 * The measurement, movement, and filter noise values must use the same
	 * unit.
	 */
	return analogRead(34) * (100.0f / 4095.0f);
}

void setup()
{
	Serial.begin(115200);

	/*
	 * Start near 50 units with a 10-unit spread. We expect about 0.5 units
	 * of unmodelled movement and 2 units of measurement noise.
	 */
	if (particle_filter_init(&filter, particles, weights, scratch,
				 particle_count, 50.0f, 10.0f, 0.5f,
				 2.0f, 1234U)) {
		Serial.println("Particle filter initialization failed");
		while (true)
			delay(1000);
	}
}

void loop()
{
	float known_movement = 0.0f;
	float measurement = read_position_sensor();

	/*
	 * Replace known_movement with the state change predicted by your
	 * application since the previous loop. Call predict before update.
	 */
	if (particle_filter_predict(&filter, known_movement) ||
	    particle_filter_update(&filter, measurement)) {
		Serial.println("Particle filter update failed");
		delay(1000);
		return;
	}

	Serial.print("measurement=");
	Serial.print(measurement);
	Serial.print(" estimate=");
	Serial.println(particle_filter_get_estimate(&filter));

	delay(100);
}
