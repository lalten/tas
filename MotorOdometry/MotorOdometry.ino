/* Motor Odometry
 *
 * This sketch reads in three hall encoder sensor lines from a sensored
 * brushless motor. It decodes the tri-phase quadrature signal and publishes it
 * via rosserial to the ROS network, where it can be used as odometry signal.
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

#include <ros.h>
#include <std_msgs/Int32.h>

// IO pin numbers
static const uint8_t pinA = 0;
static const uint8_t pinB = 1;
static const uint8_t pinC = 2;
static const uint8_t pinLED = 13;

// Ros objects
ros::NodeHandle nh;
std_msgs::Int32 msg;
ros::Publisher pub("motor_odom", &msg);

// The three hall encoder data lines
volatile bool A, B, C;
// The "step" counter. It's 32b, because with 16b, at 17000rpm (max motor speed) it would overflow 3 times a second
volatile int32_t counter = 0;
// Flag telling if new data is available
volatile bool dirty = false;
// Flag telling if there was a error during quadrature decoding
volatile bool error = false;

// There was an error, handle this gracefully
void error_reset() {
	// temporarily disable interrupts
	cli();

	// Re-read channel values
	A = digitalRead(pinA);
	B = digitalRead(pinB);
	C = digitalRead(pinC);
	dirty = false;

	// re-enable interrupts
	sei();
}

void isr_a() {
	if (A) { // channel was high before, so this is a falling edge
		if (B && C) // the other channels are both high
			counter++; // this indicates forward motion
		else if (!B && !C)
			counter--; // both low means backwards motion
		else
			// any other state indicates either input overflow or wrong logic (interchanged wires...)
			error_reset();
	} else { // rising edge
		if (!B && !C)
			counter++;
		else if (B && C)
			counter--;
		else
			error_reset();
	}
	// Update channel value and dirty flag
	A = digitalRead(pinA);
	dirty = true;
}

void isr_b() {
	if (B) {
		if (!A && C)
			counter++;
		else if (A && !C)
			counter--;
		else
			error_reset();
	} else {
		if (A && !C)
			counter++;
		else if (!A && C)
			counter--;
		else
			error_reset();
	}
	B = digitalRead(pinB);
	dirty = true;
}

void isr_c() {
	if (C) {
		if (!A && !B)
			counter++;
		else if (A && B)
			counter--;
		else
			error_reset();
	} else {
		if (A && B)
			counter++;
		else if (!A && !B)
			counter--;
		else
			error_reset();
	}
	C = digitalRead(pinC);
	dirty = true;
}

//The setup function is called once at startup of the sketch
void setup() {

	// Set up IOs
	pinMode(pinA, INPUT);
	pinMode(pinB, INPUT);
	pinMode(pinC, INPUT);

	// Read initial logic states
	A = digitalRead(pinA);
	B = digitalRead(pinB);
	C = digitalRead(pinC);

	// Set up ISR vectors
	attachInterrupt(digitalPinToInterrupt(pinA), isr_a, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pinB), isr_b, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pinC), isr_c, CHANGE);

	// Setup ROS
	nh.initNode();
	nh.advertise(pub);
}

// The loop function is called in an endless loop
void loop() {

	// Blink LED with LSB of counter
	digitalWrite(pinLED, counter & 0x01);

	// If new data is available, publish it
	if (dirty) {
		dirty = false;
		msg.data = counter;
		counter = 0;
		pub.publish(&msg);
	}

	nh.spinOnce();
}

