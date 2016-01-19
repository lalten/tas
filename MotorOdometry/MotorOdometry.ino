/* Motor Odometry
 *
 * This sketch reads in three hall encoder sensor lines from a sensored
 * brushless motor. It decodes the tri-phase quadrature signal and publishes it
 * via rosserial to the ROS network, where it can be used as odometry signal.
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

// This define is required when an native-USB Arduino like the Micro is used!
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int32.h>

// IO pin numbers
static const uint8_t pinA = 3;
static const uint8_t pinB = 2;
static const uint8_t pinC = 7;

// WHen this time has passed, publish a message even though nothing changed
static const uint16_t pub_max_idle_ms = 5000;

// ROS objects
ros::NodeHandle nh;
std_msgs::Int32 msg;
ros::Publisher pub("motor_encoder", &msg);

// The three hall encoder data lines
volatile bool A, B, C;
// The encoder counter. It's 32b, because with 16b, at 17000rpm (max motor speed) it would overflow 3 times a second
volatile int32_t counter = 0;
// Flag telling if new data is available
volatile bool dirty = false;
// Flag telling if there was a error during quadrature decoding
volatile bool error = false;

// Time of last message having been published
unsigned long last_sent = 0;

// counter for toggling the LED with its LSB
uint8_t ledcounter = 0;

// There was an error, handle this gracefully
void error_reset() {
	// temporarily disable interrupts
	uint8_t oldSREG = SREG;
	cli();

	// Re-read channel values
	A = digitalRead(pinA);
	B = digitalRead(pinB);
	C = digitalRead(pinC);
	dirty = false;

	// re-enable interrupts
	SREG = oldSREG;
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

// The setup function is called once at startup of the sketch
void setup() {

	// Set up IOs
	pinMode(pinA, INPUT);
	pinMode(pinB, INPUT);
	pinMode(pinC, INPUT);
	TX_RX_LED_INIT;
	digitalWrite(pinA, HIGH); // activate int pull-up
	digitalWrite(pinB, HIGH);
	digitalWrite(pinC, HIGH);

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

	// save time
	unsigned long now = millis();

	// If new data is available, publish it
	// Also publish 0s every second if no update happens
	if (dirty || now - last_sent > pub_max_idle_ms) {
		dirty = false;
		msg.data = counter;
		ledcounter += counter;
		counter = 0;
		pub.publish(&msg);
		last_sent = now;
	}

	// Handle ROS
	nh.spinOnce();

	// TXLED blinks when data is published
	if(now - last_sent < 100)
		TXLED0; //on
	else
		TXLED1; //off

	// RXLED blinks with changing counter
	if(ledcounter & 0x01)
		RXLED0;
	else
		RXLED1;


}

