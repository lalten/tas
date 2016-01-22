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
#include <tas_odometry/Encoder.h>
#include <util/atomic.h>

// IO pin numbers
static const uint8_t pinA = 3;
static const uint8_t pinB = 2;
static const uint8_t pinC = 7;

// When this time (ms) has passed, publish a message even though nothing changed
static const uint16_t pub_0vel_after = 100;
bool published_0vel = false;

// ROS objects
ros::NodeHandle nh;
tas_odometry::Encoder msg;
ros::Publisher pub("motor_encoder", &msg);

// The three hall encoder data lines
volatile bool A, B, C;
// The encoder counter. It's 32b, because with 16b, at 17000rpm (max motor speed) it would overflow 3 times a second
volatile int32_t counter = 0;
// Timestamp of last encoder tick (in 1us, 4us resolution)
volatile unsigned long last_tick = 0;
// Time between ticks that actually get published
volatile unsigned long accumulated_duration = 0;
// Flag telling if new data is available
volatile bool dirty = false;

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
	// Measure inter-tick duration right at ISR entry and add it to time since last publishing
	accumulated_duration += micros() - last_tick;
	last_tick = micros();
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
	accumulated_duration += micros() - last_tick;
	last_tick = micros();
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
	accumulated_duration += micros() - last_tick;
	last_tick = micros();
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
	msg.header.frame_id = "base_motor";
	msg.header.seq = 0;
	nh.initNode();
	nh.advertise(pub);
}

// The loop function is called in an endless loop
void loop() {

	// save time
	unsigned long now = millis();

	// If new data is available, publish it
	if (dirty) {
		msg.header.seq ++;
		msg.header.stamp = nh.now(); // like ros::Time::now(), but optimized for HW
		// Don't let race conditions happen!
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			msg.duration = accumulated_duration;
			msg.encoder_ticks = counter;
			counter = 0;
			accumulated_duration = 0;
		}
		ledcounter += msg.encoder_ticks;
		dirty = false;
		published_0vel = false;
		last_sent = now;
		pub.publish(&msg);
	} else
	// If no update happens, after 100ms publish 0 - once!
	if (!published_0vel && now - last_sent > pub_0vel_after) {
		published_0vel = true;
		msg.header.seq ++;
		msg.header.stamp = nh.now();
		msg.duration = 1; // avoid accidental divisions by zero...
		msg.encoder_ticks = 0;
		pub.publish(&msg);
	}

	// Handle ROS
	nh.spinOnce();

	// TXLED blinks when data is published
	if(now - last_sent < 20)
		TXLED0; //on
	else
		TXLED1; //off

	// RXLED blinks with changing counter
	if(ledcounter & 0x01)
		RXLED0;
	else
		RXLED1;


}

