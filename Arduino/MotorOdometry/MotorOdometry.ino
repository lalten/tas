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

// IO pin numbers
static const uint8_t pinA = 2;
static const uint8_t pinB = 3;
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
volatile unsigned long accumulated_duration = 1;
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
	bool A_old = A;
	bool B_old = B;
	bool C_old = C;
	A = digitalRead(pinA);
	B = digitalRead(pinB);
	C = digitalRead(pinC);
	dirty = false;

	// Efficiently build a string like "E: 100/110"
	char err_str[] = {'E', ':', ' ',
			(char) (A_old+'0'), (char) (B_old+'0'), (char) (C_old+'0'),
			'/',
			(char) (A+'0'), (char) (B+'0'), (char) (C+'0'),
			0};

	// re-enable interrupts
	SREG = oldSREG;

	// Write the string to the roslog
	nh.logwarn(err_str);
}

// This ISR (Interrupt Service Routine) will be called whenever there is a change on pinA. Normal code execution is halted until the ISR returns.
void isr_a() {
	// Measure inter-tick duration right at ISR entry and add it to time since last publishing
	accumulated_duration += micros() - last_tick;
	last_tick = micros();
	if (!A) { // Rising edge (b/c A was low before)
		if (B && !C) // if channel B is high, channel C is low
			counter++; // this indicates forward motion
		else if (!B && C) // the other way round?
			counter--; // then backwards
		else
			// any other state indicates either input overflow or wrong logic (interchanged wires...)
			error_reset();
	} else { // Falling edge
		if (!B && C)
			counter++;
		else if (B && !C)
			counter--;
		else
			error_reset();
	}
	// Set dirty flag
	dirty = true;
	// Update channel value
	A = digitalRead(pinA);
}

void isr_b() { // analogous to isr_a
	accumulated_duration += micros() - last_tick;
	last_tick = micros();
	if (!B) {
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
	dirty = true;
	B = digitalRead(pinB);
}

void isr_c() { // analogous to isr_a
	accumulated_duration += micros() - last_tick;
	last_tick = micros();
	if (!C) {
		if (A && !B)
			counter++;
		else if (!A && B)
			counter--;
		else
			error_reset();
	} else {
		if (!A && B)
			counter++;
		else if (A && !B)
			counter--;
		else
			error_reset();
	}
	dirty = true;
	C = digitalRead(pinC);
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
	if (dirty) {
//		uint8_t oldSREG = SREG;
//		cli();		// Temporarily disable interrupts to prevent race conditions
		msg.duration = accumulated_duration;
		msg.encoder_ticks = counter;
		counter = 0;
		accumulated_duration = 0;
//		SREG = oldSREG; // reactivate ISRs
		ledcounter += msg.encoder_ticks;
		dirty = false;
		published_0vel = false;
		last_sent = now;
		pub.publish(&msg);
	} else
	// If no update happens, after 100ms publish 0 - once!
	if (!published_0vel && now - last_sent > pub_0vel_after) {
		published_0vel = true;
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

