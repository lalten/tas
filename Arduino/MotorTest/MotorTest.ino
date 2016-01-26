/* Motor Odometry Test
 *
 * This sketch reads in three hall encoder sensor lines and prints changes
 *
 * Laurenz 2015-01
 * ga68gug / TUM LSR TAS
 */

// IO pin numbers
static const uint8_t pinA = 2;
static const uint8_t pinB = 3;
static const uint8_t pinC = 7;


void isr_a() {
  if(digitalRead(pinA))
    Serial.println("A-");
  else
    Serial.println("A+");
}

void isr_b() {
  if(digitalRead(pinB))
    Serial.println("B-");
  else
    Serial.println("B+");
}

void isr_c() {
  if(digitalRead(pinC))
    Serial.println("C-");
  else
    Serial.println("C+");
}

// The setup function is called once at startup of the sketch
void setup() {

  // Set up IOs
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinC, INPUT);
  digitalWrite(pinA, HIGH); // activate int pull-up
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  attachInterrupt(digitalPinToInterrupt(pinA), isr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isr_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinC), isr_c, CHANGE);
}

// The loop function is called in an endless loop
void loop() {

}

