long frontDuration, frontDistance, backDuration, backDistance;

void checkDistance() {
  PORTD &= B01111111; // SET PIN 7 LOW (LEFT TRIGGER)
  delayMicroseconds(5);
  PORTD |= B10000000; // SET PIN 7 HIGH (LEFT TRIGGER)
  delayMicroseconds(10);
  PORTD &= B01111111; // SET PIN 7 LOW (LEFT TRIGGER)
  DDRB &= B111110; // DECLARE PIN 8 INPUT (LEFT ECHO)
  frontDuration = pulseIn(8, HIGH);
  frontDistance = (frontDuration * 0.0343) / 2;
  Serial.print("Distance from front wall: ");
  Serial.print(frontDistance);
  Serial.println("cm.");

  delay(60); // wait for first ping over
  
  PORTB &= B101111; // SET PIN 12 TO LOW (RIGHT TRIGGER)
  delayMicroseconds(5);
  PORTB |= B010000; // SET PIN 12 TO HIGH (RIGHT TRIGGER)
  delayMicroseconds(10);
  PORTB &= B101111; // SET PIN 12 TO LOW (RIGHT TRIGGER)
  DDRB &= B011111; // DECLARE PIN 13 AS INPUT RIGHT ECHO
  backDuration = pulseIn(13, HIGH);
  backDistance = (backDuration * 0.0343) / 2;
  Serial.print("Distance from back wall: ");
  Serial.print(backDistance);
  Serial.println("cm.");
  delay(1000);
}

void setup() {
  Serial.begin (57600);
  DDRD |= B10000000; // DECLARE PIN 7 OUTPUT
  DDRB |= B010000; // DECLARE PIN 12 OUTPUT
}

void loop() {
  checkDistance();
}
