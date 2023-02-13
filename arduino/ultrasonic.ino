#include <Wire.h> // I2C

constexpr uint8_t I2C_ADDR = 0x10;

constexpr unsigned int TRIGGER = 50;
constexpr size_t NUM_ULTRASONICS = 2;
constexpr unsigned int ECHO[NUM_ULTRASONICS] = {26, 28};

constexpr double SPEED_OF_SOUND = 343.0 * 100 / 1000000; // m/s to cm/s to cm/microsecond

// data that is sent over i2c on request
long ultrasonic_distances[NUM_ULTRASONICS];

void setup() {
  // set mode of trigger and echo pins
  pinMode(TRIGGER, OUTPUT);
  for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
    pinMode(ECHO[i], INPUT);
  }

  // start i2c communication as slave
  Wire.begin(I2C_ADDR);
  Wire.onRequest(handle_i2c_request); // register event
  Serial.begin(9600); // TODO: remove me
}

struct PulseInfo {
  long start = micros();
  long end;
  bool complete = false;
};

void loop() {
  // clear trigger
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);

  // set trigger
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  // wait for all triggers to go high
  bool all_high = false;
  while (!all_high) {
    all_high = true;
    for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
      if (digitalRead(ECHO[i]) == LOW) {
        all_high = false;
      }
    }
  }

  // keep track of start and end of each pulse
  PulseInfo pulses[NUM_ULTRASONICS];

  // wait for all echos to go low, keep tracking of duration of pulse
  bool all_complete = false;
  while (!all_complete) {
    all_complete = true;
    for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
      if (digitalRead(ECHO[i]) == LOW) {
        pulses[i].end = micros();
        pulses[i].complete = true;
      } else {
        all_complete = false;
      }
    }
  }

  // compute distance from pulse duration
  for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
    long duration = pulses[i].end - pulses[i].start;
    // divide by 2 since sound wave has to travel out and back
    ultrasonic_distances[i] = duration * SPEED_OF_SOUND / 2;

    // TODO: remove me
    // Serial.print("Sensor " + String(i) + ": ");
    // Serial.print(ultrasonic_distances[i]);
    // Serial.print("  start at ");
    // Serial.println(pulses[i].start);
  }
}

#include <assert.h>

constexpr unsigned int REQUEST_ULTRASONICS = 1;

void handle_i2c_request() {
  // Serial.println("Got i2c request!");

  // Serial.print("Sending back: ");
  // for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
  //   Serial.print(ultrasonic_distances[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // send distances over i2c
  Wire.write(
    (const char*)(&ultrasonic_distances),
    NUM_ULTRASONICS * sizeof(ultrasonic_distances[0])
  );
}
