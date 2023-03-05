#include <Wire.h> // I2C
#include <assert.h>

constexpr bool DEBUG = false;

constexpr uint8_t I2C_ADDR = 0x10;

constexpr size_t NUM_ULTRASONICS = 7;
constexpr unsigned int TRIGGER = 12;
constexpr unsigned int ECHO[NUM_ULTRASONICS] = {
  2, // right side
  9, // front right housing
  4, // front right side
  5, // back
  6, // front left side
  7, // front left housing
  8, // left side
};

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

  if constexpr (DEBUG) {
    Serial.begin(9600);
  }
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

  // wait for all sensors to send pulse
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
      if (digitalRead(ECHO[i]) == HIGH) {
        all_complete = false;
      } else if (!pulses[i].complete) {
        pulses[i].end = micros();
        pulses[i].complete = true;
      }
    }
  }

  // compute distance from pulse duration
  for (size_t i = 0; i < NUM_ULTRASONICS; ++i) {
    long duration = pulses[i].end - pulses[i].start;
    // divide by 2 since sound wave has to travel out and back
    ultrasonic_distances[i] = duration * SPEED_OF_SOUND / 2;

    if constexpr (DEBUG) {
      Serial.print("Sensor " + String(i) + ": ");
      Serial.println(ultrasonic_distances[i]);
    }
  }
}

void handle_i2c_request() {
  // send distances over i2c
  size_t ret = Wire.write(
    (const char*)(&ultrasonic_distances),
    NUM_ULTRASONICS * sizeof(ultrasonic_distances[0])
  );
  assert(ret == NUM_ULTRASONICS * sizeof(ultrasonic_distances[0]));
}
