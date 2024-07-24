#include <SPI.h>

// MAX6675 pins
const int MAX6675_CS = 8;
const int MAX6675_SCK = 13;
const int MAX6675_SO = 12;

// PWM pin for heating element
const int PWM_pin = 10;
const int pwmPin2 = 5; // PWM pin connected to the 12V motor

// Offset variables
// Coefficients for the intervals
float a[] = {6.6, 8.5, 8.5, 12.2, 14.0, 16.0, 18.0};
float b_coef[] = {0.2712332531776024, 0.027533493644795598, 0.18863277224321534, 0.32793541738234283, 0.14962555822741333, 0.2135623497080041, 0.19612504294057026};
float c[] = {-2.2204460492503132e-17, -0.024369975953280654, 0.04047990381312263, -0.02654963929920988, 0.008718653383716926, -0.002324974235657847, 0.0005812435589144619};
float d[] = {-0.0008123325317760211, 0.0021616626588801097, -0.0022343181037444166, 0.0011756097560975602, -0.0003681209206458258, 9.687392648574364e-05, -1.9374785297148728e-05};

float x[] = {30, 40, 50, 60, 70, 80, 90, 100};

// Variables
float temperature_read = 0.0;
float offset = 20;
float set_temperature = 40 + offset;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float coolThreshold = 10;
int PID_value = 0;
bool cooling = true; // Initially set to cooling mode

// PID constants
float kp = 9.1, ki = 0.3, kd = 1.8;
float PID_p = 0, PID_i = 0, PID_d = 0;

// Function to evaluate the spline
float evaluateSpline(float x_val) {
  int n = sizeof(a) / sizeof(a[0]);
  for (int i = 0; i < n; i++) {
    if (x_val >= x[i] && x_val <= x[i + 1]) {
      float dx = x_val - x[i];
      return a[i] + b_coef[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }
  }
  return NAN; // x is out of the interpolation range
}

double readThermocouple() {
  uint16_t v;
  digitalWrite(MAX6675_CS, LOW);
  delay(1);
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) {
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;
  }
  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;
  // The remaining bits are the number of 0.25 degree (C) counts
  return v * 0.25;
}

void setup() {
  pinMode(PWM_pin, OUTPUT);
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  digitalWrite(MAX6675_CS, HIGH);
  // Set PWM frequency for pin 10
  TCCR1B = TCCR1B & B11111000 | 0x01; // Pin 10 PWM frequency of 31.37 kHz
  // Initialize Serial communication
  Serial.begin(9600);
  // Ask the user for the desired temperature
  Serial.println("Enter the desired temperature:");
  while (!Serial.available()) {
    // Wait for user input
  }
  float user_temperature = Serial.parseFloat();
  offset = evaluateSpline(user_temperature);
  Serial.println("offset:");
  Serial.println(offset);
  set_temperature = user_temperature + offset;

  analogWrite(pwmPin2, 0); // Start cooling

  // Clear any remaining serial input
  while (Serial.available() > 0) {
    Serial.read();
  }
  // Set the initial time
  Time = millis();
}

void loop() {
  // Check for new serial input and update setpoint if available
  if (Serial.available() > 0) {
    Serial.println("Enter the desired temperature:");
    while (!Serial.available()) {
      // Wait for user input
    }
    float user_temperature = Serial.parseFloat();
    offset = evaluateSpline(user_temperature);
    Serial.println("offset:");
    Serial.println(offset);
    set_temperature = user_temperature + offset;
    cooling = true; // Reset to cooling mode for new setpoint

    // Clear any remaining serial input
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // Read the current temperature
  temperature_read = readThermocouple();

  if (cooling) {
    if (temperature_read > 28) {
      // Cooling down to 28 degrees
      analogWrite(pwmPin2, 0); // Turn on cooler
      analogWrite(PWM_pin, 0); // Turn off heater
    } else {
      // Reached 28 degrees, stop cooling and start heating
      analogWrite(pwmPin2, 255); // Turn off cooler
      cooling = false; // Switch to heating mode
    }
  } else {
    // Heating mode
    PID_error = set_temperature - temperature_read;

    // Calculate the P value
    PID_p = kp * PID_error;

    // Calculate the I value within a range of +-3
    if (-3 < PID_error && PID_error < 3) {
      PID_i = PID_i + (ki * PID_error);
    }

    // Calculate the D value
    timePrev = Time;
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000.0;
    PID_d = kd * ((PID_error - previous_error) / elapsedTime);

    // Final total PID value
    PID_value = PID_p + PID_i + PID_d;

    // Constrain the PID value
    PID_value = constrain(PID_value, 0, 255);

    // Write the PWM signal to the MOSFET on digital pin D10
    analogWrite(PWM_pin, 255 - PID_value);
    previous_error = PID_error;
  }

  // Print temperature values to Serial for plotting
  Serial.print("Set Temp: ");
  Serial.print(set_temperature - offset, 1);
  Serial.print(" C, ");
  Serial.print("Read Temp: ");
  Serial.print(temperature_read, 1);
  Serial.print(" C, ");
  Serial.print("PID Value: ");
  Serial.println(PID_value);

  // Delay to stabilize the loop
  delay(300);
}

