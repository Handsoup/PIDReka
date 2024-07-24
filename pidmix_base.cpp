#include <SPI.h>
// MAX6675 pins
const int MAX6675_CS = 8;
const int MAX6675_SCK = 13;
const int MAX6675_SO = 12;
// PWM pin for heating element
int PWM_pin = 10;

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
int PID_value = 0;
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

void setup() {
 pinMode(PWM_pin, OUTPUT);
 pinMode(MAX6675_CS, OUTPUT);
@@ -41,6 +64,9 @@ void setup() {
 // Wait for user input
 }
 float user_temperature = Serial.parseFloat();
 offset = evaluateSpline(user_temperature);
 Serial.println("offset:\n");
 Serial.println(offset);
 set_temperature = user_temperature + offset;

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
 set_temperature = user_temperature + offset;
 // Clear any remaining serial input
 while (Serial.available() > 0) {
 Serial.read();
 }
 }
 // First, we read the real value of temperature
 temperature_read = readThermocouple();
 // Next, we calculate the error between the setpoint and the real value
 PID_error = set_temperature - temperature_read;
 // Calculate the P value
 PID_p = kp * PID_error;
 // Calculate the I value in a range on +-3
 if (-3 < PID_error && PID_error < 3) {
 PID_i = PID_i + (ki * PID_error);
 }
 // For derivative, we need real-time to calculate speed change rate
 timePrev = Time; // the previous time is stored before the actual time read
 Time = millis(); // actual time read
 elapsedTime = (Time - timePrev) / 1000.0;
 // Now we can calculate the D value
 PID_d = kd * ((PID_error - previous_error) / elapsedTime);
 // Final total PID value is the sum of P + I + D
 PID_value = PID_p + PID_i + PID_d;
 // We define PWM range between 0 and 255
 if (PID_value < 0) {
 PID_value = 0;
 }
 if (PID_value > 255) {
 PID_value = 255;
 }
 // Now we can write the PWM signal to the MOSFET on digital pin D10
 analogWrite(PWM_pin, 255 - PID_value);
 previous_error = PID_error; // Remember to store the previous error for the next loop
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
