#include <SPI.h>
#include "max6675.h"

// Pin definitions
const int ledPin = 9; // Digital pin connected to the LED
const int pwmPin1 = 10; // PWM pin connected to heating elements
const int pwmPin2 = 5; // PWM pin connected to the 12V motor
const int csPin = 8; // Chip Select pin for MAX6675ISA
const int soPin = 12; // Data Out (DO) pin for SPI communication
const int sckPin = 13; // Clock (SCK) pin for SPI communication

// Temperature sensor
MAX6675 thermocouple(sckPin, csPin, soPin);

double setpoint = 28.0; // Target temperature (Celsius)
double Kp = 2, Ki = 0.5, Kd = 0; // PID coefficients (tune these parameters as needed)

double lastError = 0, integral = 0;
unsigned long lastTime = 0; // Timer for PID control

unsigned long lastReadTime = 0; // Timer for sensor readings
const int readInterval = 500; // Read sensor data every 0.5 second

void setup() {
 pinMode(ledPin, OUTPUT); // Set the LED pin as output
 pinMode(pwmPin1, OUTPUT);
 pinMode(pwmPin2, OUTPUT);
 analogWrite(pwmPin1, 128); // Start PWM with 0 (heaters off)
 analogWrite(pwmPin2, 128); // Start motor PWM with 0 (motor off)

 Serial.begin(9600);

 // Print initial message to serial monitor
 Serial.println("Heat Module Control via Serial Communication");
 Serial.print("Setpoint: ");
 Serial.println(setpoint);

 lastTime = millis(); // Initialize the lastTime variable
}

double pidControl(double temperature, double setpoint) {
 unsigned long currentTime = millis();
 double timeChange = (double)(currentTime - lastTime) / 1000.0; // Time change in seconds

 double error = setpoint - temperature;
 integral += error * timeChange;

 // Anti-windup: limit the integral term
 integral = constrain(integral, -50, 50);

 double derivative = (error - lastError) / timeChange;
 double output = Kp * error + Ki * integral + Kd * derivative;

 lastError = error;
 lastTime = currentTime;

 return output;
}

void loop() {
 unsigned long currentTime = millis();
 if (currentTime - lastReadTime >= readInterval) {
 lastReadTime = currentTime;

 double temperature = thermocouple.readCelsius();

 Serial.print("Temperature: ");
 Serial.print(temperature);
 Serial.println(" C");

 if (temperature < setpoint) {
 // Apply PID control to the heating elements
 double outputPower = pidControl(temperature, setpoint);

 // Map the PID output to a PWM range
 int pwmValue = (int)constrain(map(outputPower, 0, 100, 0, 255), 0, 255);

 // Apply PWM value to the heating elements
 analogWrite(pwmPin1, pwmValue);

 // Debugging PWM value
 Serial.print("Heating PWM Value: ");
 Serial.println(pwmValue);

 // Ensure the cooling motor is off
 analogWrite(pwmPin2, 255);
 } else {
 // Turn off the heating elements
 analogWrite(pwmPin1, 0);

 // Turn on the cooling motor
 analogWrite(pwmPin2, 128); // Full speed for cooling

 // Debugging motor status
 Serial.println("Cooling Motor ON");
 }
 }
}
