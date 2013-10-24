/**
 * Accelerometer
 *
 * A program that reads orientation and inertia
 * data from an accelerometer.
 *
 * Hardware:      Freescale MMA7455
 * Concept Usage: Wii Controller Clone
 */

#include <Motion.h>
#include <Wire.h>

// Acceleromerter.
Motion accelerometer(GS_2G_MODE|MEASUREMENT);

// Global storage for efficiency.
byte sensitivity;
double dX, dY, dZ;
int error;

void setup() {
  Serial.begin(9600);

  // Grab the sensitivity.
  sensitivity = accelerometer.getSensitivity();

  Serial.print(F("Initialised with sensitivity: "));
  Serial.println(sensitivity);
}

void loop() {
  int x, y, z;
  error = accelerometer.orientation(&x, &y, &z);

  // Calculate the 'g' values.
  dX = (double)x / sensitivity;
  dY = (double)y / sensitivity;
  dZ = (double)z / sensitivity;

  Serial.print(F("G-force = "));
  Serial.print(dX, 3);
  Serial.print(F(", "));
  Serial.print(dY, 3);
  Serial.print(F(", "));
  Serial.print(dZ, 3);
  Serial.print(F(" (error: "));
  Serial.print(error, DEC);
  Serial.println(F(")"));

  delay(1000);
}
