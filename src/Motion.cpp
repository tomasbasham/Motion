#include "Motion.h"
#include <Wire.h>

Motion::Motion(uint8_t mask) {

  // Set the accelerometer mode and sensitivity.
  this->setMode(mask);

  // Calibrate the accelerometer.
  this->calibrate();
}

int Motion::setMode(uint8_t mask) {
  int error = this->write(MODE_CONTROL, &mask);
  if(error != SUCCESS) {
    return error;
  }

  this->mode = mask & 0x03; // Extract the measurement mode from the mask.
  this->sensitivity = mask & 0x0C; // Extract the sensitivity from the mask.

  return SUCCESS;
}

int Motion::setInterrupts(uint8_t mask) {
  int error = this->write(CTL1, &mask);
  if (error != SUCCESS) {
    return error;
  }

  return SUCCESS;
}

int Motion::clearInterruptLatch() {
  int error = this->write(INTRESET, (uint8_t *)CLEAR);
  if (error != SUCCESS) {
    return error;
  }

  error = this->write(INTRESET, (uint8_t *)ENABLE);
  if (error != SUCCESS) {
    return error;
  }

  return SUCCESS;
}

int Motion::orientation(int *x, int *y, int *z) {
  accelerometer xyz;

  // Read 6 bits (XL, XH, YL, YH, ZL, ZH) from the
  // accelerometer registers consecutively.
  int error = this->read(XL_OUT, (uint8_t *)&xyz, 6);
  if (error != SUCCESS) {
    return error;
  }

  // If the sign bit is set, indicating a negative
  // 10 bit number then stretch out the sign bit so
  // that it represents a negative 16 bit integer
  // using 2's complement.
  if (xyz.registers.x_msb & 0x02) {
    xyz.registers.x_msb |= 0xFC;
  }

  if (xyz.registers.y_msb & 0x02) {
    xyz.registers.y_msb |= 0xFC;
  }

  if (xyz.registers.z_msb & 0x02) {
    xyz.registers.z_msb |= 0xFC;
  }

  // Copy the results into the buffers passed
  // to the method.
  *x = xyz.values.x;
  *y = xyz.values.y;
  *z = xyz.values.z;

  return SUCCESS;
}

uint8_t Motion::getMode() {
  return this->mode;
}

uint8_t Motion::getSensitivity() {
  if (this->sensitivity == GS_8G_MODE) {
    return 0x10;
  } else if (this->sensitivity == GS_2G_MODE) {
    return 0x40;
  } else if (this->sensitivity == GS_4G_MODE) {
    return 0x20;
  }

  // If for some reason everything goes wrong, return 1.
  return 0x01;
}

void Motion::calibrate() {
  int i = 0;

  // Here I need to calculate and calibrate the offset so that
  // the accelerometer always reads 0,0,64 when flat. This assumes
  // that the accelerometer is flat upon configuration which is
  // most likely not going to be the case.
  accelerometer acc;
  uint8_t sensitivity = this->getSensitivity();

  do {
    // Grab orientation values.
    this->orientation (&acc.values.x, &acc.values.y, &acc.values.z);

    // The sensor wants double values.
    acc.values.x += 2 * -acc.values.x;
    acc.values.y += 2 * -acc.values.y;
    acc.values.z += 2 * -(acc.values.z - sensitivity);

    // Write the offset.
    this->write(XL_OFF, (uint8_t *)&acc, 6);

    // Wait 200 miliseconds before repeating the calibration.
    delay(200);
  } while (++i < 3);
}

int Motion::read(uint8_t address, uint8_t *buffer, int size) {
  int i = 0;

  // Begin a transmission to the I2C slave device with the address of
  // the accelerometer. This allows for the device to be open to send
  // and receive data to/from the Arduino microcontroller.
  Wire.beginTransmission(ACCELEROMETER);

  // Tell the device which register we wish to write to.
  uint8_t n = Wire.write(address);
  if (n != 1) {
    return NOWRITE;
  }

  // Stop transmission to the device, holding the I2C bus.
  n = Wire.endTransmission(false);
  if (n != 0) {
    return n;
  }

  // Again begin a transmission with the accelerometer and request from
  // the device n number of bytes. After reading from the device the
  // I2C bus will be released.
  n = Wire.requestFrom(ACCELEROMETER, size);
  if (n != size) {
    return BADLENGTH;
  }

  // If there are bytes to read then add them to the buffer.
  while (Wire.available () > 0 && i < size) {
    buffer[i++] = Wire.read();
  }

  if (i != size) {
    return BADLENGTH;
  }

  return SUCCESS;
}

int Motion::write(uint8_t address, const uint8_t *buffer, int size) {

  // Begin a transmission to the I2C slave device with the address of
  // the accelerometer. This allows for the device to be open to send
  // and receive data to/from the Arduino microcontroller.
  Wire.beginTransmission(ACCELEROMETER);

  // Tell the device which register we wish to write to.
  uint8_t n = Wire.write(address);
  if (n != 1) {
    return NOWRITE;
  }

  // Write the value.
  n = Wire.write(buffer, size);
  if (n != size) {
    return NOWRITE;
  }

  // Stop transmission to the device, releasing the I2C bus.
  int error = Wire.endTransmission();
  if (error != SUCCESS) {
    return error;
  }

  return SUCCESS;
}
