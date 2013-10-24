//
// Motion
//
// An Arduino library to interface with the Freescale MMA7455
// accelerometer module. Given a mask to the constructor, a
// multitude of measurements can be taken.
//

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

// DEVICE
#define ACCELEROMETER 0x1D // Address of the accelerometer

// REGISTERS
#define XL_OUT        0x00 // Register containing the value for X LSB
#define XH_OUT        0x01 // Register containing the value for X HSB
#define YL_OUT        0x02 // Register containing the value for Y LSB
#define YH_OUT        0x03 // Register containing the value for Y HSB
#define ZL_OUT        0x04 // Register containing the value for Z LSB
#define ZH_OUT        0x05 // Register containing the value for Z HSB

#define X_OUT         0x06 // 8-bit register containing the value for X
#define Y_OUT         0x07 // 8-bit register containing the value for Y
#define Z_OUT         0x08 // 8-bit register containing the value for Z

#define STATUS        0x09 // Status ready bit
#define DETECTION     0x0A // Detection source register
#define TEMPERATURE   0x0B // Temperature reading
#define RESERVED      0x0C // Reserved
#define I2C           0x0D // I2C device address
#define USER          0x0E // User information
#define WHOAMI        0x0F // WHOAMI

#define XL_OFF        0x10 // Offset drift X value LSB register
#define XH_OFF        0x11 // Offset drift X value HSB register
#define YL_OFF        0x12 // Offset drift Y value LSB register
#define YH_OFF        0x13 // Offset drift Y value HSB register
#define ZL_OFF        0x14 // Offset drift Z value LSB register
#define ZH_OFF        0x15 // Offset drift Z value HSB register

#define MODE_CONTROL  0x16 // Mode control register
#define INTRESET      0x17 // Interrupt latch reset
#define CTL1          0x18 // Control 1 register
#define CTL2          0x19 // Control 2 register

#define LEVEL_THRESH  0x1A // Level detection threshold limit
#define PULSE_THRESH  0x1B // Pulse detection threshold limit
#define PULSE_DUR     0x1C // Pulse duration
#define LATENCY_TIME  0x1D // Latency time
#define TIME_WINDOW   0x1E // Time window for second pulse
#define RESERVED2     0x1F // Reserved

// MODE $16
#define STANDBY       0x00 // Standby mode
#define MEASUREMENT   0x01 // Measurement mode
#define LEVEL         0x02 // Level detection mode
#define PULSE         0x03 // Pulse detection mode

// SENSITIVITY $16
#define GS_8G_MODE    0x00 // Set sensitivity to 8g (8 or 10 bit)
#define GS_2G_MODE    0x04 // Set sensitivity to 2g (8 bit)
#define GS_4G_MODE    0x08 // Set sensitivity to 4g (8 bit)

// SELF TEST $16
#define STOF          0x00 // Self-test is off
#define STON          0x10 // Self-test is enabled

// SPI MODE $16
#define SPI_4         0x00 // SPI is 4 wire mode
#define SPI_3         0x20 // SPI is 3 wire mode

// DATA READY $16
#define DRDY_PIN      0x00 // Send ready status to INT1 pin
#define DRDY_NO_PIN   0x40 // Do not send ready status to INT1 pin

// LATCH $17
#define CLEAR         0x03 // Clear both latches
#define ENABLE        0x00 // Enable registers

// INTERRUPTS $18
#define LEV_PUL       0x00 // Level int1, Pulse int2
#define PUL_LEV       0x02 // Pulse int1, Level int2
#define DOUBLE_PULSE  0x04 // Pulse int1, Pulse or Double Pulse int2

// LEVEL DETECTION $19
#define LMOTION       0x00 // Positive polarity (OR 3 axis)
#define LFREEFALL     0x01 // Negative polarity (AND 3 axis)

// PULSE DETECTION $19
#define PMOTION       0x00 // Positive polarity (OR 3 axis)
#define PFREEFALL     0x02 // Negative polarity (AND 3 axis)

// RETURN VALUES
#define SUCCESS       0x00 // Success
#define TOOLONG       0x01 // Data too long to fit in transmit buffer
#define NACKA         0x02 // Received NACK on transmit of address
#define NACKD         0x03 // Received NACK on transmit of data
#define UNKNOWN       0x04 // Unknown or other error
#define NODATA        0x05 // No data returned
#define NOWRITE       0x06 // Did not write any data
#define BADLENGTH     0x07 // Buffer length not equal to actual size

class Motion {
private:

  /**
   * The mode of the accelerometer as a byte mask.
   * The valid modes are: standby, measurement,
   * level detection and pulse detection.
   *
   * @type {uint8_t}
   */
  uint8_t mode;

  /**
   * Sensitivity of the accelerometer as a byte mask.
   * The valid sensitivity levels are: 2g, 4g and 8g.
   *
   * @type {uint8_t}
   */
  uint8_t sensitivity;

  /**
   * Calculate the calibration offset of the device
   * such that it reads 0, 0, 64 when flat.
   *
   * @method calibrate
   */
  void calibrate();

  /**
   * Read from one of the devices registers and return
   * the value into a buffer.
   *
   * @method read
   *
   * @param {uint8_t} address
   *   Address of the register.
   *
   * @param {uint8_t *} buffer
   *   Byte array in which to store data read from the register.
   *
   * @param {int} size
   *   The number of byes to read from the register. Default is 1 byte.
   *
   * @return {int}
   *   0 if the read was successful, otherwise some other error code.
   */
  int read(uint8_t address, uint8_t *buffer, int size = 1);

  /**
   * write to one of the devices registers the value
   * stored within a buffer.
   *
   * @method write
   *
   * @param {uint8_t} address
   *   Address of the register.
   *
   * @param {const uint8_t *} buffer
   *   Byte array in which to read data to send to the register.
   *
   * @param {int} size
   *   The number of byes to write to the register. Default is 1 byte.
   *
   * @return {int}
   *   0 if the write was successful, otherwise some other error code.
   */
  int write(uint8_t address, const uint8_t *buffer, int size = 1);

protected:

  /**
   * Simple union structure that can be used either to
   * send values to the accelerometer of read values
   * from.
   *
   * @type {union}
   */
  union accelerometer {
    struct {
      uint8_t x_lsb;
      uint8_t x_msb;
      uint8_t y_lsb;
      uint8_t y_msb;
      uint8_t z_lsb;
      uint8_t z_msb;
    } registers;
    struct {
      int x;
      int y;
      int z;
    } values;
  };

public:

  /**
   * Accelerometer constructor. By default the accelerometer
   * will be put into standby mode.
   *
   * @method Motion
   *
   * @param {uint8_t} modeMask
   *   The accelerometer meansurement mode and sensitivity. Default is standby.
   */
  Motion(uint8_t modeMask = STANDBY);

  /**
   * Set the measurement mode of the accelerometer.
   *
   * @method setMode
   *
   * @param {uint8_t} mask
   *   The accelerometer meansurement mode and sensitivity. Default is standby.
   *
   * @return {int}
   *  0 if the accelerometer measurement mode was successfully set, otherwise some other error code.
   */
  int setMode(uint8_t mask);

  /**
   * The accelerometer has a series of interrups that can be
   * set to fire upon specific pulse or level measurement
   * events.
   *
   * @method setInterrupts
   *
   * @param {uint8_t} mask
   *   The accelerometer interrupt mask.
   *
   * @return {int}
   *  0 if the accelerometer interrupt mask was successfully set, otherwise some other error code.
   */
  int setInterrupts(uint8_t mask);

  /**
   * Once an iterrupt has been fired it needs to be reset
   * before it can detect any further events.
   *
   * @method clearInterruptLatch
   *
   * @return {int}
   *  0 if the interrupts are cleared successfully, otherwise some other error code.
   */
  int clearInterruptLatch();

  /**
   * Get the current orientation of the accelerometer, returning
   * all 3 axis.
   *
   * @method orientation
   *
   * @param {int *} x
   *   Buffer to store the orientation value along the x axis.
   *
   * @param {int *} y
   *   Buffer to store the orientation value along the y axis.
   *
   * @param {int *} z
   *   Buffer to store the orientation value along the z axis.
   *
   * @return {int}
   *  0 if the orientation is read successfully, otherwise some other error code.
   */
  int orientation(int *x, int *y, int *z);

  /**
   * Get the current accelerometer measurement mode.
   *
   * @method getMode
   *
   * @return {uint8_t}
   *   The current accelerometer measurement mode.
   */
  uint8_t getMode();

  /**
   * Get the current accelerometer sensitivity.
   *
   * @method getSensitivity
   *
   * @return {uint8_t}
   *   The current accelerometer sensitivity.
   */
  uint8_t getSensitivity();
};

#endif
