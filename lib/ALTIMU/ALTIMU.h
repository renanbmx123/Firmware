#ifndef __ALTIMU_LIB
#define __ALTIMU_LIB
#include "mbed.h"

#define LSM303_ADDR    0x3A
#define L3GD20_ADDR    0xD6
#define LPS25H_ADDR    (0x5d << 1)

class Altimu
{
    public:
      /** Create a new interface for a Altimu10V4
       *
       * @param sda is the pin for the I2C SDA line
       * @param scl is the pin for the I2C SCL line
       */
       Altimu(PinName sda, PinName scl);

       /** Initialize  Altimu10V4
        *
        * @conf configure the i2c frequency.``
        * @conf configure fifo filter.
        * @conf set sample mode.
        */
        void Init(void);

       // The section below, is reserved all three IMU function's.

       /** read the raw accelerometer and compass values
        *
        * @param ax,ay,az is the accelerometer 3d vector, written by the function
        * @param mx,my,mz is the magnetometer 3d vector, written by the function
        */
        void read_LSM303D(float *ax, float *ay, float *az, float *mx, float *my, float *mz);
        void read_L3GD20(float *gx, float *gy, float *gz);
        void read_LPS25H(float *press, float *alt);

    private:
      I2C _ALTIMU;           // I2C object

      bool write_reg(int addr_i2c,int addr_reg, char v);    // I2C Write data register.
      bool read_reg(int addr_i2c,int addr_reg, char *v);    // I2C Read data register.
      bool recv(char sad, char sub, char *buf, int length); // I2C Read n data
};
#endif
