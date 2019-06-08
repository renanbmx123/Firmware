/**
 */
 #include "ALTIMU.h"

// Public Methods //////////////////////////////////////////////////////////////

// Constructor
Altimu::Altimu(PinName sda, PinName scl):
    _ALTIMU(sda, scl)
{
    // Setting I2C operation frequency.
    _ALTIMU.frequency(200000);
    // L3GD20 configuration.
    // DRDY_HL (DRDY active high);I2C_dis = (I2C & SPI enable); SW = (Normal Mode); Low_ODR = (Low speed disable).
    write_reg(L3GD20_ADDR,0x39,0x00);
    // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled).
    write_reg(L3GD20_ADDR,0x20,0x5F); //Setting CTRL_REG1
    // End L3GD20 configuration.
    // LSM303 configuration
        // Acell configuration
          // 50 Hz X/Y/Z axis enable.
    write_reg(LSM303_ADDR, 0x20, 0x57);
        // mag
            //continuous mag
    write_reg(LSM303_ADDR, 0x24, 0x78);
    write_reg(LSM303_ADDR, 0x26, 0x00);
    // End LSM303 configuration.
   
    // LPS25H configuration
    //write_reg(LPS25H_ADDR, 0x10, 0x05);
    //write_reg(LPS25H_ADDR, 0x2e, 0xdf);
    //write_reg(LPS25H_ADDR, 0x21, 0x40);
    write_reg(LPS25H_ADDR, 0x20, 0x90);
}
// L3GD30 read data function,
void Altimu::read_L3GD20(float *gx, float *gy, float *gz) {
    char gyr[6];
    recv(L3GD20_ADDR, 0x28, gyr, 6);
    //scale is 8.75 mdps/digit
        *gx = float(short(gyr[1] << 8 | gyr[0]))*0.00875;
        *gy =  float(short(gyr[3] << 8 | gyr[2]))*0.00875;
        *gz =  float(short(gyr[5] << 8 | gyr[4]))*0.00875;
}

// LSM303D read data function.
void Altimu::read_LSM303D(float *ax, float *ay, float *az, float *mx, float *my, float *mz) {
        char acc[6], mag[6];
        recv(LSM303_ADDR, 0x28, acc, 6) && recv(LSM303_ADDR, 0x08, mag, 6); 
        *ax = float(short(acc[1] << 8 | acc[0]))*0.061;  //32768/4=8192
        *ay =  float(short(acc[3] << 8 | acc[2]))*0.061;
        *az =  float(short(acc[5] << 8 | acc[4]))*0.061;
        //+-4gauss
        *mx = float(short(mag[0] << 8 | mag[1]))*0.16;
        *mz = float(short(mag[2] << 8 | mag[3]))*0.16;
        *my = float(short(mag[4] << 8 | mag[5]))*0.16;
}

// LPS25H read data function.
void Altimu::read_LPS25H(float *press, float *alt)
{
    char dt[3]; // 3 bytes for reading i2c data(Press_X_L Press_L Press_H, Temp_L Temp_H).
    float t;    // Store internal temperature sensor, for compesate altitude calculation.
    
    // Reading 3 bytes from pressure sensor.
    recv(LPS25H_ADDR, 0x28, dt, 3); 
    // Put togheter three bytes of pressure and make a calculation to present it on hPa values.
    *press = (double)((dt[2] << 16) | (dt[1] << 8) | dt[0])/4096.0;
    // Reading 2 bytes of internal temperature sensor.
    recv(LPS25H_ADDR, 0x2B, dt, 2);
    // Put the two temperature data togheter.
    t = dt[1] << 8 | dt[0];
    // Calculate temperature in celcius.
    t = (t/480 + 42.5)/10;
    // Calculate altitude value from pressure and temperature.
     *alt = (1-pow((*press/1013.25), 0.190262525))*((t+273.15)/0.0065);
}

// I2C functions
    
    // Write a byte in a register address. 
bool Altimu::write_reg(int addr_i2c,int addr_reg, char v)
{
    char data[2] = {addr_reg, v}; // 
    // return boolean value of write operation, if fails return 0, else 1.
    return Altimu::_ALTIMU.write(addr_i2c, data, 2) == 0;
}
    // Read a byte from register
bool Altimu::read_reg(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg;
    bool result = false;
    
    __disable_irq();
    if ((_ALTIMU.write(addr_i2c, &data, 1) == 0) && (_ALTIMU.read(addr_i2c, &data, 1) == 0)){
        *v = data;
        result = true;
    }
    __enable_irq();
    return result;
}
     // Read n bytes from mem address.
bool Altimu::recv(char sad, char sub, char *buf, int length) {
    if (length > 1) sub |= 0x80;

    return _ALTIMU.write(sad, &sub, 1, true) == 0 && _ALTIMU.read(sad, buf, length) == 0;
}
