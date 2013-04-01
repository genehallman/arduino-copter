// OSEPP Gyroscope Sensor Example Sketch
// by OSEPP <http://www.osepp.com>

// This sketch demonstrates interactions with the Gyroscope Sensor

#include <Wire.h>

// Possible sensor addresses (suffix correspond to DIP switch positions)
#define GYRO_SENSOR_ADDR_OFF  (0x69)
#define GYRO_SENSOR_ADDR_ON   (0x68)

#define ACCEL_SENSOR_ADDR_OFF (0x1D)
#define ACCEL_SENSOR_ADDR_ON  (0x53)

#define COMP_SENSOR_ADDR  (0x1E)

// gyro sensor register addresses (gotten from datasheet)
#define GYRO_REG_ACCEL_ADDR       (0x14)
#define GYRO_REG_DATAX1_ADDR      (0x1D)
#define GYRO_REG_DATAX0_ADDR      (0x1E)
#define GYRO_REG_DATAY1_ADDR      (0x1F)
#define GYRO_REG_DATAY0_ADDR      (0x20)
#define GYRO_REG_DATAZ1_ADDR      (0x21)
#define GYRO_REG_DATAZ0_ADDR      (0x22)
#define GYRO_REG_USERC_ADDR       (0x3D)

// accel sensor register addresses (gotten from datasheet)
#define ACCEL_REG_DEVID_ADDR        (0x00)
#define ACCEL_REG_THRESH_TAP_ADDR   (0x1D)
#define ACCEL_REG_TAP_DUR_ADDR      (0x21)
#define ACCEL_REG_TAP_LATENCY_ADDR  (0x22)
#define ACCEL_REG_TAP_WINDOW_ADDR   (0x23)      
#define ACCEL_REG_BW_RATE_ADDR      (0x2C)
#define ACCEL_REG_PWR_CTL_ADDR      (0x2D)
#define ACCEL_REG_INT_ENABLE_ADDR   (0x2E)
#define ACCEL_REG_DATA_FORMAT_ADDR  (0x31)
#define ACCEL_REG_DATAX0_ADDR       (0x32)
#define ACCEL_REG_DATAX1_ADDR       (0x33)
#define ACCEL_REG_DATAY0_ADDR       (0x34)
#define ACCEL_REG_DATAY1_ADDR       (0x35)
#define ACCEL_REG_DATAZ0_ADDR       (0x36)
#define ACCEL_REG_DATAZ1_ADDR       (0x37)
#define ACCEL_REG_FIFO_CTL_ADDR     (0x38)

// compass sensor register addresses (gotten from datasheet)
#define COMP_REG_CONFA_ADDR        (0x0)
#define COMP_REG_CONFB_ADDR        (0x1)
#define COMP_REG_MODE_ADDR         (0x2)
#define COMP_REG_DATAX0_ADDR       (0x3)
#define COMP_REG_DATAX1_ADDR       (0x4)
#define COMP_REG_DATAY0_ADDR       (0x7)
#define COMP_REG_DATAY1_ADDR       (0x8)
#define COMP_REG_DATAZ0_ADDR       (0x5)
#define COMP_REG_DATAZ1_ADDR       (0x6)

const uint8_t gyroAddr = GYRO_SENSOR_ADDR_ON;
const uint8_t accelAddr = ACCEL_SENSOR_ADDR_OFF;
const uint8_t compAddr = COMP_SENSOR_ADDR;

// One-time setup
void setup()
{
   // Start the serial port for output
   Serial.begin(9600);

   // Join the I2C bus as master
   Wire.begin();

   // gyro setup
   WriteByte(gyroAddr, ACCEL_REG_INT_ENABLE_ADDR, accelAddr); // set accel slace addr
   WriteByte(gyroAddr, GYRO_REG_USERC_ADDR, 0x20); // set AUX_IF_EN to 1 on user control

   // accel setup
   WriteByte(accelAddr, ACCEL_REG_BW_RATE_ADDR, 0x08); // Set 25 Hz output data rate and 25 Hz bandwidth and disable low power mode
   WriteByte(accelAddr, ACCEL_REG_PWR_CTL_ADDR, 0x08); // Disable auto sleep
   WriteByte(accelAddr, ACCEL_REG_INT_ENABLE_ADDR, 0x0); // Disable interrupts
      
   // compass setup
   WriteByte(compAddr, COMP_REG_CONFA_ADDR, 0x10); // default settings (from datasheet)
   WriteByte(compAddr, COMP_REG_CONFB_ADDR, 0xE0); // set gain to ± 8.1 Ga
   WriteByte(compAddr, COMP_REG_MODE_ADDR, 0x0); // Set compass to continuous-measurement mode
}

// Main program loop
void loop()
{
   // gyro
   uint8_t g_x_msb;   // X-axis most significant byte
   uint8_t g_x_lsb;   // X-axis least significant byte
   uint8_t g_y_msb;   // Y-axis most significant byte
   uint8_t g_y_lsb;   // Y-axis least significant byte
   uint8_t g_z_msb;   // Z-axis most significant byte
   uint8_t g_z_lsb;   // Z-axis least significant byte
   uint16_t g_x;
   uint16_t g_y;
   uint16_t g_z;

   // accel
   uint8_t a_x_msb;   // X-axis most significant byte
   uint8_t a_x_lsb;   // X-axis least significant byte
   uint8_t a_y_msb;   // Y-axis most significant byte
   uint8_t a_y_lsb;   // Y-axis least significant byte
   uint8_t a_z_msb;   // Z-axis most significant byte
   uint8_t a_z_lsb;   // Z-axis least significant byte
   uint16_t a_x;
   uint16_t a_y;
   uint16_t a_z;

   // comp
   uint8_t c_x_msb;   // X-axis most significant byte
   uint8_t c_x_lsb;   // X-axis least significant byte
   uint8_t c_y_msb;   // Y-axis most significant byte
   uint8_t c_y_lsb;   // Y-axis least significant byte
   uint8_t c_z_msb;   // Z-axis most significant byte
   uint8_t c_z_lsb;   // Z-axis least significant byte
   uint16_t c_x;
   uint16_t c_y;
   uint16_t c_z;

   // Get the value from the sensors
   if ((ReadByte(gyroAddr, GYRO_REG_DATAX1_ADDR, &g_x_msb) == 0) &&
       (ReadByte(gyroAddr, GYRO_REG_DATAX0_ADDR, &g_x_lsb) == 0) &&
       (ReadByte(gyroAddr, GYRO_REG_DATAY1_ADDR, &g_y_msb) == 0) &&
       (ReadByte(gyroAddr, GYRO_REG_DATAY0_ADDR, &g_y_lsb) == 0) &&
       (ReadByte(gyroAddr, GYRO_REG_DATAZ1_ADDR, &g_z_msb) == 0) &&
       (ReadByte(gyroAddr, GYRO_REG_DATAZ0_ADDR, &g_z_lsb) == 0) &&
       (ReadByte(accelAddr, ACCEL_REG_DATAX1_ADDR, &a_x_msb) == 0) &&
       (ReadByte(accelAddr, ACCEL_REG_DATAX0_ADDR, &a_x_lsb) == 0) &&
       (ReadByte(accelAddr, ACCEL_REG_DATAY1_ADDR, &a_y_msb) == 0) &&
       (ReadByte(accelAddr, ACCEL_REG_DATAY0_ADDR, &a_y_lsb) == 0) &&
       (ReadByte(accelAddr, ACCEL_REG_DATAZ1_ADDR, &a_z_msb) == 0) &&
       (ReadByte(accelAddr, ACCEL_REG_DATAZ0_ADDR, &a_z_lsb) == 0) &&
       (ReadByte(compAddr, COMP_REG_DATAX1_ADDR, &c_x_msb) == 0) &&
       (ReadByte(compAddr, COMP_REG_DATAX0_ADDR, &c_x_lsb) == 0) &&
       (ReadByte(compAddr, COMP_REG_DATAY1_ADDR, &c_y_msb) == 0) &&
       (ReadByte(compAddr, COMP_REG_DATAY0_ADDR, &c_y_lsb) == 0) &&
       (ReadByte(compAddr, COMP_REG_DATAZ1_ADDR, &c_z_msb) == 0) &&
       (ReadByte(compAddr, COMP_REG_DATAZ0_ADDR, &c_z_lsb) == 0))
   {
      g_x = (g_x_msb << 8) | g_x_lsb;
      g_y = (g_y_msb << 8) | g_y_lsb;
      g_z = (g_z_msb << 8) | g_z_lsb;
      
      a_x = (a_x_msb << 8) | a_x_lsb;
      a_y = (a_y_msb << 8) | a_y_lsb;
      a_z = (a_z_msb << 8) | a_z_lsb;

      c_x = (c_x_msb << 8) | c_x_lsb;
      c_y = (c_y_msb << 8) | c_y_lsb;
      c_z = (c_z_msb << 8) | c_z_lsb;

      // Perform 2's complement
      int16_t real_gx = ~(g_x - 1);
      int16_t real_gy = ~(g_y - 1);
      int16_t real_gz = ~(g_z - 1);

      int16_t real_ax = ~(a_x - 1);
      int16_t real_ay = ~(a_y - 1);
      int16_t real_az = ~(a_z - 1);

      Serial.print("{\"gx\":");
      Serial.print(real_gx);
      Serial.print(",\"gy\":");
      Serial.print(real_gy);
      Serial.print(",\"gz\":");
      Serial.print(real_gz);
      Serial.print(",\"ax\":");
      Serial.print(real_ax);
      Serial.print(",\"ay\":");
      Serial.print(real_ay);
      Serial.print(",\"az\":");
      Serial.print(real_az);
      Serial.print(",\"cx\":");
      Serial.print(c_x);
      Serial.print(",\"cy\":");
      Serial.print(c_y);
      Serial.print(",\"cz\":");
      Serial.print(c_z);
      Serial.println("}");
   }
   else
   {
      Serial.println("{err:'Failed to read from sensor'}");
   }

   // Run again in 1 s (1000 ms)
   delay(100);
}

// Read a byte on the i2c interface
int ReadByte(uint8_t addr, uint8_t reg, uint8_t *data)
{
   // Do an i2c write to set the register that we want to read from
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.endTransmission();

   // Read a byte from the device
   Wire.requestFrom(addr, (uint8_t)1);
   if (Wire.available())
   {
      *data = Wire.read();
   }
   else
   {
      return -1;
   }
   return 0;
}

// Write a byte on the i2c interface
void WriteByte(uint8_t addr, uint8_t reg, byte data)
{
   // Begin the write sequence
   Wire.beginTransmission(addr);

   // First byte is to set the register pointer
   Wire.write(reg);

   // Write the data byte
   Wire.write(data);

   // End the write sequence; bytes are actually transmitted now
   Wire.endTransmission();
}
