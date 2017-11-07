//From: https://github.com/Frogofmagic/MPU9250/blob/master/MPU9250Main/MPU9250Main.ino
//WFW: Modified for the MPU-6500 (no magnetic sensor)
//WFW: Works fine with ESP8266
//
//	No addition library needed (except Wire).
//	Note: If you get constant readings only, the MPU-6500 might not operate.
//	If strange readings occur repower the ESP
//
//		Hardware connection:
//				VCC-3V3, 
//				GND-GND, 
//				SDA-D2, 
//				SCL-D1


#include <Wire.h>

#define    MPU6500_ADDRESS            0x68
#define    GYRO_CONFIG                0x27
#define    ACCEL_CONFIG               0x28
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
#define	   WHO_AM_I					 0x75 // Should return 0x71 for MPU9250 and 0x70 for MPU6500
#define	   CONFIG					0x1A
#define    LOWPASS_10HZ             0x05
#define    LOWPASS_05HZ             0x06



// Initializations
void setup()
{
	Wire.begin();
	Serial.begin(115200);
	delay(10);
	Serial.println(" ");
	// Configure gyroscope range
	I2CwriteByte(MPU6500_ADDRESS, GYRO_CONFIG, GYRO_FULL_SCALE_250_DPS);
	// Configure accelerometers range
	I2CwriteByte(MPU6500_ADDRESS, ACCEL_CONFIG, ACC_FULL_SCALE_2_G);
	// Configure Low-Pass filtering 
	I2CwriteByte(MPU6500_ADDRESS, CONFIG, LOWPASS_05HZ);

	//pinMode(D3, OUTPUT);
	Serial.println("For acceleration: 1g = 16384 ");
	// Read the WHO_AM_I register, this is a good test of communication
	Serial.println("MPU9250 or MPU6500 motion sensor...");
	byte c = I2CreadByte(MPU6500_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register for MPU-9250/MPU-6500
	Serial.print("Identification from WHO_AM_I register: >>"); Serial.print(c, HEX); 
	Serial.println("<<. ");
	Serial.println("It should be 0x71 (MPU9250) or 0x70 (MPU6500).");
	delay(1000);
}

void loop()
{
	// Read accelerometer and gyroscope
	uint8_t Buf[14];
	int nrLoops = 4;
	int32_t axsum = 0;	int32_t aysum = 0; 	int32_t azsum = 0;
	int32_t gxsum = 0; 	int32_t gysum = 0; 	int32_t gzsum = 0; int32_t temsum =0;
	int16_t ax, ay, az, gx, gy, gz, tem;
	for (size_t i = 0; i < nrLoops; i++)
	{
		I2Cread(MPU6500_ADDRESS, 0x3B, 14, Buf);

		// Create 16 bits values from 8 bits data
		// Accelerometer
		ax = Buf[0] << 8 | Buf[1];
		ay = Buf[2] << 8 | Buf[3];
		az = Buf[4] << 8 | Buf[5];
		// Gyroscope
		gx = Buf[8] << 8 | Buf[9];
		gy = Buf[10] << 8 | Buf[11];
		gz = Buf[12] << 8 | Buf[13];
		// Temperature
		tem = Buf[6] << 8 | Buf[7];
		axsum += ax; aysum += ay; azsum += az;
		gxsum += gx; gysum += gy; gzsum += gz;
		temsum += tem;
		delay(2);
	}
	ax = axsum / nrLoops;	ay = aysum / nrLoops;	az = azsum / nrLoops;
	gx = gxsum / nrLoops;	gy = gysum / nrLoops;	gz = gzsum / nrLoops;
	tem = temsum / nrLoops;
	// Display values

	// Accelerometer
	Serial.print("Accelerometer: ax=");
	Serial.print(ax, DEC);
	Serial.print(" ay=");
	Serial.print(ay, DEC);
	Serial.print(" az=");
	Serial.print(az, DEC);
	Serial.print("\n");

	// Gyroscope
	Serial.print("Gyroscope: x=");
	Serial.print(gx, DEC);
	Serial.print(" y=");
	Serial.print(gy, DEC);
	Serial.print(" z=");
	Serial.print(gz, DEC);
	Serial.print("\n");

	// Temperature
	Serial.print("Temp: ");
	float tempCelsius = 21.0 + tem / 333.87;
	Serial.print(tempCelsius); Serial.print(" °C \n");

	uint8_t dim; 
	if (true) {
		dim = abs(ax) / 64;
		analogWrite(D3, dim);
	}

	//Serial.print("dim = ");
	//Serial.println(dim);
	delay(600);
}

// #####################################################################################
// ######################## Subroutines ################################################
// #####################################################################################

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
	Wire.beginTransmission(Address);	// Set register address
	Wire.write(Register);
	Wire.endTransmission();
	Wire.requestFrom(Address, Nbytes);	// Read Nbytes
	uint8_t index = 0;
	while (Wire.available()) Data[index++] = Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
	Wire.beginTransmission(Address);	// Set register address
	Wire.write(Register);
	Wire.write(Data);
	Wire.endTransmission();
}

// Read a byte (return value) in device (Address) at register (Register)
uint8_t I2CreadByte(uint8_t Address, uint8_t Register) {
	uint8_t Data; 
	Wire.beginTransmission(Address);         // Initialize the Tx buffer
	Wire.write(Register);	                 // Put slave register address in Tx buffer
	Wire.endTransmission();
	Wire.requestFrom(Address, (size_t)1);    // Read one byte from slave register address 
	Data = Wire.read();                      // Fill Rx buffer with result
	return Data;                             // Return data read from slave register
}