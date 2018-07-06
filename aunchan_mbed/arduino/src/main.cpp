#include <Arduino.h>
#include <SoftWire.h>

#define I2C_SDA PB9
#define I2C_SCL PB8

SoftWire SWire(I2C_SCL, I2C_SDA, SOFT_FAST);

int16_t Gyro[3],Accel[3],Temp;
void setup(){
	SWire.setClock(400000);
	SWire.begin();
	delay(250);

    Serial1.begin(9600);

	SWire.beginTransmission(0x68);
	SWire.write(0x6B);
	SWire.write(0x00);
	SWire.endTransmission();
    Serial1.println("I2C Scanner");
}

void loop() {

	SWire.beginTransmission(0x68);
	SWire.write(0x3B);
	SWire.endTransmission();
	SWire.requestFrom(0x68,14);
	Accel[0] = SWire.read()<<8 | SWire.read();
	Accel[1] = SWire.read()<<8 | SWire.read();
	Accel[2] = SWire.read()<<8 | SWire.read();
	Temp = SWire.read()<<8 | SWire.read();
	Gyro[0] = SWire.read()<<8 | SWire.read();
	Gyro[1] = SWire.read()<<8 | SWire.read();
	Gyro[2] = SWire.read()<<8 | SWire.read();
	Serial1.print("X:");
	Serial1.print(Accel[0]);
	Serial1.print("\tY:");
	Serial1.print(Accel[1]);
	Serial1.print("\tZ:");
	Serial1.print(Accel[2]);
	Serial1.print("\t\tX:");
	Serial1.print(Gyro[0]);
	Serial1.print("\tY:");
	Serial1.print(Gyro[1]);
	Serial1.print("\tZ:");
	Serial1.println(Gyro[2]);
	delay(100);


}