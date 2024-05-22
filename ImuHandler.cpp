#include <iostream>
#include <chrono>
#include <thread>

#include "ImuHandler.h"

#include "BBB_I2C.h"
#include "MPU6050.h"

int16_t ax,ay,az;
int16_t gx,gy,gz;

int offset_ax, offset_ay, offset_az;
int offset_gx, offset_gy, offset_gz;

MPU6050 MPU;
BBB_I2C BBB_I2C;
uint8_t rxbuf;
uint8_t txbuf = 0xaa;
uint8_t addr = 0x68;  // i2c slave address for MPU6050
uint8_t offset = 0x55;
uint8_t bitnum = 0x00;
uint8_t DEV_ID;

uint8_t bitlength = 0x04;
uint8_t bitstart = 0x00;

ImuHandler::ImuHandler() {
	if (MPU.testConnection() < 1){
		printf ("Device ID not match!\n");
		exit(1);
	}
	
    printf ("Initializing IMU ... ");
	if (MPU.initialize() < 1) {
		printf ("MPU initialize fail!\n");
		exit(1);
	}
    printf (" complete.\n");
}

void ImuHandler::calibrateImu() {
    printf ("Calibrating IMU ... \r");

    for (int i = 0; i < NUM_IMU_CALIBRATION_VALS; i++) {
		MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        offset_ax += ax;
        offset_ay += ay;
        offset_az += az;
        offset_gx += gx;
        offset_gy += gy;
        offset_gz += gz;

        if (i % (NUM_IMU_CALIBRATION_VALS / 100) == 0) {
            printf ("Calibrating IMU ... %.0f %% \r", ((double)i / (double)NUM_IMU_CALIBRATION_VALS * 100.0) );
        }
    }

    offset_ax /= NUM_IMU_CALIBRATION_VALS;
    offset_ay /= NUM_IMU_CALIBRATION_VALS;
    offset_az /= NUM_IMU_CALIBRATION_VALS;
    offset_gx /= NUM_IMU_CALIBRATION_VALS;
    offset_gy /= NUM_IMU_CALIBRATION_VALS;
    offset_gz /= NUM_IMU_CALIBRATION_VALS;

    printf ("IMU calibration complete with %d measurements. Offsets:\n", NUM_IMU_CALIBRATION_VALS);
    printf ("\tAccel (x,y,z): %d, %d, %d\n", offset_ax, offset_ay, offset_az);
    printf ("\tGyro  (x,y,z): %d, %d, %d\n", offset_gx, offset_gy, offset_gz);
}

void ImuHandler::read() {
    while (true) {
        MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        float accelX = (float)(ax)/4096.0;
        float accelY = (float)(ay)/4096.0;
        float accelZ = (float)(az)/4096.0;
        int16_t roll = atan(accelY / sqrt(accelX*accelX + accelZ*accelZ))/(3.14159/180);
        int16_t pitch = atan(accelX / sqrt(accelY*accelY + accelZ*accelZ))/(3.14159/180);

        // printf ("Gyro  (x,y,z): %d, %d, %d \n", (gx-offset_gx)/65, (gy-offset_gy)/65, (gz-offset_gz)/65);
        // printf ("Accl  (x,y,z): %d, %d, %d \n", (ax-offset_ax)/4096, (ay-offset_ay)/4096, (az-offset_az)/4096);
        // printf("Roll: %d Pitch: %d Gravity: %d \n", roll, pitch, accelZ);

        printf("%.2f %.2f %.2f\n", accelX, accelY, accelZ);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
