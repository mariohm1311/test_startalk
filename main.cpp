#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)
int calib_data_addr = 0x0000;
const int imu_id = 55;
adafruit_bno055_offsets_t calib_data;
bool saveCal = true;

Adafruit_BNO055 bno = Adafruit_BNO055(imu_id);
sensor_t imu_sensor;
uint8_t sys, gyro, accel, mag;

unsigned long now = 0;
unsigned long now_long = 0;
unsigned long dt = 0;
uint16_t niter = 0;

#include <madgwick.h>
Madgwick filter;

Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();
float pres_pa, temp_C, alt_m;

void displayCalStatus(void)
{
    sys = gyro = accel = mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    if (!sys)
    {
        Serial.print("! ");
    }

    Serial.print("Sys=");
    Serial.print(sys, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
}

void displaySensorOffsets(adafruit_bno055_offsets_t &calib_data)
{
    Serial.print("Accelerometer: ");
    Serial.print(calib_data.accel_offset_x);
    Serial.print(" ");
    Serial.print(calib_data.accel_offset_y);
    Serial.print(" ");
    Serial.print(calib_data.accel_offset_z);
    Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calib_data.gyro_offset_x);
    Serial.print(" ");
    Serial.print(calib_data.gyro_offset_y);
    Serial.print(" ");
    Serial.print(calib_data.gyro_offset_z);
    Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calib_data.mag_offset_x);
    Serial.print(" ");
    Serial.print(calib_data.mag_offset_y);
    Serial.print(" ");
    Serial.print(calib_data.mag_offset_z);
    Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calib_data.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calib_data.mag_radius);
}

void bnoStartup(void) 
{
    long bno_id;
    bool found_calib = false;
    bno.getSensor(&imu_sensor);

    if (saveCal) {
        EEPROM.get(calib_data_addr, bno_id);

        if (bno_id != imu_sensor.sensor_id)
        {
            Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
            delay(500);
        }
        else
        {
            Serial.println("\nFound Calibration for this sensor in EEPROM.");
            calib_data_addr += sizeof(long);
            EEPROM.get(calib_data_addr, calib_data);

            displaySensorOffsets(calib_data);

            Serial.println("\n\nRestoring Calibration data to the BNO055...");
            bno.setSensorOffsets(calib_data);

            Serial.println("\n\nCalibration data loaded into BNO055");
            found_calib = true;
        }
    }
    
    delay(1000);
    bno.setExtCrystalUse(true); // Crystal must be configured AFTER calibration

    if (found_calib)
    {
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

            Serial.print("X: ");
            Serial.print(ang.x(), 4);
            Serial.print("\tY: ");
            Serial.print(ang.y(), 4);
            Serial.print("\tZ: ");
            Serial.print(ang.z(), 4);

            Serial.print("\t");
            displayCalStatus();
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    bno.getSensorOffsets(calib_data);
    displaySensorOffsets(calib_data);

    if (saveCal) {
        Serial.println("\n\nStoring calibration data to EEPROM...");

        calib_data_addr = 0x0000;
        bno.getSensor(&imu_sensor);
        bno_id = imu_sensor.sensor_id;
        EEPROM.put(calib_data_addr, bno_id);

        calib_data_addr += sizeof(long);
        EEPROM.put(calib_data_addr, calib_data);
        Serial.println("Data stored to EEPROM.");
    }

    bno.configMaxPerfAMG();
}

void setup()
{
    Serial.begin(115200);

    Serial.println("##############################################");
    Serial.println("                IMU SENS FUSE                 ");
    Serial.println("##############################################");

    if (!bno.begin())
    {
        Serial.println("No IMU detected");
        while (1)
            ;
    }

    delay(1000);

    bnoStartup();

    if (!mpl.begin())
    {
        Serial.println("No altimeter detected");
        return;
    }
    delay(1000);

    Serial.println("Calibration values:\n0=uncalibrated, 3=fully calibrated");
    Serial.println("");
    // Serial.println("----------------------------------------------");

    // filter.begin(93.5f);

    now = micros();
    now_long = micros();
}

void loop()
{
    // pres_pa = temp_C = alt_m = 0.0;

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    // Serial.print("CALIBRATION: ");
    // displayCalStatus();

    // uint8_t sys_stat, self_test, sys_err;
    // bno.getSystemStatus(&sys_stat, &self_test, &sys_err);
    // Serial.println(sys_stat);
    // Serial.println(self_test);
    // Serial.println(sys_err);
    // Serial.println(bno._mode);

    // imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // imu::Vector<3> angvel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // filter.update(angvel.x(), angvel.y(), angvel.z(),
    //               acc.x(), acc.y(), acc.z(),
    //               mag.x(), mag.y(), mag.z());

    niter++;
    dt = micros() - now;
    now = micros();

    if (niter % 100 == 0) {
        float freq = niter * 1e+6;
        freq /= (float)(micros() - now_long);
        Serial.println(freq);

        niter = 0;
        now_long = micros();
    }

    pres_pa = mpl.getPressure();
    // Serial.print("AMBIENT PRESSURE:    ");
    // Serial.print(pres_pa / 101325, 4);
    // Serial.println(" atm");

    // // Altitude can be calculated directly through the following equation
    alt_m = 44330.77 * (1 - pow(pres_pa / 101326.0, 0.1902632));
    // // alt_m = mpl.getAltitude();
    // Serial.print("BAROMETRIC ALTITUDE: ");
    Serial.print(alt_m, 2);
    Serial.println(" m");

    // temp_C = mpl.getTemperature();
    // Serial.print("AMBIENT TEMPERATURE:  ");
    // Serial.print(temp_C, 2);
    // Serial.println(" *C");

    // Serial.println(buf);
    // Serial.print(acc.x());
    // Serial.print(",");
    // Serial.print(acc.y());
    // Serial.print(",");
    // Serial.print(acc.z());
    // Serial.print(",");
    // Serial.print(angvel.x());
    // Serial.print(",");
    // Serial.print(angvel.y());
    // Serial.print(",");
    // Serial.print(angvel.z());
    // Serial.print(",");
    // Serial.print(mag.x());
    // Serial.print(",");
    // Serial.print(mag.y());
    // Serial.print(",");
    // Serial.print(mag.z());
    // Serial.print(",");
    // Serial.println(dt);

    // Serial.print(acc.x());
    // Serial.print(",");
    // Serial.print(acc.y());
    // Serial.print(",");
    // Serial.print(acc.z());
    // Serial.print(",");
    // Serial.print(filter.getPitch());
    // Serial.print(",\t");
    // Serial.print(filter.getYaw());
    // Serial.print(",\t");
    // Serial.print(filter.getRoll());
    // Serial.print(",\t");
    // Serial.print(mag.x());
    // Serial.print(",");
    // Serial.print(mag.y());
    // Serial.print(",");
    // Serial.print(mag.z());
    // Serial.print(",");
    Serial.println(dt);

    

    // Serial.println("----------------------------------------------");

    // delay(BNO055_SAMPLERATE_DELAY_MS);
}