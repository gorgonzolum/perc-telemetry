/*  MFSC PAXC Rocket Telemetry Code
 *  License: MIT
 *
 *  Libraries Needed:
 *    Sparkfun MPU9250: https://github.com/sparkfun/MPU-9250_Breakout
 *    Adafruit BMP085: https://github.com/adafruit/Adafruit-BMP085-Library
 */
#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>

#include <Adafruit_BMP085.h>

#include <SoftwareSerial.h>

// Toggle debugging output to the Serial Monitor
// WARNING: blocks if no FTDI cable is connected to the Arduino ProTrinket!
#define DEBUG_TO_MONTIOR 0

// Toggle debugging output over the HC12 radio
#define DEBUG 1

// Status LED
const byte statusPin = 13;

// Pin for Beeper/Speaker
const byte beeperPin = 10;

// Tx/Rx pins for HC12
const byte hc12RxPin = 4;
const byte hc12TxPin = 5;
const byte hc12SetPin = 6;

// Transmit settings for HC12
// Transmit rate, valid values
// are 1200, 2400, 4800, 9600, 19200, 38400, 57600, or 115200bps
// Smaller rate = higher data throughput
const byte hc12Rate = 9600;

// Channel from 001 to 127, keep below 100
const byte hc12Channel = 1;

MPU9250_DMP imu;
Adafruit_BMP085 bmp;
SoftwareSerial hc12(hc12TxPin, hc12RxPin);

typedef struct _Vec3 {
  float x;
  float y;
  float z;
} Vec3;

typedef struct _IMUData {
  Vec3 accel;
  Vec3 gyro;
  Vec3 mag;
  float seaPressure;
  float pressure;
  float altitude;
  float temp;
} IMUData;
 
// Initializes the HC12 UART
void setupHC12 (void) {
  logInfo("Configuring HC12 Transmitter... ");
  
  hc12.begin(9600);
  
  pinMode(hc12SetPin, OUTPUT);
  digitalWrite(hc12SetPin, LOW);

  // Wait for command mode to engage
  delay(100);

  hc12.write("AT+DEFAULT");
  delay(100);
  String statusInfo = hc12.readString();
  Serial.println(statusInfo);
  if (!statusInfo.startsWith("OK")) {
    Serial.println("[HC12] ERROR Communicating with HC12!");
  }
  
  delay(50);
  hc12.write("AT+RX");
  delay(100);
  String transmitParams = hc12.readString();

  //TODO: Setup commands here
  //TODO: verify operation here

  // Exit Command Mode and change serial baud
  digitalWrite(hc12SetPin, HIGH);
   
  logInfo("Transmit Parameters:\n" + transmitParams);
  Serial.println("Transmit Parameters:\n" + transmitParams);
  logInfo("[HC12 OK]");
}

// Initializes the MPU-9250
void setupIMU(void) {
  logInfo("Initalizing HC12... ");
  
  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      logInfo("Unable to communicate with MPU-9250");
      logInfo("Check connections, and try again.");
      logInfo();
      delay(1000);
    }
  }

  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  logInfo("[MPU9250 OK]");
}

void setupBMP(void) {
  logInfo("Initializing BMP180... ");
  while (!bmp.begin()) {
    logInfo("[ERR]");
    logInfo("Unable to communicate with BMP185! Retrying...");
    delay(1000);
  }
  logInfo("[BMP180 OK]");
}

void setup() {
  Serial.begin(9600);
  setupHC12();
  setupIMU();
  setupBMP();

  pinMode(statusPin, OUTPUT);

  pinMode(beeperPin, OUTPUT);
  beep(200);
}

void beep(int duration) {
  digitalWrite(beeperPin, HIGH);
  delay(duration);
  digitalWrite(beeperPin, LOW);
  delay(duration);
}

void loop() {
  if (imu.dataReady()) {
    IMUData data;

    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    
    data.accel.x = imu.calcAccel(imu.ax);
    data.accel.y = imu.calcAccel(imu.ay);
    data.accel.z = imu.calcAccel(imu.az);
    data.gyro.x = imu.calcGyro(imu.gx);
    data.gyro.y = imu.calcGyro(imu.gy);
    data.gyro.z = imu.calcGyro(imu.gz);
    data.mag.x = imu.calcMag(imu.mx);
    data.mag.y = imu.calcMag(imu.my);
    data.mag.z = imu.calcMag(imu.mz);

    // Get data from BMP
    data.temp = bmp.readTemperature();
    data.pressure = bmp.readPressure();
    data.altitude = bmp.readAltitude();
    data.seaPressure = bmp.readSealevelPressure();

    // TODO: specify current sealevel pressure for more accurate reading
    //bmp.readAltitude();

    transmitIMUData(data);
  }
  
  // polling delay
  delay(400);
}

void logInfo() {
  logInfo("");
}

void logInfo(String msg) {
  #if DEBUG_TO_MONITOR
    Serial.println(msg);
  #elif DEBUG
    hc12.println(msg);
  #endif
}

void transmitIMUData(IMUData data) {
  digitalWrite(statusPin, HIGH);
  hc12.print(String(data.accel.x) + "\t" + String(data.accel.y) + "\t" + String(data.accel.z) + "\t");
  hc12.print(String(data.gyro.x) + "\t" + String(data.gyro.y) + "\t" + String(data.gyro.z) + "\t");
  hc12.print(String(data.mag.x) + "\t" + String(data.mag.y) + "\t" + String(data.mag.z) + "\t");
  hc12.print(String(data.pressure) + "\t" + String(data.temp) + "\n");
  delay(100);
  digitalWrite(statusPin, LOW);
}
