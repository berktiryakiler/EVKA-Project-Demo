
#include <ESP8266WiFi.h>

#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include <WiFiUdp.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#ifndef STASSID
#define STASSID "Evkas"
#define STAPSK  "1q2w3e4r"
#endif

#define REMOTE_UDP_PORT 2230

unsigned int localPort = 8888;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back
byte txBuffer[32];

WiFiUDP Udp;

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define A 4
#define B 5
#define Z 6
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
byte AB,ABold,printdata;
int counter=5000,sayi;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  //pinMode(2,INPUT);
  //pinMode(3,INPUT);
  //pinMode(A,INPUT);
  //pinMode(B,INPUT);
  //pinMode(Z,INPUT);
  //attachInterrupt(digitalPinToInterrupt(2),encoder,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(3),encoder,CHANGE);
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  WiFi.begin(STASSID, STAPSK);
    delay(1000);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);
  
  Wire.begin(13,12);
    
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);  
  }
  delay(100); // Wait for sensor to stabilize



  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accX, accY) * RAD_TO_DEG;  //accY, accZ
  double pitch = atan(accZ / sqrt(accY * accY + accX * accX)) * RAD_TO_DEG;  //-accX / sqrt(accY * accY + accZ * accZ
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanX.setQbias(0.001f);
  kalmanY.setQbias(0.001f);
  kalmanX.setRmeasure(0.01f);
  kalmanY.setRmeasure(0.01f);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

   ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void encoder() {
  AB=2*digitalRead(A)+digitalRead(B);
  if (AB==0) {
    if (ABold==1)
      counter++;
    else
      counter--;
  }
  else if (AB==1) {
    if (ABold==3)
      counter++;
    else
      counter--;
  }
  else if (AB==3) {
    if (ABold==2)
      counter++;
    else
      counter--;
  }
  else {
    if (ABold==0)
      counter++;
    else
      counter--;
  }
//  if (digitalRead(Z)==1)
//    counter=0;
  ABold=AB;
}

void loop() {

  ArduinoOTA.handle();
  
  /*
  getData();
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
                  packetSize,
                  Udp.remoteIP().toString().c_str(), Udp.remotePort(),
                  Udp.destinationIP().toString().c_str(), Udp.localPort(),
                  ESP.getFreeHeap());

    // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

  }
  //Serial.printf("IP: %s\n:", WiFi.gatewayIP().toString().c_str());
  
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(WiFi.gatewayIP(), REMOTE_UDP_PORT);
    
    int angleY = int(10.0*kalAngleY)+1800;
    int angleX = int(10.0*kalAngleX)+1800;

    memset(&txBuffer[0], angleX, 2);
    memset(&txBuffer[2], angleY, 2);
    memset(&txBuffer[4], counter, 2);
    Udp.write(&txBuffer[0], 6);

    Udp.endPacket();
    
  delay(10);*/
}

void getData()
{

  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accX, accY) * RAD_TO_DEG;  //accY, accZ
  double pitch = atan(accZ / sqrt(accY * accY + accX * accX)) * RAD_TO_DEG;  //-accX / sqrt(accY * accY + accZ * accZ
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroZ / 131.0; // Convert to deg/s
  double gyroYrate = gyroX / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  printdata=1;
  /* Print Data */
  if (printdata==0){ // Set to 1 to activate
    Serial.print(accX); Serial.print(" ");
    Serial.print(accY); Serial.print(" ");
    Serial.print(accZ); Serial.print(" ");
  
    Serial.print(gyroX); Serial.print(" ");
    Serial.print(gyroY); Serial.print(" ");
    Serial.print(gyroZ); Serial.print(" ");
    Serial.print("\r\n");}
  else if (printdata==1) {
    Serial.print(roll); Serial.print("dfsdf");
    Serial.print(int(10.0*kalAngleX)+1800); Serial.print(" ");
  
    Serial.print(pitch); Serial.print(" ");
    Serial.print(int(10.0*kalAngleY)+1800); Serial.print(" ");

    Serial.print(counter);
    Serial.print("\r\n");}
  else {
    Serial.write(32);
    delay(5);
    sayi=int(10.0*kalAngleX)+1800;
    Serial.write(sayi/256);
    delay(5);
    Serial.write(sayi-256*(sayi/256));
    delay(5);
    sayi=int(10.0*kalAngleY)+1800;
    Serial.write(sayi/256);
    delay(5);
    Serial.write(sayi-256*(sayi/256));
    delay(5);
    Serial.write(counter/256);
    delay(5);
    Serial.write(counter-256*(counter/256));}
}
