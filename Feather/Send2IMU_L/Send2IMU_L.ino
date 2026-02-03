// Send via BLE Quaternions
// Name of the device for the communication: B_L_Upp


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery over bluetooth

// battery
#define VBAT_PIN          (A7) //(31)

// 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_MV_PER_LSB   (0.73242188F)

// 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER      (0.71275837F)

// Compensation factor for the VBAT divider
#define VBAT_DIVIDER_COMP (1.403F)

int vbat_raw;
uint8_t vbat_per;
float vbat_mv;

#define BAT_LED          (A3)
const int buzzer = 7; // (D7)
bool toggle_tone = false;
int count_bat = 0;
//battery end


//config LED
#define GYRO_LED          (A0)
#define ACCEL_LED         (A1)
#define MAG_LED           (A2)



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno1 = Adafruit_BNO055(-1, 0x28);

// buffer for IMU1 data
uint8_t buffer1_angle[6]; // buffer for right thigh pitch angle data 
uint8_t buffer1_velocity[6]; // buffer for right thigh pitch velocity data
//uint8_t buffer1_gra[6]; // buffer for gravity in the sagittal plane



//send data
char char_buffer[4 + 1] = {0}; // size buffer in byte
//calib
uint8_t sys1, gyro1, accel1, mag1;
    
//timing
int t0 = 0;
int t1 = 0;
int t_diff = 0;
float t_frequ = 40; //in Hz --- max 500Hz


void setup()
{
  //IMU 1
  memset(buffer1_angle, 0, 6); //initialize to 0
  memset(buffer1_velocity, 0, 6); //initialize to 0
  //memset(buffer1_gra, 0, 6); //initialize to 0
  
  
  Serial.begin(115200);

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial ) yield();
#endif

  //Initialise the sensor
  if (!bno1.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

   // AXES REMAP ACCORDING TO SENSOR ORIENTATION (look user manual pg. 25)
  bno1.setAxisRemap((Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)0x06); // X=Z, Y=Y, Z=X
  bno1.setAxisSign((Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)0x01); // -Z
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Angle__L");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setUuid(0x063B);
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  //battery
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);  // Can be 8, 10, 12 or 14
  delay(1);
  pinMode(buzzer, OUTPUT);
  pinMode(BAT_LED, OUTPUT);

  //sensor config
  pinMode(GYRO_LED, OUTPUT);
  pinMode(ACCEL_LED, OUTPUT);
  pinMode(MAG_LED, OUTPUT);

  //set buzzer to low by default
  digitalWrite(buzzer, LOW);
  
}




void loop()
{
  t0 = millis();
  
  // Read IMU1 data (4 bytes)
  //bno1.readLen(Adafruit_BNO055::BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer1, 8);// read quaternionn data
  bno1.readLen(Adafruit_BNO055::BNO055_EULER_H_LSB_ADDR, buffer1_angle, 6); // read euler angle data
  bno1.readLen(Adafruit_BNO055::BNO055_GYRO_DATA_X_LSB_ADDR, buffer1_velocity, 6); // read angular velocity data
  //bno1.readLen(Adafruit_BNO055::BNO055_GRAVITY_DATA_X_LSB_ADDR, buffer1_gra, 6);// read gra data
  //--------- IMU1 --------
  char_buffer[0] = buffer1_angle[4];
  char_buffer[1] = buffer1_angle[5];
  char_buffer[2] = buffer1_velocity[0];
  char_buffer[3] = buffer1_velocity[1];
  //char_buffer[4] = buffer1_gra[2];
  //char_buffer[5] = buffer1_gra[3];  

  bleuart.write(char_buffer,4); //send BLE data

  //LED and Buzzer controll
  if (count_bat > 100) {
    

    bno1.getCalibration(&sys1, &gyro1, &accel1, &mag1);    

    if (gyro1 == 3) {
      digitalWrite(GYRO_LED, LOW);
    } else {
      digitalWrite(GYRO_LED, HIGH);
    }

    if (accel1 == 3) {
      digitalWrite(ACCEL_LED, LOW);
    } else {
      digitalWrite(ACCEL_LED, HIGH);
    }

    if (mag1 == 3) {
      digitalWrite(MAG_LED, LOW);
    } else {
      digitalWrite(MAG_LED, HIGH);
    }


    vbat_raw = analogRead(VBAT_PIN);
    vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);

    vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

    if (toggle_tone) {
      if (vbat_per < 10) {
        tone(buzzer, 3000);
      }
      if (vbat_per < 20) {
        digitalWrite(BAT_LED, HIGH);
      }
    }
    else {
      noTone(buzzer);
      digitalWrite(BAT_LED, LOW);
    }

    toggle_tone = !toggle_tone;
    count_bat = 0;
  }
  count_bat++;
  
  t1 = millis();
  t_diff = t1-t0;
  while(t_diff < ((1/t_frequ) * 1000)){
    t1 = millis();
    t_diff = t1-t0;
  }
  
  //Serial.println(t1-t0);
  
}


void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addService(bledis);//for uuid

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();


  // Start Advertising
  // - Enable auto advertising if disconnected
  // - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
  // - Timeout for fast mode is 30 seconds
  // - Start(timeout) with timeout = 0 will advertise forever (until connected)
  //
  // For recommended advertising interval
  // https://developer.apple.com/library/content/qa/qa1931/_index.html
  //
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}


// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

//battery level in percent
uint8_t mvToPercent(float mvolts) {
  uint8_t battery_level;

  if (mvolts >= 3000) {
    battery_level = 100;
  } else if (mvolts > 2900) {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  } else if (mvolts > 2740) {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  } else if (mvolts > 2440) {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  } else if (mvolts > 2100) {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  } else {
    battery_level = 0;
  }

  return battery_level;
}
