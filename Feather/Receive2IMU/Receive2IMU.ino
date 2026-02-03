// Receiver BLE Left Arm: I2C address 45

#include <Wire.h>
#include <math.h>

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
//#include "utility/quat_calc.h"


int findConnHandle(uint16_t conn_handle);
void bleuart_rx_callback(BLEClientUart& uart_svc);
void scan_callback(ble_gap_evt_adv_report_t* report);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
bool comp_name(char name[], char ref[]);


#define MAX_CON (2)

float fakeBuff[4]; // RIGHT IMU 2+2(Up bluetooth+cable); Left IMU 2+2(Up bluetooth+cable);

//float triggerPin;

byte buff_I2C[16]; // 12 signal (number*4byte)

// Struct containing peripheral info
typedef struct
{
  char name[7 + 1]; // size name sender

  uint16_t conn_handle;

  // Each prph need its own bleuart client service
  BLEClientUart bleuart;

  uint8_t buffer1[4]; // buffer sender size
  int cur = 0;

  bool rdy = false;
  uint8_t buffer2[4]; // buffer sender size

} prph_info_t;

prph_info_t prphs[MAX_CON];

int connection_num = 0;

//for angle
union u_double
{
  double  dbl;
  char    data[sizeof(double)];
};

double angle_old = 0;

union u_split {
  byte to_byte[4];
  float to_float;
} split;




/* Set the delay between fresh samples */
#define SAMPLERATE_DELAY_MMS (3000)

//timing
int t0 = 0;
int t1 = 0;
int t_diff = 0;
//int count = 0;
float t_frequ = 200; //in Hz --- max 500Hz

void setup()
{
  Serial.begin(115200);

  Wire.begin(45); // set ID i2c communication
  Wire.setClock(400000);
  Wire.onRequest(requestEvent);

  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  //switch
  //pinMode(7, INPUT); // trigger
  //pinMode(11, OUTPUT); // power pin
  //digitalWrite(11, HIGH); 
  
  // Initialize Bluefruit with max concurrent connections as Peripheral = 0, Central = 4
  // SRAM usage required by SoftDevice will increase with number of connections

  Bluefruit.begin(0, MAX_CON);

  // Set Name
  Bluefruit.setName("HipExtioRe");
                     

  // Init peripheral pool
  for (uint8_t idx = 0; idx < MAX_CON; idx++) {

    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;

    // All of BLE Central Uart Serivce
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);


  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Filter only accept bleuart service in advertising
     - Don't use active scan (used to retrieve the optional scan response adv packet)
     - Start(0) = will scan forever since no timeout is given
  */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
//  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.setInterval(8, 8);       // in units of 0.625 ms
  //Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.filterUuid(0x063A,0x063B);
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds


}

/**
   Callback invoked when scanner picks up an advertising packet
   @param report Structural advertising data
*/
void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}

/**
   Callback invoked when an connection is established
   @param conn_handle
*/
void connect_callback(uint16_t conn_handle) {
  // Find an available ID to use
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Eeek: Exceeded the number of connections !!!
  if ( id < 0 ) return;

  char name[8 + 1];
  Bluefruit.Connection(conn_handle)->getPeerName(name, sizeof(name) - 1);
  
  char id0[8 + 1] = "Angle__R";
  char id1[8 + 1] = "Angle__L";

  if (comp_name(name, id0)) {
    id = 0;
    
  }
  else if (comp_name(name, id1)) {
    id = 1;
  }
  else {
    id = -1;
  }

  if ( id < 0 ) {
    Bluefruit.disconnect(conn_handle);
    return;
  }

  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;

  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name) - 1);

  if ( peer->bleuart.discover(conn_handle) )
  {
    peer->bleuart.enableTXD();

    Bluefruit.Scanner.start(0);
  } else
  {
    //  since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }

  connection_num++;
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

}

/**
   Callback invoked when BLE UART data is received
   @param uart_svc Reference object to the service where the data
   arrived.
*/
void bleuart_rx_callback(BLEClientUart& uart_svc) {
  // uart_svc is prphs[conn_handle].bleuart
  uint16_t conn_handle = uart_svc.connHandle();

  int id = findConnHandle(conn_handle);
  prph_info_t* peer = &prphs[id];

  memset(peer->buffer1, 0, 4); // buffer sender size
  uart_svc.read(peer->buffer1, 4);

  for (int i = 0; i < 4; i++) {
    peer->buffer2[i] = peer->buffer1[i]; 
  }
  peer->rdy = true;

}
void loop()
{
  
  // trigger
  //triggerPin = digitalRead(7);
  
  t0 = millis();
  // Right
  int16_t zR11, zR12, zR13;//right up bluetooth leg IMU euler/vel/gra
  // Left  
  int16_t zL11, zL12, zL13;//left up bluetooth leg IMU euler/vel/gra
  // initializing right and left
  zR11 = zR12 = 0;
  zL11 = zL12 = 0;
  
  // Right leg IMU euler/vel/gra ---> 1 2 3
  zR11 = ((int16_t)prphs[0].buffer2[0]) | (((int16_t)prphs[0].buffer2[1]) << 8);
  zR12 = ((int16_t)prphs[0].buffer2[2]) | (((int16_t)prphs[0].buffer2[3]) << 8);
  //zR13 = ((int16_t)prphs[0].buffer2[4]) | (((int16_t)prphs[0].buffer2[5]) << 8);
  // Left leg IMU euler/vel/gra
  zL11 = ((int16_t)prphs[1].buffer2[0]) | (((int16_t)prphs[1].buffer2[1]) << 8);
  zL12 = ((int16_t)prphs[1].buffer2[2]) | (((int16_t)prphs[1].buffer2[3]) << 8);
  //zL13 = ((int16_t)prphs[1].buffer2[4]) | (((int16_t)prphs[1].buffer2[5]) << 8);
  
  //right leg
  fakeBuff[0] = ((double)zR11) / 16.0;//Euler
  fakeBuff[1] = ((double)zR12) / 16.0;//angRate
  //fakeBuff[2] = ((double)zR13) / 100.0;//gra
 //left leg
  fakeBuff[2] = ((double)zL11) / 16.0;//Euler
  fakeBuff[3] = ((double)zL12) / 16.0;//angRate
  //fakeBuff[5] = ((double)zL13) / 100.0;//gra
//if (count %5 == 0){  
//  Serial.print(fakeBuff[0]);
//  Serial.print(" ");
//  Serial.print(fakeBuff[1]);
//  Serial.print(" ");
//  Serial.print(fakeBuff[2]);
//  Serial.print(" ");
//  Serial.print(fakeBuff[3]);
//  Serial.print(" ");
//  Serial.print(fakeBuff[4]);
//  Serial.print(" ");
//  Serial.print(fakeBuff[5]);
////  Serial.print(" | ");
////  Serial.print(fakeBuff[6]);
////  Serial.print(" ");
//  Serial.print(fakeBuff[7]);
//  Serial.print(" ");
////  Serial.print(fakeBuff[8]);
////  Serial.print(" ");
////  Serial.print(fakeBuff[9]);
////  Serial.print(" ");
//  Serial.print(fakeBuff[10]);
//  Serial.print(" ");
//  Serial.print(fakeBuff[11]);
// 
//  Serial.println(); 
//}
//  count += 1;
  
  t1 = millis();
  t_diff = t1-t0;
  while(t_diff < ((1/t_frequ) * 1000)){
    t1 = millis();
    t_diff = t1-t0;
  }
  /*
  Serial.print(" ");
  Serial.println(t1-t0);
  */
}

/**
   Find the connection handle in the peripheral array
   @param conn_handle Connection handle
   @return array index if found, otherwise -1
*/

int findConnHandle(uint16_t conn_handle) {
  for (int id = 0; id < MAX_CON; id++)
  {
    if (conn_handle == prphs[id].conn_handle)
    {
      return id;
    }
  }

  return -1;
}

bool comp_name(char name[], char ref[]) {

  for (int i = 0; i < 8; i++) {
    if (name[i] != ref[i]) {
      return false;
    }
  }
  return true;

}

// no Serial.print or delay in the function otherwise doesn't work!
void requestEvent () {
// buffer size in single (num*4)
  for (int i = 0; i < 4; i++) {
    split.to_float = fakeBuff[i];
    for (int j = 0; j < 4; j++) {

      buff_I2C[(4 * i) + j] = split.to_byte[j];
    }
  }

  Wire.write(buff_I2C, 16);

}
