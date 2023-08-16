#include <Arduino.h>
#include <bluefruit.h>

float adcvalue = 0;




bool buttonPressed=false;
float mv_per_lsb = 3600.0F/1024.0F; 
/************************ ADS1292 FUNCTIONS *************************************************************/
String hex_to_char(int hex_in) {
  int precision = 2;
  char tmp[16];
  char format[128];
  sprintf(format, "0x%%.%dX", precision);
  sprintf(tmp, format, hex_in);
  //Serial.print(tmp);
  return(String(tmp));
}

// BLE Service


BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery


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
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
 void lowPower() {
  //NRF_POWER_MODE_LOWPWR     /**< Low power mode. See power management in the reference manual. */
  if (checkForSoftDevice() == 1) {
    // SoftDevice enabled
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  } else {
    // No SoftDevice
    NRF_POWER->TASKS_LOWPWR = 1;
  }
}
uint8_t checkForSoftDevice() {
  uint8_t check;
  sd_softdevice_is_enabled(&check);

  return check;
}
void powerOff() {
  if (checkForSoftDevice() == 1) {
    // SoftDevice enabled
    sd_power_system_off();
  } else {
    // No SoftDevice
    NRF_POWER->SYSTEMOFF = 1;
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
 // powerOff();
 
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
void setupBluetooth(){
 
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();
  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries:)");
  bledis.setModel("Fea Bluefruit  :=)");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

}
void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}


void buttonHandler() {
  delay(20000);
  if ((digitalRead(A3) == 1) || (digitalRead(A5) == 1)){
      buttonPressed = true;
      powerOff();
   }
   
}
void setup(){
    // initialize the serial communication:
  Serial.begin(115200);
  pinMode(A1, INPUT); // Setup for leads off detection LO +
  pinMode(A3, INPUT); // Setup for leads off detection LO -

  setupBluetooth();
 // attachInterrupt(digitalPinToInterrupt(A1), buttonHandler, RISING);
 // attachInterrupt(digitalPinToInterrupt(A3), buttonHandler, RISING);
  
}

int j=0;
int estado=0;

void loop() {
 
  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
          Serial.println("Recibí ");

    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }
   String cad;
   for(int i=0;i<40;i++){
     if ((digitalRead(A1) == 1) || (digitalRead(A3) == 1)) {
         Serial.println('!');
    }
    else {
      // send the value of analog input 0:
     // adcvalue=0;
     //  for(int j=0;j<5;j++){
       //   adcvalue+=analogRead(A5);
         // }
      // adcvalue=adcvalue/5;
      String cad1=String(int(analogRead(A5)) )+'\n';
    // Serial.println(cad1);
    
     cad = cad+cad1;
   }
   delay(10);
  }
     bleuart.write( cad.c_str(),cad.length());
   


   
}
