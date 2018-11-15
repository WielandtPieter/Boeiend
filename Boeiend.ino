


//GPS
#include<TinyGPS.h>
#include<SoftwareSerial.h>
//Accelerometer
#include <Wire.h>
//Temperature sensors
//#include <OneWire.h>
//#include <DallasTemperature.h>
//LoRa Module
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//Sleep
#include <DeepSleepScheduler.h>

#define VERBOSE

//Accelerometer parameters
#define ACC_ID          0x53  //ADXL345 Device ID
#define ACC_POWER_CTL   0x2D  //Power Control Register
#define ACC_DATA_FORMAT 0x31
#define ACC_START_BYTE  0x32  //X-Axis Data 0
#define ACC_BYTES       0x06  //Number of databytes
#define ACC_GAIN        0.00376390  //Convertion factor to g

//GPS declaraties
TinyGPS gps;
SoftwareSerial ss(4, 3);

byte acc_buffer[ACC_BYTES];

//Temperature sensor parameters
//#define TEMP_BUS      7     //Onewire pin
//#define NUM_TEMP_SENSORS  1  //Number of temperature sensors 

//OneWire oneWire(TEMP_BUS);                  //Setup onwire on selected pin
//DallasTemperature temp_sensors(&oneWire);   //Setup temperature sensor
//DeviceAddress tempSensorAddresses[NUM_TEMP_SENSORS];

#define POWER_ENABLE_PIN 8
#define MEASURE_INTERVAL 10*1000UL   //Measurement interval time in ms each quarter

//#define LORA_LPP_TEMP       0x67
#define LORA_LPP_ACC        0x71
#define LORA_LPP_ANALOG_OUT 0x03
#define LORA_LPP_GPS        0x88
//uint8_t temp_sensor_channel[1] = {0};


#define LORA_PACKET_SIZE (8+4+11)//GPS toegevoegd 9 bytes



// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = {0xEA, 0xC8, 0x01, 0x73, 0x1B, 0xCA, 0xD2, 0xFC, 0xB2, 0xCC, 0x14, 0x5D, 0xD6, 0x31, 0xFA, 0x44};
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = {0xCA, 0xE1, 0x3F, 0x57, 0x30, 0x20, 0x96, 0xB3, 0xC8, 0x04, 0x40, 0xF8, 0xA6, 0x0A, 0xF8, 0x2E};
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011EF1;

static uint8_t lora_stream[LORA_PACKET_SIZE];
static osjob_t sendjob;

// Schedule TX every this many seconds
// (might become longer due to duty
// cycle limitations in the ISM band).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 6,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};


void setup()
{

  //softwareserial voor GPS
  ss.begin(9600);

  //uint8_t detectedTempSensors;

  //Enable Serial Port
#ifdef VERBOSE
  Serial.begin(115200);
#endif

  //Enable Pin 3V3 LDO
  pinMode(POWER_ENABLE_PIN, OUTPUT);
#ifdef VERBOSE
  Serial.println(F("Dramco-UNO"));
#endif
 /* //Start up temperature sensors
  Serial.print(F("Adding temperature sensors, expecting "));
  Serial.print(NUM_TEMP_SENSORS);
  Serial.println(F(" sensors."));

  Serial.println(F("Add one sensor at a time in order to know the order of the temp. sensors."));


#endif

  bool stop = false;
  int i = 0;

  do {

    temp_sensors.begin();
    detectedTempSensors = temp_sensors.getDeviceCount();
#ifdef VERBOSE
    Serial.print(detectedTempSensors);
    Serial.println(F(" temperature sensor(s) detected"));
#endif

    for (int j = 0; j < detectedTempSensors; j++) {
      temp_sensors.getAddress(tempSensorAddresses[j], j);
#ifdef VERBOSE
      Serial.print(F("Sensor "));
      Serial.print(j);
      Serial.print(F(": "));
      printAddress(tempSensorAddresses[j]);
#endif
    }

    // read to clear rx buffer
    while (Serial.available() > 0) { Serial.read();}

    Serial.println(F(" Scan again (Y/N)?"));
    while (Serial.available() == 0) { }

    
    if (Serial.read() == 'N') stop = true;
    i++;
    // read to clear rx buffer
    while (Serial.available() > 0) { Serial.read();}
    
  } while (i < NUM_TEMP_SENSORS && !stop);


  temp_sensors.setResolution(10);   //10 bit -> conversion time 187.5 ms //resolutie lager maken?*/

  //Start measurement scheduler
  scheduler.schedule(measure);
}

// function to print a device address
/*void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

*/
void measure() {
  //uint8_t detectedTempSensors;
  //float temp_buffer[NUM_TEMP_SENSORS];
  //Enable 3V3 LDO
  digitalWrite(POWER_ENABLE_PIN, HIGH);

  //Start Lora

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF11, 14);

  //Start up Accelerometer
  Wire.begin();        // join i2c bus (address optional for master)

  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(ACC_DATA_FORMAT, 0x00);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(ACC_POWER_CTL, 0x08);

 /* temp_sensors.requestTemperatures(); // Send the command to get temperature readings
  for (uint8_t i = 0; i < NUM_TEMP_SENSORS; i++) {
    temp_buffer[i] = temp_sensors.getTempC(tempSensorAddresses[i]);

    lora_stream[0 + 4 * i] =0; //temp_sensor_channel[i];
    lora_stream[1 + 4 * i] =0; // LORA_LPP_TEMP;
    lora_stream[2 + 4 * i] =0; // (uint8_t)((int16_t)(temp_buffer[i] * 10) >> 8);
    lora_stream[3 + 4 * i] =0; //(uint8_t)((int16_t)(temp_buffer[i] * 10) & 0xFF);
#ifdef VERBOSE
    Serial.print("Temp ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(temp_buffer[i]);
#endif
 }*/
#ifdef VERBOSE
  Serial.println();
#endif

  //Read accellerometer data (Doesn't work without Temp sensor)
  readFrom(ACC_START_BYTE, ACC_BYTES, acc_buffer); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int acc_x = (int)(((((int)acc_buffer[1]) << 8) | acc_buffer[0]) * 3.76390);
  int acc_y = (int)(((((int)acc_buffer[3]) << 8) | acc_buffer[2]) * 3.76390);
  int acc_z = (int)(((((int)acc_buffer[5]) << 8) | acc_buffer[4]) * 3.76390);

  lora_stream[/*4 * NUM_TEMP_SENSORS */0] = 3;
  lora_stream[/*4 * NUM_TEMP_SENSORS */1] = LORA_LPP_ACC;
  lora_stream[/*4 * NUM_TEMP_SENSORS */ 2] = acc_x >> 8;
  lora_stream[/*4 * NUM_TEMP_SENSORS */3] = acc_x & 0xFF;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/4] = acc_y >> 8;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/5] = acc_y & 0xFF;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/6] = acc_z >> 8;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/7] = acc_z & 0xFF;
#ifdef VERBOSE
  Serial.print(F("x: "));
  Serial.print(acc_x);
  Serial.print(F(" y: "));
  Serial.print(acc_y);
  Serial.print(F(" z: "));
  Serial.println(acc_z);
#endif

  //Read Voltage
  long batteryVoltage = readVcc();
#ifdef VERBOSE
  Serial.print(F("Vbatt: "));
  Serial.println(batteryVoltage, DEC);
#endif
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/8] = 3;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/9] = LORA_LPP_ANALOG_OUT;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/10] = (uint8_t)((int16_t)(batteryVoltage / 10) >> 8);
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/11] = (uint8_t)((int16_t)(batteryVoltage / 10) & 0xFF);

  // Start job
  do_send(&sendjob);

  os_runloop_once();

  delay(1100); // only read every 0,5 seconds

  //Sleep
  digitalWrite(POWER_ENABLE_PIN, LOW);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  scheduler.scheduleDelayed(measure, MEASURE_INTERVAL);

  //GPS CODE

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  float flat, flon;
  if (newData)
  {
    //float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");


/*
  long latt =((long)flat);
  int longg = (int)(((((int)acc_buffer[3]) << 8) | acc_buffer[2]) * 3.76390);
  int altt = (int)(((((int)acc_buffer[5]) << 8) | acc_buffer[4]) * 3.76390);

  */

  lora_stream[/*4 * NUM_TEMP_SENSORS */12] = 3;
  lora_stream[/*4 * NUM_TEMP_SENSORS */13] = LORA_LPP_GPS;
  lora_stream[/*4 * NUM_TEMP_SENSORS */ 14] = (uint32_t)flat >> 16;
  lora_stream[/*4 * NUM_TEMP_SENSORS */ 15] = (uint32_t)flat >> 8;
  lora_stream[/*4 * NUM_TEMP_SENSORS */16] = (uint32_t)flat & 0xFF;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/17] = (uint32_t)flon >> 16;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/18] = (uint32_t)flon >> 8;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/19] = (uint32_t)flon & 0xFF;
  
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/20] = 0x00;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/21] = 0x00;
  lora_stream[/*4 * NUM_TEMP_SENSORS +*/22] = 0x00;

  
    
}

void loop()
{
  scheduler.execute();
}

//LoRa Module Functions
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      Serial.println(os_getTime() + sec2osticks(TX_INTERVAL));
      break;
    default:
      Serial.println(F("LoRa someting happened"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lora_stream, sizeof(lora_stream), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
//Accelero Functions
void writeTo(byte address, byte val) {
  Wire.beginTransmission(ACC_ID); // start transmission to device
  Wire.write(address);            // send register address
  Wire.write(val);                // send value to write
  Wire.endTransmission();         // end transmission
}

void readFrom(byte address, int num, byte _buff[]) {
  int i = 0;

  Wire.beginTransmission(ACC_ID); // start transmission to device
  Wire.write(address);            // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(ACC_ID); // start transmission to device
  Wire.requestFrom(ACC_ID, num);  // request 6 bytes from device

  while (Wire.available()) {      // device may send less than requested (abnormal)
    acc_buffer[i] = Wire.read();  // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}


long readVcc() {
  long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
