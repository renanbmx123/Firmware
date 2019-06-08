// Main library's.
#include "mbed.h"
// Sensor library's.
#include "ALTIMU.h"
#include "TinyGPSPlus.h"
#include "MQ4.h"
#include "DHT.h"
// Communication library.
#include "XBeeLib.h"
// Defines
#define TIMEOUT                30    // Set ticker time, for time interrupt.
#define NEW_PANID             0x1234 // Network Pan ID.
#define NEW_CHANNEL_MASK      0x7FFF // Network Channel.
#define SEARCH_NETWORK_RETRIES  20   // Numbers of retries search network.
#define DEBUG_LEDS                   // USED FOR DEBUG.
#define SAMPLES                 10   // Read and send some samples to network.
#define SLEEP_STATE             2    // Sleep state, shutdown all modules and sensor then go to sleep.
#define SEARCH_NETWORK_STATE    1    // Search for network, stays on this state if network doenst show up.
#define READ_SEND_DATA_STATE    0    // This state send data after reading it.
// Global objects.
using namespace XBeeLib;      // Namespace for XbeeLib
DigitalOut* sleep_req = NULL; //Xbee sleep request pin.
DigitalIn* on_sleep = NULL;   // Xbee sleep read pin, for check sleep state.
AnalogIn bat_pin(p16);        // Batery level read analog pin 16.
DigitalOut mq4_heater(p17);   // MQ4 heater power pin control.
DigitalOut GpsPwrPin(p15);    // GPS power pin control`
// This is used for debug, these leds are built in leds.
#if defined(DEBUG_LEDS)
DigitalOut led1(LED1);        // Debug Led 1
DigitalOut led2(LED2);        // Debug Led 1
DigitalOut led3(LED3);        // Debug Led 1
DigitalOut led4(LED4);        // Debug Led 1
#endif
// Control variables
bool mcu_sleep = false;   // microcontroler sleep control.
Ticker tim; // Ticker, for timer interrupt .
int conn_retries = 0; // Count numbers of trying to connect

XBeeZB xbee = XBeeZB(RADIO_TX, RADIO_RX, RADIO_RESET, NC, NC, 9600);
//Serial  pc(USBTX, USBRX,9600);// Serial Debug.
// Auxliar Function.
float bat_level();
bool is_radio_sleeping();
void sleep_radio();
void awake_radio();
void sleepManager();
TxStatus send_data_to_coordinator(XBeeZB& xbee, char *data);
float bat_level();
void XbeeNetworkSearch();
// Main function.
int main(){
  // Serial Configuration.
  sleep_req = new DigitalOut(RADIO_SLEEP_REQ); // Xbee Sleep request pin
  on_sleep = new DigitalIn(RADIO_ON_SLEEP);  //   Xbee on sleep pin.
  uint8_t state;
  char        data[200],  // Data to send
  NorthSouth = 'F',       // North South direction
  EastWest = 'F';         // East West direction
  uint16_t    id;         // Node identification
  int         time[3];    // UTC Time.
  float       g[3],       // Gyroscope
  acc[3]                  // Accelerometer
  ,mag[3],                // Magnetrometer
  dht_t,                  // DHT22 temperature sensor
  dht_h,                  // DHT22 humidity sensor
  piezo,                  // Vibration sensor
  vbat,                   // Batery percentage.
  lat,                    // Latitude .
  lon,                    // Longitude.
  alt,                    // Altitude from gps module.
  Balt,                   // Altitude from barometer sensor.
  speed;                  // Speed over the ground
  float      Bpress;      // Barometric pressure
  // Instantiate objects.
  DHT dht_th(p23,DHT22);                     // Dht (Digital humidity temperature) sensor on digital pin 23.
  AnalogIn piezo_sensor(p20);                // Piezo sensor on analog pin 20.
  MQ4 mq4(p18);                              // Gás sensor on analog pin 18.
  Serial GPSSerial(p9, p10,9600);
  Altimu lib_imu(p28,p27);
  TinyGPSPlus tgps;
  // Xbee object configuration.
  RadioStatus radioStatus = xbee.init();
  // Configure radio and get id number.
  uint32_t serialn_low;
  uint64_t current_panid;
  uint16_t current_channel_mask;
  // Get 2 bytes of the address module, set Node id with it.
  xbee.get_param("SL", &serialn_low);
  id = (uint16_t)serialn_low;
  // Set PAN_ID if is not configured.
  xbee.get_operating_panid(&current_panid);
  if (current_panid != NEW_PANID) {
    xbee.set_panid(NEW_PANID);
  }
  // Set CN (channel mask) if its is not set.
  xbee.get_channel_mask(&current_channel_mask);
  if (current_channel_mask != NEW_CHANNEL_MASK) {
    xbee.set_channel_mask(NEW_CHANNEL_MASK);
    xbee.write_config();
  }

  mq4_heater = 1;         // Set heater on, for Mq4 sensor set up.
  GpsPwrPin = 1;          // Set gps on.
  #if defined(DEBUG_LEDS)
  led1 = 1;
  #endif
  //wait(120);              // Wait sensor warm up on cold start, first start.
  #if defined(DEBUG_LEDS)
  led1 = 0;
  #endif
  MQ4_data_t MQ4_data;    // Store gas sensor information.
  mq4.begin();            // Begin te R0 calculation and sensor calibration.
  // Verify network connection at first start.
  XbeeNetworkSearch();
  state = READ_SEND_DATA_STATE;

  while(1){

    switch (state) {
      case READ_SEND_DATA_STATE:
      //Read data
        for (int j = 0; j < 10; j++){
        // Read 3 axis gyroscope sensor.
        lib_imu.read_L3GD20(&g[0],&g[1],&g[2]);
        // Read linear accleration and magnecti field.
        lib_imu.read_LSM303D(&acc[0],&acc[1],&acc[2],&mag[0],&mag[1],&mag[2]);
        // Read imu pressure in hPa and Altitude in metters.
        lib_imu.read_LPS25H(&Bpress, &Balt);
        // Read piezoeletric sensor.
        piezo = piezo_sensor.read();
        // Read 9V baterry charge.
        vbat = bat_level();
        dht_th.readData();
        // Read DHT22 temperature in celcius.
        dht_t = dht_th.ReadTemperature(CELCIUS);
        // Read DHT22 relative humidity
        dht_h = dht_th.ReadHumidity();
        // Read MQ4 gas sensor
        mq4.read(&MQ4_data);
        // Reading GPS data...
        for(int i =0;i<100;i++){
          if(GPSSerial.readable()){
            while (!tgps.encode(GPSSerial.getc()));
            i = 100;
          }
        }
        if(tgps.location.isValid()){
          lat = tgps.location.lat();
          lon = tgps.location.lng();
        }else{
          lat = 0;
          lon = 0;
        }
        if(tgps.altitude.isValid()){
          alt = tgps.altitude.meters();
        }else{
          alt = 0;
        }
        if(tgps.speed.isValid()){
          speed = tgps.speed.mps();
        }else{
          speed = 0;
        }
        if((tgps.NorthSouth == 'S') |
           (tgps.NorthSouth =='N')){
          NorthSouth = tgps.NorthSouth;
        }
        else{
          NorthSouth = 'F';
        }
        if ((tgps.EastWest == 'W') |
            (tgps.EastWest == 'E')){
          EastWest = tgps.EastWest;
        }else{
          EastWest = 'F';
        }
        if (tgps.time.isValid())
        {
          time[0] = tgps.time.hour();
          time[1] = tgps.time.minute();
          time[2] = tgps.time.second();
        }else{
          time[0] = 0;
          time[1] = 0;
          time[2] = 0;
        }
        //Store data into a vector.
        sprintf(data,"%4x,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d%d%d,%.3f,%.3f,%.2f,%c,%c,%.2f,%.2f,%.2f",id,
        g[0], g[1], g[2],acc[0],acc[1], acc[2], mag[0], mag[1], mag[2],Bpress, dht_h,dht_t,MQ4_data.ch4,piezo, time[0],time[1],time[2], lon, lat, speed, NorthSouth, EastWest, Balt, alt,vbat);
        while(send_data_to_coordinator(xbee, data) != TxStatusSuccess) // try to send data, every 100 ms
        {
          #if defined(DEBUG_LEDS)
          led1 = !led1;
          #endif
          wait_ms(500);
        }
        wait_ms(800);
      }
        state = SLEEP_STATE;
      break;
      case SLEEP_STATE:
        //Debug Led
        #if defined(DEBUG_LEDS)
        led2 = !led2;
        #endif

        sleep_radio();          // Set radio to sleep.
        mq4_heater = 0;         // Disable Mq4 sensor heater resistence.
        GpsPwrPin = 0;
        tim.attach(&sleepManager,TIMEOUT);  // attach timeout function
        sleep();
        //Debug Led
        #if defined(DEBUG_LEDS)
        led2 = !led2;
        #endif

        mq4_heater = 1;           // Disable Mq4 sensor heater resistence.
        GpsPwrPin = 1;            // Enable gps module.
        awake_radio();
        //wait(30);
        state = SEARCH_NETWORK_STATE;
        // whaiting for xbee join to network.
      break;
      case SEARCH_NETWORK_STATE:    // Looking for network before
        XbeeNetworkSearch();
        state = READ_SEND_DATA_STATE;
      break;
    }

  }
}
// Auxliar Function.
bool is_radio_sleeping()            // Set radio to sleep, return 0 if radio sleep.
{
  assert(on_sleep != NULL);
  return on_sleep->read() == 0;
}

void sleep_radio()                 // Sleep xbee radio function.
{
  assert(sleep_req != NULL);
  sleep_req->write(1);
}

void awake_radio()                // Wake up xbee radio funcion.
{
  assert(sleep_req != NULL);
  sleep_req->write(0);
  /* Wait until radio awakes. Typically 14 mS */
  while(is_radio_sleeping());
}

void sleepManager()              // Sleep Manager function.
{
  tim.detach();   // Deatach timer interrupt.
  #if defined(DEBUG_LEDS)
  led4 = !led4;
  #endif
}
TxStatus send_data_to_coordinator(XBeeZB& xbee, char *data)
{
  const TxStatus txStatus = xbee.send_data_to_coordinator((const uint8_t *)data, strlen(data));
  return txStatus;
}

float bat_level(){
  return bat_pin.read()*5;
}

void XbeeNetworkSearch(){
  // whaiting for xbee join to network.
  while (!xbee.is_joined()) {
    wait_ms(1000);
    conn_retries ++;
    if(conn_retries == SEARCH_NETWORK_RETRIES){
      tim.attach(&sleepManager,TIMEOUT);  // Call this function every time when reach TIMEOUT value.
      sleep();  // Sleep for a while when network is not reach.
      conn_retries = 0; // Reset the count variable.
    }
    #if defined(DEBUG_LEDS)
    led3 = !led3;
    #endif
  }
  led3 = 0;
}
