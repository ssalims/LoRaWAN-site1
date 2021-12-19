//stesyen 02
//Sun 28Nov2021-jadi
//https://www.influxdata.com/blog/connecting-the-things-network-to-influxdb/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "DHT.h"
#define DHTTYPE DHT11   // DHT 11
#define DHTPIN 3    
DHT dht(DHTPIN, DHTTYPE);

static const PROGMEM u1_t NWKSKEY[16] = { 0x26, 0xFE, 0x77, 0xDC, 0x96, 0x36, 0x35, 0x01, 0xA0, 0x32, 0x04, 0xBD, 0x7E, 0xD9, 0x0E, 0xC5 };
static const u1_t PROGMEM APPSKEY[16] = { 0xA0, 0x8E, 0x0E, 0xAA, 0x15, 0xD8, 0x65, 0xF6, 0x1C, 0xBD, 0x99, 0x85, 0x15, 0xF6, 0xBE, 0x4A };
static const u4_t DEVADDR = 0x260BFF2C ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10; //default 60

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
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
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    float temp_hum_val[2] = {0};
    static uint8_t message[4];
    volatile float averageVcc = 0.0;
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(fport, data, data_length, confirmed);
        ////LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
            
        if (!dht.readTempAndHumidity(temp_hum_val)) {
          

          
          volatile float humid_float = (float)temp_hum_val[0]; //humid
          delay(100);
          volatile float temp_float = (float)temp_hum_val[1]; //temp
          delay(100);
          
          int16_t fHumid = round(humid_float *100);
          int16_t fTemp = round(temp_float *100);

          if(humid_float >= 0 && humid_float <= 100)
            {
                message[0] = highByte(fHumid);
                message[1] = lowByte(fHumid);
                message[2] = highByte(fTemp);
                message[3] = lowByte(fTemp);   
            } 
 
           // Serial.print (message[0]);Serial.print(" xx ");Serial.print (message[1]);Serial.print(" xx ");
           // Serial.print (message[2]);Serial.print(" xx ");Serial.print (message[3]);Serial.println(" zzzz ");
            
            Serial.print("Humid:");
            Serial.print(humid_float);
            
            Serial.print(" Temp:");
            Serial.println(temp_float);
           //at port 2
          LMIC_setTxData2(2, message, sizeof(message), 0);    
        } else {
        Serial.println("Failed to get temprature and humidity value.");
         }
        
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    dht.begin();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9; //szs

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14); //szs
    ////LMIC_setDrTxpow(DR_SF12,20);  //lowest Datarate possible in 915MHz region
    
    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
