/*
 * Open Home Security - Extension node with RFM69, Power 12V
 * 8 Analog and 1 Digital zones
 * Relay for Siren/Horn, AC_OFF, BATTERY_OK signals.
 * 
 * Board v.1.00
 */

#include <SPI.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <avr/eeprom.h> // Global configuration for in chip EEPROM

// Node settings
#define NODEID           5    // This is our address 
#define VERSION          100  // Version of EEPROM struct
#define PING_DELAY       1200000 // In milliseconds, 20 minutes
#define SENSOR_DELAY     600000  // In milliseconds, 10 minutes
// Constants
#define REG_LEN          21   // Size of one conf. element
#define NODE_NAME_SIZE   16   // As defined in gateway
#define ALARM_ZONES      9    // Fixed number by hardware
// Pins
#define DE               3    // RS 485 DE pin
#define BOX_TAMPER       4    // 
#define AC_OFF           5    // 
#define BATTERY_OK       6    // 
#define RELAY            9    //
// Radio
#define NETWORKID       100  // Do not change, defined in gateway
#define GATEWAYID       1    // Do not change gateway address
#define RADIO_REPEAT    5    // Repeat sending
#define FREQUENCY       RF69_868MHZ // Match this with the version of your gateway (others: RF69_433MHZ, RF69_915MHZ)
#define KEY             "ABCDABCDABCDABCD" // Has to be same 16 characters/bytes on all nodes, not more not less!
//#define ENABLE_ATC      // Comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI        -75
// Alarm definition constants
#define ALARM_PIR_LOW    560
#define ALARM_PIR        715
#define ALARM_PIR_HI     900
#define ALARM_OK_LOW     270
#define ALARM_OK         380
#define ALARM_OK_HI      490
#define ALARM_UNBALANCED 100
#define ALARM_TAMPER     0
// Some macro helpers
#define GET_ZONE_SENT(x)             ((x) & 0b1)
#define GET_ZONE_SENT_TAMPER(x)      ((x >> 1U) & 0b1)
#define SET_ZONE_SENT(x)             x |= 1
#define SET_ZONE_SENT_TAMPER(x)      x |= (1 << 1U)
#define CLEAR_ZONE_SENT(x)           x &= ~1
#define CLEAR_ZONE_SENT_TAMPER(x)    x &= ~(1 << 1U)
#define GET_CONF_ZONE_ENABLED(x)     ((x) & 0b1)
#define GET_CONF_ZONE_PIR_AS_TMP(x)  ((x >> 1U) & 0b1)
#define GET_CONF_ZONE_BALANCED(x)    ((x >> 2U) & 0b1)
#define GET_CONF_ZONE_TYPE(x)        ((x >> 7U) & 0b1)

#ifdef ENABLE_ATC 
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

// Hardware pin to software zone number map
static const uint8_t zonePins[] = {A5, A4, A3, A2, A1, A0, A7, A6};

// Global variables
int8_t   resp;
uint8_t  mode = 0;
uint8_t  pos;
uint8_t  msg[REG_LEN+1], out_msg[1+(ALARM_ZONES*2)];
uint8_t  radioLength;
uint8_t  toSend = 0;
uint8_t  acOff = 0;
uint16_t val;
unsigned long zoneMillis;
unsigned long aliveMillis;
uint16_t pirDelay = 60000; // 60 seconds to allow PIR to settle

// Configuration struct
struct config_t {
  uint16_t version;
  char     reg[REG_LEN * 11]; // Number of elements on this node
} conf;

// Float conversion
union u_tag {
    uint8_t  b[4];
    float    fval;
} u;

// Zone runtime variables
typedef struct {
  char    lastEvent = 'U';
  uint8_t setting = 0;
} zone_t;
zone_t zone[ALARM_ZONES];

/* 
 * Registration 
 */
void sendConf(){
  int8_t result;
  uint16_t count = 0;

  // Wait some time to avoid contention
  delay(NODEID * 1000);   

  Serial.print(F("Conf:"));

  while (count < sizeof(conf.reg)) {
    msg[0] = 'R'; // Registration flag
    memcpy(&msg[1], &conf.reg[count], REG_LEN);    
    result = radio.sendWithRetry(GATEWAYID, msg, REG_LEN + 1, RADIO_REPEAT);
    Serial.print(F(" ")); Serial.print(result);
    count += REG_LEN;
  }  
  
  Serial.println(F("."));
}
/*
 * Set defaults
 */
void setDefault(){
  conf.version = VERSION;   // Change VERSION to take effect
  conf.reg[0+(REG_LEN*0)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*0)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*0)]  = 12;        // Zone number
  conf.reg[3+(REG_LEN*0)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*0)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*0)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*1)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*1)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*1)]  = 13;        // Zone number
  conf.reg[3+(REG_LEN*1)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*1)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*1)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*2)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*2)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*2)]  = 14;        // Zone number
  conf.reg[3+(REG_LEN*2)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*2)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*2)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*3)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*3)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*3)]  = 15;        // Zone number
  conf.reg[3+(REG_LEN*3)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*3)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*3)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*4)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*4)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*4)]  = 16;        // Zone number
  conf.reg[3+(REG_LEN*4)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*4)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*4)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*5)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*5)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*5)]  = 17;        // Zone number
  conf.reg[3+(REG_LEN*5)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*5)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*5)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*6)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*6)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*6)]  = 18;        // Zone number
  conf.reg[3+(REG_LEN*6)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*6)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*6)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*7)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*7)]  = 'A';       // Analog
  conf.reg[2+(REG_LEN*7)]  = 19;        // Zone number
  conf.reg[3+(REG_LEN*7)]  = B10000100; // Analog, balanced
  conf.reg[4+(REG_LEN*7)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*7)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*8)]  = 'Z';       // Zone
  conf.reg[1+(REG_LEN*8)]  = 'D';       // Digital
  conf.reg[2+(REG_LEN*8)]  = 20;        // Zone number
  conf.reg[3+(REG_LEN*8)]  = B00000010; // Digital, unbalanced, PIR as tamper
  conf.reg[4+(REG_LEN*8)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*8)], 0, NODE_NAME_SIZE);
  strcpy(&conf.reg[5+(REG_LEN*8)], "Box tamper"); // Set default name
  conf.reg[0+(REG_LEN*9)]  = 'H';       // Horn/Siren 
  conf.reg[1+(REG_LEN*9)]  = 'D';       // Digital 
  conf.reg[2+(REG_LEN*9)]  = 1;         // Local index, !Used in code bellow to match!
  conf.reg[3+(REG_LEN*9)]  = B00000000; // Default setting
  conf.reg[4+(REG_LEN*9)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*9)], 0, NODE_NAME_SIZE);
  strcpy(&conf.reg[5+(REG_LEN*9)], "Remote siren"); // Set default name
  conf.reg[0+(REG_LEN*10)]  = 'S';       // Sensor
  conf.reg[1+(REG_LEN*10)]  = 'D';       // Digital 
  conf.reg[2+(REG_LEN*10)]  = 1;         // Local index, !Used in code bellow to match!
  conf.reg[3+(REG_LEN*10)]  = B00000000; // Default setting
  conf.reg[4+(REG_LEN*10)]  = B00011110; // Default setting, group='not set', disabled   
  memset(&conf.reg[5+(REG_LEN*10)], 0, NODE_NAME_SIZE);
  strcpy(&conf.reg[5+(REG_LEN*10)], "Remote AC Off"); // Set default name
  
  eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration 
  
  Serial.println(F("Default conf."));
}
/*
 * Send ping command to gateway 
 */
void sendPing(void) {
  msg[0] = 'C'; // Command
  msg[1] = 2;   // PING = 2
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 2);
}
/*
 * Process incoming radio data
 */
void checkRadio(){
  // Look for incomming transmissions
  if (radio.receiveDone()) {
    radioLength = radio.DATALEN; 
    if (radio.ACKRequested()) { 
      delay(5); // wait after receive, we need this delay or gateway will not see ACK!!!
      radio.sendACK();
      //Serial.print(F("ACK:"));
    }
    //for (uint8_t ii=0; ii < radioLength; ii++){ Serial.print((char)radio.DATA[ii], HEX); Serial.print("-"); }; Serial.println(F("<"));
    if ((char)radio.DATA[0] == 'C') {
      Serial.print(F("C:")); Serial.println(radio.DATA[1]);
      // Commands from gateway
      switch (radio.DATA[1]) {
        case 1: // Request for registration
          sendConf(); 
          // Clear sent zones, maybe GW was restarted
          for(uint8_t ii = 0; ii < ALARM_ZONES; ii++) {
            CLEAR_ZONE_SENT(zone[ii].setting);
            SET_ZONE_SENT_TAMPER(zone[ii].setting); // To send OK for balanced
          }
          break;
        default: break;
      }
    }
    if ((char)radio.DATA[0] == 'R') { // Registration
      Serial.print(F("R:"));
      // Replace part of conf string with new paramters.
      pos = 0; 
      while (((conf.reg[pos] != radio.DATA[1]) || (conf.reg[pos+1] != radio.DATA[2]) ||
              (conf.reg[pos+2] != radio.DATA[3])) && (pos < sizeof(conf.reg))) {
        pos += REG_LEN; // size of one conf. element
      }
      if (pos < sizeof(conf.reg)) {
        Serial.println(pos/REG_LEN); // Show # of updated element       
        memcpy(&conf.reg[pos], &radio.DATA[1], REG_LEN);
        // Save it to EEPROM
        conf.version = VERSION;
        eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration              
      }
    }
    // Siren/Horn relay
    if ((char)radio.DATA[0] == 'H') {
      Serial.print(F("H:"));
      Serial.print(radio.DATA[1], DEC);
      Serial.print(F("="));
      Serial.println(radio.DATA[2], DEC);
      // Number matches local conf.freg.
      if (radio.DATA[1] == conf.reg[2+(REG_LEN*9)]) digitalWrite(RELAY, radio.DATA[2]);
    }
  }
}
/*
 * Send float value of one element to gateway 
 */
void sendValue(uint8_t element, float value) {
  u.fval = value; 
  msg[0] = conf.reg[(REG_LEN*element)];
  msg[1] = conf.reg[1+(REG_LEN*element)];
  msg[2] = conf.reg[2+(REG_LEN*element)];
  msg[3] = u.b[0]; msg[4] = u.b[1]; msg[5] = u.b[2]; msg[6] = u.b[3];
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 7);
}
/*
 * Setup
 */
void setup() {
  pinMode(A0, INPUT); pinMode(A1, INPUT);
  pinMode(A2, INPUT); pinMode(A3, INPUT);
  pinMode(A4, INPUT); pinMode(A5, INPUT);
  pinMode(A6, INPUT); pinMode(A7, INPUT);
  pinMode(BOX_TAMPER, INPUT);
  pinMode(DE, OUTPUT);
  pinMode(AC_OFF, INPUT);
  pinMode(BATTERY_OK, INPUT);
  pinMode(RELAY, OUTPUT); digitalWrite(RELAY, LOW);
  // Free IO ports
  //pinMode(7, OUTPUT); digitalWrite(7, LOW); 
  //pinMode(8, OUTPUT); digitalWrite(8, LOW); 

  // RFM69
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower();  // uncomment only for RFM69HW!
  // radio.encrypt(KEY); // uncomment if you use encryption
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif
    
  Serial.begin(115200); 
  Serial.println(); Serial.println(F("Start"));

  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  if (conf.version != VERSION) setDefault();

  sendConf();

  zoneMillis  = millis();
  aliveMillis = millis();

  // Let's start
  delay(1000);
}
/*
 * Main loop
 */
void loop() {
  // Check battery state
  // The signal is "Low" when the voltage of battery is under 11V
  if (digitalRead(BATTERY_OK) == LOW) {    
    return; // Go to loop(), and do nothing
  }

  // Check AC state
  // The signal turns to be "High" when the power supply turns OFF
  if ((digitalRead(AC_OFF) == HIGH) && (acOff == 0)) {
    acOff = 1;    
    sendValue(10, (float)1); // Send it to GW
  }
  if ((digitalRead(AC_OFF) == LOW) && (acOff == 1)) {
    acOff = 0;
    sendValue(10, (float)0); // Send it to GW
  }

  // Process radio data
  checkRadio();

  // Zones
  if ((unsigned long)(millis() - zoneMillis) > pirDelay) {
    zoneMillis = millis(); 
    pirDelay = 250; // PIR settled
    // Clear toSend
    toSend = 0;
    // Cycle trough zones
    for(uint8_t zoneNum = 0; zoneNum < ALARM_ZONES; zoneNum++) {
      if (GET_CONF_ZONE_ENABLED(conf.reg[4+(REG_LEN*zoneNum)])) {
        // Digital 0, Analog 1
        if (GET_CONF_ZONE_TYPE(conf.reg[3+(REG_LEN*zoneNum)])) {
          // Do ADC
          val = analogRead(zonePins[zoneNum]);;
          // Force unbalanced for analog zones
          if (!GET_CONF_ZONE_BALANCED(conf.reg[3+(REG_LEN*zoneNum)])){
            if (val < ALARM_UNBALANCED) val = ALARM_OK;
            else                        val = ALARM_PIR;
          }
        } else {
          // We have just one digital zone
          if (digitalRead(BOX_TAMPER)) val = ALARM_PIR;
          else                         val = ALARM_OK;
        }

        // alarm as tamper && is PIR, then make it tamper
        if ((GET_CONF_ZONE_PIR_AS_TMP(conf.reg[3+(REG_LEN*zoneNum)])) && 
            (val >= ALARM_PIR_LOW && val <= ALARM_PIR_HI)) val = ALARM_TAMPER;

        // Decide zone state
        switch ((uint16_t)(val)) {
          case ALARM_OK_LOW ... ALARM_OK_HI:
            // Do not send OK for balanced zones, gateway swithes them to OK automatically 
            if ((zone[zoneNum].lastEvent == 'O') && !GET_ZONE_SENT(zone[zoneNum].setting) &&
                (!GET_CONF_ZONE_BALANCED(conf.reg[3+(REG_LEN*zoneNum)]) || GET_ZONE_SENT_TAMPER(zone[zoneNum].setting))) {
              SET_ZONE_SENT(zone[zoneNum].setting);
              CLEAR_ZONE_SENT_TAMPER(zone[zoneNum].setting);
              out_msg[toSend*2+1] = conf.reg[2+(REG_LEN*zoneNum)]; // Zone number as in cof.reg
              out_msg[toSend*2+2] = zone[zoneNum].lastEvent;
              toSend++;
            }
            if ((zone[zoneNum].lastEvent != 'O') && 
              (!GET_CONF_ZONE_BALANCED(conf.reg[3+(REG_LEN*zoneNum)]) || GET_ZONE_SENT_TAMPER(zone[zoneNum].setting))) {
              CLEAR_ZONE_SENT(zone[zoneNum].setting);
            }
            zone[zoneNum].lastEvent = 'O';
            break;
          case ALARM_PIR_LOW ... ALARM_PIR_HI:
            if ((zone[zoneNum].lastEvent == 'P') && !GET_ZONE_SENT(zone[zoneNum].setting)) {
              SET_ZONE_SENT(zone[zoneNum].setting);
              CLEAR_ZONE_SENT_TAMPER(zone[zoneNum].setting);
              out_msg[toSend*2+1] = conf.reg[2+(REG_LEN*zoneNum)]; // Zone number as in cof.reg
              out_msg[toSend*2+2] = zone[zoneNum].lastEvent;
              toSend++;
            }
            if (zone[zoneNum].lastEvent != 'P') CLEAR_ZONE_SENT(zone[zoneNum].setting);
            zone[zoneNum].lastEvent = 'P';
            break;
          default: // Line is cut or short or tamper, no difference to alarm event
            if ((zone[zoneNum].lastEvent == 'T') && !GET_ZONE_SENT(zone[zoneNum].setting)) {
              SET_ZONE_SENT(zone[zoneNum].setting);
              SET_ZONE_SENT_TAMPER(zone[zoneNum].setting);
              out_msg[toSend*2+1] = conf.reg[2+(REG_LEN*zoneNum)]; // Zone number as in cof.reg
              out_msg[toSend*2+2] = zone[zoneNum].lastEvent;
              toSend++;
            }
            if (zone[zoneNum].lastEvent != 'T') CLEAR_ZONE_SENT(zone[zoneNum].setting);
            zone[zoneNum].lastEvent = 'T';
            break;
        }
      } // zone enabled
    } // for each alarm zone

    // Send zones if any
    if (toSend > 0) {    
      out_msg[0] = 'Z'; // Zone
      // Send to GW 
      resp = radio.sendWithRetry(GATEWAYID, out_msg, 1 + (toSend*2));
      /*
      // Print zone data being send
      Serial.print(F("Sent: ("));
      Serial.print(toSend);     
      Serial.print(F(") "));
      Serial.print(resp);     
      Serial.print(F(". Data: "));
      for (uint8_t ii=0; ii < toSend; ii++){
        Serial.print(out_msg[(ii*2)+1], DEC);
        Serial.print(F("="));
        Serial.write((char)out_msg[(ii*2)+2]);
        Serial.print(F(", "));
      }      
      Serial.println();
      */
    }
  }

  // Send alive packet every PING_DELAY
  if ((unsigned long)(millis() - aliveMillis) > PING_DELAY){
    aliveMillis = millis();
    sendPing();
  }

} // End main loop
