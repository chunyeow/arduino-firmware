/* Copyright (c) Chun-Yeow Yeoh, 2018. All rights reserved.             */
/* Author Name: Chun-Yeow Yeoh                                          */
/* Date: 6 June 2018                                                    */
/* Version: 1.0.0                                                       */

/* RTC Support */
#include <DS3232RTC.h>
#include <Streaming.h>
#include <Time.h>
#include <avr/sleep.h>
#include <EEPROM.h>

#define APN "\"\""
#define PLMN "\"\""
#define RECEIVE_CONTROL "0"
#define UDP_SERVER_IP ""
#define UDP_PORT ""

//#define DEBUG 1
const int NBPin = 32;

// SARA N201 NB-IoT module
const char NB_CHECK[]                   PROGMEM = "AT+CEREG?";      
const char NB_CHECK_RES[]               PROGMEM = "AT+CEREG:0,1";         
const char NB_IPADDR[]                  PROGMEM = "AT+CGPADDR=1";
const char NB_GIPADDR[]                 PROGMEM = "AT+CGPADDR:1,";
const char NB_CGDCONT[]                 PROGMEM = "AT+CGDCONT=1,\"IP\",";
const char NB_CGATT[]                   PROGMEM = "AT+CGATT=1";
const char NB_COPS[]                    PROGMEM = "AT+COPS=1,2,";
const char NB_CHECKS[]                  PROGMEM = "AT+CSCON?";
const char NB_CHECKS_RES[]              PROGMEM = "AT+CSCON:0,";
const char NB_REBOOT[]                  PROGMEM = "AT+NRB";
const char NB_ENABLE[]                  PROGMEM = "AT+CFUN=1";
const char NB_IMSI[]                    PROGMEM = "AT+CIMI";
const char NB_USOCK[]                   PROGMEM = "AT+NSOCR=DGRAM,17,";
const char NB_CSOCK[]                   PROGMEM = "AT+NSOCL=";
const char NB_SDATA[]                   PROGMEM = "AT+NSOST=";
const char NB_ISOCK_R[]                 PROGMEM = "AT+NSONMI:0,";
const char NB_RDATA[]                   PROGMEM = "AT+NSORF=";
const char NB_IMEI[]                    PROGMEM = "AT+CGSN=1";
const char NB_STATS[]                   PROGMEM = "AT+NUESTATS";

const char* const table_NBIOT[] PROGMEM =
{
    NB_CHECK,                       //0
    NB_CHECK_RES,                   //1
    NB_IPADDR,                      //2
    NB_GIPADDR,                     //3
    NB_CGDCONT,                     //4
    NB_CGATT,                       //5
    NB_COPS,                        //6  
    NB_CHECKS,                      //7
    NB_CHECKS_RES,                  //8
    NB_REBOOT,                      //9
    NB_ENABLE,                      //10
    NB_IMSI,                        //11
    NB_USOCK,                       //12
    NB_CSOCK,                       //13
    NB_SDATA,                       //14
    NB_ISOCK_R,                     //15
    NB_RDATA,                       //16
    NB_IMEI,                        //17
    NB_STATS,                       //18
};

char ATCOM[30];
char PARAMS[20];
char IMSI[20];
char IMEI[20];
char CONN_RESP[20];
char IPADDR[20];
char SNDCOM[600];
char RXDATA[600];
char RSRP[6];
char TXPWR[6];
char CELLID[10];
char DLMCS[6];
char ULMCS[6];
char DCIMCS[6];
int socket;
char nbiots[512] = "";

#define RESPONSE_OK "OK\r"
String response;

enum NB_STATES {
  NB_INIT,
  NB_IDLE,
  NB_ACTIVE,
  NB_SLEEP
};

enum NB_STATES nbstate;

/* Sensors Integration */
const int enable_sensor = 33;
const int LEDPin = 34;
const int ACPin = 35;
const int LDRsensorPin = 49;
const int TempsensorPin = 52;

/* RTC Clocking */
#define RTC_CLOCK_SET 1
#define SQW_PIN 4

/* EEPROM for storing the attempt for reconnection */
int RECONN_ADDR = 0;
byte reconn_store;

/* Use for interrupt by RTC */
volatile boolean alarmIsrWasCalled = false;

/* CoAP Support */
#define COAP_HEADER_SIZE 4
#define COAP_OPTION_HEADER_SIZE 1
#define COAP_PAYLOAD_MARKER 0xFF
#define MAX_OPTION_NUM 10
#define BUF_MAX_SIZE 300
#define COAP_DEFAULT_PORT 5683
#define RESPONSE_CODE(class, detail) ((class << 5) | (detail))
#define COAP_OPTION_DELTA(v, n) (v < 13 ? (*n = (0xFF & v)) : (v <= 0xFF + 13 ? (*n = 13) : (*n = 14)))

typedef enum {
  COAP_CON = 0,
  COAP_NONCON = 1,
  COAP_ACK = 2,
  COAP_RESET = 3
} COAP_TYPE;

typedef enum {
  COAP_GET = 1,
  COAP_POST = 2,
  COAP_PUT = 3,
  COAP_DELETE = 4
} COAP_METHOD;

typedef enum {
  COAP_IF_MATCH = 1,
  COAP_URI_HOST = 3,
  COAP_E_TAG = 4,
  COAP_IF_NONE_MATCH = 5,
  COAP_URI_PORT = 7,
  COAP_LOCATION_PATH = 8,
  COAP_URI_PATH = 11,
  COAP_CONTENT_FORMAT = 12,
  COAP_MAX_AGE = 14,
  COAP_URI_QUERY = 15,
  COAP_ACCEPT = 17,
  COAP_LOCATION_QUERY = 20,
  COAP_PROXY_URI = 35,
  COAP_PROXY_SCHEME = 39
} COAP_OPTION_NUMBER;

struct CoapOption {
  uint8_t number;
  uint8_t len;
  uint8_t *buf;
};

struct CoapPacket {
  uint8_t type;
  uint8_t code;
  char *token;
  uint8_t tokenlen;
  char *payload;
  int payloadlen;
  uint16_t messageid;
  uint8_t optionnum;
  CoapOption options[MAX_OPTION_NUM];
};

#ifdef RTC_CLOCK_SET
/* Time Setting */
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}
#endif

void printDateTime(time_t t)
{
  Serial << ((day(t)<10) ? "0" : "") << _DEC(day(t)) << ' ';
  Serial << monthShortStr(month(t)) << " " << _DEC(year(t)) << ' ';
  Serial << ((hour(t)<10) ? "0" : "") << _DEC(hour(t)) << ':';
  Serial << ((minute(t)<10) ? "0" : "") << _DEC(minute(t)) << ':';
  Serial << ((second(t)<10) ? "0" : "") << _DEC(second(t));
  Serial << " --> Current RTC time." << endl;
}

/*
 * Function for reading temperature and LDR value
 * May need to optimize and calibrate
 */

float get_temp_Voltage(int pin) {
  return (analogRead(pin) * 0.004882814);
}

float get_LDR_Voltage(int pin) {
  return (analogRead(pin));
}

void sleepNow() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(INT4, alarmIsr, LOW);
  sleep_mode();
}

void alarmIsr()
{
  alarmIsrWasCalled = true;
  sleep_disable();
  detachInterrupt(INT4);
}

uint16_t sendPacket(CoapPacket packet) {
  uint16_t running_delta = 0;
  uint16_t packetSize = 0;
  uint16_t i;
  char hexDummy[2];
  char buff[BUF_MAX_SIZE];
  char *p = buff;

  // define coap packet base header
  *p = 0x01 << 6;
  *p |= (packet.type & 0x03) << 4;
  *p++ |= (packet.tokenlen & 0x0F);
  *p++ = packet.code;
  *p++ = (packet.messageid >> 8);
  *p++ = (packet.messageid & 0xFF);
  p = buff + COAP_HEADER_SIZE;
  packetSize += 4;

  // define token
  if (packet.token != NULL && packet.tokenlen <= 0x0F) {
    memcpy(p, packet.token, packet.tokenlen);
    p += packet.tokenlen;
    packetSize += packet.tokenlen;
  }

  // define option header
  for (int i = 0; i < packet.optionnum; i++)  {
    uint32_t optdelta;
    uint8_t len, delta;

    if (packetSize + 5 + packet.options[i].len >= BUF_MAX_SIZE) {
      return 0;
    }
    optdelta = packet.options[i].number - running_delta;
    COAP_OPTION_DELTA(optdelta, &delta);
    COAP_OPTION_DELTA((uint32_t)packet.options[i].len, &len);

    *p++ = (0xFF & (delta << 4 | len));
    if (delta == 13) {
      *p++ = (optdelta - 13);
      packetSize++;
    } else if (delta == 14) {
      *p++ = ((optdelta - 269) >> 8);
      *p++ = (0xFF & (optdelta - 269));
      packetSize+=2;
    } if (len == 13) {
      *p++ = (packet.options[i].len - 13);
      packetSize++;
    } else if (len == 14) {
      *p++ = (packet.options[i].len >> 8);
      *p++ = (0xFF & (packet.options[i].len - 269));
      packetSize+=2;
    }

    memcpy(p, packet.options[i].buf, packet.options[i].len);
    p += packet.options[i].len;
    packetSize += packet.options[i].len + 1;
    running_delta = packet.options[i].number;
  }

  // define payload
  if (packet.payloadlen > 0) {
    if ((packetSize + 1 + packet.payloadlen) >= BUF_MAX_SIZE) {
       return 0;
    }
    *p++ = 0xFF;
    memcpy(p, packet.payload, packet.payloadlen);
    packetSize += 1 + packet.payloadlen;
  }

  memset(nbiots, '\0', 512);
  for (i=0; i<packetSize; i++) {
    sprintf(hexDummy, "%02X", (buff[i] & 0xFF));
    strcat(nbiots, hexDummy);
  }

  return packetSize;
}

uint16_t send(char *url1, char *url2, char *url3, char *url4, char *url5, COAP_TYPE type, COAP_METHOD method, char *token, uint8_t tokenlen, char *payload, int payloadlen) {
  // format and create the packet
  CoapPacket packet;
  int i=0;

  packet.type = type;
  packet.code = method;
  packet.token = token;
  packet.tokenlen = tokenlen;
  packet.payload = payload;
  packet.payloadlen = payloadlen;
  packet.optionnum = 0;
  packet.messageid = rand();
    
  if (i==0 && url1 != NULL) {
    packet.options[packet.optionnum].buf = (uint8_t *)url1;
    packet.options[packet.optionnum].len = strlen(url1);
    packet.options[packet.optionnum].number = COAP_URI_PATH;
    packet.optionnum++;
    i++;
  }

  if (i==1 && url2 != NULL) {
    packet.options[packet.optionnum].buf = (uint8_t *)url2;
    packet.options[packet.optionnum].len = strlen(url2);
    packet.options[packet.optionnum].number = COAP_URI_PATH;
    packet.optionnum++;
    i++;
  }

  if (i==2 && url3 != NULL) {
    packet.options[packet.optionnum].buf = (uint8_t *)url3;
    packet.options[packet.optionnum].len = strlen(url3);
    packet.options[packet.optionnum].number = COAP_URI_PATH;
    packet.optionnum++;
    i++;
  }

  if (i==3 && url4 != NULL) {
    packet.options[packet.optionnum].buf = (uint8_t *)url4;
    packet.options[packet.optionnum].len = strlen(url4);
    packet.options[packet.optionnum].number = COAP_URI_PATH;
    packet.optionnum++;
    i++;
  }

  if (i==4 && url5 != NULL) {
    packet.options[packet.optionnum].buf = (uint8_t *)url5;
    packet.options[packet.optionnum].len = strlen(url5);
    packet.options[packet.optionnum].number = COAP_URI_QUERY;
    packet.optionnum++;
    i++;
  }

  // send packet
  return sendPacket(packet);
}

void software_reset() {
  asm volatile ("  jmp 0");
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int8_t sendATcommand(char* ATcommand, String expected_answer, unsigned int timeout) {
    uint8_t answer=0, x=0;
    unsigned long previous;
    
    delay(100); // Delay to be sure no passed commands interfere
    
    while( Serial1.available() > 0) Serial1.read();    // Wait for clean input buffer

#ifdef DEBUG
    Serial.print("[NB DEBUG] ");
    Serial.println(ATcommand);
#endif
    Serial1.println(ATcommand);    // Send the AT command 

    previous = millis();

    // this loop waits for the answer
    do{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(Serial1.available() != 0){    
            //*(response++) = Serial1.read();
            response = "";
            response = Serial1.readStringUntil('\n');
            // check if the desired answer is in the response of the module
            if (response == expected_answer){
                answer = 1;
            } else {
                // Additional routing to store global varibale data
                if (strstr(ATcommand, "AT+NUESTAT") != 0) {
                  x++;
                  if (x == 2) {
                      String rsrp = getValue(response, ':', 1);
                      rsrp.trim();
                      rsrp.toCharArray(RSRP, 6);
                  }
                  if (x == 4) {
                      String tpwr = getValue(response, ':', 1);
                      tpwr.trim();
                      tpwr.toCharArray(TXPWR, 6);
                  }
                  if (x == 7) {
                      String cellid = getValue(response, ':', 1);
                      cellid.trim();
                      cellid.toCharArray(CELLID, 10);
                  }
                  if (x == 8) {
                      String dlmcs = getValue(response, ':', 1);
                      dlmcs.trim();
                      dlmcs.toCharArray(DLMCS, 6);
                  }
                  if (x == 9) {
                      String ulmcs = getValue(response, ':', 1);
                      ulmcs.trim();
                      ulmcs.toCharArray(ULMCS, 6);
                  }                   
                  if (x == 10) {
                      String dcimcs = getValue(response, ':', 1);
                      dcimcs.trim();
                      dcimcs.toCharArray(DCIMCS, 6);
                  }               
                } 
                if (strstr(ATcommand, "AT+CIMI") != 0) {
                  x++;
                  if (x == 2) {
                    response.trim();
                    response.toCharArray(IMSI, 20);
                  }
                } 
                if (strstr(ATcommand, "AT+CGSN=1") != 0) {
                  x++;
                  if (x == 2) {
                    response.remove(0, 6);
                    response.trim();
                    response.toCharArray(IMEI, 20);
                  }
                } 
                if (strstr(ATcommand, "AT+CSCON?") != 0) {
                  x++;
                  if (x == 2) {
                    response.trim();
                    response.toCharArray(CONN_RESP, 20);
                  }
                }
                if (strstr(ATcommand, "AT+CGPADDR=1") != 0) {
                  x++;
                  if (x == 2) {
                    response.remove(0, 11);
                    response.trim();
                    response.toCharArray(IPADDR, 20);
                  }
                }
                if (strstr(ATcommand, "AT+NSOCR=") != 0) {
                  socket = response.toInt();
                }
                if (strstr(ATcommand, "AT+NSORF=") != 0) {
                  x++;
                  if (x == 2) {
                    String remoteIPAddr = getValue(response, ',', 1);
                    String remotePORT = getValue(response, ',', 2);
                    String rlength = getValue(response, ',', 3);
                    String rdata = getValue(response, ',', 4);
                    String relength = getValue(response, ',', 5);
#ifdef DEBUG
                    Serial.println("[NB DEBUG] Rx Data");
                    Serial.print("Remote IP Address: ");
                    Serial.println(remoteIPAddr);
                    Serial.print("Remote Port Number: ");
                    Serial.println(remotePORT);
                    Serial.print("Length: ");
                    Serial.println(rlength);
                    Serial.print("Rx Data: ");
                    Serial.println(rdata);
                    Serial.print("Remaining Length: ");
                    Serial.println(relength);
#endif
                    int rxlen = rlength.toInt();
                    rxlen = rxlen * 2;
                    rdata.trim();
                    rdata.toCharArray(RXDATA, rxlen + 1);
                  }
                }
            }
        }
    // Waits for the answer with time out
    } while((answer == 0) && ((millis() - previous) < timeout));
    
    return answer;
}

void setup() {
  uint8_t ans, i=0, exit=0;

  /* Enable NB-IoT and LED */
  pinMode(NBPin, OUTPUT);
  digitalWrite(NBPin, HIGH);

  /* Sensors Setup */
  pinMode(enable_sensor, OUTPUT);
  digitalWrite(enable_sensor, HIGH);
  pinMode(LEDPin, OUTPUT);
  pinMode(ACPin, INPUT);

  /* NB-IoT Setup */
  nbstate = NB_INIT;
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  /* Read the previous number of attempts on UE re-attach */
  reconn_store = EEPROM.read(RECONN_ADDR);
  Serial.print("[NB] Number of Signal Connection Lost: ");
  Serial.println(reconn_store, DEC);

  if (reconn_store == 0xFF)
    EEPROM.write(RECONN_ADDR, 0);

#ifdef RTC_CLOCK_SET
  /* RTC Setting - DS3231 - Configure Time*/
  if (getDate(__DATE__) && getTime(__TIME__)) {
    RTC.write(tm);
  }
#endif

  /* setSyncProvider() causes the Time library to synchronize with external RTC */
  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet){
    Serial.println("[NB] Setup RTC failed");
  }

  printDateTime( RTC.get() );

  /* Disable the default square wave of the SQW pin. */
  RTC.squareWave(SQWAVE_NONE);

  /* Attach an interrupt on the falling of the SQW pin. */
  pinMode(SQW_PIN, INPUT_PULLUP);
  digitalWrite(SQW_PIN, HIGH);
  attachInterrupt(INT4, alarmIsr, LOW); /* Set LOW not FALLING */

  /* Every 30 seconds of every minutes - become 1 minute (further investigation needed) */
  RTC.setAlarm(ALM1_MATCH_SECONDS, 30, 0, 0, 1);
  RTC.alarm(ALARM_1); /* Clear RTC interrupt flag */
  RTC.alarmInterrupt(ALARM_1, true);

  /* Set an alarm for every minute. */
  //RTC.setAlarm(ALM2_MATCH_MINUTES, 0, 5, 0, 1);
  //RTC.alarm(ALARM_2);
  //RTC.alarmInterrupt(ALARM_2, true);

  // Do this once, Reboot the node
  Serial.println("[NB] Reboot NB-IoT module");
  strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[9])));
  ans = sendATcommand(ATCOM, RESPONSE_OK, 10000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Failed to reboot NB-IoT module");
    exit = 1;
  }
 
  // Turn on the NB-IoT Radio
  Serial.println("[NB] Turn on NB-IoT module");
  strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[10])));
  ans = sendATcommand(ATCOM, RESPONSE_OK, 10000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Failed to turn on NB-IoT module");
    exit = 1;
  }

  // Get the IMSI
  Serial.println("[NB] Get the IMSI of NB-IoT module");
  strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[11])));
  ans = sendATcommand(ATCOM, RESPONSE_OK, 5000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Failed to get IMSI from NB-IoT module");
    exit = 1;
  }
  Serial.print("[NB] IMSI for NB-IoT module: ");
  Serial.println(IMSI);

  // Get the IMEI
  Serial.println("[NB] Get the IMEI of NB-IoT module");
  strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[17])));
  ans = sendATcommand(ATCOM, RESPONSE_OK, 5000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Failed to get IMEI from NB-IoT module");
    exit = 1;
  }
  Serial.print("[NB] IMEI for NB-IoT module: ");
  Serial.println(IMEI);

  // Trigger Network Attach Procedure
  Serial.println("[NB] Set PDN Connection for NB-IoT module");
  strcpy_P(PARAMS, (char*)pgm_read_word(&(table_NBIOT[4])));
  sprintf(ATCOM,"%s%s", PARAMS, APN);
  ans = sendATcommand(ATCOM, RESPONSE_OK, 5000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Failed to set PDN connection");
    exit = 1;
  }

  strcpy_P(PARAMS, (char*)pgm_read_word(&(table_NBIOT[6])));
  sprintf(ATCOM,"%s%s", PARAMS, PLMN);
  ans = sendATcommand(ATCOM, RESPONSE_OK, 5000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Failed to set PLMN");
    exit = 1;
  }

  // Check signaling is up
  i = 0;
  do {
    i++;
    memset(CONN_RESP, '\0', 20);
    strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[7])));
    ans = sendATcommand(ATCOM, RESPONSE_OK, 10000);
    if (ans == 0) {
      Serial.println("[NB ERROR] Check connection status failed");
      exit = 1;
    }
    //Serial.println(CONN_RESP);
    delay(1000);
    if (i > 60) {
      Serial.println("[NB] Software reset");
      software_reset();
    }
  } while (strstr(CONN_RESP, "+CSCON:0,1") == 0);
  Serial.println("[NB] Connected to NB-IoT network");
  
  // Check IPv4 address is allocated
  do {
    memset(IPADDR, '\0', 20);
    strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[2])));
    ans = sendATcommand(ATCOM, RESPONSE_OK, 10000);
    if (ans == 0) {
      Serial.println("[NB ERROR] Check connection status failed");
      exit = 1;
    }
  } while (strstr(IPADDR, "."/*"\r"*/) == 0);
  Serial.print("[NB] IPv4 Address: ");
  Serial.println(IPADDR);

  if (exit == 1)
     Serial.println("[NB ERROR] Problem on sending AT command");

  nbstate = NB_IDLE;
 
  delay(5000);
}

void loop() {
  uint8_t ans, exit=0;
  int txlen, packetlen, ac_power;
  char num[10] = "";
  char nbiot_str[512] = "";
  char t[10] = "", l[10] = "";
  
  Serial.println("[NB] NB-IoT Application Loop");

  // Setup UDP connection
  if (nbstate == NB_IDLE) {  
    Serial.println("[NB] Create UDP Socket");
    strcpy_P(PARAMS, (char*)pgm_read_word(&(table_NBIOT[12])));
    sprintf(ATCOM,"%s%s,%s", PARAMS, UDP_PORT, RECEIVE_CONTROL);
    ans = sendATcommand(ATCOM, RESPONSE_OK, 1000);
    if (ans == 0) {
      Serial.println("[NB ERROR] Failed to create UDP socket");
      exit = 1;
    }
    Serial.print("[NB] Socket Number: ");
    Serial.println(socket);
    nbstate = NB_ACTIVE;
  }

  if (nbstate == NB_ACTIVE) {
    delay(1000);
    Serial.println("[NB] Sensor Reading");
    digitalWrite(enable_sensor, HIGH);
    digitalWrite(LEDPin, HIGH);

    float temp_voltage, degreesC, degreesF;
    float LDR_voltage,luminanceLX;

    temp_voltage = get_temp_Voltage(TempsensorPin);
    degreesC = ((temp_voltage - 0.5) * 100.0)/2.7;
    degreesF = degreesC * (9.0/5.0) + 32.0;

    LDR_voltage = get_LDR_Voltage(LDRsensorPin);
    luminanceLX = (float(LDR_voltage)*100/1023);

    Serial.print("[NB] Temparature: ");
    Serial.println(degreesC);
    Serial.print("[NB] LDR: ");
    Serial.println(luminanceLX);
    delay(1000);

    strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[18])));
    ans = sendATcommand(ATCOM, RESPONSE_OK, 1000);
    if (ans == 0) {
      Serial.println("[NB ERROR] Failed to check UE stats");
      exit = 1;
    }

#ifdef DEBUG
    Serial.print("[NB] CELL ID: ");
    Serial.println(CELLID);
    Serial.print("[NB] RSRP: ");
    Serial.println(RSRP);
    Serial.print("[NB] TX PWR: ");
    Serial.println(TXPWR);
    Serial.print("[NB] DL MCS: ");
    Serial.println(DLMCS);
    Serial.print("[NB] UL MCS: ");
    Serial.println(ULMCS);
    Serial.print("[NB] DCI MCS: ");
    Serial.println(DCIMCS); 
#endif

    Serial.println("[NB] NB-IoT Tx/Rx is activated");

    /* Building COAP Packet
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |Ver| T |  OC   |      Code     |        Transaction ID         |
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |   Options (if any) ...
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    |   Payload (if any) ...
    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    Version: 1 (01)
    Type: Confirmable (00)
    Token Length: 4 (0100)
    Code: POST (00000010)
    0x44 0x02
    Transaction ID: rand()
    0xXX 0xXX
    Token:
    0xXX 0xXX 0xXX 0xXX
    */

    ac_power = digitalRead(ACPin);
    digitalWrite(LEDPin, ac_power);  

    dtostrf(degreesC, 4, 2, t);
    dtostrf(luminanceLX, 4, 2, l);
    sprintf(nbiot_str, "{\"dev_id\":\"%s\", \"imsi\":\"%s\", \"cell_id\":\"%s\", \"rsrp\":\"%s\", \"ac_power\":%d, \"temp\":\"%s\", \"ldr\":\"%s\"}%c\0\r\n", IMEI, IMSI, CELLID, RSRP, ac_power, t, l, 0x0A);
    sprintf(num, "%d", strlen(nbiot_str));
    txlen = atoi(num);
    // dhs4MkruPzjZkSGigFfo point 1
    // LUOtcEZTF5QjTNbzbOBf point 9
    packetlen = send("api", "v1", "dhs4MkruPzjZkSGigFfo", "telemetry", NULL, COAP_NONCON, COAP_POST, NULL, 0, nbiot_str, txlen);
      
    Serial.println("[NB] Tx UL Data");
    memset(SNDCOM, '\0', 600);
    strcpy_P(PARAMS, (char*)pgm_read_word(&(table_NBIOT[14])));
    sprintf(SNDCOM,"%s%d,%s,%s,%d,%s", PARAMS, socket, UDP_SERVER_IP, UDP_PORT, packetlen, nbiots);
    ans = sendATcommand(SNDCOM, RESPONSE_OK, 1000);
    if (ans == 0) {
      Serial.println("[NB ERROR] Failed to Tx Data");
      exit = 1;
    }

    nbstate = NB_SLEEP;
  }

  if (nbstate == NB_SLEEP) {
    Serial.println("[NB] Close UDP Socket");
    strcpy_P(PARAMS, (char*)pgm_read_word(&(table_NBIOT[13])));
    sprintf(ATCOM,"%s%d", PARAMS, socket);
    ans = sendATcommand(ATCOM, RESPONSE_OK, 1000);
    if (ans == 0) {
      Serial.println("[NB ERROR] Failed to close UDP socket");
      exit = 1;
    }
    
    nbstate = NB_IDLE;
  }   

  delay(1000);
  
  memset(CONN_RESP, '\0', 20);
  strcpy_P(ATCOM, (char*)pgm_read_word(&(table_NBIOT[7])));
  ans = sendATcommand(ATCOM, RESPONSE_OK, 10000);
  if (ans == 0) {
    Serial.println("[NB ERROR] Check connection status failed");
    exit = 1;
  }

  /* Something wrong with AT command, just do a software restart */
  if (exit == 1)
     software_reset();

  if (strstr(CONN_RESP, "+CSCON:0,1") == 0) {
    reconn_store++;
    EEPROM.write(RECONN_ADDR, reconn_store);
    Serial.println("[NB] Not connected to NB-IoT cell");
    software_reset();
  } else {
    Serial.println("[NB] Connected to NB-IoT cell");
  }

  if (alarmIsrWasCalled){
    if (RTC.alarm(ALARM_1)) {
      Serial.println("[NB] Sleep");
      RTC.read(tm);
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, (tm.Minute + 5) % 60, 0, 0);
    }
    alarmIsrWasCalled = false;
  }

  /* Sleep for a while */
  delay(100);
  sleepNow();
  delay(100);
}
