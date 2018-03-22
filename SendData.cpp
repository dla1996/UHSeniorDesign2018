#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <mbed.h>
#include "dot_util.h"
#include "RadioEvent.h"
#include "itoa.h"

#define BUFFER_SIZE 10

//note: added GPS functions , variables below (will organize better later ski
//and gps code in main is noted



///-----------------FOR GPS------BELOW----------------------------------
//

#include "MBed_Adafruit_GPS.h"


Serial * gps_Serial = new Serial(PA_2,PA_3);
//Initalize using pa2 and pa3 (D0 and D1 on the mdot) (D0->TX on gps) (D1->RX on gps)
//gps_Serial = new Serial(PA_2,PA_3); //serial object for use w/ GPS USING PA_2 and PA_3
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
const int refresh_Time = 2000; //refresh time in ms


 
// how long are max NMEA lines to parse?
#define MAXLINELENGTH 120
 
// we double buffer: read one line in and leave one for the main program
volatile char line1[MAXLINELENGTH];
volatile char line2[MAXLINELENGTH];
// our index into filling the current line
volatile uint16_t lineidx=0;
// pointers to the double buffers
volatile char *currentline;
volatile char *lastline;
volatile bool recvdflag;
volatile bool inStandbyMode;
 
 
 
 
 bool Adafruit_GPS::parse(char *nmea) {
  // do checksum check
 
  // first look if we even have one
  if (nmea[strlen(nmea)-4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
    sum += parseHex(nmea[strlen(nmea)-2]);
    
    // check checksum 
    for (uint8_t i=1; i < (strlen(nmea)-4); i++) {
      sum ^= nmea[i];
    }
    if (sum != 0) {
      // bad checksum :(
      //return false;
    }
  }
 
  // look for a few common sentences
  if (strstr(nmea, "$GPGGA")) {
    // found GGA
    char *p = nmea;
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
 
    milliseconds = fmod((double) timef, 1.0) * 1000;
 
    // parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);
 
    p = strchr(p, ',')+1;
    if (p[0] == 'N') lat = 'N';
    else if (p[0] == 'S') lat = 'S';
    else if (p[0] == ',') lat = 0;
    else return false;
 
    // parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
 
    p = strchr(p, ',')+1;
    if (p[0] == 'W') lon = 'W';
    else if (p[0] == 'E') lon = 'E';
    else if (p[0] == ',') lon = 0;
    else return false;
 
    p = strchr(p, ',')+1;
    fixquality = atoi(p);
 
    p = strchr(p, ',')+1;
    satellites = atoi(p);
 
    p = strchr(p, ',')+1;
    HDOP = atof(p);
 
    p = strchr(p, ',')+1;
    altitude = atof(p);
    p = strchr(p, ',')+1;
    p = strchr(p, ',')+1;
    geoidheight = atof(p);
    return true;
  }
  if (strstr(nmea, "$GPRMC")) {
   // found RMC
    char *p = nmea;
 
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
 
    milliseconds = fmod((double) timef, 1.0) * 1000;
 
    p = strchr(p, ',')+1;
    // Serial.println(p);
    if (p[0] == 'A') 
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else
      return false;
 
    // parse out latitude
    p = strchr(p, ',')+1;
    latitude = atof(p);
 
    p = strchr(p, ',')+1;
    if (p[0] == 'N') lat = 'N';
    else if (p[0] == 'S') lat = 'S';
    else if (p[0] == ',') lat = 0;
    else return false;
 
    // parse out longitude
    p = strchr(p, ',')+1;
    longitude = atof(p);
 
    p = strchr(p, ',')+1;
    if (p[0] == 'W') lon = 'W';
    else if (p[0] == 'E') lon = 'E';
    else if (p[0] == ',') lon = 0;
    else return false;
 
    // speed
    p = strchr(p, ',')+1;
    speed = atof(p);
 
    // angle
    p = strchr(p, ',')+1;
    angle = atof(p);
 
    p = strchr(p, ',')+1;
    uint32_t fulldate = atof(p);
    day = fulldate / 10000;
    month = (fulldate % 10000) / 100;
    year = (fulldate % 100);
 
    // we dont parse the remaining, yet!
    return true;
  }
 
  return false;
}
 
char Adafruit_GPS::read(void) {
  char c = 0;
  
  if (paused) return c;
 
    if(!gpsSerial->readable()) return c;
    c = gpsSerial->getc();
 
  //Serial.print(c);
 
  if (c == '$') {
    currentline[lineidx] = 0;
    lineidx = 0;
  }
  if (c == '\n') {
    currentline[lineidx] = 0;
 
    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }
 
    lineidx = 0;
    recvdflag = true;
  }
 
  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH)
    lineidx = MAXLINELENGTH-1;
 
  return c;
}
 
Adafruit_GPS::Adafruit_GPS (Serial *ser)
{
  common_init();     // Set everything to common state, then...
  gpsSerial = ser; // ...override gpsSwSerial with value passed.
}
 
// Initialization code used by all constructor types
void Adafruit_GPS::common_init(void) {
  gpsSerial = NULL;
  recvdflag   = false;
  paused      = false;
  lineidx     = 0;
  currentline = line1;
  lastline    = line2;
 
  hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
  lat = lon = mag = 0; // char
  fix = false; // bool
  milliseconds = 0; // uint16_t
  latitude = longitude = geoidheight = altitude =
    speed = angle = magvariation = HDOP = 0.0; // float
}
 
void Adafruit_GPS::begin(int baud)
{
  gpsSerial->baud(baud);
  wait_ms(10);
}
 
void Adafruit_GPS::sendCommand(char *str) {
  gpsSerial->printf("%s",str);
}
 
bool Adafruit_GPS::newNMEAreceived(void) {
  return recvdflag;
}
 
void Adafruit_GPS::pause(bool p) {
  paused = p;
}
 
char *Adafruit_GPS::lastNMEA(void) {
  recvdflag = false;
  return (char *)lastline;
}
 
// read a Hex value and return the decimal equivalent
uint8_t Adafruit_GPS::parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
}
 
bool Adafruit_GPS::waitForSentence(char *wait4me, uint8_t max) {
  char str[20];
 
  uint8_t i=0;
  while (i < max) {
    if (newNMEAreceived()) { 
      char *nmea = lastNMEA();
      strncpy(str, nmea, 20);
      str[19] = 0;
      i++;
 
      if (strstr(str, wait4me))
    return true;
    }
  }
 
  return false;
}
 
bool Adafruit_GPS::LOCUS_StartLogger(void) {
  sendCommand(PMTK_LOCUS_STARTLOG);
  recvdflag = false;
  return waitForSentence(PMTK_LOCUS_LOGSTARTED);
}
 
bool Adafruit_GPS::LOCUS_ReadStatus(void) {
  sendCommand(PMTK_LOCUS_QUERY_STATUS);
  
  if (! waitForSentence("$PMTKLOG"))
    return false;
 
  char *response = lastNMEA();
  uint16_t parsed[10];
  uint8_t i;
  
  for (i=0; i<10; i++) parsed[i] = -1;
  
  response = strchr(response, ',');
  for (i=0; i<10; i++) {
    if (!response || (response[0] == 0) || (response[0] == '*')) 
      break;
    response++;
    parsed[i]=0;
    while ((response[0] != ',') && 
       (response[0] != '*') && (response[0] != 0)) {
      parsed[i] *= 10;
      char c = response[0];
      if (isdigit(c))
        parsed[i] += c - '0';
      else
        parsed[i] = c;
      response++;
    }
  }
  LOCUS_serial = parsed[0];
  LOCUS_type = parsed[1];
  if (isalpha(parsed[2])) {
    parsed[2] = parsed[2] - 'a' + 10; 
  }
  LOCUS_mode = parsed[2];
  LOCUS_config = parsed[3];
  LOCUS_interval = parsed[4];
  LOCUS_distance = parsed[5];
  LOCUS_speed = parsed[6];
  LOCUS_status = !parsed[7];
  LOCUS_records = parsed[8];
  LOCUS_percent = parsed[9];
 
  return true;
}
 
// Standby Mode Switches
bool Adafruit_GPS::standby(void) {
  if (inStandbyMode) {
    return false;  // Returns false if already in standby mode, so that you do not wake it up by sending commands to GPS
  }
  else {
    inStandbyMode = true;
    sendCommand(PMTK_STANDBY);
    //return waitForSentence(PMTK_STANDBY_SUCCESS);  // don't seem to be fast enough to catch the message, or something else just is not working
    return true;
  }
}
 
bool Adafruit_GPS::wakeup(void) {
  if (inStandbyMode) {
   inStandbyMode = false;
    sendCommand("");  // send byte to wake it up
    return waitForSentence(PMTK_AWAKE);
  }
  else {
      return false;  // Returns false if not in standby mode, nothing to wakeup
  }
}
 
 
 
 
 /////FOR GPS-----------------ABOVE-------------------------------------













 
/////////////////////////////////////////////////////////////////////////////
// -------------------- DOT LIBRARY REQUIRED ------------------------------//
// * Because these example programs can be used for both mDot and xDot     //
//     devices, the LoRa stack is not included. The libmDot library should //
//     be imported if building for mDot devices. The libxDot library       //
//     should be imported if building for xDot devices.                    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libmDot-mbed5/        //
// * https://developer.mbed.org/teams/MultiTech/code/libxDot-dev-mbed5/    //
// * https://developer.mbed.org/teams/MultiTech/code/libxDot-mbed5/        //
/////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// * these options must match between the two devices in   //
//   order for communication to be successful
//-------------------MDOT variables------------------------//
/////////////////////////////////////////////////////////////
static uint8_t network_address[] = { 0x00, 0x11, 0x22, 0x33 };
static uint8_t network_session_key[] = { 0x00, 0x11, 0x22, 0x33, 0x00, 0x11, 0x22, 0x33, 0x00, 0x11, 0x22, 0x33, 0x00, 0x11, 0x22, 0x33 };
static uint8_t data_session_key[] = { 0x33, 0x22, 0x11, 0x00, 0x33, 0x22, 0x11, 0x00, 0x33, 0x22, 0x11, 0x00, 0x33, 0x22, 0x11, 0x00 };

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;
//--------------End of MDOT variables-------------------//

Serial pc(USBTX, USBRX);
Ticker Periodic;

// ADXL372 Slave I2C
I2C ADXL372(I2C_SDA, I2C_SCL);  // (D14,D15) (MISO, CS)

// ADT7410 Temperature
I2C ADT7410(I2C_SDA, I2C_SCL);      // Attempt at making I2C connection to slaves (D14,D15)
InterruptIn ADT7410_Int(D2);    // Allow this pin for ADT7410 Interrupt critical temperature notice

// DS7505s Temperature
I2C DS7505(I2C_SDA, I2C_SCL);      // Attempt at making I2C connection to slaves (D14,D15)

// Create reocurring interrupt function that could be used to periodically take temperatures
// Not working right now due to some mutex initialize error
// Suspect that it is due to it having be a RTOS task thing
// Should probably go back to using an in processor timer interrupt instead of mbed

const int ADT7410_Address_7BIT = 0x49;  // A0 set HIGH and A1 set LOW
const int ADT7410_Address_8BIT = ADT7410_Address_7BIT << 1; // Shift 1 bit to left for R/~W bit, and basic I2C format

const int ADXL372_Address_7bit = 0x1D;      // Address for the I2C if MISO pulled low, 0x53 if pulled high
const int ADXL372_Address_8bit = ADXL372_Address_7bit << 1; // Same

const int DS7505s_Address_7bit = 0x48;  // A0 set LOR, A1 set LOW, A2 set LOW
const int DS7505s_Address_8bit = DS7505s_Address_7bit << 1; // Same


int regAddress; // Used by all sensors

/*
 *  Variables used for ADT7410 Temperature
 */
// Points to the returned char pointer from called functions
char * rawTempValues;           // Could change to uint8_t, same for other char pointers
uint16_t convertedTempValue;    // Data values must be uint16_t for conversion and send prep
uint16_t temperatureBuffer[BUFFER_SIZE];


/*
 *  Variables used for mDot
 */


uint32_t tx_frequency;
uint8_t tx_datarate;
uint8_t tx_power;
uint8_t frequency_band;



/*
 *  Variables used for ADXL372 Accelerometer
 */
char *accelValues;
char Xmsb;  // Gets most significant byte
char Xlsb;  // Gets least significant byte
char Ymsb;
char Ylsb;
char Zmsb;
char Zlsb;
uint16_t XData;
uint16_t YData;
uint16_t ZData;

uint16_t XDataInterrupt[BUFFER_SIZE];
uint16_t YDataInterrupt[BUFFER_SIZE];
uint16_t ZDataInterrupt[BUFFER_SIZE];



/*
 *  Variables used for interrupt triggers
 */
bool takeTemperature = false;   // Trigger temperature reading
bool takeAccelerometer = false; // Trigger accelerometer reading
bool periodicReadingTrigger = false;  // Trigger reading both accelerometer and temperature

/*
 *  Prototype functions
 */
 
char twosComplementConversion(char value);

void ADXL372Initialize(void);
void ADXL372Reset(void);
void I2CSelfTest(void);
void accelerometerI2CWrite(int hexAddress, int hexData);
char * accelerometerI2CRead(int hexAddress);

void ADT7410Initialize(void);
void ADT7410Write(unsigned char registerAddress, unsigned char data);
char * ADT7410Read(int hex);


/*
 *  Interrupt functions
 */
void CriticalTemperatureInterrupt(void){   
    takeTemperature = true; // Take temperature because something happened
}

void CriticalAccelerometerInterrupt(void){
    takeAccelerometer = true;   // Take accelerometer because something happened
}

void takePeriodicReading(std::vector<uint8_t> tx_data){
        pc.printf("Regular periodic reading \n\r");
        /*
         *  Taking accelerometer data
         */
        regAddress = 0x08;  // This is the register address for XData
        accelValues = accelerometerI2CRead(regAddress);
        Xmsb = *(accelValues + 0);
        Xlsb = *(accelValues + 1);
        Ymsb = *(accelValues + 2);
        Ylsb = *(accelValues + 3);
        Zmsb = *(accelValues + 4);
        Zlsb = *(accelValues + 5);
        
        XData = (Xmsb << 8 | Xlsb) >> 4;  // Combine two bytes into short int, remove last 4 flag bits
        YData = (Ymsb << 8 | Ylsb) >> 4;
        ZData = (Zmsb << 8 | Zlsb) >> 4;
        
        XData = twosComplementConversion(XData);
        YData = twosComplementConversion(YData);
        ZData = twosComplementConversion(ZData);
        
        /*
         *  Taking temperature data
         */
        regAddress = 0x00;
        rawTempValues = ADT7410Read(regAddress);
        convertedTempValue = ((*(rawTempValues + 0) << 8) | *(rawTempValues + 1)) >> 3; // Combine the two bytes into 
                                                                                        // a short int variable(16 bits), remove last 3 bits
                                                                                        
        
        c = myGPS.read();   //queries the GPS
          
       // if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
         //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
            
            }    
        }
        
         //check if enough time has passed to warrant printing GPS info to screen
        //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
        if (refresh_Timer.read_ms() >= refresh_Time) {
            refresh_Timer.reset();
        
        /* pc.printf("Time: %d:%d:%d.%u\n", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);   
            pc.printf("Date: %d/%d/20%d\n", myGPS.day, myGPS.month, myGPS.year);
            pc.printf("Fix: %d\n", (int) myGPS.fix);
            pc.printf("Quality: %d\n", (int) myGPS.fixquality);*/
         if (myGPS.fix) {
                pc.printf("\r");
                pc.printf("GPS log:\n");
                pc.printf("\r");
                pc.printf("Time(GMT): %d:%d:%d.%u\n", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);   
                pc.printf("\r");
                pc.printf("Date: %d/%d/20%d\n", myGPS.day, myGPS.month, myGPS.year);
                pc.printf("\r");
                pc.printf("Fix: %d\n", (int) myGPS.fix);
                pc.printf("\r");
                pc.printf("Quality: %d\n", (int) myGPS.fixquality);
                pc.printf("\r");
                pc.printf("Location: %5.2f%c, %5.2f%c\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
                pc.printf("\r");
                pc.printf("Speed: %5.2f knots\n", myGPS.speed);
                pc.printf("\r");
                pc.printf("Angle: %5.2f\n", myGPS.angle);
                pc.printf("\r");
                pc.printf("Altitude: %5.2f\n", myGPS.altitude);
                pc.printf("\r");
                pc.printf("Satellites: %d\n", myGPS.satellites);
                pc.printf("\r");
            }
        }
                                                                                        
        pc.printf("Accelerometer::: ");
        pc.printf("X: 0x%x | Y: 0x%x | Z: 0x%x\n\r", XData, YData, ZData);
        pc.printf("Temperature::: ");
        pc.printf("Celsius: 0x%x\n\r", convertedTempValue);
        
        
        // Push 1 for temperature
		tx_data.push_back(1);
        tx_data.push_back((convertedTempValue >> 8) & 0xFF);
        tx_data.push_back(convertedTempValue & 0xFF);
		// Push 2 for AccelerometerX
		tx_data.push_back(2);
        tx_data.push_back((XData >> 8) & 0xFF);
        tx_data.push_back(XData & 0xFF);
		// Push 3 for AccelerometerY
		tx_data.push_back(3);
        tx_data.push_back((YData >> 8) & 0xFF);
        tx_data.push_back(YData & 0xFF);
		// Push 4 for AccelerometerZ
		tx_data.push_back(4);
        tx_data.push_back((ZData >> 8) & 0xFF);
        tx_data.push_back(ZData & 0xFF);
        logInfo("Temperautre: %lu [0x%04X]", convertedTempValue, convertedTempValue);
        send_data(tx_data);
        periodicReadingTrigger = false; // Flip back to no trigger
}

void takePeriodicReadingTicker(void){
        periodicReadingTrigger = true;
}

////////////////////////////////////////////////////////////////////////////////
/*                              _     
               ____ ___  ____ _(_)___ 
              / __ `__ \/ __ `/ / __ \
             / / / / / / /_/ / / / / /
            /_/ /_/ /_/\__,_/_/_/ /_/ 
                          
 *//////////////////////////////////////////////////////////////////////////////
int main(void)
{
    // Custom event handler for automatically displaying RX data
    //interruptEverything.attach(&interruptReadTemperature, 7.0);
    RadioEvent events;
    // Change baud rate in serial terminal to this value
    pc.baud(115200);
    ADXL372.frequency(300000);  // I2C devices are connected to the same clock
    ADT7410.frequency(300000);  // Redundant but whatever
    ADT7410_Int.rise(&CriticalTemperatureInterrupt);
    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
    
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    
    
    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);
    
    // Sometimes when calling this, it creates error: type specifier expected
    // Even with identical include files I would get this in another workspace.
    plan = new lora::ChannelPlan_US915();

    logInfo("Now asserting");
    assert(plan);

    // Careful when using this. The production ready libmdot-mbed5 has a void constructor
    // Therefore, can only use the libmDot-dev-mbed5 version, for now.
    dot = mDot::getInstance(plan);
    assert(dot);

    logInfo("mbed-os library version: %d", MBED_LIBRARY_VERSION);

    // start from a well-known state
    logInfo("defaulting Dot configuration");
    dot->resetConfig();

    // make sure library logging is turned on
    dot->setLogLevel(mts::MTSLog::INFO_LEVEL);

    // attach the custom events handler
    dot->setEvents(&events);

    // update configuration if necessary
    if (dot->getJoinMode() != mDot::PEER_TO_PEER) {
        logInfo("changing network join mode to PEER_TO_PEER");
        if (dot->setJoinMode(mDot::PEER_TO_PEER) != mDot::MDOT_OK) {
            logError("failed to set network join mode to PEER_TO_PEER");
        }
    }
    
    /*
     *  Get the Frequency and then choose transfer frequency, datarate, and power accordingly
     *
     */
    ////////////////////////////////////////////////////////////////////////////////
    frequency_band = dot->getFrequencyBand();
    switch (frequency_band) {
        case lora::ChannelPlan::EU868_OLD:
        case lora::ChannelPlan::EU868:
            // 250kHz channels achieve higher throughput
            // DR_6 : SF7 @ 250kHz
            // DR_0 - DR_5 (125kHz channels) available but much slower
            tx_frequency = 869850000;
            tx_datarate = lora::DR_6;
            // the 869850000 frequency is 100% duty cycle if the total power is under 7 dBm - tx power 4 + antenna gain 3 = 7
            tx_power = 4;
            break;

        case lora::ChannelPlan::US915_OLD:
        case lora::ChannelPlan::US915:
        case lora::ChannelPlan::AU915_OLD:
        case lora::ChannelPlan::AU915:
            // 500kHz channels achieve highest throughput
            // DR_8 : SF12 @ 500kHz
            // DR_9 : SF11 @ 500kHz
            // DR_10 : SF10 @ 500kHz
            // DR_11 : SF9 @ 500kHz
            // DR_12 : SF8 @ 500kHz
            // DR_13 : SF7 @ 500kHz
            // DR_0 - DR_3 (125kHz channels) available but much slower
            tx_frequency = 915500000;
            tx_datarate = lora::DR_13;
            // 915 bands have no duty cycle restrictions, set tx power to max
            tx_power = 20;
            break;

        case lora::ChannelPlan::AS923:
        case lora::ChannelPlan::AS923_JAPAN:
            // 250kHz channels achieve higher throughput
            // DR_6 : SF7 @ 250kHz
            // DR_0 - DR_5 (125kHz channels) available but much slower
            tx_frequency = 924800000;
            tx_datarate = lora::DR_6;
            tx_power = 16;
            break;

        case lora::ChannelPlan::KR920:
            // DR_5 : SF7 @ 125kHz
            tx_frequency = 922700000;
            tx_datarate = lora::DR_5;
            tx_power = 14;
            break;

        default:
            while (true) {
                logFatal("no known channel plan in use - extra configuration is needed!");
                wait(5);
            }
            break;
    }
    /////////////////////////////////////////////////////////////////////////////////////

    // in PEER_TO_PEER mode there is no join request/response transaction
    // as long as both Dots are configured correctly, they should be able to communicate
    update_peer_to_peer_config(network_address, network_session_key, data_session_key, tx_frequency, tx_datarate, tx_power);
    
    // save changes to configuration
    logInfo("saving configuration");
    if (!dot->saveConfig()) {
        logError("failed to save configuration");
    }
    // Display configuration
    // It's gonna output a lot of information onto the Serial Terminal
    display_config();





//below is acc,temp sensors
    ADT7410Initialize();
    ADXL372Initialize();
    
    Periodic.attach(&takePeriodicReadingTicker,5);
    
    while(1){
        // Create a vector of uint8_t elements to be sent later
        
        std::vector<uint8_t> tx_data;
        
        
        if(periodicReadingTrigger)
        {
            takePeriodicReading(tx_data);
        }
        
        if(takeAccelerometer || takeTemperature){
            pc.printf("INTERRUPTEDDDDDDDD: ");
            if(takeTemperature) pc.printf("Temperature triggered!!!!!!!!!!!!\n\r");
            else if(takeAccelerometer) pc.printf("AccelerometerTriggered!!!!!!!!!!!!!\n\r");
            
            for(int i = 0; i < BUFFER_SIZE; ++i){
                /*
                *   Taking accelerometer data
                */
                regAddress = 0x08;  // This is the register address for XData
                accelValues = accelerometerI2CRead(regAddress);
                Xmsb = *(accelValues + 0);
                Xlsb = *(accelValues + 1);
                Ymsb = *(accelValues + 2);
                Ylsb = *(accelValues + 3);
                Zmsb = *(accelValues + 4);
                Zlsb = *(accelValues + 5);
    
                XDataInterrupt[i] = (Xmsb << 8 | Xlsb) >> 4;  // Combine two bytes into short int, remove last 4 flag bits
                YDataInterrupt[i] = (Ymsb << 8 | Ylsb) >> 4;
                ZDataInterrupt[i] = (Zmsb << 8 | Zlsb) >> 4;
    
                XDataInterrupt[i] = twosComplementConversion(XDataInterrupt[i]);
                YDataInterrupt[i] = twosComplementConversion(YDataInterrupt[i]);
                ZDataInterrupt[i] = twosComplementConversion(ZDataInterrupt[i]);
    
                /*
                *   Taking temperature data
                */
                regAddress = 0x00;
                rawTempValues = ADT7410Read(regAddress);
                temperatureBuffer[i] = ((*(rawTempValues + 0) << 8) | *(rawTempValues + 1)) >> 3; // Combine the two bytes into 
                                                                                                // a short int variable(16 bits), remove last 3 bits    
            }
            
            for(int i = 0; i < BUFFER_SIZE; ++i){
                pc.printf("Accelerometer::: ");
                pc.printf("X: 0x%x | Y: 0x%x | Z: 0x%x\n\r", XDataInterrupt[i], YDataInterrupt[i], ZDataInterrupt[i]);
                pc.printf("Temperature::: ");
                pc.printf("Celsius: 0x%x\n\r", temperatureBuffer[i]);   
            }
            //wait(0.2);
            takeAccelerometer = false;  // Flip back to no trigger
            takeTemperature = false;    // Flip back to no trigger
            
        }
        
        
        
        
        //wait(1);
    }
    
    return 0;
}

/*******************************************************************************
 *  Not really used at the moment
 *  Not really needed. But keep just in case because I don't want to rewrite it
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
char twosComplementConversion(char value)
{
    /*
     * Go from bit 0 to bit 7 and invert them
     * Then finally add 1
     */
    char mask = value & 0x80;
    if(mask == 0x80){   // Check for sign
        value = ~value + 1;
        return value;
    } 
    return value;
}
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  Initializes whatever settings you want for the accelerometer
 *  Can change it to use the previous I2C write function instead of all this mess
 *
 ******************************************************************************/  
////////////////////////////////////////////////////////////////////////////////
void ADXL372Initialize(void){
    pc.printf("Initializing ADXL372 accelerometer\n\r");
    accelerometerI2CWrite(0x3F, 0x0F);  // Enable I2C highspeed,Low Pass, High pass and full bandwidth measurement mode
    accelerometerI2CWrite(0x38, 0x01);  // Enable the High pass filter corner 1 at register 0x38
/*     accelerometerI2CWrite(0x24, 0x01);  // X used for activity threshold
    accelerometerI2CWrite(0x26, 0x01);  // Y used for activity threshold
    accelerometerI2CWrite(0x28, 0x01);  // Z used for activity threshold  */
    pc.printf("\n\n\r");
}
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
 *  ADT7410 Initializing function
 *  Make critical temperature 24 celsius
 *  Make CRIT pin active high
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void ADT7410Initialize(void){
    pc.printf("Initializing ADT7410 Temperature\n\r");
    // Make critical temperature be 24 celsius
    ADT7410Write(0x08, 0x01);   // MSB of Temperature Crit value
    ADT7410Write(0x09, 0x80);   // LSB of Temperature Crit value
    
    // Make CRIT pin active high
    ADT7410Write(0x03, 0x08);   // Turn INT HIGH, works for the interrupt pin
    pc.printf("\n\n\r");
}
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  ADXL372 reset function
 *  Resets all registers and settings back to default
 *  Basically the same as the previous ADXL372 I2C write function
 *
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void ADXL372Reset(void){
    int flag;
//--------- One full writing cycle for ADXL372 for Z Enable ------------------//
/*    '0' - NAK was received
 *    '1' - ACK was received, <---- This good
 *    '2' - timeout
 */
    ADXL372.start();
    flag = ADXL372.write(ADXL372_Address_8bit | 0);
    if(flag == 1)
    {
        //pc.printf("Write to I2C address success\n\r");
        
        flag = ADXL372.write(0x41);
        if(flag == 1)
        {
            //pc.printf("Write to 0x41 register address success\n\r");
            flag = ADXL372.write(0x52);                             // Set bit 0
            if(flag == 1)
            {
                pc.printf("Everything has been reset\n\r");
                ADXL372.stop();
            }
        }
    }
    else ADXL372.stop();
// ---------------- End of writing cycle --------------------------//
}
////////////////////////////////////////////////////////////////////////////////

/*
 *
 *  Self-test to see if the accelerometer is working as intended
 *  Wait 300 ms.
 *  Check bit 2 for a 1 for success. Bit 1 for completion of self-test.    
 *  Returns whole register
 */
////////////////////////////////////////////////////////////////////////////////
void I2CSelfTest(void){
    char *result;
    uint8_t check;
    accelerometerI2CWrite(0x3F, 0x0F);
    accelerometerI2CWrite(0x40, 0x01);
    wait(0.3);
    result = accelerometerI2CRead(0x40);
    check = result[0];
    if(check & 0x04){
        pc.printf("Passed\n\r");
    }else {pc.printf("FAILED\n\r");}
}
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *                                                                             
 *  I2C function for the the ADXL372 accelerometer for a write sequence
 *  Param:
 *      hexAddress: Pass the hexadecimal value for what register you want
 *      hexData: Pass the hexadecimal value for what data you want to send
 *               i.e. hexadecimal represenatation of certain bits
 *  Returns from mbed library write function            
 *      0: NAK was reveived
 *      1: ACK was received <---- Good for us
 *      2: Timeout     
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void accelerometerI2CWrite(int hexAddress, int hexData){

    int flag;
    int registerAddress = hexAddress;
    int data = hexData;
    
    ADXL372.start();
    flag = ADXL372.write(ADXL372_Address_8bit);
    if(flag == 1)
    {
        //pc.printf("Write to I2C address success\n\r");
        wait(0.1);
        flag = ADXL372.write(registerAddress);
        if(flag == 1)
        {
            //pc.printf("Write to register 0x%x address success\n\r", registerAddress);
            flag = ADXL372.write(data);
            if(flag == 1)
            {
                pc.printf("Writing data 0x%x to register address 0x%d success\n\r", data, registerAddress);
                ADXL372.stop();
                return;
            }else {ADXL372.stop();} 
        }else {ADXL372.stop();}
    }else ADXL372.stop();
   
}
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
 *  I2C read sequence for the accelerometer
 *  Param:
 *      hexAddress: pass the hexadecimal representation of desired Register address
 *  Return:
 *      Char pointer to the array of read data. 
 *
 *  Right now it works only for the XData, YData, ZData because I wrote it to read
 *  6 bytes(6 registers).
 *  Should change it to be 1 byte at a time
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
char * accelerometerI2CRead(int hexAddress){
    char accelData[6];
    char registerAddress[1];
    registerAddress[0] = hexAddress;
    
    // Perform mbed's way sending a start bit, then device address[r/~w], and then the register address
    // Also if it succeeds, continue to the next operation
    if(ADXL372.write(ADXL372_Address_8bit, registerAddress, 1) == 0){
        
        // If previous sequence works, get 6 bytes into char array accelData
        // Char array because it uses 1 byte(8bits)
        // Should probably change it to uint8_t type
        if(ADXL372.read(ADXL372_Address_8bit, accelData, 6) == 0){
            return accelData;
        }else pc.printf("Failed to read\n\r");
    }else pc.printf("Failed to write\n\r");
    return 0; // Only if it fails completely 
}
////////////////////////////////////////////////////////////////////////////////




/*******************************************************************************
 *  Performs one byte write I2C protocol
 *  PARAM:
 *      registerAddress: register you want access to in device, one byte char hex format
 *      data: one byte data that you want to write to device register
 *  Return results from mbed library function:
 *      0: failure at writing i2c address
 *      1: successful write
 *      2: failure at writing data
 *      
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
void ADT7410Write(unsigned char registerAddress, unsigned char data){
    int flag;
    ADT7410.start();
    flag = ADT7410.write(ADT7410_Address_8BIT);
    if(flag == 1)
    {
        
        wait(0.1);
        flag = ADT7410.write(registerAddress);
        if(flag == 1)
        {
           
            flag = ADT7410.write(data);
            if(flag == 1)
            {
                pc.printf("Writing data 0x%x to register address 0x%x success\n\r", data, registerAddress);
                ADT7410.stop();
                
            }else {ADT7410.stop();} 
        }else {ADT7410.stop();} 
    }else ADT7410.stop();
       
    
}
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
 *  I2C Read function for ADT7410 Temperature sensor
 *  Param:
 *      hex: hexadecimal representation for desired register
 *  Return:
 *      Char pointer to the array of data values.
 *  Could also change from a char pointer to a uint8_t pointer.
 *
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////
char * ADT7410Read(int hex){
    //short int convertedVal;
    char data[2] = {0, 0};
    char cmd[1];
    cmd[0] = hex;
    //pc.printf("Register Addres is: %x \n\r", cmd[0]); 
    if(ADT7410.write(ADT7410_Address_8BIT, cmd,1) == 0){
        if(ADT7410.read(ADT7410_Address_8BIT, data, 2) == 0){
            
            return data;
            //return (data[0] << 8 | data[1])>>3;     // Explained here: https://stackoverflow.com/a/141576 SOOO GREAT
            
        }else {pc.printf("Failed to read \n\r"); return data;}
    }else {pc.printf("Failed to write \n\r"); return data;}
    
}
////////////////////////////////////////////////////////////////////////////////

