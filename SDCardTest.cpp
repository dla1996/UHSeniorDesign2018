#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <mbed.h>
#include "dot_util.h"
#include "RadioEvent.h"
#include "itoa.h"

#include "SDBlockDevice.h"
#include "FATFileSystem.h"


#define BUFFER_SIZE 10

//note: added GPS functions , variables below (will organize better later ski
//and gps code in main is noted



///-----------------FOR GPS------BELOW----------------------------------
//

#include "MBed_Adafruit_GPS.h"




//------------------------------SD CARD------------------------------------//

SDBlockDevice sd(D11, D12, D13, D10); // mosi, miso, sclk, cs
FATFileSystem fs("sd"); 
//LittleFileSystem fs("sd");


 
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
/*
 *  Variables used for mDot
 */


uint32_t tx_frequency;
uint8_t tx_datarate;
uint8_t tx_power;
uint8_t frequency_band;

void return_error(int ret_val){
  if (ret_val)
    printf("Failure. %d\r\n", ret_val);
  else
    printf("done.\r\n");
}

void errno_error(void* ret_val){
  if (ret_val == NULL)
    printf(" Failure. %d \r\n", ret_val);
  else
    printf(" done.\r\n");
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
    FILE *fp;
    
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
    
    int error = 0;
    
    if ( 0 == sd.init()) {
        printf("Init success \n\r");
    }
    pc.printf("Formatting\n\r");
    error = FATFileSystem::format(&sd);
    return_error(error);
    //	ERROR IN MOUNTING -5 CODE
    error = fs.mount(&sd);
    pc.printf("%s\n", (error ? "Fail :(\n\r" : "OK\n\r"));
    return_error(error);
    // Check if file was opened for some reason
    if(fp != NULL)
    {
        pc.printf("File was opened, closing it\n\r");
        fclose(fp);
    }
    // ERROR IN OPENING -5 CODE
    fp = fopen("/sd/sdtest.txt", "w+");
    if(fp == NULL) {
        pc.printf("Could not open file for write\n\r");
    }
    fprintf(fp, "Hello fun SD Card World!");
    
    for (int i = 0; i < 20; i++){
    printf("Writing decimal numbers to a file (%d/20)\r\n", i);
    fprintf(fp, "%d\r\n", i);
    }
    
    pc.printf("Writing decimal numbers to a file (20/20) done.\r\n");

    pc.printf("Closing file.\n\r");
    fclose(fp); 
    
    
    
    return 0;
}

