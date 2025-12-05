  //
//   last updated  2025 11 24
//  Derived from engineMasterDatagram code.  Removing all operations and structures related to keeping
//     state about regsitering and registered engines.  Let upper level command software do that.
//  Fixed bug that limited number of bytes sent to engne modules to four. (ACKs of registration
//  requests are currently five bytes.)
//
// EngineMaster is basically a conduit between upper level control software and engine modules controlling
// individual engine function.  Inputs from above come across a Serial connection, and resulting outputs are sent
// via RFM69 radio.  Inputs from engines come in over the RFM69 radio and resulting outputs are sent over
// the Serial conection.
//
// EngineMaster attempts to ensure message delivery to engines by employing reliable datagram services offered
// by RadioHead RFM69 library.  After a settable number of message delivery failures, a warning message is sent back
// across the Serial line and the failed message is discarded.
//
// engineMaster does *no message queueing*.  When it receives a message from above, it follows through on attempted
// delivery to target engine before listening for any further messages.  HOWEVER, if it receives a command to shut
// down all engines, it will broadcast that message to all engines immediately.

#include <Arduino.h>
#include <SPI.h>    // for comms with packet radio
#include <RH_RF69.h>   // Packet radio library
#include <RHReliableDatagram.h>
#include "defines.h"

#define MyAddress 250
#define clientSpecialAddress 254

RH_RF69 rf69(8, 3); // Adafruit Feather M0  CS and INT pins
RHReliableDatagram manager(rf69, MyAddress);
uint8_t from;

bool DEBUG = false; // set false when in production mode

//   *********************************************************************************************
//   *******************************************  S e t u p  *************************************
//   *********************************************************************************************
void setup() {

  Serial.begin(115200);
  while(!Serial) delay(1000);

//
//    ------------------------------------- P a c k e t   R a d i o   S e t u p ------------------------------------
//
  if (!manager.init()){
	Serial.print(engineConduitInformation);   Serial.print("  ");    Serial.println (radioFailure);
  }
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module),  No encryption
  rf69.setTxPower(17, true);  // 17 is highest power that doesn't kick in power hungry turbo mode.

} // end setup ***********************************************************************************************
// **********************************************************************************************************




#define endOfInput 0x10000
#define signalError 0X20000
//   ******************************************* m y P a r s e I n t ( )  *************************************
//                                               ***********************
uint32_t myParseInt(){
//  This routine looks for an unsigned integer input in ascii format, not exceeding 65535 (uint16_t) in value,
//  on the Serial input line.  It treats all characters that are not base ten digits preceding the
//  final integer produced as spaces to be ignored (skipped past).  Once a conversion to integer starts,
//  it stops looking when it encounters an ascii character that is not a base ten digit. 
//   e.g.:  input string "aa123bcd456" produces the binary value for 123.
//  Input strings are assumed to be termnated by the character 0x0A <-- end of line

//  if a parsed integer would exceed 65535 then the value 65537 (2^16 + 1) is returned, signalling an overflow.
//  65536 (2^16) signals end of input encountered.

  uint32_t inputValue = 0;
  char c;
  bool parseError = false;
  c = Serial.peek();
  if(c == 0xFF) return endOfInput;  // no more to read
  while(!isdigit(c)){
    Serial.read(); c = Serial.peek();
    if(c == 0xFF) return endOfInput;    // end of string encountered
    if(c == 0x0A){
      Serial.read();
      return endOfInput;    // end of string encountered
    }
  }
  while (isdigit(c)){
    inputValue = (10 * inputValue) + (c - '0');
    if (inputValue > 65535){
      inputValue = 0;
      parseError = true;
    } 
    Serial.read(); c = Serial.peek();
  }
  if (parseError) return signalError;
  else return inputValue; 
}  // end parseint


  uint8_t serBuf[200];
  uint8_t serBufLen = sizeof(serBuf);

//   ************************************* c o n v e r t  S e r i a l  I n p u t ( )  *************************************
//                                         *****************************************
byte convertSerialInput(){
  //  This function updates the contents of serBuf[] by inserting new values.
  //  It can recognize 16 bit values and place them in adjacent bytes of sefBuf[].
  //  Its return value, len, indicates the number of new bytes placed into serBuf[].
  //  Assumption: length of serBuf[] does not exceed 255.
  uint32_t binNumber;
  byte len = 0;
  binNumber = myParseInt();  
  while (binNumber < endOfInput){  // Collect converted to binary inputs from Serial stream
    if (binNumber > 0xFF) serBuf[len++] = (binNumber & 0xFF00) >> 8;  //  Handle 16 bit uints.
    serBuf[len++] = (binNumber & 0xFF);
    binNumber = myParseInt();
  }
  if (binNumber == endOfInput) return len;
  else return 0;  // parseInt returned an error indication of signalError
}



//   ****************************************** s e n d C o m m a n d ( )  *************************************
//                                              *************************
void sendUpSerial(uint8_t buf[], byte len, bool processSerial){ 
  uint8_t start = 3;
  Serial.print(buf[0]);
    if (len > 2 && processSerial){
       uint16_t SN = ((uint16_t) buf[1] << 8 | buf[2] );
       Serial.print(" "); Serial.print(SN);
    }
    else 
       start = 1; 
    for(uint8_t i = start; i < len; i++){
      Serial.print(" "); Serial.print(buf[i]);
  }
  Serial.println();
}


uint8_t radioBuf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t radioBufLen = sizeof(radioBuf);


void processSerialCommand(){
//  operates on serBuf and serBufLen
//  Commands processed here have come from control above on Serial input.
//  If its an ACK for a request to register, then pass news along along with command software index that
//  will also be used for datagram index on radio network.
//  Most will simply be passed along to engineModule
// Emergency stop and reset commands are broadcast/sent immediately

  if (serBufLen > 0)   // a buffer length of zero signals an error
    if(serBuf[0] == emergencyStop || serBuf[0] == resetAll)  //  If it's urgent then get message out immediately
        for(uint8_t i = 0; i < 3 ; i++){  // Send three times (either emergencyStop or resetAll)
          manager.sendtoWait(serBuf, 1, 255);  // by broadcast to all engines running
          delay(1000);  //               ^ broadcast address
        }
    else if(serBufLen > 3 && serBuf[0] == ACKRegistration)  // special ACK; serBuf[3] contains engine's new datagram address
        manager.sendtoWait(serBuf, serBufLen, 254); // 
        
    else if(serBufLen > 1) 
        manager.sendtoWait(serBuf, serBufLen, serBuf[1]);  // serBuf[1] is index used as datagram address also
//              engineSerial = ((uint16_t) serBuf[1] << 8 | serBuf[2] );
    else {
        Serial.print(engineConduitInformation);   Serial.print("  ");    Serial.println (unknownCommand); 
        }
}


void processRadioCommand(uint8_t sender){
//  Operates on radioBuf and radioBufLen.  "From" may have unique value of 254 in which case sender is
//  requesting a unique network ID, which will also be the engine's index, used locally.  Response here is to
//  use provided CPU Serial number to make a unique entry in the locally maintained engineRoster[]. The index
//  that references the engine's CPU number in engineRoster[] will also be the network ID used by the
//  radio network to uniquely id an engineModule on the radio network.
//  All other commands processed here will have a "from" value that is the index/network ID generated during the
//  address assignment step just noted. Operations coming from engineModules include error messages, status reports,
//  and engine CPU Serial# register requests.
//   All messages from an engineModule should contain a one byte command and two bytes of engine CPU Serial number.
    //      engineSerial = ((uint16_t) radioBuf[1] << 8 | radioBuf[2] );

  if (radioBufLen > 1){  // a buffer length less than two signals an error
     if (sender == 254 && radioBufLen > 2)  // engine trying to register
        // Notify upper level controller that engine is trying to register.
        // Note engine serial is in bytes [1] and [2] since a datagram index is not yet known.
        sendUpSerial(radioBuf, 3, true);  // Format: registerMe, serial, serial
     else         // Expecting status reports and error messages here.  Both destined for higher level controller.
        sendUpSerial(radioBuf, radioBufLen, false);  // Format:  command, index, mostly 8 bit data bytes.
  }
  else{
    //  Signal "command rejected" up across Serial to control. 
        Serial.print(engineConduitInformation);   Serial.print("  ");    Serial.println (eModBadCommand );   
//        Serial.print(engineConduitInformation);   Serial.print("  ");    Serial.println (setUpPercent );   
        }
  }


//   *********************************************************************************************
//   **************************************  M a i n   L o o p  **********************************
//   *********************************************************************************************

// ***********************************                   
void loop() {
// Check for command input from command software coming across Serial.
// Also check for msg from any engines coming in on radio
// Otherwise loop back

//  Check for and process inputs on Serial line first.  Likely leads to subsequent message being sent to engine(s)
//  across radio lines.
  if(Serial.available() > 0){
    serBufLen = convertSerialInput();  // Readies serBuf and surBufLen
    processSerialCommand();
  } 

  if (manager.available()){
	  radioBufLen = sizeof(radioBuf); 
    if (manager.recvfromAck(radioBuf, &radioBufLen, &from)){
      processRadioCommand(from);   
    }
  }
  delay(10);  
} // Loop


