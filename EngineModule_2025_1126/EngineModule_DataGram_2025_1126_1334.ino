  //
//   last updated  2025 11 26
//     Added provisions for responding to a total reset request from engineMaster
//     Added desRPS and smoothedRPS to parameter list for sendStatus
//     Set up information messages for higher levels
//     Added pullup for pin 12 (argh, 1.5 days lost on that one)
//     Fixed estimated speed computation for cruise mode;  fixed head/rear lights issue
//     Add update cruise RPS values on the fly.
//     Add provisions for detecting and reporting dead battery
//     Got reset working better, as well as management of myEngines
//     Modified for streamlined engineConduit API; added smoothing for read battery voltage
//     Added provision for parameterized low voltage cutoff, since different control boards use different divider networks.
//     Updated low voltage cutoff and smoothedVoltage initial values to avoid false triggering.
//     Modified timeout for hearing back from conduit for reg ACK to six seconds (formerly two).
//     2025 11 24: Added provisions for cutting off battery power, detected locally or commanded from external.
//     2025 11 26: Made powering off a loop that drops A2 once per second;  should depower processor quickly and reliably.
//                 Also repaired power-off issues entangled with issue with boolean state info.
//                 Also tightened up takeAction code error processing.
//
//  Code for Feather with onboard RF69 433MHz packet radio and connections to Pololu motor *driver* (not controller).  This code sets up packet radio
//  and PWM for driving motor driver, and then loops, receiving commands via packet radio, ACking them, and then applying received commands to motor(s).
//  Code also tracks flywheel revolutions in order to apply cruise control.
//   Pin use:
//     Power comes in USB 5 volt port.  DO NOT ATTACH TO USB WHEN ALTERNATE POWER SOURCE IS ENABLED
//     Pin 10:  Output.  20 kHz PWM for motor driver board.
//     Pin 11:  Output.  Direction controller for motor driver board.
//     Pin 12:  Input.  Tracks pulses off IR sensor.  Periodic interrupts capture counts for program.
//     Pins A1-A4: Output.  Engine lights control.  (Or D20, D21, D5 and D6 on newer designs)
//     Pin A5:  Input.  Read Battery voltage indicator.
//
//  Radio input is of the form:
//    byte 0:  command to respond to
//    byte 1ff:  binary data may follow if operation in byte 0 needs it, e.g. for a speed command.  Speed value is one byte, zero to 100.



#include <SPI.h>    // for comms with packet radio
#include <RH_RF69.h>   // Packet radio library 
#include <RHReliableDatagram.h>
#include "defines.h"

// myAddress is temporarily set to 254 for signaling a request for a fixed datagram address
#define serverAddress 250
uint8_t myAddress = 254;

#define reverse 1
#define forward 2
#define dirUnknown 0

#define percent 1
#define cruise 2
#define unknown 0

//   StatusReportFreq is multiples of TCC2 interrupt frequency
#define statusReportFreq 2

int resolution = 2400;   //  Related to duty cycle.  Equals (48MHz clock speed / PWM frequency).

bool startedUp = false;         //  True iff engine has been started up.
bool shuttingDown = false;      // True if shutdown requested and speed still > 0.
uint8_t  engineDirection = dirUnknown; 
uint8_t smoothedVoltage = 190;     // RC filtered(1/2, 1/2) voltage of LiPo battery.  190 is over all known cutoffs
uint8_t lowVoltageCutoff = 167;    // This value is sent to engine on startup.  Typ value is 168 or 177 depending on
                                   // divider network used on control board.  167 is a failsafe placeholder.
uint8_t flash = 0;             // For toggling onboard LED to show life
uint8_t ditchLightToggle = 1;  // used to alternate between left and right ditch lights
uint8_t statusReportCounter = statusReportFreq;  // Used to determine status report frequency (based on TCC2 ints)
bool beenToldToRestart = true;
int lowVoltageCount = 0;   //  Keeps track of repeated low voltage readings in support of power off conditions.
bool lowVoltage = false;  // Once low voltage has been seen often enough, this gets set in order to trigger shutdown/power off
bool beenToldToPowerDown = false;  //  If command setPowerToOff arrives then this gets set true and power should be cut.

int headlight;
int rearlight;
int leftDitch;
int rightDitch;

  union
  {
    uint16_t onePiece;
    uint8_t twoPieces[2];
  }myEngineNumber;  // Low order two bytes of CPUNumber


uint8_t speedControlMethod = unknown;  
bool DEBUG = false;

void TCC2_Handler();  // Prototype for one second interrupts handler



  RH_RF69 rf69(8, 3); // Adafruit Feather M0  CS and INT pins for RFM69 radio
  RHReliableDatagram manager(rf69, myAddress);  // Set up with temporary address

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Built in LED
  pinMode(A5, INPUT);  // Battery voltage monitoring pin;  12.6 volts should show up as 2.74 volts; about 840 out of 1023.
  #define dirPin 11
  pinMode(dirPin, OUTPUT);  // Connect to Dir input on Pololu motor driver.  (Pin 11 on M0)
  // Pin D10 is output used for PWM
  // Pin D12 is input used for cruise control (count pulses off IR sensor tracking flywheel strip)
  pinMode(12, INPUT_PULLUP);
  pinMode(A2, OUTPUT);   // Alive pin for soft power switch.  Keep high unless power off is requested or required.
  digitalWrite(A2, HIGH); 

  ////////////////////////////////////////////////////////////////////////////////////////
  // Generic Clock Initialisation 
  ////////////////////////////////////////////////////////////////////////////////////////

// Set up 32kHz clock for 1 Hz interrupts
  GCLK->GENDIV.reg =  GCLK_GENDIV_DIV(1) |          // Select clock divisor to 1                     
                      GCLK_GENDIV_ID(4);            // Select GLCK4         
 
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |            // Set the duty cycle to 50/50 HIGH/LOW 
                      GCLK_GENCTRL_GENEN |          // Enable GCLK                   
                      GCLK_GENCTRL_SRC_XOSC32K |    // Select GCLK source as external 32.768kHz crystal (XOSC32K)                          
                      GCLK_GENCTRL_ID(4);           // Select GCLK4             
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0
      GCLK_CLKCTRL_GEN_GCLK0 |                     // Select GCLK0 at 48MHz
      GCLK_CLKCTRL_ID_TCC0_TCC1;                   // Connect GCLK0 to timers TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);             // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |          // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK0 |      // GCLK0 at 48MHz 
                      GCLK_CLKCTRL_ID_TC4_TC5;      // As a clock source for TC4 and TC5
  
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |          // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK4 |      // GCLK4 at 32.768kHz
                      GCLK_CLKCTRL_ID_TCC2_TC3;     // As a clock source for TCC2 and TC3
//    while (GCLK->STATUS.bit.SYNCBUSY);             // Wait for synchronization


  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;           // Switch on the event system peripheral
  
//    ********************************************************************** P W M   S e t u p *************************************
// SAMD21G timers
//        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
//        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
//        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
//        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
//        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
//        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5

  //////////////////////////////////////////////////////////////////////////////////////////////
// Set-up TCC0 and TCC1 for 20kHz PWM, 0% duty-cycle (initially) on digital pin 10  pin 10    pin 10   pin 10
// Pin 10 is used for motoer speed control by controlling duty cycle on its 20kHz signal
  //////////////////////////////////////////////////////////////////////////////////////////////
//
  // Enable the port multiplexer for the PWM channel for digital pin 10
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1; 
  // Connect the TCC0 channel 2 to digital pin 10
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;        // Set the PWM output to dual-slope critical PWM mode
    while (TCC0->SYNCBUSY.bit.WAVE);             // Wait for synchronization
  TCC0->PER.reg = (resolution - 1);              // Set the frequency of the PWM to 20kHz ((48MHz/20kHz)-1) on D9
    while (TCC0->SYNCBUSY.bit.PER);              // Wait for synchronization
    TCC0->CC[2].reg = 0;                         // Set the duty-cycle to 0%
    while (TCC0->SYNCBUSY.bit.CC2);              // Wait for synchronization
  TCC0->CTRLA.bit.ENABLE = 1;                    // Enable the TCC0 timer
    while (TCC0->SYNCBUSY.bit.ENABLE);           // Wait for synchronization

  
  //////////////////////////////////////////////////////////////////////////////////////////////
  // TCC2 Initialisation - reference timer: measures a 1s period and generates an interrupt.
  //////////////////////////////////////////////////////////////////////////////////////////////     
  
  TCC2->PER.reg = 32767;           //Timer-Frequency = Generic-Clock-Frequency / (Timer-Prescaler * (Value-PER-Register + 1))
    while(TCC2->SYNCBUSY.bit.PER);
  TCC2->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;     //Set up normal PWM mode on TCC2
    while (TCC2->SYNCBUSY.bit.WAVE);
  TCC2->CC[0].reg = 16384;       //Set CounterCompare.
    while(TCC2->SYNCBUSY.bit.CC0);
  TCC2->INTENSET.reg = TCC_INTENSET_OVF;     //Enable TCC2 OVF Interrupt
  TCC2->CTRLA.bit.ENABLE = 1;           //Enable timer TCC2 and wait for synchronization
    while (TCC2->SYNCBUSY.bit.ENABLE);
  NVIC_SetPriority(TCC2_IRQn, 15);    //Set NVIC priority for TCC2 to 15 (lowest)
  NVIC_EnableIRQ(TCC2_IRQn);      //Connect TCC2 to Nested Vector Interrupt Controller (NVIC)


  ////////////////////////////////////////////////////////////////////////////////////////
  // TC4 Initialisation - measurement counter: counts the number incoming pulses on pin D12
  ////////////////////////////////////////////////////////////////////////////////////////
  
  // Enable the port multiplexer on digital pin D12 (port pin PA19)
  PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;
  // Set-up the pin as an EIC (interrupt) peripheral on D12
  PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg |= PORT_PMUX_PMUXO_A;

  // External Interrupt Controller (EIC) ///////////////////////////////////////////////////

  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO3;                                // Enable event output on external interrupt 3 (D12)
  EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE3_HIGH;                           // Set event detecting a HIGH level on interrupt 3
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT3;                               // Disable interrupts on interrupt 3
  EIC->CTRL.bit.ENABLE = 1;                                               // Enable the EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);                                       // Wait for synchronization

  // Event System //////////////////////////////////////////////////////////////////////////

  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                               // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);               // Set the event user (receiver) as timer TC4
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |   // Set event generator (sender) as external interrupt 3
                       EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0                                 
  
  // Timer Counter TC4 /////////////////////////////////////////////////////////////////////

  TC4->COUNT32.EVCTRL.reg |= TC_EVCTRL_TCEI |              // Enable asynchronous events on the TC timer
                             TC_EVCTRL_EVACT_COUNT;        // Increment the TC timer each time an event is received
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32;          // Configure TC4 together with TC5 to operate in 32-bit mode
  TC4->COUNT32.CTRLA.bit.ENABLE = 1;                       // Enable TC4
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization
  TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);        // Offset of the 32-bit COUNT register
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization


//
//    *************************************************** P a c k e t   R a d i o   S e t u p *************************************
//
  if (!manager.init())
    flashLED(2,2);
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module),  No encryption
  rf69.setTxPower(17, true);  // 17 is highest power that doesn't kick in power hungry turbo mode.


  //    *********************************************** G e t  E n g i n e   #   f r o m   C P U   I D  ***************
  volatile uint32_t CPUNumber;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  CPUNumber = *ptr1;   // Get SAMD21 CPU unique ID number
  myEngineNumber.onePiece = CPUNumber & 0xFFFF;  //Get low order two bytes of CPU serial
  if(DEBUG){
      Serial.begin(19200);   
      while (!Serial);       // Wait for user to open serial port monitor.
      Serial.print("CPU number is: ") ; 
      Serial.println(CPUNumber, HEX);
      Serial.print("My engine number is: ") ; 
      Serial.println(myEngineNumber.onePiece, HEX);
  }   

//   ############################################### G e t  u n i q u e   d a t a G r a m   a d d r e s s ###############

  getRadioDataGramAddress(); 


} // end SETUP  ////////// ***********************************************************************************************
/// ***********************************************************************************************************************


void getRadioDataGramAddress(){
  uint8_t tempData[5];
  uint8_t tempLen = sizeof(tempData);
  uint8_t from; 
  // When engine first powers up it attempts to get a radio network unique ID from engineMaster.  This is signalled
  // by using the unique "from" address of 254, which engineMaster reserves for engines requesting a unique network
  // ID.  Assumption is that engineMaster returns a message to the 254 address with a payload containing a new
  // unique ID, which this engine Module is to use as its "from" address henceforth.  This request for an ID will spin until
  // such time that enginemaster returns an ID.  There is risk in multiple engineModules making these "254 address"
  // registration requests concurrently.  EngineMaster includes an engine's CPU's serial number in a response for
  // a unique address to allow an engineModule recipient to ensure only the intended engine uses the new network ID.
  myAddress = 254;
  manager.setThisAddress(myAddress);   // set my uint8_t address for datagram communications;  temp is 254
  while(myAddress == 254){
    tempData[0] = registerMe;
    tempData[1] = myEngineNumber.twoPieces[1];
    tempData[2] = myEngineNumber.twoPieces[0];
    while (!manager.sendtoWait(tempData, 3, serverAddress)){   // Use from address of 254 to indicate request for real address
      if (DEBUG) Serial.println("No new address response from Control.");
      delay(6000); // Otherwise wait 6 seconds and then try again.
      }
    //  Wait for reply from engineConduit with new address in tempData[0]
  
    if (manager.recvfromAckTimeout(tempData, &tempLen, 6000, &from)) {
      // Got an expected message.  Is it for me?  If not, keep requesting.
      if (DEBUG){
        Serial.print(tempData[0]); Serial.print(" "); 
        Serial.print(tempData[1]); Serial.print(" "); 
        if (tempLen > 2){
          Serial.print(tempData[2]); Serial.print(" "); 
        }
        if (tempLen > 3){
          Serial.print(tempData[3]); Serial.print(" ");         
        }
        if (tempLen > 4){
          Serial.print(tempData[4]);         
        }
        Serial.println();
      }
      if (from == serverAddress && tempLen > 3 && tempData[1] == myEngineNumber.twoPieces[1] \
          && tempData[2] == myEngineNumber.twoPieces[0]){
        manager.setThisAddress(tempData[3]);   // set my uint8_t address for datagram communications
        myAddress = tempData[3];
        lowVoltageCutoff = tempData[4];
        if (DEBUG) {
          Serial.println("My datagram address is: " + String(myAddress));
           Serial.println("My digital low voltage cutoff is: " + String(lowVoltageCutoff));
        }
      }
    }
  }
  if(DEBUG)Serial.println("Restart successful");
  beenToldToRestart = false;
}



void flashLED(int onTime, int offTime){
	// this function flashes on-board LED without end, on/off times intended to indicate error that occurred
  while(1){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000*onTime);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000*offTime);
  }
}


volatile uint32_t smoothedRPS = 0;         // RC smoothed Pulse counter from IR sensor on pin 12.  Read once per second in TCC2_Handler()
uint16_t currentPercentage = 0;            // percentage power being applied to motor, represented as a percentage, used to control
                                           //   PWM duty cycle. Used in both speed control methods: 1) set percentage
                                           //   explicitly, and set percentage using input speed setting and cruise control
                                           //   and flywheel RPS. 


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////  Class power (percentage) management. ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class variables

uint16_t desiredPercent = 0;     // current max percentage change per interrupt and absolute percentage, packed bytewise,
                                //  being worked towards;  [delta, abs percentage]
 

bool schedulePercentChange(uint8_t newPercentage, uint8_t newDelta){
	// This function is called when a new percent request has just been received. New request may or may not be kept
  //  based on current operating conditions.  Will replace existing desiredPercent if engine motion is zero
  //  or desiredPercent is non-zero, or newPercentage indicates immediate stop.  Rejected if engine is slowing to zero.
	//                       -----------------

	if ((newPercentage == 0) or ((desiredPercent & 0xFF) != 0) or (currentPercentage == 0)) { // Accept newPercentage as long as engine is
                                                                                 // not slowing to zero while newPercentage != 0
		desiredPercent = (uint16_t(newDelta) << 8) | newPercentage ;  
		if(DEBUG) {Serial.print("newPercentage request accepted immediately:  "); Serial.println(newPercentage);}
    return true;
	}
	else   
    // Rejection occurs when non-zero newPercentage could interfere with stopping engine by previous action.
    if(DEBUG) {Serial.print("newPercentage request rejected:  "); Serial.println(newPercentage);}                           	
    return false;   // flag rejection of new requested percentage.
}


void updatePercentSetting(){
  // Called on a periodic (TCC2 nterrupt frequency) basis
  // desiredPercent contains a max delta in upper byte, and a desired percentage in lower byte.
  int gap;
  if((desiredPercent & 0xFF) != currentPercentage){    // If desired power doeesn't match current, then take action.
    gap =  ((desiredPercent & 0xFF) - currentPercentage);  // Note: may be negative if engine is slowing
    if(gap > 0 )     // Go faster
      currentPercentage += min(gap, ((desiredPercent & 0xFF00) >> 8));  // Increase currentPercentage accordingly.
    else                  // Go slower
      currentPercentage += max(gap, -((desiredPercent & 0xFF00) >> 8));  // Decrease currentPercentage accordingly.
    // two negative numbers: ^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^   
    
      TCC0->CC[2].reg = (currentPercentage*resolution)/100;      // Set the duty-cycle to requested percent of resolution.
      while (TCC0->SYNCBUSY.bit.CC2);                  // Wait for duty cycle setting to complete		  
  }
}

//////////////////////////////// END  Class power (percentage) management. //////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////  Class speed management. ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class variables

uint16_t RPS[23];  // Create bins for target flywheel rotations per second, indexed by requested speeds.  Bins represent
                   //  values 5 MPH apart.  RPS[0] is for zero MPH, RPS[1] is for 0 MPH,...  RPS[21] is for 100 MPH.
                   //  RPS[22] is a sentinel == to RPS[21].
uint16_t desiredRPS = 0;  // target RPS derived from a requested speed setting;
uint8_t maxPowerDelta = 0;


uint16_t interpolate(uint8_t requestedSpeed){
///////////  Should error check requestedSpeed for being an in-range index
// if requestedSpeed > 100 then ERROR
  uint8_t baseIndex = (requestedSpeed / 5) + 1;  //  Truncation division.  output range is one to 21 given +1
  uint8_t midStep = requestedSpeed % 5;   //   Output range zero to four
  return RPS[baseIndex] + uint16_t((0.2 * midStep) * (RPS[baseIndex+1] - RPS[baseIndex]));  // interpolate
}

uint8_t lastRequestedSpeed = 0;  // Needed for when cruise values are updated while engine running.

bool scheduleSpeedChange(uint8_t newSpeed, uint8_t newDelta){
  // This function is called when a new speed setting has just been received. New request may or may not be accepted
  //  based on current operating conditions.  If it is accepted then it is converted to a rotations per second (RPS)
  //  value derived through interpolation on RPS table values. Acceptance is based on new speed request being zero, or
  //  engine not in a state of slowing to zero MPH.  
  uint16_t newRPS;
  newRPS = interpolate(newSpeed);
	if ((newRPS == 0) or (desiredRPS != 0) or (currentPercentage == 0)) { // Accept newSpeed (newRPS) as long as engine is
                                                                     // not slowing to zero while newSpeed != 0
    desiredRPS = newRPS;    // Current target RPS
    maxPowerDelta = newDelta;  // Current maximum absolute percentage PWM duty cycle can increase/decrease
		if(DEBUG) {Serial.print("\nNewSpeed request accepted immediately:  "); Serial.print(newSpeed);
              Serial.print("   "); Serial.println(desiredRPS);}
    return true;
	}
	else {
    // Rejection occurs when non-zero newSpeed could interfere with stopping engine by previous action.
    if(DEBUG) {Serial.print("NewSpeed request rejected:  "); Serial.print(newRPS); 
               Serial.print("  "); Serial.println(currentPercentage);}                           	
    return false;   // flag rejection of new requested percentage.
  }
}

int RPSErrorSum = 0;  //  Used in support of PID motor control done in updateSpeedSetting()
uint8_t Kp = 0;  //        Kp is used to scale and normalize updates from RPS to PWM percent
uint8_t Ki = 0;  //       Ki is used to scale and normalize updates from RPS to PWM percent
uint8_t Kd = 0;  //       Kd is used to scale and normalize updates from RPS to PWM percent

void updateSpeedSetting(){
// smoothedRPS is the RC filtered pulse count most recently computed during latest TCC2 interrupt
// desiredRPS is the current target RPS
// currentPercentage is percentage that dictates duty cycle for the motor driver PWM
// maxPowerDelta is current maximum absolute percentage PWM duty cycle can increase/decrease per TCC2 interrupt 
  int RPSError;
  int PropFactor;  //  RPSError scaled by normalization and kp
  int SumFactor;   //  SumError scaled by normalization and ki
  int newCurrentPercentage;  // temporary for updating currentPercentage
  
  if (desiredRPS == 0 and smoothedRPS < 50){  // If engine has effectively come to a requested stop then
      RPSErrorSum = 0;  
      PropFactor = 0;                      // drive currentPercentage to zero and reset integral.
      currentPercentage = 0;
  }
  else{
      RPSError = desiredRPS - smoothedRPS;   //   +/-Int    units: RPS
      PropFactor = RPSError/Kp;         //   +/-Int    units: percent               
      if (abs(RPSErrorSum + RPSError) < 5000)  // Limit magnitude of integral; 5000 is chosen arbitrarily
          RPSErrorSum += RPSError;  //   +/-Int    units: RPS
      SumFactor = RPSErrorSum/Ki;       //   +/Int     units: percent

      newCurrentPercentage = PropFactor + SumFactor;      // The PI portion of a PID controller
      if((newCurrentPercentage - currentPercentage) > 0 )     // Note: may be negative if engine is slowing
        newCurrentPercentage = min(currentPercentage + maxPowerDelta, newCurrentPercentage);  // Increase currentPercentage accordingly.
      else                  // Go slower
        newCurrentPercentage = max(currentPercentage - maxPowerDelta, newCurrentPercentage);  // Decrease currentPercentage accordingly. 
      
      newCurrentPercentage = max(newCurrentPercentage,0);    // Keep between
      newCurrentPercentage = min(newCurrentPercentage,100);  //              zero and 100%


      currentPercentage = newCurrentPercentage;
  }
  if(DEBUG) {
        Serial.println("\nCruise control...  ");
        Serial.print("Desired RPS:  "); Serial.print(desiredRPS);  Serial.print("   Smoothed RPS:  "); Serial.print(smoothedRPS);
        Serial.print("  Proportional factor:  "); Serial.print(PropFactor);
        Serial.print("  RPSErrorSum:  "); Serial.print(RPSErrorSum);  Serial.print("    SumFactor:  "); Serial.print(SumFactor);
        Serial.print("  Current Percentage:  "); Serial.println(currentPercentage);
  }
  TCC0->CC[2].reg = (currentPercentage*resolution)/100;      // Set the duty-cycle to requested percent of resolution.
  while (TCC0->SYNCBUSY.bit.CC2);                  // Wait for duty cycle setting to complete	
}


uint8_t speedFromRPS(){
// Use smoothed RPS to compute current speed of engine.
// Find RPS values that bracket the smoothedRPS and then interpolate a speed based on
// the speeds these bracketing RPS values represent.  Each RPS index represents 5 times 
// the (index - 1)  MPH, for index >= 1.
  uint8_t foundIt = 1;
  if(smoothedRPS >= RPS[21])
    return 100;  // Watch out for these index values; unsafe since they could change in def.
  else
    while (smoothedRPS >= RPS[foundIt]){
      foundIt++;
    }
    // Assert: foundit points to first RPS[] value > smoothedRPS

 //Serial.println("RPS[foundIt-1] is:  " + String(RPS[foundIt-1]));  
 //Serial.println("RPS[foundIt] is:  " + String(RPS[foundIt]));

 //  0.1 is added to compensate for truncation in discretization of return value.
  float factor = ((float)(smoothedRPS - RPS[foundIt-1]) / ((RPS[foundIt]+1) - RPS[foundIt-1])) + 0.1;  // +1 guarantees no zero divides in
                                                                              // monotonic non-decreasing vector
  // Serial.println("Factor is:  " + String(factor));                                                                          
  return uint8_t (5* ((foundIt-2) + factor)); 
  }
 




 //////////////////////////////////////////////////////////////////////////////////////////////
 // TCC2_Handler catches one second interrupts in order to support control over acceleration/deceleration, and
 // to capture pulse counts at one second intervals from IR sensor on motor flywheel to manage cruise control.
 //////////////////////////////////////////////////////////////////////////////////////////////

volatile boolean timerInterrupt = false;   // This flag set in TCC2_Handler() so motion can be checked in main loop

void TCC2_Handler() { 
//  Read the pulse count from TC4's 32 bit counter
  smoothedRPS = (smoothedRPS >> 1) + (TC4->COUNT32.COUNT.reg >> 1);  // Read and RC filter the TC4 pulse count register; 1/2 to each
  TC4->COUNT32.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;  // Retrigger the TC4 timer, to reset it to zero
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);               // Wait for synchronization
    //   If accessing control register B (above) disables continuous read, execute the following code.
    // TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |        // Enable a continuous read request
    //                            TC_READREQ_ADDR(0x10);    // Offset of the 32-bit COUNT register
    // while (TC4->COUNT32.STATUS.bit.SYNCBUSY);            // Wait for synchronization

  if (TCC2->INTFLAG.bit.OVF == 1) {                         //if overflow happens 
    TCC2->INTFLAG.bit.OVF =  0X01;                          //resets overflow bit so counter starts counting again
    while (TCC2->SYNCBUSY.bit.CC2);                        // Wait for synchronization
  } 
  digitalWrite(LED_BUILTIN, flash);
  flash = 1 - flash;
  smoothedVoltage = (smoothedVoltage >> 1) + (analogRead(A5) >> 3);   // Check battery voltage.  Max poss A5 value is 1023. 
  if (smoothedVoltage <= lowVoltageCutoff ){
    lowVoltageCount +=1;             
    if (lowVoltageCount > 20) // Entered immediate battery protection territory;  Shut down power now.
      if (not DEBUG) digitalWrite(A2, LOW);  // This is a failsafe in case other code doesn't force powwer off in time. 
      else {
        lowVoltageCount = 20; // Some risk here... no battery protection in debug mode.                                          
//        Serial.println("\nPower would have been cut here.");
      }
    if (lowVoltageCount > 10) lowVoltage = true;  //  Initiate shutdown for battery cutoff
  }
  else lowVoltageCount = 0;  // Reset low voltage count; but note lowVoltage may have been set true previously.
  timerInterrupt = true;                                        // timerInterrupt signals main loop that TCC2 handler called.
}

///////////////////////////////////////  End Class power management. ///////////////////////////////////////////



bool schedulePowerChange(uint8_t controlMethod, uint8_t newPower, uint8_t newDelta){
    // This funtion exists to choose method used for requesting PWM change to engine motor.
    // A power request can be a percentage indicating the extent of a PWM duty cycle, or it can be a speed,
    // which is subsequently translated into a count of rotations per second of motor flywheel, and then on
    // to a best estimate PWM duty cycle.
      if (controlMethod == percent) 
        return schedulePercentChange(newPower, newDelta);
		  else if (controlMethod == cruise ){ 
        lastRequestedSpeed = newPower;
        return scheduleSpeedChange(newPower, newDelta);
      }
      else 
        return false;
}


// NOTE: Emergency stop command honored only if engine is started up.  OK?   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void takeAction(uint8_t buf[], uint8_t bufLen){
// * * * * * * * * *  Respond to input request to communicate with engine  * * * * * * * * * * *
// In the following, buf[] is of the form:
//    byte 0:  command to act on
//    bytes 1/2: Engine serial number
//    byte 3ff: data for command as needed.

  #define opLoc 0
  #define dataLoc 2
  if (DEBUG){ Serial.print("Entering takeAction.  command is:  ");  Serial.println(buf[opLoc]);}
  if (!startedUp and !shuttingDown)
  {
    if(buf[opLoc] == setUpPercent){
      if(DEBUG){ Serial.println("Motor control is set to percentage.");}
      speedControlMethod = percent;  // Motor ontrol will be done based on submitted percentages of power (PWM)
    }
    else if ((buf[opLoc] == setUpCruise) && ((bufLen - dataLoc) >= 24)){
      Kp = buf[dataLoc];      // Incoming data is for the three PID control parameters for cruise,
      Ki = buf[dataLoc+1];    // plus 21 RPS values (divided by ten) for setting target observed
      Kd = buf[dataLoc+2];    // RPS values.
      for (byte i =1; i < 22; i++)
        RPS[i] = 10*buf[i + dataLoc +2];  // Save rotations per second indexed by increments of 5MPH
                                // bytes received have been divided by ten to fit in a byte each for transmission.
      RPS[0] = 0;         // lower end sentinel for interpolation
      RPS[22] = RPS[21];  // upper end sentinel for interpolation
      if(DEBUG){ Serial.println("Motor control is set to cruise.");}
      speedControlMethod = cruise;  // Motor control will be done based on submitted speeds.
      lastRequestedSpeed = 0;
      }
    else if ((buf[opLoc] == startUpEngine) && (speedControlMethod != unknown)){
	    if(DEBUG){ Serial.print("start up idle engine requested:  ");    Serial.println(myAddress);}
      // Startup command passes GPIO numbers for light control
      headlight = buf[dataLoc];  rearlight = buf[dataLoc+1];  leftDitch = buf[dataLoc+2];    rightDitch = buf[dataLoc+3];
//      headlight = A0;  rearlight = A1;  leftDitch = A2;    rightDitch = A3;
      pinMode(headlight, OUTPUT);    pinMode(rearlight, OUTPUT);   pinMode(leftDitch, OUTPUT);   pinMode(rightDitch, OUTPUT);

      if(DEBUG){ Serial.print("Headlight: "); Serial.println(headlight);}

      digitalWrite(headlight, LOW);  digitalWrite(rearlight, LOW);  digitalWrite(leftDitch, LOW);  digitalWrite(rightDitch, LOW); // Turn off lights
      digitalWrite(dirPin, forward);  // Set direction pin on motor driver to forward;  
		  engineDirection = forward;
      startedUp = true; 
    }
  }  

  else if (startedUp and !shuttingDown){  // Engine is on-line and knows its speedControlMethod

    switch (buf[opLoc])
    {  
      case shutDownEngine: 
        if(DEBUG){ Serial.print("shut down engine "); Serial.println(myAddress, HEX);}
        shuttingDown = true; 
        if(not schedulePowerChange(speedControlMethod, 0, 20)){   // Bring engine to a stop with 20% delta 
          reportTakeActionIssue(schedulingIssue);
        }  
        break;

      case goForward:
        if(DEBUG){ Serial.print("set to forward, engine ");  Serial.println(myEngineNumber.onePiece, HEX);}    
        engineDirection = forward;
        if(not schedulePowerChange(speedControlMethod, 0, 20)){   // Bring engine to a stop with 20% delta
          reportTakeActionIssue(schedulingIssue);
        }
        break; 

      case goRearward:
        if(DEBUG){ Serial.print("set to reverse, engine ");  Serial.println(myEngineNumber.onePiece, HEX);}        
        engineDirection = reverse;
        if(not schedulePowerChange(speedControlMethod, 0, 20)){   // Bring engine to a stop with 20% delta
          reportTakeActionIssue(schedulingIssue);
          }
        break;      

      case motorControl: 
        if(DEBUG){ Serial.print("update motor setting ");  Serial.println(myEngineNumber.onePiece, HEX);}
        if(not schedulePowerChange(speedControlMethod, buf[dataLoc], 20)){   // Update power with max 20% delta   
          reportTakeActionIssue(speedSettingIssue);
        }
        break;

      case updateCruiseValues:
        if(DEBUG){ Serial.print("update cruise values ");  Serial.println(myEngineNumber.onePiece, HEX);}    
        if ((bufLen - dataLoc) >= 24){
              Kp = buf[dataLoc];      // Incoming data is for the three PID control parameters for cruise,
              Ki = buf[dataLoc+1];    // plus 21 RPS values (divided by ten) for setting target observed
              Kd = buf[dataLoc+2];    // RPS values.
              for (byte i =1; i < 22; i++)
                RPS[i] = 10*buf[i + dataLoc +2];  // Save rotations per second indexed by increments of 5MPH
                                        // bytes received have been divided by ten to fit in a byte each for transmission.
              RPS[0] = 0;         // lower end sentinel for interpolation
              RPS[22] = RPS[21];  // upper end sentinel for interpolation
        }
        if (speedControlMethod == cruise) // Engine started up and in cruise -> need to refresh requested speed
                                          // since RPS values have just changed (could change desiredRPS value)
          if(not schedulePowerChange(speedControlMethod, lastRequestedSpeed, 20)){   // Update power with max 20% delta   
            reportTakeActionIssue(speedSettingIssue);
          }
        break;        

      case emergencyStop: 
        if(DEBUG){ Serial.print("emergency stop for engine "); Serial.println(myEngineNumber.onePiece, HEX);}
        if(not schedulePowerChange(speedControlMethod, 0, 50)){   // Set current percentage to zero  ASAP -- 50% delta
          reportTakeActionIssue(speedSettingIssue);
        }
        break;

      default:
        if(DEBUG){ Serial.println("everything else");}
        reportTakeActionIssue(unhandledCommand);
        break;
      }
    }

  else{
    reportTakeActionIssue(unhandledCommand);
  }

} // takeAction

void reportTakeActionIssue(uint8_t action ){
  uint8_t buflocal[3];
  buflocal[0] = engineModuleInformation;
  buflocal[1] = myAddress;  // Should be unnecessary since incoming msg had this value in buf[1]
  buflocal[2] = action;
  sendToMaster(buflocal, 3);
}


void sendToMaster(uint8_t toSend[], uint8_t sz){
//
//  If this send fails to get through, the engine should go into a fail stop mode.
//
  while (!manager.sendtoWait(toSend, sz, serverAddress)){
    if(DEBUG) Serial.println("Master not ACKing.");
    delay(1000);
    }
  // if(DEBUG){ 
  //   Serial.print("Just sent to master: " + String(toSend[0]) + "  ");
  //   Serial.print(toSend[1]);  Serial.print(toSend[2]);
  //   for (uint8_t i = 3; i < sz; i++){
  //     Serial.print("  " + String(toSend[i]));
  //   }
  //   Serial.println();
  // } 
} //sendToMaster

void sendStatus(uint8_t dir,uint8_t spdCtl, uint8_t spd, uint8_t readVolts, uint16_t desRPS, uint32_t obsRPS){
  uint8_t temp[10];
  uint16_t tempSmoothed;
// Send status to EngineConduit
  temp[0] = statusRpt;  // signals status message; 
  temp[1] = myAddress;
  temp[2] = dir;
  temp[3] = spdCtl;
  temp[4] = spd;
  temp[5] = readVolts;
  temp[6] = (desRPS & 0xFF);
  temp[7] = (desRPS & 0xFF00) >> 8;
  if (obsRPS > 65535){
     temp[8] = 0xFF; temp[9] = 0xFF;
     }
  else{
    tempSmoothed = obsRPS;
    temp[8] = (tempSmoothed & 0xFF);
    temp[9] = (tempSmoothed & 0xFF00) >> 8;
  }
  // if(DEBUG){ Serial.print("Sending status report ");  Serial.println(temp[1]);}
  sendToMaster(temp, sizeof(temp));
} //sendStatus


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (timerInterrupt){ // TCC2 one second interrupt handler was called, (and perhaps pulse counter on D12 collected.)
	  timerInterrupt = false;   //  Clear one second interrupt (TCC2) flag;

    if (!DEBUG and (lowVoltage && not beenToldToPowerDown)){
      buf[1] = myAddress;   
      buf[0] = shutDownEngine;             // Initiate shutdown
      takeAction(buf, 2);  
      beenToldToPowerDown = true;     //  Trigger eventual power off after shutdown
    }

  // Is it time for a status report?
	  if (statusReportCounter >0)
		  statusReportCounter--;  // Ticking down to send time
      else { // Yes, time to issue a status report
        statusReportCounter = statusReportFreq;
          // Special case status report when power is low...
          //////  These messages will only be sent until engine power is ultimately cut off
        if (lowVoltage || beenToldToPowerDown){
          buf[1] = myAddress;   
          buf[0] = engineModuleInformation;
          buf[2] = batteryProtectionMode;
          buf[3] = smoothedVoltage;
          sendToMaster(buf, 4);
        }
              // send normal status report
        if (speedControlMethod == percent)
            sendStatus(engineDirection, speedControlMethod, currentPercentage, smoothedVoltage, 0, smoothedRPS);
        else if(speedControlMethod == cruise)
            sendStatus(engineDirection, speedControlMethod, speedFromRPS(), smoothedVoltage, desiredRPS, smoothedRPS);
          else  // speedControlMethod is unknown ... not set yet.
            sendStatus(engineDirection, speedControlMethod, 0, smoothedVoltage, 0, smoothedRPS);
      }

    if(startedUp){  // Startup being true ensures engiineDirection is forward or reverse
      if(currentPercentage == 0){  
        digitalWrite(dirPin, engineDirection == forward);  // Set direction pin on motor driver; this action is idempotent;
        digitalWrite(headlight, engineDirection == forward);  // Set headlights according to
        digitalWrite(rearlight, engineDirection == reverse);  // engine direction.
        digitalWrite(leftDitch, LOW);  // At zero motion, ditch lights
        digitalWrite(rightDitch, LOW);  //   should be off.
      }
      else 
        if (engineDirection == forward){  // non-zero forward motion -> ditch lights operating
          digitalWrite(leftDitch, ditchLightToggle);  // Toggle
          digitalWrite(rightDitch, 1-ditchLightToggle);  // ditch lights
          ditchLightToggle = 1-ditchLightToggle;
        }
      // Modify motor power settings to establish desired power/speed outcomes.  percent and cruise are only
            // possibilities since unknown was captured above.
      if (speedControlMethod == percent) updatePercentSetting();
      else if (speedControlMethod == cruise ) updateSpeedSetting();
      } // End, if started up  
                  
    if(startedUp && shuttingDown && (currentPercentage == 0) ) {  // Time for going into shutdown state?
      startedUp = false;      // Together define
      shuttingDown = false;   // shutdown state
      engineDirection = dirUnknown;
      digitalWrite(headlight, LOW);  // Turn off headlight forward
      digitalWrite(rearlight, LOW);  // Turn off headlight reverse
      digitalWrite(leftDitch, LOW);  // At zero speed, ditch lights
      digitalWrite(rightDitch, LOW);  //   should be off.
      if (speedControlMethod == percent)
          sendStatus(engineDirection, speedControlMethod, currentPercentage, smoothedVoltage, 0, smoothedRPS);
      else 
          sendStatus(engineDirection, speedControlMethod, speedFromRPS(), smoothedVoltage, desiredRPS, smoothedRPS);
    }

  } // End timer interrupt based reactions.

  
    if ((beenToldToRestart || beenToldToPowerDown) && !startedUp && !shuttingDown && (currentPercentage == 0)){
      if (beenToldToRestart){
          if (DEBUG) Serial.println(" Been told to restart.");
          speedControlMethod = unknown;
          getRadioDataGramAddress();
      }
      else if (beenToldToPowerDown){  // Engine idle and been told to cut power off
          if (DEBUG) Serial.println(" Been told to power down.");
          while(1){
            digitalWrite(A2, LOW);  // This should shut everything down at soft switch.
            delay(1000);
          }
      }
    }
    else  // Look for a new command from engineMaster
      if (manager.recvfromAckTimeout(buf, &len, 400, &from)){   // New msg has come in
          if(DEBUG){
            Serial.print("\ngot message: [");  Serial.print(buf[0]);   Serial.print("] " ); Serial.print(myAddress);
            Serial.print(";\t\tLast RSSI: ");
            Serial.println(rf69.lastRssi(), DEC);
          }
          if(buf[0] == engineReset || buf[0] == resetAll){
            if(not beenToldToPowerDown){
              buf[0] = shutDownEngine;             // Initiate shutdown before doing reset
              buf[1] = myAddress;
              takeAction(buf, 2);  // After this the engine may still be started up, but shutting down will be true for sure.
            }
            beenToldToRestart = true;   // This will lead to resetting engine module to initial state;
          }

          if(buf[0] == setPowerToOff){  // higher level control is commanding power off conditions.
              buf[0] = shutDownEngine;             // Initiate shutdown
              buf[1] = myAddress;
              takeAction(buf, 2);  // After this the engine may still be started up, but shutting down will be true for sure.
              beenToldToPowerDown = true; // Eventually leads to alive pin (A2) being set low
          }
          else takeAction(buf, len);  //  Most commands coming from engineConduit will be acted on here.
      }
  
}  // loop








//  Useful online comments about interrupts and timers

// 2025 04 24 Martin L code (well known Arduino commenter)
//  


//     Can I put D12 on D6? (D2..D9, A0..A5) would be free. The pin probably works with other interrupts, channels, EIC etc.
//  I am not familiar with the settings. What changes do I have to make?  
//     Yes, it's possible to route any interrupt pin
// through to any timer, via the event system.   The information for what interrupt channel to use can be found from
// Arduino Zero schematic and the SAMD21's Port Multiplexing table in section 7.

// From the schematic (on sheet 2) it's possible to find the port pin for D6, it PA20.
// In the port multiplexing table, row PA20, column A, the external interrupt is on channel 4: EXTINT[4]. As PA20 is
// an even port pin, we select the multiplexer switch for position A (EIC) on the even port: PORT_PMUX_PMUXE_A.

// Just change the following lines of code and the input should work on D6:

  // PORT->Group[PORTA].PINCFG[20].bit.PMUXEN = 1;                            // Enable the port multiplexer on port pin PA20 (D6)
  // PORT->Group[PORTA].PMUX[20 >> 1].reg |= PORT_PMUX_PMUXE_A;               // Set-up PA20 (D6) as an EIC (interrupt)
 
  // EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO4;                                 // Enable event output on external interrupt 4
  // EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE4_HIGH;                            // Set event detecting a HIGH level
  // EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT4;                                // Clear the interrupt flag on channel 4
  // EIC->CTRL.bit.ENABLE = 1;                                                // Enable EIC peripheral
  // while (EIC->STATUS.bit.SYNCBUSY);                                        // Wait for synchronization
 
  // EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                    // EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);                // Set the event user (receiver) as timer TC4 event
                   
  // EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection
                       // EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous                     
                       // EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_4) |    // Set event generator (sender) as external interrupt 4
                       // EVSYS_CHANNEL_CHANNEL(0); 
