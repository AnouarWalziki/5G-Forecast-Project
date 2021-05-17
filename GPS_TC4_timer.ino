#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

void setup() {
  PORT->Group[PORTA].DIRSET.reg = PORT_PA21;               // Set D7 as a digital output
  
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |                 // Enable GCLK0 for TC4 and TC5
        GCLK_CLKCTRL_GEN_GCLK0 |             // Select GCLK0 at 48MHz
        GCLK_CLKCTRL_ID_TC4_TC5;             // Feed GCLK0 output to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);                       // Wait for synchronization

  TC4->COUNT16.CC[0].reg = 29999;                          // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;             // Enable TC4 overflow (OVF) interrupts

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC |     // Reset timer on the next prescaler clock
        TC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 1, 48MHz/1 = 48MHz
        TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode 
        TC_CTRLA_MODE_COUNT16;         // Set the timer to 16-bit mode      
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable the TC4 timer
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization
}

void loop() {}

void TC4_Handler()                                         // Interrupt Service Routine (ISR) for timer TC4
{      
  //if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)       // Optionally check for overflow (OVF) interrupt      
  //{   
  PORT->Group[PORTA].OUTTGL.reg = PORT_PA21;             // Toggle the D7 output
  TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;             // Clear the OVF interrupt flag
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Fix: ");
    Serial.print(GPS.fix);
    Serial.print("\n");
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 4);
      Serial.print("\n");
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("\n");
    }
  }
}
  //}
}