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
Serial.println("Adafruit GPS library basic parsing test!");

// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
GPS.begin(9600);
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {
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