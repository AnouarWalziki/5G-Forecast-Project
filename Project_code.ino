#include <stdio.h>
#include "LoRa.h"
#include <Adafruit_GPS.h>

#define PIN_ANALOGIQUE 15 // 15 => pin A0
#define RESOLUTION 12 // ADC 12 bits => 4096 values
#define PERIODE_CAN_MS 10 // période d'écahtillonage (en négligeant les traitements)
#define NOMBRE_MOYENNE 1000


// Cartographie du capteur et étalonage
#define NBR_POINT 10
double  carto_dBm[NBR_POINT] = {   -30,   -25,   -20,  -15,   -10,    -5,    0,    5,   10,  12}; // correspond à l'axe y de la cartographie
double carto_Volt[NBR_POINT] = { 0.124, 0.129, 0.146, 0.19, 0.315, 0.543, 1.05, 1.94, 3.29, 3.3}; // @3.6GHz, correpond à l'axe x de la cartographie // carte connecteur SMA

#define GAIN_ANTENNE 32.74 // rapport dBm -> dBm/m


#define VISTESSE_USB 115200 // vitesse de l'USB

#define LED_PIN 5

// Variables
unsigned short int i; // Multifunction
unsigned short int moyenne = 1; // Count the number of averages
unsigned short int resultCAN = 0; // The result of A/D convertion
double mesure = 0; // The result of to Volts convertion
double mesure_moy = 0; // Averaged result
double mesure_max = 0; // Result max
double puissance_dBm = 0; // The power in dBm before the antenna
double puissance_dBm_max = 0;
double puissance = 0; // The power in W/m2 in the air
double puissance_max = 0;
double champElectrique = 0; // The electric field in V/m in the air
double champElectrique_max = 0;

// Connect to the GPS on the hardware port
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();

float fakeResult=-200;
int flag =0;

//LoRa variables
#define WITH_APPKEY
unsigned int freq = 865200000;//HZ
unsigned int id_node=233;
unsigned int id_champ_electrique_max=543;
unsigned int id_champ_electrique=544;
unsigned int id_latitude=545;
unsigned int id_longitude=546;
unsigned short id_frame = 0;
#ifdef WITH_APPKEY
uint8_t my_appKey[4]={5, 6, 7, 8};
#endif
char message[100];

//Location variables
float longitude = 0;
float latitude = 0;


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  analogReadResolution(RESOLUTION); // configuration du CAN
  
  Serial.begin(115200);
  
  // Set timer TC4 to call the TC4_Handler every 0.625 ms: (1 * (29999 + 1)) / 48MHz = 0.625ms
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

  
  //-----------GPS Setup----------------//

  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  
  //-----------LORA Setup----------------//
  
  if (!LoRa.begin(freq)) {
   Serial.println("Starting LoRa failed!");
   while (1); 
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN); 
}


void loop() // run over and over again
{
    if (GPS.fix)
    {
        //----------Loop donnée de champ--------------//
      
        // reset of measurement variables and number of averages
        moyenne = 1;
        mesure_moy = 0;
        mesure_max = 0;
        
        while(1)
        {
            // time delay before new A/D conversion
            delay(PERIODE_CAN_MS);
            
            resultCAN = analogRead(PIN_ANALOGIQUE); // Conversion A/D
    
            mesure = (double)resultCAN * 3.3 / 4095; // conversion to Volts
    
            if (mesure > mesure_max)
            {
                mesure_max = mesure;
            }
    
            mesure_moy = (mesure_moy*(moyenne-1) + mesure) / (moyenne) ; //  measurements average
    
            moyenne++;
            if (moyenne >= NOMBRE_MOYENNE)
            {
                break;
            }
        }  
        
        for (i = 0 ; i < NBR_POINT ; i++){ // mapping scan
          if (mesure_moy <= carto_Volt [i]) 
          {
            break;
          }
        }
    
        if (i == 0){ // linear interpolation approximation of the mapping
          puissance_dBm = carto_dBm[0];
        } else {
          puissance_dBm = (mesure_moy - carto_Volt[i-1]) * ( (carto_dBm[i]-carto_dBm[i-1]) / (carto_Volt[i]-carto_Volt[i-1]) ) + carto_dBm[i-1];
        }
        
        for (i = 0 ; i < NBR_POINT ; i++){ // mapping scan
          if (mesure_max <= carto_Volt [i]) {
            break;
          }
        }
    
        if (i == 0){ // linear interpolation approximation of the mapping
          puissance_dBm_max = carto_dBm[0];
        } else {
          puissance_dBm_max = (mesure_max - carto_Volt[i-1]) * ( (carto_dBm[i]-carto_dBm[i-1]) / (carto_Volt[i]-carto_Volt[i-1]) ) + carto_dBm[i-1];
        }
        
        // Calculation of electromagnetic quantities
        puissance = pow (10, (puissance_dBm - 30 + GAIN_ANTENNE) / 10);
        puissance_max = pow (10, (puissance_dBm_max - 30 + GAIN_ANTENNE) / 10);
        champElectrique = sqrt(puissance * 377);
        champElectrique_max = sqrt(puissance_max * 377);
    
        // display of results on the serial port (computer terminal)
        Serial.print(resultCAN); //this value is not averaged
        Serial.print(" quantum\n");
        Serial.print(mesure, 3);
        Serial.print(" V\n");
        if ((puissance_dBm < -28.5) && (puissance_dBm_max < -24.5)){
          Serial.print("< -28.5 dBm  /!\\ puissance trop faible\n");
          Serial.print("< 2.65 mW/m2  /!\\ Puissance trop faible\n");
          Serial.print("< 1 V/m    /!\\ Champ trop faible\n");
          Serial.print("\n");
        } else if (puissance > 5) {
          Serial.print(puissance_dBm, 3);
          Serial.print(" dBm   /!\\ puissance DANGEREUSE NE PAS DÉPASSER 12 dBm /!\\\n");
          Serial.print(puissance*1000, 3);
          Serial.print(" mW/m2  /!\\ PUISSANCE DANGEREUSE /!\\\n");
          Serial.print(champElectrique, 3);
          Serial.print(" V/m  /!\\ CHAMP DANGEREUX /!\\\n");
          Serial.print("\n");
        } else {
          Serial.print(puissance_dBm, 3);
          Serial.print(" dBm (max : ");
          Serial.print(puissance_dBm_max, 3);
          Serial.print(")\n");
          Serial.print(puissance*1000, 3);
          Serial.print(" mW/m2 (max : ");
          Serial.print(puissance_max*1000, 3);
          Serial.print(")\n");
          Serial.print(champElectrique, 3);
          Serial.print(" V/m  (max : ");
          Serial.print(champElectrique_max, 3);
          Serial.print(")\n");
          Serial.print("\n");
        }
        
    
        //-----------LORA loop----------------//
      
        long startSend;
        long endSend;
        uint8_t app_key_offset = 0;
        int e;
      
        #ifdef WITH_APPKEY
              app_key_offset = sizeof(my_appKey);
              memcpy(message,my_appKey,app_key_offset);
        #endif
              uint8_t r_size;
              char final_str[80] = "\\";
              char aux[6] = "";
              char id[1] = "";
        sprintf(final_str, "%s!%i!%hd", final_str,id_node, id_frame++);      
    
        //adding 5G electric field value to the LoRa frame
    
        fakeResult=-20;
         
        if (flag ==0)
        {
            flag=1;   
            sprintf(final_str, "%s#%d/%2.4f", final_str, id_champ_electrique, fakeResult); //sending fake data
    
            //adding longitude data to the LoRa frame
            sprintf(final_str, "%s#%d/%2.4f", final_str, id_longitude, longitude);
    
            //adding latitude data to the LoRa frame
            sprintf(final_str, "%s#%d/%2.4f", final_str, id_latitude, latitude);
    
            r_size=sprintf(message+app_key_offset,final_str);
            startSend=millis();
            Serial.println(message);
            
            LoRa.beginPacket();
            LoRa.print(message);
            LoRa.endPacket();
            Serial.println("Switch to puissance saving mode\n");
            LoRa.sleep();
        }
        else
        {
            flag=0;
            
            sprintf(final_str, "%s#%d/%2.4f", final_str, id_champ_electrique, champElectrique); //adding elctromagnetic field value to the LoRa frame
            sprintf(final_str, "%s#%d/%2.4f", final_str, id_longitude, longitude);//adding longitude data to the LoRa frame
            sprintf(final_str, "%s#%d/%2.4f", final_str, id_latitude, latitude); //adding latitude data to the LoRa frame
        
            r_size=sprintf(message+app_key_offset,final_str);
            startSend=millis();
            Serial.println(message);
            
            LoRa.beginPacket();
            LoRa.print(message);
            LoRa.endPacket();
            Serial.println("Switch to power saving mode\n");
            LoRa.sleep();
        }  
    }
    else
    {
    }
}
void TC4_Handler()                                         // Interrupt Service Routine (ISR) for timer TC4
{      
  //if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)       // Optionally check for overflow (OVF) interrupt      
  //{   
    PORT->Group[PORTA].OUTTGL.reg = PORT_PA21;             // Toggle the D7 output
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;             // Clear the OVF interrupt flag
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
    if (millis() - timer > 2000) 
    {
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
          longitude=GPS.longitudeDegrees;
          latitude=GPS.latitudeDegrees;
        }
   //  }
  }

}
