#include <stdio.h>
#include "LoRa.h"

#define WITH_APPKEY

unsigned int freq = 865200000;//HZ
unsigned int id_node=233;
unsigned int id_data1=543;
unsigned int id_data2=544;

unsigned short id_frame = 0;

#ifdef WITH_APPKEY
uint8_t my_appKey[4]={5, 6, 7, 8};
#endif

float information1;
float information2;

char message[100];

void setup(){
  Serial.println("Set LoRa modulation\r\n");
  Serial.println("Set LoRa\r\n");
  

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


void loop()
{
  //making the LoRa frame
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

      //Adding data to the LoRa frame
      sprintf(final_str, "%s#%d/%2.4f", final_str, id_data1, information1);
      sprintf(final_str, "%s#%d/%2.4f", final_str, id_data2, information2);
      
      r_size=sprintf(message+app_key_offset,final_str);
      startSend=millis();
      Serial.println(message);

      //start sending the LoRa frame
      LoRa.beginPacket();
      LoRa.print(message);
      LoRa.endPacket();
      Serial.println("Switch to power saving mode\n");
      LoRa.sleep();
      delay(10000); // wait for 10s  
}
}