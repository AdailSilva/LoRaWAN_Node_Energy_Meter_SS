/* 
 *   
 *  Project:          IoT Energy Meter with C/C++, Java/Spring, TypeScript/Angular and Dart/Flutter;
 *  About:            End-to-end implementation of a LoRaWAN network for monitoring electrical quantities;
 *  Version:          1.0;
 *  Backend Mote:     ATmega328P/ESP32/ESP8266/ESP8285/STM32;
 *  Radios:           RFM95w and LoRaWAN EndDevice Radioenge Module: RD49C;
 *  Sensors:          Peacefair PZEM-004T 3.0 Version TTL-RTU kWh Meter;
 *  Backend API:      Java with Framework: Spring Boot;
 *  LoRaWAN Stack:    LoRaMac-node Library (LoRaWAN L2 1.0.3 - Released / API via AT commands);
 *  Activation mode:  Activation by Personalization (ABP) or Over-the-Air Activation (OTAA);
 *  Author:           Adail dos Santos Silva
 *  E-mail:           adail101@hotmail.com
 *  WhatsApp:         +55 89 9 9433-7661
 *  
 *  This project was conceived using the LoRaWAN library,
 *  whose copyright belongs to the RD49C module manufacturer, the company Radioenge from Brazil.
 *  All the remaining implementation is authored by the creator of the LoRaWAN Electricity Meter project.
 *  
 *  WARNINGS:
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the “Software”), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 */

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

/* Includes */
#include "LoRaWAN.h"
#include <SoftwareSerial.h>
#include <stdint.h>

/* SoftwareSerial handles */
SoftwareSerial *hSerialCommands = NULL;


/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

/* Keys for AES-128 Encryption */
//// CHIRPSTACK - CS:
char DEVEUI[] = "00:12:F8:00:00:00:0D:4E"; /* MSB ( 0012F80000000D4E ) */
char APPEUI[] = "00:00:00:00:00:00:00:00"; /* MSB or LSB (Zerada para ChirpStack NetworkServer) ( 0000000000000000 ) */
char APPKEY[] = "c9:7e:a4:d2:05:fd:7d:86:77:ad:94:d1:0d:a4:80:65"; /* MSB */
//char APPKEY[] = "65:80:a4:0d:d1:94:ad:77:86:7d:fd:05:d2:a4:7e:c9"; /* LSB */
//
// or:
//
// EVERYNET - ATC:
//char DEVEUI[] = "00:12:F8:00:00:00:0D:4E"; /* MSB ( 0012F80000000D4E ) */
//char APPEUI[] = "00:00:00:00:00:00:00:00"; /* MSB or LSB ( 0000000000000000 ) */
////char APPKEY[] = "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00"; /* MSB ( 00000000000000000000000000000000 ) */
//char APPKEY[] = "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00"; /* LSB ( 00000000000000000000000000000000 ) */
//
// or:
//
// THE THINGS NETWORK - TTN:
//char DEVEUI[] = "00:12:F8:00:00:00:0D:4E"; /* MSB ( 0012F80000000D4E ) */
//char APPEUI[] = "70:B3:D5:7E:D0:03:17:01"; /* MSB ( 70B3D57ED0031701 ) */
//char APPKEY[] = "A9:CC:11:21:AE:D0:76:F7:9D:05:58:BD:67:A5:50:C6"; /* MSB ( A9CC1121AED076F79D0558BD67A550C6 ) */
//char APPKEY[] = "C6:50:A5:67:BD:58:05:9D:F7:76:D0:AE:21:11:CC:A9"; /* LSB ( C650A567BD58059DF776D0AE2111CCA9 ) */


/* Masks according to the frequency plan adopted, in this AU915-928 (LA915-928A) project. */
// CHIRPSTACK - CS:
//char CHMASK[] = "FF00:0000:0000:0000:0002:0000"; // Second Option (Channels: 8 at 15 + 65).
// "00FF:0000:0000:0000:0001:0000"; // Mask to Network Server: ChirpStack - CS.
// Channels: 0 at 7 + 64 (915.2 MHz - 916.6 MHz - 927.8 MHz)
// and:
// "FF00:0000:0000:0000:0002:0000"
// Channels: 8 at 15 + 65 (916.8 MHz - 918.2 MHz - 915.9 MHz)
//
// or:
//
// EVERYNET - ATC:
//char CHMASK[] = "00FF:0000:0000:0000:0001:0000";
// "00FF:0000:0000:0000:0001:0000"; // Mask to Network Server: Everynet - ATC.
// Channels: 0 at  7 (915.2 MHz - 916.6 MHz - 000.00 MHz)
//
// or:
//
// THE THINGS NETWORK - TTN:
char CHMASK[] = "FF00:0000:0000:0000:0002:0000";
// "FF00:0000:0000:0000:0002:0000"; // Mask to Network Server: The Things Network - TTN.
// Channels: 8 at 15 + 65 (916.8 MHz - 918.2 MHz - 915.9 MHz)


char str_counter[128];
int counter = 0;


/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup()
{

    Serial.begin(115200); /* Initialize monitor serial */
    Serial.println(F("Initializing the relevant settings..."));
    delay(5000);

    /* Initialize SoftwareSerial */
    /* Arduino UNO */
    hSerialCommands = SerialCommandsInit(7, 6, 9600); // RX, TX, BaudRate.
    
    /* ESPDuino-32 */
    //hSerialCommands = SerialCommandsInit(14, 27, 9600); // RX, TX, BaudRate.

    SendRaw("ATZ\r\n");
    Serial.println(F("Reset module..."));
    delay(5000);

    SendRaw("AT+AJOIN=0\r\n");
    Serial.println(F("Configurando Auto Join >> in Off <<"));
    delay(1000);

    SendRaw("AT+DR=5\r\n");
    Serial.println(F("Configurando ADR para 5 = SF7"));
    delay(1000);

    /* Configure the EndDevice as OTAA */
    InitializeOTAA(APPKEY, APPEUI);
    SendAtCommand(AT_CHMASK, AtSet, CHMASK);
    delay(1000);

    /* Joinning the EndDevice as OTAA */
    //SendRaw("AT+JOIN\r\n");
    Serial.println(F("AT+JOIN (Tentando Ingressar em uma Rede LoRaWAN...)"));
    //delay(1000);

    if (CHMASK != 0){
      Serial.print(F("Channel Mask: "));
      Serial.println(CHMASK);
      }

    String mask = CHMASK;
    String networkServer = "";

    if (mask == "00FF:0000:0000:0000:0000:0000") /* Channels: 0 a 7 */
    {
        networkServer = "NetworkServer Everynet.";
        Serial.print(F("Everynet."));        
    }
    else if (mask == "00FF:0000:0000:0000:0001:0000") /* Channels: 0 a 7 + 64 */
    {
        networkServer = "ChirpStack.";
        Serial.print(F("NetworkServer ChirpStack."));
    }
    else if (mask ==  "FF00:0000:0000:0000:0002:0000") /* Channels: 8 at 15 + 65 */
    {
        networkServer = "NetworkServer ChirpStack - CS AND/OR The Things Network - TTN.";

        Serial.print(F("Sending JOIN --> "));
        Serial.println(networkServer);

        SendAtCommand(AT_JOIN, AtGet, NULL);
        if (JoinNetwork(0) == RAD_OK)
        {
            Serial.println(F("EndDevice has joined sucessfully. \\('~')/"));
        }
        else
        {
            Serial.println(F("Error joining the network. :-/"));
        }
    }
}


/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop()
{
    /* Sends a string containing a counter every 10s */
    
    Serial.println();
    Serial.println(counter);
    Serial.println(F("Counter"));
    
    sprintf(str_counter, "{\"variable_1\":\"counter\",\"value\":%d} \r\n\0", counter++);
    Serial.println(str_counter);
    
    Serial.println(F("##### ##### ##### ##### ##### #####"));

    /* Send to NetworServer */
    SendString(str_counter, 101);
    delay(5000); /* O tempo aqui setado é acrescentado a 5 segundos. */
}
