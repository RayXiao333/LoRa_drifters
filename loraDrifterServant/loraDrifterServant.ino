#include "src/loraDrifterLibs/loraDrifter.h"

// F. Functions
void startWebServer(const bool webServerOn);
String IpAddress2String(const IPAddress& ipAddress);

SPIClass SDSPI(HSPI);

// =======================================================================================
// A. Global variables
// =======================================================================================
TinyGPSPlus gps;
String drifterName = "D08";   // ID send with packet
int drifterTimeSlotSec = 5; // seconds after start of each GPS minute
int nSamplesFileWrite = 300;      // Number of samples to store in memory before file write
const char* ssid = "DrifterServant";   // Wifi ssid and password
const char* password = "Tracker1";
String hour, minute, second, year, month, day, tTime, tDate;
String csvOutStr = "";                 // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);
bool webServerOn = false;
String csvFileName = "";
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
int webServerPin = BUTTON_PIN;
int gpsLastSecond = -1;

int ledState = LED_ON;
int ledPin = BOARD_LED;

// H. This is the string literal for the main web page

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <title>UWA LoRa Drifters</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <link rel="icon" href="data:,">
      <style>
        html {
          font-family: Arial; display: inline-block; text-align: center;
        }
        h2 {
          font-size: 3.0rem;
        }
        p {
          font-size: 3.0rem;
        }
        table, th, td {
           border: 1px solid black;
        }
        body {
          max-width: 700px; margin:0px auto; padding-bottom: 25px;
        }
      </style>
    </head>
    <body>
      <h2>LoRa Drifters</h2>
      <h4>Servant Node</h4>
      <table>
        <tr>
          <td>Filename</td>
          <td>Get Data Link</td>
          <td>Last File Write GPS Time</td>
          <td>Erase Data (NO WARNING)</td>
        </tr>
      %SERVANT%
      </table>
      <br><br>
      <h4>Configuration</h4>
      <form action="/configure" method="get">
        <table>
          <tr>
            <td>Setting</td>
            <td>Current Values</td>
            <td>New Values</td>
            <td>Guidance</td>
          </tr>
          <tr>
            <td><label for="fname">Drifter ID:</label></td>
            <td>%DRIFTERID%</td>
            <td><input type="text" id="fname" name="drifterID"></td>
            <td>Drifter IDs from D00 to D11</td>
          </tr>
          <tr>
            <td><label for="lname">LoRa Sending Second:</label> </td>
            <td>%LORASENDSEC%</td>
            <td><input type="text" id="lname" name="loraSendSec"></td>
            <td>Sending second is from 0 to 59 seconds</td>
          </tr>
        </table>
        <input type="submit" value="Configure">
      </form>
    </body>
  </html>
)rawliteral";

// =======================================================================================
// B. Setup
// =======================================================================================

void setup() {
  // A. TTGO Power ups and init radio
  initBoard();
  delay(50);
  
  // B. Setup LEDs for information   
  // Need to setup this part
  // pinMode(BOARD_LED, OUTPUT);
  // digitalWrite(ledPin, ledState);     // will change state when a LoRa packet is received
  pinMode(webServerPin, INPUT);

  // C. Local GPS
  // Started inside initBoard()

  //Setup LoRa
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if(!LoRa.begin(LoRa_frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  delay(50);
  
  // G. SPIFFS to write data to onboard Flash
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS - need to add retry");
    while(1);
  }
  delay(50);

  //H. Read config file if exists
  file = SPIFFS.open("/config.txt", FILE_READ);
  if(!file) {
    Serial.println("Failed to open config.txt configuration file");
  } else {
    String inData = file.readStringUntil('\n');
    int comma = inData.indexOf(",");
    drifterName = inData.substring(0, comma);
    drifterTimeSlotSec = inData.substring(comma + 1).toInt();
    Serial.println(inData);
  }
  delay(50);

  csvFileName="/svt" + String(drifterName)+".csv";
  webServerOn = false;
  Serial.println("init ok");
  delay(50);
}

// =======================================================================================
// C. Loop
// =======================================================================================
void loop() {
  // A. Check for button press
  if(digitalRead(BUTTON_PIN) == LOW) {
    if(webServerOn) {
      Serial.println("Web server already started");
      webServerOn = false;
      startWebServer(webServerOn);
      delay(1000);
    } else {
      webServerOn = true;
      startWebServer(webServerOn);
      Serial.println("Web server started");
      delay(1000);
    }
  }

  if(!webServerOn) {
    // A. Receive and Decode GPS data and send via LoRa if in time slot 
    SerialGPSDecode(Serial1, gps);
    delay(10);
    
    // B. Write data to onboard flash if nSamples is large enough
    // Serial.println("nSamples:" + String(nSamples));
    if(nSamples >= nSamplesFileWrite) {  // only write after collecting a good number of samples
      Serial.println("Dump data into the memory");
      writeData2Flash();
    }
  } else {
    Serial.println("Web server is ON, not GPS data or saving during the time");
    digitalWrite(BOARD_LED, HIGH);
    delay(40);
    digitalWrite(BOARD_LED, LOW); 
    delay(40);
  }
}

// =======================================================================================
// D. Functions
// =======================================================================================

// D0. Write data to flash
void writeData2Flash (){
  file = SPIFFS.open(csvFileName, FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
    ESP.restart();
  } else {
    if(file.println(csvOutStr)) {
      Serial.println("Wrote data in file, current size: ");
      Serial.println(file.size());
      csvOutStr = ""; 
      nSamples = 0;
      lastFileWrite = tTime;
    } else {
      lastFileWrite = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS) {
  Packet packet;
  // Read GPS and run decoder
  unsigned long start = millis();
  do {
    while(Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  } while(millis() - start < 500);
  // C. If this is a new GPS record then save it
  if(gps.time.second() != gpsLastSecond) {
    strcpy(packet.name, drifterName.c_str());
    packet.drifterTimeSlotSec = drifterTimeSlotSec;
    packet.hour = gps.time.hour();
    packet.minute = gps.time.minute();
    packet.second = gps.time.second();
    packet.year = gps.date.year();
    packet.month = gps.date.month();
    packet.day = gps.date.day();
    packet.lng = gps.location.lng();
    packet.lat = gps.location.lat();
    packet.nSamples = nSamples;
    packet.age = gps.location.age();

    // hour = String(gps.time.hour());
    // minute = String(gps.time.minute());
    // second = String(gps.time.second());
    // year = String(gps.date.year());
    // month = String(gps.date.month());
    // day = String(gps.date.day());
    // if(hour.length() == 1) {
    //     hour = "0" + hour;
    // }
    // if(minute.length() == 1) {
    //     minute = "0" + minute;
    // }
    // if(second.length() == 1) {
    //     second = "0" + second;
    // }
    // if(month.length() == 1) {
    //     month = "0" + month;
    // }
    // if(day.length() == 1) {
    //     day = "0" + day;
    // }
    // tDate = year + "-" + month + "-" + day;
    // tTime = hour + ":" + minute + ":" + second;
    // String tLocation = String(gps.location.lng(), 8) + "," + String(gps.location.lat(), 8) + "," + String(gps.location.age());
    // String sendPacket = String(drifterName) + "," + String(drifterTimeSlotSec) + "," + tDate + "," + tTime + "," + tLocation + "," + String(nSamples) + "\n";
    gpsLastSecond = gps.time.second();
    
    if((gps.location.lng() != 0.0) && (gps.location.age() < 1000)) {
      // csvOutStr += tDate + "," + tTime + "," + tLocation + "\n";
      nSamples += 1;
      // B. Send GPS data on LoRa if it is this units timeslot
      if(gps.time.second() == drifterTimeSlotSec) {
        Serial.println("Sending packet via LoRa");
        // Serial.println(sendPacket);
        // csvOutStr += sendPacket; // Save any packets that are sent (debugging purposes).
        LoRa.beginPacket();
        LoRa.write((const uint8_t*)&packet, sizeof(packet));
        LoRa.endPacket();
        delay(50); // Don't send more than 1 packet
      }
    } else {
      Serial.println("NO GPS FIX, NOT SENDING OR WRITING");
    }
  }
  delay(50);
}

// D3. Web Server Setup
void startWebServer(const bool webServerOn) {
  if(!webServerOn) {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
  } else {
    WiFi.softAP(ssid, password);
    Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

    // F. Web Server Callbacks setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/configure", HTTP_GET, [](AsyncWebServerRequest * request) {
      int paramsNr = request->params();
      Serial.println(paramsNr);
      for(int ii = 0; ii < paramsNr; ii++) {
        AsyncWebParameter* p = request->getParam(ii);
        if(p->name() == "drifterID") {
          drifterName = p->value();
        }
        if(p->name() == "loraSendSec") {
          drifterTimeSlotSec = String(p->value()).toInt();
        }
      }
      csvFileName="/svt" + String(drifterName) + ".csv";
      
      file = SPIFFS.open("/config.txt", FILE_WRITE);
      if(!file) {
        Serial.println("Could not open config.txt for writing");
        request->send(200, "text/plain", "Failed writing configuration file config.txt!");
      } else {
        file.print(drifterName + "," + String(drifterTimeSlotSec));
        file.close();
        request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
      }
    });
    
    server.on("/getServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      writeData2Flash();
      request->send(SPIFFS, csvFileName, "text/plain", true);
    });
    
    server.on("/deleteServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      SPIFFS.remove(csvFileName);
      file = SPIFFS.open(csvFileName, FILE_WRITE);
      if(!file) {
        Serial.println("There was an error opening the file for writing");
        return;
      }
      if(file.println("#FILE ERASED at " + lastFileWrite)) {
        Serial.println("File was created");
      } else {
        Serial.println("File creation failed");
      }
      file.close();
      lastFileWrite = "";
      request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
    });
    server.begin();
  }
}

// D4. Used to update sections of the webpages
// Replaces placeholder with button section in your web page

String processor(const String& var) {
  if(var == "SERVANT") {
     String servantData = "";
     servantData += "<td>" + csvFileName + "</td>";
     servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getServant\"> GET </a></td>";
     servantData += "<td>" + lastFileWrite + "</td>";
     servantData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteServant\"> ERASE </a></td>";
     servantData += "</tr>";
     return servantData;
  }
  if(var == "DRIFTERID") {
    return drifterName;
  }
  if(var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
  return String();
}
