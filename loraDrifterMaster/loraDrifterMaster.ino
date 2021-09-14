#include "loraDrifterMaster.h"

// =======================================================================================
// A. Global variables
// =======================================================================================
TinyGPSPlus gps;                      // decoder for GPS stream
const char* ssid = "DrifterMaster";   // Wifi ssid and password
const char* password = "Tracker1";
Master m;                             // Master data
Servant s[nServantsMax];              // Servants data array
String masterData = "";               // Strings for tabular data output to web page
String servantsData = "";
String csvOutStr = "";                // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);            // Create AsyncWebServer object on port 80
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
//int ledState = LOW;
//int ledPin = 14;
int gpsLastSecond = -1;
int webServerPin = BUTTON_PIN;

// =======================================================================================
// B. Setup
// =======================================================================================

void setup() {
  // A. Init Board
  initBoard();
  delay(500);
  
  // B. Setup LEDs for information
  // pinMode(ledPin, OUTPUT);
  // digitalWrite(ledPin, ledState);     // will change state when a LoRa packet is received
  // pinMode(BUTTON_PIN, INPUT);

  // C. Local GPS
  // moved inside initBoard();

  // D. LoRa Setup
  // moved inside initBoard();
  LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DI0_PIN);
  if(!LoRa.begin(LoRa_frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  // register the receive callback
  LoRa.onReceive(onReceive);
  // put the radio into receive mode
  LoRa.receive();
  delay(50);
  
  // E. WiFi Access Point start up, by default it is always on
  // could think of saving energy and fire up on demand (i.e. BUTTON_PIN)
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

  // F. Web Server Callbacks setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/getMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    writeData2Flash();
    request->send(SPIFFS, "/master.csv", "text/html", true);
  });
  server.on("/deleteMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    SPIFFS.remove("/master.csv");
    file = SPIFFS.open("/master.csv", FILE_WRITE);
    if(!file) {
      Serial.println("There was an error opening the file for writing");
      return;
    }
    if(file.println("#FILE ERASED at " + String(m.hour, DEC) + ":" + String(m.minute, DEC) + ":" + String(m.second, DEC))) {
      Serial.println("File was erased / reinit OK");
    } else {
      Serial.println("File reinit failed");
    }
    file.close();
    lastFileWrite="";
    request->send(200, "text/html", "<html><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "\">Success!  BACK </a></html>");
  });
  server.begin();
  delay(50);
 
  // G. SPIFFS to write data to onboard Flash
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS - need to add retry");
    while (1);
  }
  delay(50);
  Serial.println("init setup ok");
}


// =======================================================================================
// C. Loop
// =======================================================================================
void loop() {
  // A. LoRa is on interrupt / callback

  // B. Read and decode Master GPS
  SerialGPSDecode(Serial1, gps);
  delay(10);
  
  // C. Make Servants Data HTML
  servantsData = "";
  for(int ii = 0; ii < nServantsMax; ii++) {
    if(s[ii].active) {
      servantsData += "<tr>";
      servantsData += "<td>" + String(s[ii].ID) + "</td>";
      servantsData += "<td>" + String(s[ii].drifterTimeSlotSec) + "</td>";
      servantsData += "<td>" + String((millis() - s[ii].lastUpdateMasterTime) / 1000) + "</td>";
      servantsData += "<td>" + String(s[ii].hour) + ":" + String(s[ii].minute) + ":" + String(s[ii].second) + "</td>";
      servantsData += "<td>" + String(s[ii].lng, 6) + "</td>";
      servantsData += "<td>" + String(s[ii].lat, 6) + "</td>";
      servantsData += "<td>" + String(s[ii].dist) + "</td>";
      servantsData += "<td>" + String(s[ii].bear) + "</td>";
      servantsData += "<td>" + String(s[ii].nSamples) + "</td>";
      servantsData += "<td>" + String(s[ii].rssi) + "</td>";
      servantsData += "</tr>";
    }
  }

  // D. Write data to onboard flash
  if (nSamples > nSamplesFileWrite) {  // only write after collecting a good number of samples
    writeData2Flash();
  }
  // Save to SD Card file ???
}

// =======================================================================================
// D. Functions
// =======================================================================================

void onReceive(int packetsize) {
  // received a packet
  Serial.println("Received packet:");
  uint8_t buffer[sizeof(Packet)];
  for(uint8_t ii = 0; ii < sizeof(Packet); ii++) {
    buffer[ii] = LoRa.read();
  }
  Packet * packet;
  memset(&packet, 0, sizeof(packet));
  packet = (Packet *)buffer;
  // Get ID and then send to class for decoding
  const String name = String(packet->name);
  if(!strcmp(name.substring(0, 1).c_str(), "D")) {
    Serial.println("Drifter signal found!");
    // csvOutStr += recv; // Save all packets recevied (debugging purposes)
    const int id = name.substring(1, 3).toInt();
    s[id].ID = id;
    s[id].decode(packet);
    s[id].rssi = LoRa.packetRssi();
    s[id].updateDistBear(m.lng, m.lat);
    s[id].active = true;
    Serial.println("RX from LoRa - decoding completed");
  }
  delay(50);
}

// D0. Write data to flash
// 
void writeData2Flash() {
  file = SPIFFS.open("/master.csv", FILE_APPEND);
  if(!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
    ESP.restart();
  } else {
    if(file.println(csvOutStr)) {
      csvOutStr = "";
      nSamples = 0;
      Serial.println("Wrote data in file, current size: ");
      Serial.println(file.size());
      lastFileWrite = String(m.hour, DEC) + ":" + String(m.minute, DEC) + ":" + String(m.second, DEC);
    } else {
      lastFileWrite = "FAILED WRITE, RESTARTING";
      ESP.restart();
    }
  }
  file.close();
  delay(50);
}

// D2. Processing the onboard GPS
// The function:
//   Reads the GPS for 500ms  (allows interrupts)
//   Encodes the data via TinyGPS
//   Updates the Master data class object

void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS) {
  // Read GPS and run decoder
  unsigned long start = millis();
  do {
    while(mySerial.available() > 0) {
      myGPS.encode(mySerial.read());
    }
  } while(millis() - start < 500);

  if(gps.time.second() != gpsLastSecond) {
    m.lng = gps.location.lng();
    m.lat = gps.location.lat();
    // TODO: Need to add 8 hours onto gps time
    m.year = gps.date.year();
    m.month = gps.date.month();
    m.day = gps.date.day();
    m.hour = gps.time.hour();
    m.minute = gps.time.minute();
    m.second = gps.time.second();
    m.age = gps.location.age();

    #ifdef DEBUG_MODE
    Serial.print("m.lng: ");
    Serial.println(m.lng);
    Serial.print("m.lat: ");
    Serial.println(m.lat);
    Serial.print("m.year: ");
    Serial.println(m.year);
    Serial.print("m.month: ");
    Serial.println(m.month);
    Serial.print("m.day: ");
    Serial.println(m.day);
    Serial.print("m.hour: ");
    Serial.println(m.hour);
    Serial.print("m.minute: ");
    Serial.println(m.minute);
    Serial.print("m.second: ");
    Serial.println(m.second);
    Serial.print("m.age: ");
    Serial.println(m.age);
    #endif

    const String tDate = String(m.year) + "-" + String(m.month) + "-" + String(m.day);
    const String tTime = String(m.hour) + ":" + String(m.minute) + ":" + String(m.second);
    masterData =  "<tr><td>" + tDate + " " + tTime + "</td><td>" + String(m.lng, 6) + "</td><td>" + String(m.lat, 6) + "</td><td>" + String(m.age) + "</td>";
    masterData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/getMaster\"> GET </a></td>";
    masterData += "<td>" + lastFileWrite + "</td>";
    masterData += "<td><a href=\"http://" + IpAddress2String(WiFi.softAPIP()) + "/deleteMaster\"> ERASE </a></td>";
    masterData += "</tr>";
    // Update String to be written to file
    if((m.lng != 0.0) && (m.age < 1000)) {
      csvOutStr += tDate + "," + tTime + "," + String(m.lng, 8) + "," + String(m.lat, 8) + "," + String(m.age) + "\n";
      #ifdef DEBUG_MODE
      Serial.print("csvOutStr: ");
      Serial.println(csvOutStr);
      #endif
      nSamples += 1;
    } else {
      Serial.println(" NO GPS FIX, not WRITING LOCAL DATA !");
    }
    gpsLastSecond = gps.time.second();
    // Serial.println("nSamples: " + String(nSamples));
  }
}

// D3. Used to update sections of the webpages
// Replaces placeholder with button section in your web page
String processor(const String& var) {
  if (var == "SERVANTS") {  return servantsData;  }
  if (var == "MASTER") {    return masterData;  }
  return String();
}
