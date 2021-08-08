#include "src/loraDrifterLibs/loraDrifter.h"

#define nServantsMax                12       // Maximum number of servant drifters (just for setting array size)
#define nSamplesFileWrite           300      // Number of samples to store in memory before file write

// F. Function definitions
void loraProcessRXData(int packetSize);

// G. Classes for Master and Servant Data

class Master {
  public:
    Master() = default;
    float lon = 0.f;
    float lat = 0.f;
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    int age = 0;
    ~Master() = default;
};

class Servant {
  public:
    Servant() = default;
    void decode(String packet);
    void updateDistBear(const float fromLon, const float fromLat);
    int ID = 0;
    int loraUpdatePlanSec = 0;
    int lastUpdateMasterTime = 0;
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    float lon = 0.f;
    float lat = 0.f;
    int age = 0;
    int count = 0;
    float dist = 0.f;
    float bear = 0.f;
    int rssi = 0;
    ~Servant() = default;
};

void Servant::updateDistBear(const float fromLon, const float fromLat) {
   dist = TinyGPSPlus::distanceBetween(fromLat, fromLon, lat, lon);
   bear = TinyGPSPlus::courseTo(fromLat, fromLon, lat, lon);
}

void Servant::decode(String packet) {
  // Example:  D01,15,yyyy-mm-dd,HH:MM:SS,-31.97758433,115.88428733,151,4104
  
  // Update Plan
  int comma1 = packet.indexOf(",");
  int comma2 = packet.indexOf(",", comma1 + 1);
  loraUpdatePlanSec = packet.substring(comma1 + 1, comma2).toInt();
  lastUpdateMasterTime = millis();

  // Date
  comma1 = comma2;  comma2 = packet.indexOf(",", comma1 + 1);
  String timeFull = packet.substring(comma1 + 1, comma2);
  
  int colon1 = timeFull.indexOf("-");
  int colon2 = timeFull.indexOf("-", colon1 + 1);
  year = timeFull.substring(0, colon1).toInt();
  month = timeFull.substring(colon1 + 1, colon2).toInt();
  day = timeFull.substring(colon2 + 1).toInt();

  // Time
  comma1 = comma2;  comma2 = packet.indexOf(",", comma1 + 1);
  timeFull = packet.substring(comma1 + 1, comma2);
  
  colon1 = timeFull.indexOf(":");
  colon2 = timeFull.indexOf(":", colon1 + 1);
  hour = timeFull.substring(0, colon1).toInt();
  minute = timeFull.substring(colon1 + 1, colon2).toInt();
  second = timeFull.substring(colon2 + 1).toInt();
  
  // Longitude
  comma1 = comma2;  comma2 = packet.indexOf(",", comma1 + 1);
  lon = packet.substring(comma1 + 1,comma2).toFloat();

  // Latitude
  comma1 = comma2;  comma2 = packet.indexOf(",", comma1 + 1);
  lat = packet.substring(comma1 + 1, comma2).toFloat();
  
 // Age
  comma1 = comma2;  comma2 = packet.indexOf(",", comma1 + 1);
  age = packet.substring(comma1 + 1,comma2).toInt();

  // Count
  comma1 = comma2;  comma2 = packet.indexOf(",", comma1 + 1);
  count = packet.substring(comma1 + 1, comma2).toInt(); 
}

// H. This is the string literal for the main web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>UWA LoRa Drifters</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="refresh" content="1" >
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
      .switch {
        position: relative; display: inline-block; width: 120px; height: 68px
      } 
      .switch input {
        display: none
      }
      .slider {
        position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 6px
      }
      .slider:before {
        position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px
      }
      input:checked + .slider {
        background-color: #b30000
      }
      input:checked + .slider:before {
        -webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)
      }
    </style>
  </head>
  <body>
    <h2>LoRa Drifters</h2>
    <h4>Master Node</h4>
    <table>
      <tr><td>GPS Time</td>
        <td>Longitude</td>
        <td>Latitude</td>
        <td>GPS Age [milliSec]</td>
        <td>Get Data Link </td>
        <td>Last File Write GPS Time</td>
        <td>Erase Data (NO WARNING)</td>
      </tr>
      %MASTER%
    </table>
    <br><br>
    <h4>Servants</h4>
    <table>
      <tr>
        <td>ID</td>
        <td>Lora Update Plan [sec]</td>
        <td>Last Update Master Time [sec] Ago</td>
        <td>Time</td>
        <td>Longitude</td>
        <td>Latitude</td>
        <td>Distance [m]</td>
        <td>Bearing [degN to]</td>
        <td>Count</td>
        <td>RSSI</td>
      </tr>
      %SERVANTS%
    </table>
  </body>
</html>
)rawliteral";
