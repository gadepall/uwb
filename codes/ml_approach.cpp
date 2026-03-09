/* --- ML approach for UWB Positioning system ---
   Anchors: A1(0,0), A2(10,0), A3(5,8)
*/

#include "WiFi.h"
#include <WebServer.h>
#include <math.h>

// ==========================================
// 1. CONFIGURATION & CALIBRATION
// ==========================================
float anchors[3][2] = {
  {0.00,  0.00},  // Anchor 01
  {10.00, 0.00},  // Anchor 02
  {5.00,  8.00}   // Anchor 03
};
String anchorNames[] = {"FTM_Anchor_01", "FTM_Anchor_02", "FTM_Anchor_03"};

// LEAST SQUARES CALIBRATION CONSTANTS (Calculated from your 25-point dataset)
// Format: d_true = (m * d_measured) + c
float A1_m = 0.4614; float A1_c = 2.5134;
float A2_m = 0.6145; float A2_c = 0.3300;
float A3_m = 0.2887; float A3_c = 2.4097;

// ==========================================
// 2. FILTERING & CACHED VARIABLES
// ==========================================
float dist_history[3][5]; 
int history_idx[3] = {0, 0, 0};
float smoothed_distances[3] = {0, 0, 0};
float calibrated_distances[3] = {0, 0, 0}; 
bool first_reading[3] = {true, true, true}; 

volatile bool got_data = false;
float last_raw_val = 0;

uint8_t anchorBSSIDs[3][6];
int anchorChannels[3];
bool anchorFound[3] = {false, false, false};

float posX = 0.0;
float posY = 0.0;

WebServer server(80);

// ==========================================
// 3. THE WEB UI (HTML + JAVASCRIPT)
// ==========================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Indoor GPS</title>
  <style>
    body { font-family: Arial; text-align: center; background-color: #222; color: #fff; margin: 0; padding: 20px;}
    canvas { background-color: #111; border: 2px solid #555; border-radius: 5px; box-shadow: 0px 0px 15px rgba(0,255,255,0.2); margin-top: 10px; max-width: 100%;}
    .stats { font-size: 1.2em; font-weight: bold; color: #00FFCC; margin-top: 15px; }
    .debug { font-size: 0.9em; color: #aaa; margin-top: 5px; }
  </style>
</head>
<body>
  <h2>Live Tracking (10m x 8m)</h2>
  <canvas id="mapCanvas" width="400" height="320"></canvas>
  <div class="stats" id="coords">X: 0.00 m | Y: 0.00 m</div>
  <div class="debug" id="dists">A1: 0m | A2: 0m | A3: 0m</div>

  <script>
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');
    const scale = 40; 
    
    function drawGrid() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = "#333";
      for(let i=0; i<10; i++) {
        ctx.beginPath(); ctx.moveTo(i*scale, 0); ctx.lineTo(i*scale, 320); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i*scale); ctx.lineTo(400, i*scale); ctx.stroke();
      }
      ctx.fillStyle = "#00FFCC";
      ctx.fillRect(0, 320 - 10, 10, 10); 
      ctx.fillRect(10 * scale - 10, 320 - 10, 10, 10); 
      ctx.fillRect(5 * scale - 5, 320 - (8 * scale), 10, 10); 
      
      ctx.fillStyle = "#fff"; ctx.font = "12px Arial";
      ctx.fillText("A1(0,0)", 15, 310); ctx.fillText("A2(10,0)", 340, 310); ctx.fillText("A3(5,8)", 215, 20);
    }

    function updateMap() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          drawGrid();
          let cx = data.x * scale; let cy = 320 - (data.y * scale);
          if(cx < 0) cx = 5; if(cx > 400) cx = 395;
          if(cy < 0) cy = 5; if(cy > 320) cy = 315;

          ctx.beginPath(); ctx.arc(cx, cy, 8, 0, 2 * Math.PI);
          ctx.fillStyle = "#FF3366"; ctx.fill();
          ctx.strokeStyle = "#fff"; ctx.stroke();
          
          document.getElementById('coords').innerText = "X: " + data.x.toFixed(2) + " m | Y: " + data.y.toFixed(2) + " m";
          document.getElementById('dists').innerText = "A1: " + data.r1.toFixed(1) + "m | A2: " + data.r2.toFixed(1) + "m | A3: " + data.r3.toFixed(1) + "m";
        }).catch(err => console.log(err));
    }
    drawGrid(); setInterval(updateMap, 400); 
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 4. FTM EVENT HANDLER
// ==========================================
void onFtmReport(arduino_event_t *event) {
  wifi_event_ftm_report_t *report = &event->event_info.wifi_ftm_report;
  if (report->status == FTM_STATUS_SUCCESS) {
    last_raw_val = (report->dist_est / 100.0); // Offset removed, handled entirely by LS Calibration now
    got_data = true;
  }
}

// ==========================================
// 5. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA); 
  WiFi.softAP("ESP32-Live-Map", "12345678"); 
  WiFi.onEvent(onFtmReport, ARDUINO_EVENT_WIFI_FTM_REPORT);
  WiFi.setSleep(false);
  
  server.on("/", []() { server.send(200, "text/html", index_html); });
  server.on("/data", []() { 
    char json[100];
    sprintf(json, "{\"x\":%.2f, \"y\":%.2f, \"r1\":%.2f, \"r2\":%.2f, \"r3\":%.2f}", 
            posX, posY, calibrated_distances[0], calibrated_distances[1], calibrated_distances[2]);
    server.send(200, "application/json", json);
  });
  server.begin();
  
  Serial.println("\n--- 10m Calibrated Tracker Starting ---");
  Serial.println("Scanning for Anchors...");
  int n = WiFi.scanNetworks(false, false, false, 150); 
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 3; j++) {
      if (WiFi.SSID(i) == anchorNames[j]) {
        anchorChannels[j] = WiFi.channel(i);
        memcpy(anchorBSSIDs[j], WiFi.BSSID(i), 6);
        anchorFound[j] = true;
        Serial.printf("Found %s on Channel %d\n", anchorNames[j].c_str(), anchorChannels[j]);
      }
    }
  }
  WiFi.scanDelete();
  
  Serial.println("Connect Phone to Wi-Fi: ESP32-Live-Map");
  Serial.println("Open Browser to: http://192.168.4.1");
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  server.handleClient(); 

  for (int j = 0; j < 3; j++) {
    if (anchorFound[j]) {
      got_data = false;
      if (WiFi.initiateFTM(32, 0, anchorChannels[j], anchorBSSIDs[j])) {
         unsigned long start = millis();
         while(!got_data && millis() - start < 1000) { delay(10); server.handleClient(); }
         
         if(got_data && last_raw_val > 0.1 && last_raw_val < 30.0) {
           if (first_reading[j]) {
             for (int k = 0; k < 5; k++) dist_history[j][k] = last_raw_val;
             first_reading[j] = false;
           } else {
             dist_history[j][history_idx[j]] = last_raw_val;
             history_idx[j] = (history_idx[j] + 1) % 5;
           }
           
           float temp[5];
           for(int k=0; k<5; k++) temp[k] = dist_history[j][k];
           for(int p=0; p<4; p++) {
             for(int q=0; q<4-p; q++) {
               if(temp[q] > temp[q+1]) { float t = temp[q]; temp[q] = temp[q+1]; temp[q+1] = t; }
             }
           }
           smoothed_distances[j] = temp[2]; 
         }
      }
      unsigned long d_start = millis();
      while(millis() - d_start < 50) { server.handleClient(); delay(1); }
    }
  }

  if (!first_reading[0] && !first_reading[1] && !first_reading[2]) {
    calculateXY();
  }
}

// ==========================================
// 7. CALIBRATION & TRILATERATION
// ==========================================
void calculateXY() {
  // Apply Least Squares Calibration Math mapping measured distances to true distances
  calibrated_distances[0] = (smoothed_distances[0] * A1_m) + A1_c;
  calibrated_distances[1] = (smoothed_distances[1] * A2_m) + A2_c;
  calibrated_distances[2] = (smoothed_distances[2] * A3_m) + A3_c;

  float r1 = calibrated_distances[0];
  float r2 = calibrated_distances[1];
  float r3 = calibrated_distances[2];
  
  float d = anchors[1][0]; 
  float i = anchors[2][0]; 
  float j = anchors[2][1]; 

  float x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
  float y = (pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2) - (2 * i * x)) / (2 * j);

  if (isnan(x) || isnan(y)) return;

  posX = x;
  posY = y;

  // Print raw vs calibrated data for debugging
  Serial.printf("RAW -> A1:%.2f A2:%.2f A3:%.2f | CALIB -> A1:%.2f A2:%.2f A3:%.2f | X:%.2f Y:%.2f\n", 
                smoothed_distances[0], smoothed_distances[1], smoothed_distances[2], 
                r1, r2, r3, x, y);
}
