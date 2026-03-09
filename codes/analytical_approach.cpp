/* --- Analytical approach for UWB Positioning system  ---
   Anchors: A1(0,0), A2(10,0), A3(5,8)

*/

#include "WiFi.h"
#include <WebServer.h>
#include <math.h>

// ==========================================
// 1. CONFIGURATION
// ==========================================
float anchors[3][2] = {
  {0.00,  0.00},  // Anchor 01
  {10.00, 0.00},  // Anchor 02
  {5.00,  8.00}   // Anchor 03
};
String anchorNames[] = {"FTM_Anchor_01", "FTM_Anchor_02", "FTM_Anchor_03"};
float distance_offset = 0.0; 

// ==========================================
// 2. FILTERING & CACHED VARIABLES
// ==========================================
float dist_history[3][5]; 
int history_idx[3] = {0, 0, 0};
float smoothed_distances[3] = {0, 0, 0};
bool first_reading[3] = {true, true, true}; 

volatile bool got_data = false;
float last_raw_val = 0;

// Cached Wi-Fi Data (Prevents scanning in the loop)
uint8_t anchorBSSIDs[3][6];
int anchorChannels[3];
bool anchorFound[3] = {false, false, false};

// Coordinate Globals for Web UI
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
    const scale = 40; // 40 pixels = 1 meter
    
    function drawGrid() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      // Draw Grid Lines
      ctx.strokeStyle = "#333";
      for(let i=0; i<10; i++) {
        ctx.beginPath(); ctx.moveTo(i*scale, 0); ctx.lineTo(i*scale, 320); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i*scale); ctx.lineTo(400, i*scale); ctx.stroke();
      }

      // Draw Anchors
      ctx.fillStyle = "#00FFCC";
      ctx.fillRect(0, 320 - 10, 10, 10); // A1 (0,0)
      ctx.fillRect(10 * scale - 10, 320 - 10, 10, 10); // A2 (10,0)
      ctx.fillRect(5 * scale - 5, 320 - (8 * scale), 10, 10); // A3 (5,8)
      
      ctx.fillStyle = "#fff";
      ctx.font = "12px Arial";
      ctx.fillText("A1(0,0)", 15, 310);
      ctx.fillText("A2(10,0)", 340, 310);
      ctx.fillText("A3(5,8)", 215, 20);
    }

    function updateMap() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          drawGrid();
          
          // Draw Tag
          let cx = data.x * scale;
          let cy = 320 - (data.y * scale); // Invert Y for canvas
          
          // Constrain dot to canvas bounds visually
          if(cx < 0) cx = 5; if(cx > 400) cx = 395;
          if(cy < 0) cy = 5; if(cy > 320) cy = 315;

          ctx.beginPath();
          ctx.arc(cx, cy, 8, 0, 2 * Math.PI);
          ctx.fillStyle = "#FF3366";
          ctx.fill();
          ctx.strokeStyle = "#fff";
          ctx.stroke();
          
          document.getElementById('coords').innerText = "X: " + data.x.toFixed(2) + " m | Y: " + data.y.toFixed(2) + " m";
          document.getElementById('dists').innerText = "A1: " + data.r1.toFixed(1) + "m | A2: " + data.r2.toFixed(1) + "m | A3: " + data.r3.toFixed(1) + "m";
        })
        .catch(err => console.log(err));
    }

    drawGrid();
    setInterval(updateMap, 400); // Fetch data every 0.4s
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
    last_raw_val = (report->dist_est / 100.0) + distance_offset;
    got_data = true;
  }
}

// ==========================================
// 5. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Wi-Fi Setup: AP mode for Web Server, STA mode for FTM
  WiFi.mode(WIFI_AP_STA); 
  WiFi.softAP("ESP32-Live-Map", "12345678"); // Hotspot Name & Password
  WiFi.onEvent(onFtmReport, ARDUINO_EVENT_WIFI_FTM_REPORT);
  WiFi.setSleep(false);
  
  // Web Server Endpoints
  server.on("/", []() { 
    server.send(200, "text/html", index_html); 
  });
  
  server.on("/data", []() { 
    char json[100];
    sprintf(json, "{\"x\":%.2f, \"y\":%.2f, \"r1\":%.2f, \"r2\":%.2f, \"r3\":%.2f}", 
            posX, posY, smoothed_distances[0], smoothed_distances[1], smoothed_distances[2]);
    server.send(200, "application/json", json);
  });
  
  server.begin();
  
  Serial.println("\n--- 10m Stabilized System Starting ---");
  Serial.println("Scanning for Anchors...");

  // ONE-TIME SCAN FOR ANCHORS
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
  WiFi.scanDelete(); // Free memory

  Serial.println("Connect Phone to Wi-Fi: ESP32-Live-Map");
  Serial.println("Open Browser to: http://192.168.4.1");
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  server.handleClient(); // Keep the website alive

  for (int j = 0; j < 3; j++) {
    if (anchorFound[j]) {
      got_data = false;
      
      // Request FTM using CACHED BSSID and Channel
      if (WiFi.initiateFTM(32, 0, anchorChannels[j], anchorBSSIDs[j])) {
         unsigned long start = millis();
         
         // Wait for data, feeding the web server
         while(!got_data && millis() - start < 1000) { 
           delay(10); 
           server.handleClient(); 
         }
         
         if(got_data && last_raw_val > 0.1 && last_raw_val < 30.0) {
           
           // Fill array on first reading to prevent ramp-up lag
           if (first_reading[j]) {
             for (int k = 0; k < 5; k++) dist_history[j][k] = last_raw_val;
             first_reading[j] = false;
           } else {
             dist_history[j][history_idx[j]] = last_raw_val;
             history_idx[j] = (history_idx[j] + 1) % 5;
           }
           
           // --- MEDIAN FILTER LOGIC ---
           float temp[5];
           
           // 1. Copy the history array
           for(int k=0; k<5; k++) {
             temp[k] = dist_history[j][k];
           }
           
           // 2. Bubble Sort the temporary array
           for(int p=0; p<4; p++) {
             for(int q=0; q<4-p; q++) {
               if(temp[q] > temp[q+1]) {
                 float t = temp[q];
                 temp[q] = temp[q+1];
                 temp[q+1] = t;
               }
             }
           }
           
           // 3. Extract the median value (middle element)
           smoothed_distances[j] = temp[2]; 
           // ---------------------------
         }
      }
      
      // Brief delay, feeding server
      unsigned long d_start = millis();
      while(millis() - d_start < 50) { server.handleClient(); delay(1); }
    }
  }

  // Only calculate XY if we have valid readings for all 3 anchors
  if (!first_reading[0] && !first_reading[1] && !first_reading[2]) {
    calculateXY();
  }
}

// ==========================================
// 7. TRILATERATION MATH
// ==========================================
void calculateXY() {
  float r1 = smoothed_distances[0];
  float r2 = smoothed_distances[1];
  float r3 = smoothed_distances[2];
  
  float d = anchors[1][0]; // 10.0
  float i = anchors[2][0]; // 5.0
  float j = anchors[2][1]; // 8.0

  float x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2)) / (2 * d);
  float y = (pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2) - (2 * i * x)) / (2 * j);

  if (isnan(x) || isnan(y)) return;

  // Update globals for the Web UI
  posX = x;
  posY = y;

  Serial.printf("DIST -> A1:%.2fm A2:%.2fm A3:%.2fm | POS -> X:%.2fm, Y:%.2fm\n", r1, r2, r3, x, y);
}
