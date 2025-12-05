// RoverDashboardFirmware.ino
// WiFi + Ultrasonic + JSON API

#include <Arduino.h>
#include <WiFiNINA.h>   

char ssid[] = "Novexa";
char pass[] = "00000007";

WiFiServer server(80);

// === Ultrasonic pins ===
const int LEFT_TRIG  = 33;
const int LEFT_ECHO  = 32;
const int RIGHT_TRIG = 31;
const int RIGHT_ECHO = 30;

struct LegState { int coxa; int femur; int tibia; bool standing; };
LegState legs[6];

struct PoseState { bool isStanding; } pose;

String currentCommand = "stop";

float readUltrasonicMeters(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return -1.0;

  float distanceCm = duration * 0.0343f / 2.0f;
  return distanceCm / 100.0f;
}

String classify(float d) {
  if (d < 0) return "No Echo";
  if (d < 0.30) return "Very Near";
  if (d < 1.00) return "Near";
  return "Far";
}

String buildStatusJson() {
  float left  = readUltrasonicMeters(LEFT_TRIG, LEFT_ECHO);
  float right = readUltrasonicMeters(RIGHT_TRIG, RIGHT_ECHO);

  String j = "{";

  // legs
  j += "\"legs\":[";
  for (int i=0;i<6;i++) {
    if (i>0) j += ",";
    j += "{";
    j += "\"coxa\":" + String(legs[i].coxa);
    j += ",\"femur\":" + String(legs[i].femur);
    j += ",\"tibia\":" + String(legs[i].tibia);
    j += ",\"standing\":" + String(legs[i].standing?"true":"false");
    j += "}";
  }
  j += "],";

  // pose
  j += "\"pose\":{\"isStanding\":" + String(pose.isStanding?"true":"false") + "},";

  // ultrasonic
  j += "\"ultrasonic\":{";
    j += "\"left\":{";
      j += "\"distance_m\":" + String(left,2);
      j += ",\"label\":\"" + classify(left) + "\"";
    j += "},";
    j += "\"right\":{";
      j += "\"distance_m\":" + String(right,2);
      j += ",\"label\":\"" + classify(right) + "\"";
    j += "}";
  j += "}";

  j += "}";
  return j;
}

void handleClient(WiFiClient &c) {
  String req = c.readStringUntil('\r');
  c.readStringUntil('\n');

  while (true) {
    String h = c.readStringUntil('\r');
    c.readStringUntil('\n');
    if (h.length()==0) break;
  }

  if (req.startsWith("GET /status")) {
    String body = buildStatusJson();
    c.println("HTTP/1.1 200 OK");
    c.println("Content-Type: application/json");
    c.print("Content-Length: "); c.println(body.length());
    c.println("Connection: close");
    c.println();
    c.print(body);
  }
  else if (req.startsWith("POST /command")) {
    delay(10);
    String data="";
    while (c.available()) data += (char)c.read();

    int k = data.indexOf("\"action\"");
    if (k>=0) {
      int q1 = data.indexOf("\"", k+8);
      int q2 = data.indexOf("\"", q1+1);
      if (q1>=0 && q2>q1)
        currentCommand = data.substring(q1+1, q2);
    }

    String r = "{\"ok\":true}";
    c.println("HTTP/1.1 200 OK");
    c.println("Content-Type: application/json");
    c.print("Content-Length: "); c.println(r.length());
    c.println("Connection: close");
    c.println();
    c.print(r);
  }
  else {
    c.println("HTTP/1.1 404 Not Found");
    c.println("Content-Type: text/plain");
    c.println("Connection: close");
    c.println();
    c.println("Not found");
  }
}

void setupLegs() {
  legs[0]={90,165,165,true};
  legs[1]={90,15,15,true};
  legs[2]={90,165,165,true};
  legs[3]={90,15,15,true};
  legs[4]={90,15,15,true};
  legs[5]={90,165,165,true};

  pose.isStanding = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_TRIG,OUTPUT);
  pinMode(LEFT_ECHO,INPUT);
  pinMode(RIGHT_TRIG,OUTPUT);
  pinMode(RIGHT_ECHO,INPUT);

  setupLegs();

  WiFi.begin(ssid, pass);
  while (WiFi.status()!=WL_CONNECTED) { delay(500); }

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
    client.stop();
  }

  // TODO: Add gait control here using currentCommand
  //       Update legs[] values as servos move.
}
