#include <DRV8835MotorShield.h>
#include <Arduino.h>
#include <vector>
#include <WiFi.h>
#include <WebServer.h>

using namespace std;

#define MAX_OUTPUT 400

/* PID values */
double BASE_SPEED = 330;
double KP = 2.40;
double KD = 110;
double KI = 0.1;

double error = 0;
double lastError = 0;
double errorDiff = 0;
double errorInt = 0;

double error_cap = 100;  // New field to be adjusted in the web interface

const char* ssid = "";
const char* password = "";

/* DRV8835MotorShield */
#define M1DIR 19  // Custom pin for M1A
#define M1PWM 18  // Custom pin for M2A
#define M2DIR 16  // Custom pin for M1B
#define M2PWM 17  // Custom pin for M2B
DRV8835MotorShield motors(M1DIR, M1PWM, M2DIR, M2PWM);
/* DRV8835MotorShield */

/* QTRX-MD-13RC */
#define SENSOR_CNT 13
const int sensorArray[SENSOR_CNT] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13, 15};
/* QTRX-MD-13RC */

double last_calculated_weighted_error = 0;

/* Line Sensor */
class LineSensor {
 private:
    vector<int> sensor_pins;
    int sensor_count;

  public:
    double sensorWeights[SENSOR_CNT] = {-300, -135, -95, -65, -30, -2, 0, 2, 30, 65, 95, 135, 300};  // Full mirrored array

    LineSensor(const int* sensors, int count) : sensor_pins(sensors, sensors + count), sensor_count(count) {}

    void initializePins() {
      for (int pin : sensor_pins) {
        pinMode(pin, INPUT);
      }
    }

    double calculateError() {
      double weightedSum = 0;
      int activeCount = 0;

      for (int i = 0; i < sensor_count; ++i) {
        if (digitalRead(sensor_pins[i])) {  // Assuming active-low sensors
          weightedSum += sensorWeights[i];
          activeCount++;
        }
      }

      if (activeCount > 0) {
        double calculatedError = weightedSum / activeCount;  // Calculate average position

        // Apply deadzone
        if (abs(calculatedError) < 5) {
          return 0;
        } else {
          return calculatedError;
        }

        last_calculated_weighted_error = calculatedError;
      } else {
        return last_calculated_weighted_error;  // Return neutral value if no sensors are active
      }

      Serial.print("last error: ");
      Serial.println(last_calculated_weighted_error);

    }

    void printSensorValues() {
      Serial.println("-----------------------------");
      Serial.println("Sensor Values:");
      for (int i = 0; i < sensor_count; ++i) {
        int sensorValue = digitalRead(sensor_pins[i]);
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(" (Pin ");
        Serial.print(sensor_pins[i]);
        Serial.print("): ");
        Serial.println(sensorValue);
      }
      Serial.println("-----------------------------");
    }
};
LineSensor lineSensor(sensorArray, SENSOR_CNT);


double PID(double input) {
  double output;

  // Apply filtered error to dampen oscillations
  error = input;
  double filteredError = 0.6 * lastError + 0.4 * error;

  // Compute error difference using filtered derivative
  errorDiff = filteredError - lastError;
  double filteredErrorDiff = (error - lastError) * 0.9 + errorDiff * 0.1;
  
  errorInt += error;
  errorInt = constrain(errorInt, -MAX_OUTPUT, MAX_OUTPUT); // Prevent integral windup

  if (error == 0) {
    errorInt = 0;
  }

  output = (KP * filteredError) + (KD * filteredErrorDiff) + (KI * errorInt);
  lastError = error;

  return output;
}


/* WebInterface Class */
class WebInterface {
private:
  const char* ssid;
  const char* password;
  WebServer server;
  double* BASE_SPEED_ptr;
  double* KP_ptr;
  double* KD_ptr;
  double* KI_ptr;
  double* sensorWeights_ptr;

  void handleRoot();
  void handleAjaxUpdate();

public:
  WebInterface(const char* ssid, const char* password,
               double* BASE_SPEED_ptr, double* KP_ptr, double* KD_ptr, double* KI_ptr, double* sensorWeights_ptr);

  void setup();
  void handleClient();
};

WebInterface::WebInterface(const char* ssid, const char* password,
                           double* BASE_SPEED_ptr, double* KP_ptr, double* KD_ptr, double* KI_ptr, double* sensorWeights_ptr)
  : ssid(ssid), password(password), server(80),
    BASE_SPEED_ptr(BASE_SPEED_ptr), KP_ptr(KP_ptr), KD_ptr(KD_ptr), KI_ptr(KI_ptr),
    sensorWeights_ptr(sensorWeights_ptr) {}

void WebInterface::setup() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  server.on("/", std::bind(&WebInterface::handleRoot, this));
  server.on("/ajax_update", std::bind(&WebInterface::handleAjaxUpdate, this));
  server.begin();
}

void WebInterface::handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 PID Control</title></head>";
  html += "<script>";
  html += "function updateVariables() {";
  html += "  var baseSpeed = document.getElementById('baseSpeed').value;";
  html += "  var kp = document.getElementById('kp').value;";
  html += "  var kd = document.getElementById('kd').value;";
  html += "  var ki = document.getElementById('ki').value;";

  // Collect sensorWeights
  html += "  var sensorWeights = '';";
  for (int i = 0; i < SENSOR_CNT; ++i) {
    html += "  sensorWeights += '&sw" + String(i) + "=' + document.getElementById('sw" + String(i) + "').value;";
  }

  html += "  var xhttp = new XMLHttpRequest();";
  html += "  xhttp.open('POST', '/ajax_update', true);";
  html += "  xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');";
  html += "  xhttp.send('baseSpeed=' + baseSpeed + '&kp=' + kp + '&kd=' + kd + '&ki=' + ki + sensorWeights);";
  html += "  xhttp.onreadystatechange = function() {";
  html += "    if (this.readyState == 4 && this.status == 200) {";
  html += "      document.getElementById('status').innerHTML = 'Variables updated!';";
  html += "    }";
  html += "  };";
  html += "}";
  html += "</script></head><body>";
  html += "<h1>ESP32 PID Variable Control</h1>";
  html += "<p>Base Speed: <input type='number' id='baseSpeed' value='" + String(*BASE_SPEED_ptr) + "'></p>";
  html += "<p>Kp: <input type='number' step='0.1' id='kp' value='" + String(*KP_ptr) + "'></p>";
  html += "<p>Kd: <input type='number' step='0.1' id='kd' value='" + String(*KD_ptr) + "'></p>";
  html += "<p>Ki: <input type='number' step='0.1' id='ki' value='" + String(*KI_ptr) + "'></p>";

  // Add sensorWeights inputs
  html += "<h2>Sensor Weights</h2>";
  for (int i = 0; i < SENSOR_CNT; ++i) {
    html += "<p>Sensor " + String(i + 1) + " Weight: <input type='number' step='0.1' id='sw" + String(i) + "' value='" + String(sensorWeights_ptr[i]) + "'></p>";
  }

  html += "<button onclick='updateVariables()'>Update Variables</button>";
  html += "<p id='status'></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void WebInterface::handleAjaxUpdate() {
  if (server.hasArg("baseSpeed")) *BASE_SPEED_ptr = server.arg("baseSpeed").toDouble();
  if (server.hasArg("kp")) *KP_ptr = server.arg("kp").toDouble();
  if (server.hasArg("kd")) *KD_ptr = server.arg("kd").toDouble();
  if (server.hasArg("ki")) *KI_ptr = server.arg("ki").toDouble();

  // Update sensorWeights
  for (int i = 0; i < SENSOR_CNT; ++i) {
    String argName = "sw" + String(i);
    if (server.hasArg(argName)) {
      sensorWeights_ptr[i] = server.arg(argName).toDouble();
    }
  }

  server.send(200, "text/plain", "OK");
}

void WebInterface::handleClient() {
  server.handleClient();
}

/* Create WebInterface instance */
WebInterface webInterface(ssid, password, &BASE_SPEED, &KP, &KD, &KI, lineSensor.sensorWeights);

void setup() {
  Serial.begin(115200);
  lineSensor.initializePins();

  webInterface.setup();
}

void loop() {
  webInterface.handleClient();

  lineSensor.printSensorValues();
  double sensor_error = lineSensor.calculateError();

  Serial.print("sensor_error: ");
  Serial.println(sensor_error);

  double pid_output = constrain(PID(sensor_error), -MAX_OUTPUT, MAX_OUTPUT);

  Serial.print("PID: ");
  Serial.println(pid_output);

  double left = BASE_SPEED - pid_output; 
  double right = BASE_SPEED + pid_output;

  Serial.print("left = ");
  Serial.println(left);

  Serial.print("right = ");
  Serial.println(right);

  motors.setM1Speed(right);
  motors.setM2Speed(left);

  delay(5);
}
