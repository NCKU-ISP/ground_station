#include <AccelStepper.h>
#include <Arduino.h>

// location: 22.937262,120.274538,36;
// 22.937510,120.274729,36;

// Define stepper motor connections and motor interface type. Motor interface
// type must be set to 1 when using a driver:
#define dirPin_yaw 8
#define stepPin_yaw 7
#define dirPin_pitch 11
#define stepPin_pitch 10
#define motorInterfaceType 1

#define YAW_MAX_SPEED 4000
#define PITCH_MAX_SPEED 4000
#define MAX_ACC 2000

#define YAW_GEAR_RATIO 250047.0 / 4913.0   // 1:51
#define PITCH_GEAR_RATIO 226233.0 / 3179.0 // 1:71
#define EARTH_RADIUS 6378137           // m

bool set_mode = false;
int set_step = 0;

double local_long_h = 0, local_lat_h = 0;
double local_long_t = 0, local_lat_t = 0;
double local_alt = 0;

const int btn_up = A1;
const int btn_down = A4;

double yaw_angle = 0;
double pitch_angle = 0;

// Create a new instance of the AccelStepper class:
AccelStepper yaw_axis =
    AccelStepper(motorInterfaceType, stepPin_yaw, dirPin_yaw);
AccelStepper pitch_axis =
    AccelStepper(motorInterfaceType, stepPin_pitch, dirPin_pitch);

void btnClicked(double *angle, double moving_steps = 1) {
  if (!digitalRead(btn_up)) {
    (*angle) += moving_steps;
    Serial.print("angle:");
    Serial.println(*angle);
  } else if (!digitalRead(btn_down)) {
    (*angle) -= moving_steps;
    Serial.print("angle:");
    Serial.println(*angle);
  }
}

int32_t Deg2Step(double deg, double gearRatio) {
  return (int32_t)(deg / 360 * 200 * 16 * gearRatio);
}

bool isDouble(String subStr) {
  int signCount = 0;
  int decPointCount = 0;
  int digitCount = 0;
  int otherCount = 0;
  for (int j = 0; j < subStr.length(); j++) {
    char ch = subStr.charAt(j);
    if (j == 0 && (ch == '-' || ch == '+'))
      signCount++; // leading sign
    else if (ch >= '0' && ch <= '9')
      digitCount++; // digit
    else if (ch == '.')
      decPointCount++; // decimal point
    else
      otherCount++; // that's bad :(
  }
  if (signCount > 1 || decPointCount > 1 || digitCount < 1 || otherCount > 0) {
    return false;
  } else {
    return true;
  }
}

void extractFloat(String digits, double *head, double *tail) {
  *head = digits.substring(0, digits.indexOf('.')).toDouble();
  *tail = digits.substring(digits.indexOf('.')).toDouble();
}

bool cmd_flag = false;

void setting() {
  set_mode = true;
  while (set_mode) {
    static String cmd = "";
    if (cmd_flag) {
      cmd = "";
      cmd_flag = false;
    }
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        cmd_flag = true;
        // Serial.print("current state: ");
        // Serial.println(set_step);
        // Serial.println(cmd);
        break;
      }
      cmd += c;
    }
    delay(50);
    switch (set_step) {
    case 0:
      Serial.println("Set yaw axis zero (heading north) [press y to finish]");
      Serial.println("Press up down button to move.");
      Serial.println(
          "(input angle for single button press (default is 1 degree))");
      set_step++;
    case 1:
      static double move_steps = 1;
      if (cmd == "y") {
        Serial.println("Set yaw axis zero success!");
        yaw_axis.setCurrentPosition(0);
        set_step++;
        break;
      } else if (isDouble(cmd) && cmd_flag) {
        Serial.print("Set single step to ");
        Serial.print(cmd.toDouble());
        Serial.println(" degree.");
        move_steps = cmd.toDouble();
      }
      btnClicked(&yaw_angle, move_steps);
      yaw_axis.runToNewPosition(Deg2Step(yaw_angle, YAW_GEAR_RATIO));
      break;
    case 2:
      Serial.println(
          "Set pitch axis zero (horizontal to ground) [press y to finish]");
      set_step++;
    case 3:
      if (cmd == "y") {
        Serial.println("Set pitch axis zero success!");
        pitch_axis.setCurrentPosition(0);
        set_step++;
        break;
      } else if (isDouble(cmd) && cmd_flag) {
        Serial.print("Set single step to ");
        Serial.print(cmd.toDouble());
        Serial.println(" degree.");
        move_steps = cmd.toDouble();
      }
      btnClicked(&pitch_angle, move_steps);
      pitch_axis.runToNewPosition(Deg2Step(pitch_angle, PITCH_GEAR_RATIO));
      break;
    case 4:
      Serial.println("Set antenna [latitude,longitude,altitude;]:");
      set_step++;
    case 5:
      if (cmd_flag && cmd.endsWith(";")) {
        extractFloat(cmd.substring(0, cmd.indexOf(',')), &local_lat_h,
                     &local_lat_t);
        extractFloat(cmd.substring(cmd.indexOf(',') + 1, cmd.lastIndexOf(',')),
                     &local_long_h, &local_long_t);
        local_alt = cmd.substring(cmd.lastIndexOf(',') + 1).toDouble();
        String msg = "cmd:" + String(local_lat_h, 0) + "+" +
                     String(local_lat_t, 6) + "," + String(local_long_h, 0) +
                     "+" + String(local_long_t, 6) + "," + local_alt;
        Serial.println(msg);
        Serial.println("Setting complete!");
        set_mode = false;
        cmd_flag = false;
      }
      break;
    }
  }
}

double Rad2Deg(double rad) { return rad * 180.0 / PI; }
double Deg2Rad(double deg) { return deg * PI / 180.0; }

#define SIGN(x) (x) >= (0) ? (1) : (-1)
double yaw_calc(double latitude_h, double latitude_t, double longitude_h,
                double longitude_t) {
  double latitude_diff =
      (latitude_h - local_lat_h) + (latitude_t - local_lat_t);
  double longitude_diff =
      (longitude_h - local_long_h) + (longitude_t - local_long_t);
  double y = ((double)SIGN(latitude_diff)) * (1 - cos(latitude_diff));
  double x = ((double)SIGN(longitude_diff)) * (1 - cos(longitude_diff));
  // Serial.println((1 - cos(latitude_diff)));
  // Serial.println(y);
  // Serial.println((1 - cos(longitude_diff)));
  // Serial.println(x);
  return Rad2Deg(HALF_PI - atan2(y, x));
}

// HaversineDistance
double pitch_calc(long double latitude_h, long double latitude_t,
                  long double longitude_h, long double longitude_t,
                  double altitude) {
  double latitude_diff =
      (latitude_h - local_lat_h) + (latitude_t - local_lat_t);
  double longitude_diff =
      (longitude_h - local_long_h) + (longitude_t - local_long_t);

  long double bal_lat = latitude_h + latitude_t;
  long double local_lat = local_lat_h + local_lat_t;

  double nA = pow(sin(Deg2Rad(latitude_diff) / 2), 2) +
              cos(Deg2Rad(bal_lat)) * cos(Deg2Rad(local_lat)) *
                  pow(sin(Deg2Rad(longitude_diff) / 2), 2);

  double nC = 2 * atan2(sqrt(nA), sqrt(1 - nA));
  double nD = EARTH_RADIUS * nC;
  Serial.println(nD, 5);
  return Rad2Deg(atan2((altitude - local_alt), nD));
}

// 22.9368341,120.2749937

void setup() {
  // Set the maximum speed in steps per second:
  Serial.begin(115200);
  yaw_axis.setMaxSpeed(YAW_MAX_SPEED);
  pitch_axis.setMaxSpeed(PITCH_MAX_SPEED);

  yaw_axis.setAcceleration(MAX_ACC);
  pitch_axis.setAcceleration(MAX_ACC);

  yaw_axis.setCurrentPosition(0);
  pitch_axis.setCurrentPosition(0);

  pinMode(btn_up, INPUT_PULLUP);
  pinMode(btn_down, INPUT_PULLUP);

  setting();
}

double bal_lat_h = 0, bal_lat_t = 0;
double bal_long_h = 0, bal_long_t = 0;
double bal_alt = 0;

void loop() {
  static String data = "";
  if (cmd_flag) {
    data = "";
    cmd_flag = false;
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == ';') {
      cmd_flag = true;
      Serial.println(data);
      break;
    }
    data += c;
  }
  if (cmd_flag) {
    // Update balloon location
    extractFloat(data.substring(0, data.indexOf(',')), &bal_lat_h, &bal_lat_t);
    extractFloat(data.substring(data.indexOf(',') + 1, data.lastIndexOf(',')),
                 &bal_long_h, &bal_long_t);
    bal_alt = data.substring(data.lastIndexOf(',') + 1).toDouble();
    String msg = "data:" + String(bal_lat_h, 0) + "+" + String(bal_lat_t, 6) +
                 "," + String(bal_long_h, 0) + "+" + String(bal_long_t, 6) +
                 "," + bal_alt;
    Serial.println(msg);

    double yaw_target_deg =
        yaw_calc(bal_lat_h, bal_lat_t, bal_long_h, bal_long_t);
    double pitch_target_deg =
        pitch_calc(bal_lat_h, bal_lat_t, bal_long_h, bal_long_t, bal_alt);

    double yaw_target = Deg2Step(yaw_target_deg, YAW_GEAR_RATIO);
    double pitch_target = Deg2Step(pitch_target_deg, PITCH_GEAR_RATIO);

    Serial.print("yaw target: ");
    Serial.println(yaw_target_deg);
    Serial.print("pitch target: ");
    Serial.println(pitch_target_deg);
    Serial.println();

    yaw_axis.moveTo(yaw_target);
    pitch_axis.moveTo(pitch_target);
  }

  yaw_axis.run();
  pitch_axis.run();
}