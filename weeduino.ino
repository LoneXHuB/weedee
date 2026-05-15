#define LEFT_CH 2
#define RIGHT_CH 7

#define PWM_LEFT 3
#define DIR_LEFT 4
#define PWM_RIGHT 5
#define DIR_RIGHT 6

#define DEADZONE 40

// ----- MOTOR TUNING -----
#define START_PWM   60
#define MAX_PWM     125
#define MAX_DELTA   500

// ---- FILTERING ----
#define FILTER_ALPHA 0.2   // 0.0–1.0 (lower = more smoothing)

float filteredLeft = 1500;
float filteredRight = 1500;

void setup() {
  pinMode(LEFT_CH, INPUT);
  pinMode(RIGHT_CH, INPUT);

  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  Serial.begin(115200);
  Serial.println("EDGE → DC MOTOR CONVERTER READY");
}

int readPulse(int pin) {
  unsigned long v = pulseIn(pin, HIGH, 25000);
  if (v == 0) return 1500;
  return constrain(v, 1000, 2000);
}

void drive(int pwmPin, int dirPin, int pulse) {

  int delta = pulse - 1500;

  if (abs(delta) < DEADZONE) {
    analogWrite(pwmPin, 0);
    return;
  }

  digitalWrite(dirPin, delta > 0 ? HIGH : LOW);

  int mag = abs(delta) - DEADZONE;

  int pwm = map(mag, 0, MAX_DELTA - DEADZONE,
                START_PWM, MAX_PWM);

  pwm = constrain(pwm, START_PWM, MAX_PWM);

  analogWrite(pwmPin, pwm);
}

void loop() {

  int rawLeft = readPulse(LEFT_CH);
  int rawRight = readPulse(RIGHT_CH);
  // Drive motors (ACTIVE for testing)
  drive(PWM_LEFT, DIR_LEFT, rawLeft);
  drive(PWM_RIGHT, DIR_RIGHT, rawRight);

  // Print RAW + FILTERED for debugging
  Serial.print("RAW L: "); Serial.print(rawLeft);
  Serial.print("  |  RAW R: "); Serial.println(rawRight);
}
