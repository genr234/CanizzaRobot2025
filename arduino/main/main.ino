#include <Servo.h>

// Definizione pin sensore colore 1
#define S2_1_PIN 8
#define S3_1_PIN 9
#define OUT_1_PIN 10

// Definizione pin sensore colore 2
#define S2_2_PIN 5
#define S3_2_PIN 4
#define OUT_2_PIN 3

// Definizione pin sistema
#define BUTTON_PIN 2
#define LED_ROSSO_PIN 13
#define LED_VERDE_PIN 12
#define TRIG_ULTRASONIC_PIN 7
#define ECHO_ULTRASONIC_PIN 6

// Soglie primo sensore
#define WHITE_THRESHOLD 60
#define RED_THRESHOLD 100
#define BLUE_THRESHOLD 80
#define DARK_THRESHOLD 110

// Soglie secondo sensore
#define WHITE_THRESHOLD_2 15
#define NESSUN_OGGETTO_THRESHOLD_2 17

Servo servo;

// Variabili stato
bool inizio = false;
bool start = false;
bool buttonPressed = false;

// Variabili debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
int lastButtonState = LOW;

// Variabili secondo sensore
int red2, green2, blue2, clear2;

void setup() {
  Serial.begin(115200);
  
  // Sensore colore 1
  pinMode(S2_1_PIN, OUTPUT);
  pinMode(S3_1_PIN, OUTPUT);
  pinMode(OUT_1_PIN, INPUT);
  
  // Sensore colore 2
  pinMode(S2_2_PIN, OUTPUT);
  pinMode(S3_2_PIN, OUTPUT);
  pinMode(OUT_2_PIN, INPUT);
  
  // Sistema
  pinMode(LED_ROSSO_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_VERDE_PIN, OUTPUT);
  pinMode(ECHO_ULTRASONIC_PIN, INPUT);
  pinMode(TRIG_ULTRASONIC_PIN, OUTPUT);
  servo.attach(11);
}

int readColorMode(char color) {
  int samples[6];
  for (int i = 0; i < 6; i++) {
    switch(color) {
      case 'R': digitalWrite(S2_1_PIN, LOW);  digitalWrite(S3_1_PIN, LOW);  break;
      case 'G': digitalWrite(S2_1_PIN, HIGH); digitalWrite(S3_1_PIN, HIGH); break;
      case 'B': digitalWrite(S2_1_PIN, LOW);  digitalWrite(S3_1_PIN, HIGH); break;
    }
    samples[i] = pulseIn(OUT_1_PIN, LOW, 5000);
  }
  int modeVal = samples[0];
  int maxCount = 1;
  for (int i = 0; i < 6; i++) {
    int count = 1;
    for (int j = i + 1; j < 6; j++) {
      if (samples[j] == samples[i]) count++;
    }
    if (count > maxCount) {
      maxCount = count;
      modeVal = samples[i];
    }
  }
  return modeVal;
}

String rilevaColore() {
  int r = readColorMode('R');
  int g = readColorMode('G');
  int b = readColorMode('B');

  if (r < WHITE_THRESHOLD && g < WHITE_THRESHOLD && b < WHITE_THRESHOLD) {
    return "1"; //BIANCO
  } else if (r < RED_THRESHOLD && g > RED_THRESHOLD && b > BLUE_THRESHOLD) {
    return "2"; //ROSSO
  } else if ((r > DARK_THRESHOLD || g > DARK_THRESHOLD || b > DARK_THRESHOLD) && 
             (r < 140 || g < 140 || b < 140)) {
    return "3"; //SCURO
  } else {
    return "0"; //SCONOSCIUTO
  }
}

void updateRGB2() {
  red2 = green2 = blue2 = clear2 = 0;

  for (int i = 0; i < 5; i++) {
    digitalWrite(S2_2_PIN, LOW);
    digitalWrite(S3_2_PIN, LOW);
    red2 += pulseIn(OUT_2_PIN, LOW);
    delay(1);

    digitalWrite(S2_2_PIN, HIGH);
    digitalWrite(S3_2_PIN, LOW);
    clear2 += pulseIn(OUT_2_PIN, LOW);
    delay(1);

    digitalWrite(S2_2_PIN, HIGH);
    digitalWrite(S3_2_PIN, HIGH);
    green2 += pulseIn(OUT_2_PIN, LOW);
    delay(1);

    digitalWrite(S2_2_PIN, LOW);
    digitalWrite(S3_2_PIN, HIGH);
    blue2 += pulseIn(OUT_2_PIN, LOW);
    delay(1);
  }

  red2 /= 5;
  green2 /= 5;
  blue2 /= 5;
  clear2 /= 5;
}

String getColor2() {
  if (clear2 > NESSUN_OGGETTO_THRESHOLD_2) {
    return "0"; //SCONOSCIUTO
  }

  float R_G = (float)red2 / green2;
  float G_B = (float)green2 / blue2;
  float B_R = (float)blue2 / red2;

  if (R_G > 0.75 && R_G < 1.1 &&
      G_B > 0.9 && G_B < 1.4 &&
      B_R > 0.75 && B_R < 1.5 &&
      clear2 <= WHITE_THRESHOLD_2) {
    return "1"; //BIANCO
  }

  if (R_G <= 0.8 && G_B >= 1.1 && G_B <= 1.4 && B_R >= 1.2) {
    return "2"; //ROSSO
  }

  if (R_G > 0.9 && G_B < 1.2 && B_R < 1.45) {
    return "4"; //VERDE
  }

  if (R_G >= 0.9 && G_B >= 1.2 && B_R < 0.8) {
    return "5"; //BLU
  }

  if (R_G >= 0.7 && G_B < 1.2 && B_R >= 1) {
    return "6"; //GIALLO
  }

  return "0"; //SCONOSCIUTO
}

String distanza(){
  // Lettura ultrasuoni con mediana
      const int numReadings = 5;
      long readings[numReadings];
      
      for (int i = 0; i < numReadings; i++) {
        digitalWrite(TRIG_ULTRASONIC_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_ULTRASONIC_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_ULTRASONIC_PIN, LOW);

        readings[i] = pulseIn(ECHO_ULTRASONIC_PIN, HIGH, 30000) * 0.0343 / 2;
        delay(5);
      }

      // Ordinamento efficiente
      for (int i = 0; i < numReadings - 1; i++) {
        int minIdx = i;
        for (int j = i + 1; j < numReadings; j++) {
          if (readings[j] < readings[minIdx]) minIdx = j;
        }
        long temp = readings[i];
        readings[i] = readings[minIdx];
        readings[minIdx] = temp;
      }

      long dist = readings[numReadings / 2];
      return String((dist >= 2 && dist <= 400) ? dist : 0);
}

void loop() {
  if (!inizio) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');  // Legge fino al newline
      input.trim();
      if (input == "1") {
        digitalWrite(LED_ROSSO_PIN, HIGH);
        inizio = true;
        Serial.println("SYS|1");  // Aggiungi newline
      } else if (input == "7"){
          updateRGB2();
          Serial.println("COL1|"+rilevaColore()+"|COL2|"+getColor2()+"|DISTANZA|"+distanza());
          return;
      }
      return;
    }
  }

  // Gestione pulsante con debounce non bloccante
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == HIGH && !buttonPressed) {
      buttonPressed = true;
      start = !start;
      
      if (start && inizio) {
        servo.write(0);
        digitalWrite(LED_VERDE_PIN, HIGH);
        digitalWrite(LED_ROSSO_PIN, LOW);
        Serial.println("SYS|2"); 
      } else if (!start && inizio) {
        digitalWrite(LED_VERDE_PIN, LOW);
        digitalWrite(LED_ROSSO_PIN, LOW);
        inizio = false;
        while (true){
         Serial.println("SYS|3"); 
        }
      }
    }
    else if (buttonState == LOW && buttonPressed) {
      buttonPressed = false;
    }
  }
  lastButtonState = buttonState;

  if (start && Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "4") {
      Serial.print("DIST|");
      Serial.println(distanza());
    }
    else if (cmd == "5") {
      Serial.println("COL1|"+rilevaColore());
    }
    else if (cmd == "6") {
      updateRGB2();
      Serial.print("COL2|");
      Serial.println(getColor2());
    }
    else if (cmd.startsWith("SERVO1|")) {
      int angle = cmd.substring(7).toInt();
      
      if (angle >= 0 && angle <= 180) {
        servo.write(angle);
        Serial.println("SERVO|OK");
      } else {
        Serial.println("SERVO|-1");
      }
    }
    else if (cmd == "3") {
      start = false;
      inizio = false;
      digitalWrite(LED_VERDE_PIN, LOW);
      digitalWrite(LED_ROSSO_PIN, LOW);
      Serial.println("SYS|3");
    }
  }
}