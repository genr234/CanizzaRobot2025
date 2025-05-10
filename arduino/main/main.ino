#include <Servo.h>

// Definizione pin sensore colore 1
#define S2_1_PIN 8
#define S3_1_PIN 9
#define OUT_1_PIN 10


// Definizione pin sistema
#define BUTTON_PIN 2
#define LED_ROSSO_PIN 13
#define LED_VERDE_PIN 12
#define TRIG_ULTRASONIC_PIN 7
#define ECHO_ULTRASONIC_PIN 6
#define TRIG2_ULTRASONIC_PIN 4
#define ECHO2_ULTRASONIC_PIN 5

// Soglie primo sensore
#define WHITE_THRESHOLD 50         // ipotetico per bianco
#define RED_R_MAX 70               // R < 70
#define RED_G_MIN 90               // G > 90
#define RED_B_MIN 75               // B > 75
#define DARK_R_MIN 100             // R > 100
#define DARK_G_MIN 95              // G > 90
#define DARK_B_MIN 55  

// Soglie secondo sensore
#define WHITE_THRESHOLD_2 15
#define NESSUN_OGGETTO_THRESHOLD_2 17

Servo servo;
Servo servo2;

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
  
  // Sistema
  pinMode(LED_ROSSO_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_VERDE_PIN, OUTPUT);
  pinMode(ECHO_ULTRASONIC_PIN, INPUT);
  pinMode(TRIG_ULTRASONIC_PIN, OUTPUT);
  pinMode(ECHO2_ULTRASONIC_PIN, INPUT);
  pinMode(TRIG2_ULTRASONIC_PIN, OUTPUT);
  servo.attach(3);
  servo2.attach(11);
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

  // BIANCO = tutti bassi (tanta luce riflessa)
  if (r < WHITE_THRESHOLD && g < WHITE_THRESHOLD && b < WHITE_THRESHOLD) {
    return "1"; // BIANCO
  }

  // Rileva ROSSO (basso R, alti G e B)
  else if (r < RED_R_MAX && g > RED_G_MIN && b > RED_B_MIN) {
    return "2"; // ROSSO
  }

  // Rileva SCURO (nero/blu) se almeno due dei valori sono sopra soglia
  else if ((r > DARK_R_MIN && g > DARK_G_MIN) ||
           (r > DARK_R_MIN && b > DARK_B_MIN) ||
           (g > DARK_G_MIN && b > DARK_B_MIN)) {
    return "3"; // SCURO
  }

  return "0"; // SCONOSCIUTO
}

String distanza(int TRIG, int ECHO){
  // Lettura ultrasuoni con mediana
      const int numReadings = 5;
      long readings[numReadings];
      
      for (int i = 0; i < numReadings; i++) {
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);

        readings[i] = pulseIn(ECHO, HIGH, 30000) * 0.0343 / 2;
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
          Serial.println("COL1|"+rilevaColore()+"|DIST2|"+distanza(TRIG2_ULTRASONIC_PIN, ECHO2_ULTRASONIC_PIN)+"|DISTANZA|"+distanza(TRIG_ULTRASONIC_PIN, ECHO_ULTRASONIC_PIN));
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

  if (!start && Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "2"){
        servo.write(0);
        digitalWrite(LED_VERDE_PIN, HIGH);
        digitalWrite(LED_ROSSO_PIN, LOW);
        Serial.println("SYS|2");
        start = true;
      }
  }

  if (start && Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "4") {
      Serial.print("DIST|");
      Serial.println(distanza(TRIG2_ULTRASONIC_PIN, ECHO2_ULTRASONIC_PIN));
    }
    else if (cmd == "5") {
      Serial.println("COL1|"+rilevaColore());
    }
    else if (cmd == "6") {
      Serial.print("DIST2|");
      Serial.println(distanza(TRIG_ULTRASONIC_PIN, ECHO_ULTRASONIC_PIN));
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
  else if (cmd.startsWith("SERVO2|")) {
    int angle = cmd.substring(7).toInt();
    
    if (angle >= 0 && angle <= 180) {  // 🔧 Sistemato il limite
      servo2.write(angle);
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
