#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// LCD setup: I2C address 0x27, 16 columns, 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

Servo servo; // Servo motor object

// SoftwareSerial for Bluetooth module: RX pin 4, TX pin 5
SoftwareSerial bluetoothSerial(4, 5);

int bt_data = -1; // Stores incoming Bluetooth data, initialized to -1

// Pin definitions for components
#define WHITE_LAMP_PIN 12
#define FAN_PIN      11
#define BUZZER_PIN    9
#define GREEN_LED_PIN  7
#define RED_LED_PIN    8
#define SERVO_PIN    10

// --- Device Control Functions ---

void setWhiteLampState(bool state) {
  digitalWrite(WHITE_LAMP_PIN, state);
  lcd.setCursor(0, 1); // Move to second line for status
  lcd.print("Lamp: ");
  lcd.print(state == HIGH ? "OFF " : "ON"); // Display status
  lcd.print("      "); // Clear remaining characters
}

void setFanState(bool state) {
  digitalWrite(FAN_PIN, state);
  lcd.setCursor(0, 1); // Move to second line for status
  lcd.print("Fan: ");
  lcd.print(state == HIGH ? "OFF " : "ON"); // Display status
  lcd.print("      "); // Clear remaining characters
}

// NOTE: Red LED state is now managed directly within controlServo,
// but we keep the function if you decide to add separate control later.
void setRedLedState(bool state) {
  digitalWrite(RED_LED_PIN, state);
  // No LCD update here as it's typically tied to servo status
}


void controlServo(bool open) {
  if (open) {
    for (int i = servo.read(); i < 90; i++) { // Open to 90 degrees
      servo.write(i);
      delay(20);
    }
    // Action when door opens: two short beeps and blinks
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(RED_LED_PIN, HIGH); // Turn Red LED ON
      delay(100); // Beep/Blink duration
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW); // Turn Red LED OFF
      delay(200); // Pause between beeps/blinks
    }
  } else {
    for (int i = servo.read(); i > 0; i--) { // Close to 0 degrees
      servo.write(i);
      delay(20);
    }
    // Action when door closes: two short beeps and blinks
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(RED_LED_PIN, HIGH); // Turn Red LED ON
      delay(100); // Beep/Blink duration
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW); // Turn Red LED OFF
      delay(200); // Pause between beeps/blinks
    }
  }
  lcd.setCursor(0, 1); // Update LCD for servo status
  lcd.print("Door: ");
  lcd.print(open ? "OPEN " : "CLOSE");
  lcd.print("     "); // Clear remaining characters
}


void setAllDevicesState(bool turnOn) {
  setWhiteLampState(turnOn ? HIGH : LOW);
  setFanState(turnOn ? HIGH : LOW);
  // NOTE: Buzzer and Red LED are NOT included in "All Devices" control
  // as they are specifically for door alerts now.
  lcd.setCursor(0, 1); // Update LCD for master control status
  lcd.print("All Devices: ");
  lcd.print(turnOn ? "OFF " : "ON");
  lcd.print("   "); // Clear remaining characters
}


void setup() {
  servo.attach(SERVO_PIN); // Attach servo to its pin

  // Configure device pins as outputs
  pinMode(WHITE_LAMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HOME AUTOMATION");

  // Initialize serial communication
  Serial.begin(9600);     // For debugging (PC)
  bluetoothSerial.begin(9600); // For Bluetooth module

  digitalWrite(GREEN_LED_PIN, HIGH); // Green LED indicates system ready
  delay(40);

  // Initial state: turn all devices off and close servo
  setAllDevicesState(true);
  controlServo(false); // This will also trigger the close alert
}

void loop() {
  // Read data from Bluetooth if available
  if (bluetoothSerial.available() > 0) {
    bt_data = bluetoothSerial.read();
    Serial.print("Received: ");
    Serial.println(bt_data);
  }

  // Handle incoming commands
  switch (bt_data) {
    // White Lamp commands (1 & 12 for ON, 2 & 11 for OFF)
    case 1:
    case 12:
      setWhiteLampState(HIGH);
      break;
    case 2:
    case 11:
      setWhiteLampState(LOW);
      break;

    // Fan commands (5 & 16 for ON, 6 & 15 for OFF)
    case 5:
    case 16:
      setFanState(HIGH);
      break;
    case 6:
    case 15:
      setFanState(LOW);
      break;

    // Servo commands (9 & 19 for Open, 10 & 20 for Close)
    case 9:
    case 19:
      controlServo(true); // This now includes the buzzer/red LED alert
      break;
    case 10:
    case 20:
      controlServo(false); // This now includes the buzzer/red LED alert
      break;

    // Master commands (22 for ALL ON, 21 for ALL OFF)
    case 22:
      setAllDevicesState(true);
      break;
    case 21:
      setAllDevicesState(false);
      break;

    default:
      // No action for other commands or if no new data
      break;
  }

  // Reset bt_data to avoid continuous command execution
  if (bt_data != -1) {
    bt_data = -1;
  }

  delay(30); // Short delay for stable operation
}