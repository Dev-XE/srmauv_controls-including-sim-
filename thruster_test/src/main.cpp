#include <Arduino.h>
#include <Servo.h>
#include <USBHost_t36.h>

// USB Host Setup for direct Xbox Controller connection
USBHost myusb;
USBHub hub1(myusb);
JoystickController joystick(myusb);

// Thruster Configuration
const int NUM_THRUSTERS = 8;
// Use PWM-capable pins on the Teensy 4.1
const int THRUSTER_PINS[NUM_THRUSTERS] = {2, 3, 4, 5, 6, 7, 8, 9};
Servo thrusters[NUM_THRUSTERS];

int active_thruster = 0; // 0-indexed (Thruster 1)

// Standard ROV ESC PWM bounds (in microseconds)
const int PWM_MIN = 1100;
const int PWM_NEUTRAL = 1500;
const int PWM_MAX = 1900;
const int DEADBAND = 30; // Prevents slight stick drift from spinning the motors

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for serial connection

  Serial.println("ROV Thruster Test Initializing...");
  
  // Start USB Host
  myusb.begin();
  Serial.println("USB Host started. Plug in the Xbox Controller.");

  // Attach thrusters and send neutral signal to arm ESCs
  for (int i = 0; i < NUM_THRUSTERS; i++) {
    thrusters[i].attach(THRUSTER_PINS[i]);
    thrusters[i].writeMicroseconds(PWM_NEUTRAL); 
  }

  Serial.println("ESCs initialized and armed (1500us sent).");
  Serial.println("\n--- COMMANDS ---");
  Serial.println("Type '1' through '8' in the Serial Monitor to select a thruster.");
  Serial.println("Use the Left Joystick (Up/Down) to throttle.");
  Serial.print("Currently Active: Thruster ");
  Serial.println(active_thruster + 1);
}

void loop() {
  // Update USB Host tasks
  myusb.Task();

  // 1. Handle Serial Monitor Input for Thruster Selection
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    
    if (incoming >= '1' && incoming <= '8') {
      // Safety: Force the previously active thruster to stop before switching
      thrusters[active_thruster].writeMicroseconds(PWM_NEUTRAL);
      
      active_thruster = incoming - '1'; // Convert char to 0-7 array index
      
      Serial.print(">>> Switched to Thruster ");
      Serial.println(active_thruster + 1);
    }
  }

  // 2. Handle Joystick Input
  int target_pwm = PWM_NEUTRAL; // Default to stopped

  if (joystick.available()) {
    // Axis 1 is typically the Left Joystick Y-Axis on standard controllers.
    // The USBHost_t36 library maps axis data from 0 to 65535 (32768 is center).
    uint32_t left_y = joystick.getAxis(1); 

    // Map the 16-bit joystick value to the 1100-1900us PWM range
    // Note: Depending on the specific controller, you may need to swap PWM_MAX and PWM_MIN 
    // in the map function if forward/backward are inverted.
    target_pwm = map(left_y, 0, 65535, PWM_MIN, PWM_MAX);

    // Apply deadband
    if (abs(target_pwm - PWM_NEUTRAL) < DEADBAND) {
        target_pwm = PWM_NEUTRAL;
    }

    joystick.joystickDataClear(); // Clear the flag so we don't read stale data
  }

  // 3. Command the Thrusters
  for (int i = 0; i < NUM_THRUSTERS; i++) {
    if (i == active_thruster) {
      thrusters[i].writeMicroseconds(target_pwm); // Send joystick command
    } else {
      thrusters[i].writeMicroseconds(PWM_NEUTRAL); // Keep all others disarmed/stopped
    }
  }

  delay(5); // Small loop delay for stability
}