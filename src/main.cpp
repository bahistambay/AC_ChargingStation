#include <Arduino.h>
#include "../EVSE.h"
#include "PWM.h"

// Pin definitions
#define CP_OUT 10           // Control Pilot signal generator output
#define CP_IN A0            // Control Pilot signal reader input
#define RELAY_PIN 16        // Main contactor relay
#define VENTILATION_PIN 17  // Ventilation control relay (new)
#define LED_TOP 6           // Top status LED
#define LED_BOTTOM 2        // Bottom status LED  
#define LED_MIDDLE_D 3      // Middle down LED
#define LED_MIDDLE_U 4      // Middle up LED

// EVSE Parameters
#define PWM_FREQUENCY 1000  // 1 kHz PWM frequency per J1772 standard
#define MAX_PWM_VALUE 255   // Maximum PWM value (12V)
#define MIN_PWM_VALUE 0     // Minimum PWM value (-12V)

// Charging current configuration
#define MAX_CHARGING_CURRENT 10  // Maximum charging current in Amperes
#define MIN_CHARGING_CURRENT 6   // J1772 minimum
#define PWM_DUTY_MULTIPLIER 0.6  // J1772 standard: Duty% = Current/0.6

// Voltage thresholds (ADC values for 10-bit ADC)
// Based on empirical testing: PWM=255 gave 740 ADC counts, PWM=0 gives 416 ADC counts
#define VOLTAGE_THRESHOLD_A 730  // +12V, State A: No vehicle connected. Actual ADC ~740
#define VOLTAGE_THRESHOLD_B 685  // +9V, State B: Vehicle connected. Actual ADC ~701
#define VOLTAGE_THRESHOLD_C 640  // +6V, State C: Ready to charge. Actual ADC ~664
#define VOLTAGE_THRESHOLD_D 595  // +3V, State D: Vent required //Not used for Lithium Batteries
#define VOLTAGE_THRESHOLD_E 550  // 0V, cable disconnected or error

// Diode check specific thresholds
#define DIODE_CHECK_EXPECTED_FORWARD_ADC 640 //~6V
#define DIODE_CHECK_EXPECTED_REVERSE_ADC 416 //-12V
#define DIODE_CHECK_TOLERANCE 40

// Below 100: State F (fault condition, -12V or disconnect)
// NOTE: These values are based on actual circuit behavior, not theoretical calculations

// Timing constants
#define DEBUG_INTERVAL 100       // Debug message interval (main loop cycles)
#define LED_BLINK_INTERVAL 50    // LED blink interval during charging
#define PEAK_DETECTION_SAMPLES 100 // Reduced from 1000 for better responsiveness
#define VENTILATION_DELAY 2000   // 2 second delay to start ventilation before charging
#define DIODE_CHECK_DURATION 1000 // 1 second for diode check procedure
#define DIODE_CHECK_SETTLE_TIME 100 // 100ms settle time between voltage changes

// State enumeration for better code readability
enum EVSEState {
  STATE_A = 'A',  // Standby - no vehicle
  STATE_B = 'B',  // Vehicle connected
  STATE_C = 'C',  // Charging
  STATE_D = 'D',  // Charging with ventilation required
  STATE_E = 'E',  // No power available
  STATE_F = 'F',  // Fault
  STATE_DIODE_CHECK_FAIL = 'X',  // Diode check failure
  STATE_DIODE_CHECK_ACTIVE = 'T'  // Diode check in progress
};

// Note: The J1772 standard requires a diode in the vehicle's CP circuit
// to ensure galvanic isolation and to allow the EVSE to verify a real,
// properly isolated OBC is connected. This prevents ground loops and
// ensures safe, compliant operation before closing the contactor.
// Diode check states
enum DiodeCheckState {
  DIODE_CHECK_IDLE,
  DIODE_CHECK_INIT,
  DIODE_CHECK_POSITIVE_TEST,
  DIODE_CHECK_NEGATIVE_TEST,
  DIODE_CHECK_COMPLETE,
  DIODE_CHECK_FAILED
};

// Global variables
static int debug_counter = 0;
static int led_blink_counter = 0;
static int peak_voltage = 0;
static EVSEState current_state = STATE_A;
static bool is_charging = false;
static bool relay_state = false;
static bool ventilation_state = false;
static bool diode_check_active = false;
static int diode_check_samples = 0;
static unsigned long last_state_change = 0;
static unsigned long ventilation_start_time = 0;
static const unsigned long STATE_CHANGE_DEBOUNCE = 500; // 500ms debounce

// Diode check variables
static DiodeCheckState diode_check_state = DIODE_CHECK_IDLE;
static unsigned long diode_check_start_time = 0;
static unsigned long diode_check_step_time = 0;
static int diode_check_positive_voltage = 0;
static int diode_check_negative_voltage = 0;
static bool diode_check_passed = false;
static int diode_check_attempts = 0;
static const int MAX_DIODE_CHECK_ATTEMPTS = 3;
static int disconnect_counter = 0;

// Function prototypes
int findPeakVoltage();
void updateLEDs();
void setChargingCurrent(int current_amps);
void handleStateTransition(EVSEState new_state);
void emergencyStop();
bool isValidStateTransition(EVSEState from, EVSEState to);
void controlVentilation(bool enable);
bool isVentilationReady();
bool performDiodeCheck();
void updateDiodeCheck();
void startDiodeCheck();
void completeDiodeCheck(bool passed);
bool isDiodeCheckRequired(EVSEState state);
void resetDiodeCheck();

/**
 * Initialize EVSE system
 */
void initEVSE() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("EVSE System Initializing...");

  // Initialize PWM system
  InitTimersSafe();
  bool pwm_success = SetPinFrequencySafe(CP_OUT, PWM_FREQUENCY);
  
  if (pwm_success) {
    pinMode(CP_OUT, OUTPUT);
    pwmWrite(CP_OUT, MAX_PWM_VALUE); // Start with +12V (State A)
    Serial.println("PWM initialized successfully");
  } else {
    Serial.println("ERROR: PWM initialization failed!");
    return;
  }

  // Initialize digital pins
  pinMode(LED_TOP, OUTPUT);
  pinMode(LED_BOTTOM, OUTPUT);
  pinMode(LED_MIDDLE_D, OUTPUT);
  pinMode(LED_MIDDLE_U, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(VENTILATION_PIN, OUTPUT);

  // Set initial states
  digitalWrite(RELAY_PIN, LOW);         // Ensure relay is open
  digitalWrite(VENTILATION_PIN, LOW);   // Ensure ventilation is off
  digitalWrite(LED_TOP, HIGH);          // Top LED indicates system ready
  digitalWrite(LED_BOTTOM, LOW);
  digitalWrite(LED_MIDDLE_D, LOW);
  digitalWrite(LED_MIDDLE_U, LOW);

  // Initialize state
  current_state = STATE_A;
  is_charging = false;
  relay_state = false;
  ventilation_state = false;
  last_state_change = millis();
  ventilation_start_time = 0;
  
  // Initialize diode check
  resetDiodeCheck();

  Serial.println("EVSE System Ready - State A");
}

/**
 * Main EVSE update loop
 */
void updateEVSE() {
    // --- Fault state safe pulse-recovery logic ---
  static unsigned long last_fault_pulse = 0;
  const unsigned long FAULT_PULSE_INTERVAL = 5000; // 5 seconds between pulses
  const unsigned long FAULT_PULSE_DURATION = 100;  // 100 ms pulse duration
  static bool pulse_active = false;
  static unsigned long pulse_start_time = 0;

  if (current_state == STATE_F) {
    unsigned long now = millis();
    if (!pulse_active && now - last_fault_pulse > FAULT_PULSE_INTERVAL) {
      // Start a +12V pulse
      pwmWrite(CP_OUT, MAX_PWM_VALUE); // +12V
      pulse_start_time = now;
      pulse_active = true;
      last_fault_pulse = now;
      Serial.println("FAULT: Sending +12V pulse to check for unplug");
    }
    if (pulse_active && now - pulse_start_time > FAULT_PULSE_DURATION) {
      // Check ADC during/after pulse
      int measured_voltage = findPeakVoltage();
      if (measured_voltage > VOLTAGE_THRESHOLD_A) {
        Serial.println("Recovered from fault: Returning to State A (unplug detected)");
        handleStateTransition(STATE_A);
        pulse_active = false; // End pulse
        pwmWrite(CP_OUT, MAX_PWM_VALUE); // Keep at +12V in State A
        return;
      } else {
        // End pulse and return to -12V (stay in fault)
        pwmWrite(CP_OUT, MIN_PWM_VALUE); // -12V
        pulse_active = false;
        Serial.println("FAULT: Plug still present or not unplugged");
      }
    }
  }

  // Debug output
  debug_counter++;
  if (debug_counter >= DEBUG_INTERVAL) {
    debug_counter = 0;
    Serial.print("State: ");
    Serial.print((char)current_state);
    Serial.print(" | Peak Voltage: ");
    Serial.print(peak_voltage);
    Serial.print(" | Charging: ");
    Serial.print(is_charging ? "YES" : "NO");
    Serial.print(" | Relay: ");
    Serial.print(relay_state ? "CLOSED" : "OPEN");
    Serial.print(" | Ventilation: ");
    Serial.print(ventilation_state ? "ON" : "OFF");
    Serial.print(" | Diode Check: ");
    Serial.println(diode_check_active ? "ACTIVE" : "IDLE");
  }

  // Handle diode check procedure
  if (diode_check_active) {
    updateDiodeCheck();
    return; // Don't process normal state logic during diode check
  }

  // Handle charging LED animation
  if (is_charging) {
    led_blink_counter++;
    if (led_blink_counter >= LED_BLINK_INTERVAL) {
      led_blink_counter = 0;
      // Alternate middle LEDs during charging
      if (digitalRead(LED_MIDDLE_D) == HIGH) {
        digitalWrite(LED_MIDDLE_D, LOW);
        digitalWrite(LED_MIDDLE_U, HIGH);
      } else {
        digitalWrite(LED_MIDDLE_D, HIGH);
        digitalWrite(LED_MIDDLE_U, LOW);
      }
    }
  }

  // Read CP voltage and determine state
  int measured_voltage = findPeakVoltage();
  EVSEState detected_state;

  // Determine state based on voltage thresholds
  if (measured_voltage > VOLTAGE_THRESHOLD_A) {
    detected_state = STATE_A;
  } else if (measured_voltage > VOLTAGE_THRESHOLD_B) {
    detected_state = STATE_B;
  } else if (measured_voltage > VOLTAGE_THRESHOLD_C) {
    detected_state = STATE_C;
  } else if (measured_voltage > VOLTAGE_THRESHOLD_D) {
    detected_state = STATE_D;
  } else if (measured_voltage > VOLTAGE_THRESHOLD_E) {
    detected_state = STATE_E;
  } else {
    detected_state = STATE_F;
  }

  // Handle state changes with debouncing
  if (detected_state != current_state) {
    unsigned long current_time = millis();
    if (current_time - last_state_change > STATE_CHANGE_DEBOUNCE) {
      if (isValidStateTransition(current_state, detected_state)) {
        // Check if diode check is required before transitioning to charging states
        if (isDiodeCheckRequired(detected_state)) {
          startDiodeCheck();
          return;
        }
        
        handleStateTransition(detected_state);
        last_state_change = current_time;
      } else {
        Serial.println("WARNING: Invalid state transition detected");
        emergencyStop();
      }
    }
  }

  // Handle State D ventilation timing
  if (current_state == STATE_D && ventilation_start_time > 0) {
    if (millis() - ventilation_start_time >= VENTILATION_DELAY && !is_charging) {
      if (isVentilationReady()) {
        // Start charging after ventilation delay
        is_charging = true;
        relay_state = true;
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("State D: Ventilation ready - starting charging");
      }
    }
  }

  // Detect unexpected CP voltage drop during charging (State C)
  if (current_state == STATE_C && !diode_check_active) {
    if (peak_voltage < VOLTAGE_THRESHOLD_C - 20) {
      disconnect_counter++;
      if (disconnect_counter >= 5) {  // Allow 5 bad readings
        Serial.println("ERROR: CP drop detected during charging. Stopping.");
        emergencyStop();
      }
    } else {
      disconnect_counter = 0; // Reset if normal
    }
  }

  // Update LEDs based on current state
  updateLEDs();
}

/**
 * Start diode check procedure
 */
void startDiodeCheck() {
  Serial.println("Starting diode check procedure...");
  diode_check_active = true;
  diode_check_state = DIODE_CHECK_INIT;
  diode_check_start_time = millis();
  diode_check_step_time = millis();
  diode_check_attempts++;
  current_state = STATE_DIODE_CHECK_ACTIVE;
  
  // Initialize test values
  diode_check_positive_voltage = 0;
  diode_check_negative_voltage = 0;
  diode_check_passed = false;
}

/**
 * Update diode check procedure
 */
void updateDiodeCheck() {
  unsigned long current_time = millis();
  
  switch (diode_check_state) {
    case DIODE_CHECK_INIT:
      Serial.println("Diode check: Initializing...");
      // Start with -12V output
      pwmWrite(CP_OUT, MIN_PWM_VALUE);
      diode_check_state = DIODE_CHECK_NEGATIVE_TEST;
      diode_check_step_time = current_time;
      break;
      
    case DIODE_CHECK_POSITIVE_TEST:
      // Wait for voltage to settle
      if (current_time - diode_check_step_time >= DIODE_CHECK_SETTLE_TIME) {
        diode_check_positive_voltage = findPeakVoltage();
        Serial.print("Diode check: Positive test voltage = ");
        Serial.println(diode_check_positive_voltage);
        
        diode_check_state = DIODE_CHECK_COMPLETE;
      }
      break;
      
    case DIODE_CHECK_NEGATIVE_TEST:
      // Wait for voltage to settle
      if (current_time - diode_check_step_time >= DIODE_CHECK_SETTLE_TIME) {
        diode_check_negative_voltage = findPeakVoltage();
        Serial.print("Diode check: Negative test voltage = ");
        Serial.println(diode_check_negative_voltage);
        
         // Switch to +12V output
        pwmWrite(CP_OUT, MAX_PWM_VALUE);
        diode_check_state = DIODE_CHECK_POSITIVE_TEST;
        diode_check_step_time = current_time;
        
      }
      break;
      
    case DIODE_CHECK_COMPLETE:
      // Analyze results
      bool positive_ok = (diode_check_positive_voltage >= DIODE_CHECK_EXPECTED_FORWARD_ADC - DIODE_CHECK_TOLERANCE) &&
                        (diode_check_positive_voltage <= DIODE_CHECK_EXPECTED_FORWARD_ADC + DIODE_CHECK_TOLERANCE);

      bool negative_ok = (diode_check_negative_voltage <= DIODE_CHECK_EXPECTED_REVERSE_ADC + DIODE_CHECK_TOLERANCE);

      bool diode_behavior_ok = (diode_check_positive_voltage > diode_check_negative_voltage + 100);

      diode_check_passed = positive_ok && negative_ok && diode_behavior_ok;

      Serial.print("Diode check results: Positive=");
      Serial.print(positive_ok ? "PASS" : "FAIL");
      Serial.print(", Negative=");
      Serial.print(negative_ok ? "PASS" : "FAIL");
      Serial.print(", Behavior=");
      Serial.print(diode_behavior_ok ? "PASS" : "FAIL");
      Serial.print(", Overall=");
      Serial.println(diode_check_passed ? "PASS" : "FAIL");

      completeDiodeCheck(diode_check_passed);
      break;

      
    case DIODE_CHECK_FAILED:
      // Handle failure
      Serial.println("Diode check failed - entering fault state");
      completeDiodeCheck(false);
      break;
  }
  
  // Timeout check
  if (current_time - diode_check_start_time > DIODE_CHECK_DURATION) {
    Serial.println("Diode check timeout - assuming failure");
    completeDiodeCheck(false);
  }
}

/**
 * Complete diode check procedure
 */
void completeDiodeCheck(bool passed) {
  diode_check_active = false;
  diode_check_state = DIODE_CHECK_IDLE;
  
  if (passed) {
    Serial.println("Diode check PASSED - proceeding with charging");
    diode_check_attempts = 0; // Reset attempt counter on success
    
    // Start with +12V output
    //pwmWrite(CP_OUT, MAX_PWM_VALUE);

    // Restore appropriate PWM for current state and proceed
    int measured_voltage = findPeakVoltage();
    EVSEState detected_state;
    Serial.print("Post diode check peak voltage ");
    Serial.print(measured_voltage);
    // Re-determine state after diode check
    if (measured_voltage > VOLTAGE_THRESHOLD_A) {
      detected_state = STATE_A;
    } else if (measured_voltage > VOLTAGE_THRESHOLD_B) {
      detected_state = STATE_B;
    } else if (measured_voltage > VOLTAGE_THRESHOLD_C) {
      detected_state = STATE_C;
    } else if (measured_voltage > VOLTAGE_THRESHOLD_D) {
      detected_state = STATE_D;
    } else if (measured_voltage > VOLTAGE_THRESHOLD_E) {
      detected_state = STATE_E;
    } else {
      detected_state = STATE_F;
    }
    
    handleStateTransition(detected_state);
  } else {
    Serial.print("Diode check FAILED (attempt ");
    Serial.print(diode_check_attempts);
    Serial.print("/");
    Serial.print(MAX_DIODE_CHECK_ATTEMPTS);
    Serial.println(")");
    
    if (diode_check_attempts >= MAX_DIODE_CHECK_ATTEMPTS) {
      Serial.println("Maximum diode check attempts reached - entering fault state");
      current_state = STATE_DIODE_CHECK_FAIL;
      emergencyStop();
    } else {
      Serial.println("Retrying diode check...");
      delay(1000); // Wait before retry
      startDiodeCheck();
    }
  }
}

/**
 * Check if diode check is required for the given state
 */
bool isDiodeCheckRequired(EVSEState state) {
  // Diode check is required before entering charging states (C or D)
  return (state == STATE_C || state == STATE_D) && !diode_check_passed;
}

/**
 * Reset diode check status
 */
void resetDiodeCheck() {
  diode_check_active = false;
  diode_check_state = DIODE_CHECK_IDLE;
  diode_check_passed = false;
  diode_check_attempts = 0;
  diode_check_start_time = 0;
  diode_check_step_time = 0;
  diode_check_positive_voltage = 0;
  diode_check_negative_voltage = 0;
}

/**
 * Find peak voltage on CP line
 */
int findPeakVoltage() {
  int peak = 0;
  int current_reading;
  
  // Sample the CP line multiple times to find peak
  for (int i = 0; i < PEAK_DETECTION_SAMPLES; i++) {
    current_reading = analogRead(CP_IN);
    if (current_reading > peak) {
      peak = current_reading;
    }
    delayMicroseconds(10); // Small delay between samples
  }
  
  peak_voltage = peak;
  return peak;
}

/**
 * Handle state transitions
 */
void handleStateTransition(EVSEState new_state) {
  Serial.print("State transition: ");
  Serial.print((char)current_state);
  Serial.print(" -> ");
  Serial.println((char)new_state);

  current_state = new_state;

  switch (new_state) {
    case STATE_A:
      // Standby - no vehicle connected
      is_charging = false;
      relay_state = false;
      ventilation_state = false;
      ventilation_start_time = 0;
      digitalWrite(RELAY_PIN, LOW);
      controlVentilation(false);
      pwmWrite(CP_OUT, MAX_PWM_VALUE); // +12V
      resetDiodeCheck(); // Reset diode check when vehicle disconnects
      Serial.println("Vehicle disconnected");
      break;

    case STATE_B:
      // Vehicle connected, not ready to charge - maintain solid +9V
      is_charging = false;
      relay_state = false;
      ventilation_state = false;
      ventilation_start_time = 0;
      digitalWrite(RELAY_PIN, LOW);
      controlVentilation(false);
      resetDiodeCheck(); // Reset diode check when vehicle disconnects
      Serial.println("Vehicle connected - ready to charge");
      break;

    case STATE_C:
      // Charging state - only enter if diode check passed
      
      if (!diode_check_passed) {
        Serial.println("ERROR: Attempting to enter State C without passing diode check");
        emergencyStop();
        return;
      }
      
      is_charging = true;
      relay_state = true;
      ventilation_state = false;
      ventilation_start_time = 0;
      digitalWrite(RELAY_PIN, HIGH);
      controlVentilation(false);
      setChargingCurrent(MAX_CHARGING_CURRENT);
      Serial.println("Charging started");
      break;

    case STATE_D:
      // Charging with ventilation required - only enter if diode check passed
      if (!diode_check_passed) {
        Serial.println("ERROR: Attempting to enter State D without passing diode check");
        emergencyStop();
        return;
      }
      is_charging = false; // Will be set to true after ventilation delay
      relay_state = false;
      ventilation_state = true;
      ventilation_start_time = millis();
      digitalWrite(RELAY_PIN, LOW); // Keep relay open until ventilation is ready
      controlVentilation(true);
      setChargingCurrent(MAX_CHARGING_CURRENT);
      Serial.println("Vehicle requires ventilation - starting ventilation system");
      break;

    case STATE_E:
      // No power available - maintain 0V
      is_charging = false;
      relay_state = false;
      ventilation_state = false;
      ventilation_start_time = 0;
      digitalWrite(RELAY_PIN, LOW);
      controlVentilation(false);
      pwmWrite(CP_OUT, 85); // Approximate PWM value for 0V (adjust based on your circuit)
      Serial.println("No power available");
      break;

    case STATE_DIODE_CHECK_ACTIVE:
      // Diode check in progress - maintain current relay/ventilation state
      Serial.println("Diode check in progress");
      break;

    case STATE_DIODE_CHECK_FAIL:
      // Diode check failure - emergency stop
      emergencyStop();
      Serial.println("DIODE CHECK FAILURE: Vehicle diode malfunction detected");
      break;

    case STATE_F:
      // Fault state
      emergencyStop();
      Serial.println("FAULT: Emergency stop activated");
      break;
  }
}

/**
 * Update LED display based on current state
 */
void updateLEDs() {
  switch (current_state) {
    case STATE_A:
      digitalWrite(LED_TOP, HIGH);      // System ready
      digitalWrite(LED_BOTTOM, LOW);
      digitalWrite(LED_MIDDLE_D, LOW);
      digitalWrite(LED_MIDDLE_U, LOW);
      break;

    case STATE_B:
      digitalWrite(LED_TOP, HIGH);      // System ready
      digitalWrite(LED_BOTTOM, HIGH);   // Vehicle connected
      digitalWrite(LED_MIDDLE_D, LOW);
      digitalWrite(LED_MIDDLE_U, LOW);
      break;

    case STATE_C:
      digitalWrite(LED_TOP, HIGH);      // System ready
      digitalWrite(LED_BOTTOM, HIGH);   // Vehicle connected
      // Middle LEDs handled by charging animation
      break;

    case STATE_D:
      digitalWrite(LED_TOP, HIGH);      // System ready
      digitalWrite(LED_BOTTOM, HIGH);   // Vehicle connected
      // Show ventilation status with rapid blinking
      if (ventilation_state && !is_charging) {
        // Rapid blink to indicate ventilation mode
        if ((millis() / 100) % 2) {
          digitalWrite(LED_MIDDLE_D, HIGH);
          digitalWrite(LED_MIDDLE_U, LOW);
        } else {
          digitalWrite(LED_MIDDLE_D, LOW);
          digitalWrite(LED_MIDDLE_U, HIGH);
        }
      }
      // If charging in State D, use normal charging animation
      break;

    case STATE_E:
      digitalWrite(LED_TOP, LOW);       // No power
      digitalWrite(LED_BOTTOM, LOW);
      digitalWrite(LED_MIDDLE_D, LOW);
      digitalWrite(LED_MIDDLE_U, LOW);
      break;

    case STATE_DIODE_CHECK_ACTIVE:
      // Slow pulse all LEDs to indicate diode check in progress
      if ((millis() / 500) % 2) {
        digitalWrite(LED_TOP, HIGH);
        digitalWrite(LED_BOTTOM, HIGH);
        digitalWrite(LED_MIDDLE_D, HIGH);
        digitalWrite(LED_MIDDLE_U, HIGH);
      } else {
        digitalWrite(LED_TOP, LOW);
        digitalWrite(LED_BOTTOM, LOW);
        digitalWrite(LED_MIDDLE_D, LOW);
        digitalWrite(LED_MIDDLE_U, LOW);
      }
      break;

    case STATE_DIODE_CHECK_FAIL:
      // Flash all LEDs to indicate diode failure
      if ((millis() / 200) % 2) {
        digitalWrite(LED_TOP, HIGH);
        digitalWrite(LED_BOTTOM, HIGH);
        digitalWrite(LED_MIDDLE_D, HIGH);
        digitalWrite(LED_MIDDLE_U, HIGH);
      } else {
        digitalWrite(LED_TOP, LOW);
        digitalWrite(LED_BOTTOM, LOW);
        digitalWrite(LED_MIDDLE_D, LOW);
        digitalWrite(LED_MIDDLE_U, LOW);
      }
      break;

    case STATE_F:
      digitalWrite(LED_TOP, LOW);       // System fault
      digitalWrite(LED_BOTTOM, LOW);
      digitalWrite(LED_MIDDLE_D, LOW);
      digitalWrite(LED_MIDDLE_U, LOW);
      break;
  }
}

/**
 * Set charging current by adjusting PWM duty cycle
 */
void setChargingCurrent(int current_amps) {
  if (current_amps < MIN_CHARGING_CURRENT || current_amps > MAX_CHARGING_CURRENT) {
    Serial.println("ERROR: Invalid charging current specified");
    return;
  }

  // J1772 standard: Duty cycle = Current / 0.6
  float duty_cycle = (float)current_amps / PWM_DUTY_MULTIPLIER / 100.0;
  int pwm_value = (int)(MAX_PWM_VALUE * duty_cycle);
  
  // Clamp PWM value to valid range
  pwm_value = constrain(pwm_value, 15, 240); // Avoid extremes
  
  pwmWrite(CP_OUT, pwm_value);
  
  Serial.print("Charging current set to ");
  Serial.print(current_amps);
  Serial.print("A (PWM: ");
  Serial.print(pwm_value);
  Serial.println(")");
}

/**
 * Control ventilation system
 */
void controlVentilation(bool enable) {
  ventilation_state = enable;
  digitalWrite(VENTILATION_PIN, enable ? HIGH : LOW);
  
  Serial.print("Ventilation system: ");
  Serial.println(enable ? "ON" : "OFF");
}

/**
 * Check if ventilation system is ready
 */
bool isVentilationReady() {
  // In a real implementation, this would check:
  // Only for Lead-Acid Batteries
  // - Ventilation fan speed
  // - Air flow sensors
  // - Gas detection sensors
  // For now, we assume ready after the delay period
  return ventilation_state && (millis() - ventilation_start_time >= VENTILATION_DELAY);
}

/**
 * Emergency stop function
 */
void emergencyStop() {
  is_charging = false;
  relay_state = false;
  ventilation_state = false;
  current_state = STATE_F;
  ventilation_start_time = 0;
  
  // Stop any active diode check
  resetDiodeCheck(); // Reset diode check at fault

  // Immediately open relay, stop ventilation, and set fault PWM
  digitalWrite(RELAY_PIN, LOW);
  controlVentilation(false);
  pwmWrite(CP_OUT, MIN_PWM_VALUE); // -12V indicates fault
  
  // Turn off all LEDs except fault indication
  digitalWrite(LED_TOP, LOW);
  digitalWrite(LED_BOTTOM, LOW);
  digitalWrite(LED_MIDDLE_D, LOW);
  digitalWrite(LED_MIDDLE_U, LOW);
  
  Serial.println("EMERGENCY STOP: All systems halted");
}

/**
 * Validate state transitions according to J1772 standard
 */
bool isValidStateTransition(EVSEState from, EVSEState to) {
  // Define valid state transitions
  switch (from) {
    case STATE_A:
      return (to == STATE_B || to == STATE_E || to == STATE_F);
    case STATE_B:
      return (to == STATE_A || to == STATE_C || to == STATE_D || to == STATE_E || to == STATE_F || to == STATE_DIODE_CHECK_ACTIVE);
    case STATE_C:
      return (to == STATE_B || to == STATE_A || to == STATE_E || to == STATE_F);
    case STATE_D:
      return (to == STATE_B || to == STATE_A || to == STATE_E || to == STATE_F);
    case STATE_E:
      return (to == STATE_A || to == STATE_F);
    case STATE_DIODE_CHECK_ACTIVE:
      return (to == STATE_C || to == STATE_D || to == STATE_DIODE_CHECK_FAIL || to == STATE_F || to == STATE_A || to == STATE_B);
    case STATE_DIODE_CHECK_FAIL:
      return (to == STATE_F || to == STATE_A); // Can only go to fault or recovery
    case STATE_F:
      return (to == STATE_A); // Recovery only to State A
    default:
      return false;
  }
}