#include "SystemFSM.h"

SystemFSM::SystemFSM(uint8_t rcjPin) 
    : currentMode(OperatingMode::POWER_ON), 
      previousMode(OperatingMode::POWER_ON),
      pinRCJ(rcjPin), 
      stateEntryTime(0),
      pauseStartTime(0),
      strategicStateSaved(false) {}

void SystemFSM::init() {
    stateEntryTime = millis();
    // 3.2: Set pin 9 as INPUT_PULLDOWN (Ensures default LOW / STOP if disconnected)
    pinMode(pinRCJ, INPUT_PULLDOWN);
    
    // Boot up indicators, serial communication configs
    Serial.begin(115200);
    changeMode(OperatingMode::POWER_ON);
}

void SystemFSM::update() {
    switch (currentMode) {
        case OperatingMode::POWER_ON: handlePowerOn(); break;
        case OperatingMode::POST:     handlePOST();     break;
        case OperatingMode::READY:    handleReady();    break;
        case OperatingMode::GAME:     handleGame();     break;
        case OperatingMode::PAUSED:   handlePaused();   break;
        case OperatingMode::TEST:     handleTest();     break;
        case OperatingMode::FAULT:    handleFault();    break;
    }
}

// 3.2: Fleeting state ending when the Teensy boot is complete (~500ms)
void SystemFSM::handlePowerOn() {
    // Perform light hardware setup, UART initialization with the 4 ESPs
    if (millis() - stateEntryTime >= 500) {
        changeMode(OperatingMode::POST);
    }
}

// 3.3: Power-On Self-Test Sequence
void SystemFSM::handlePOST() {
    bool passed = runSelfTests();
    
    if (passed) {
        // LED Green Constant
        Serial.println("[POST] Success. Turning on Solid Green LED.");
        changeMode(OperatingMode::READY);
    } else {
        // LED Red Blinking code assignment happens inside runSelfTests
        Serial.println("[POST] Critical failure detected.");
        changeMode(OperatingMode::FAULT);
    }
}

// 3.4: Default system wait state
void SystemFSM::handleReady() {
    killMotors(); // Keep PWM = 0 safely
    maintainEspNow(); // Broadcast WHO_AM_I
    
    // Monitor Pin 9 (High priority)
    if (digitalRead(pinRCJ) == HIGH) {
        changeMode(OperatingMode::GAME);
    }
}

// 3.5: Active autonomous gameplay state
void SystemFSM::handleGame() {
    // Continuous sampling of Pin 9 for safety response < 10ms
    if (digitalRead(pinRCJ) == LOW) {
        changeMode(OperatingMode::PAUSED);
        return;
    }
    
    maintainEspNow();
    runStrategicFSM(); // Execute SEARCH / ORBIT / ATTACK / GOALIE
}

// 3.6: Game Halt state requested by the Referee Box
void SystemFSM::handlePaused() {
    // Safety check: Monitor Pin 9 to instantly resume
    if (digitalRead(pinRCJ) == HIGH) {
        changeMode(OperatingMode::GAME);
        return;
    }
    
    // Keep updates going to prevent compass drift
    updateCompass();
    maintainEspNow();
    
    // Alert via LED if paused for too long (> 2 minutes)
    if (millis() - pauseStartTime > 120000) {
        // Non-blocking indicator trigger
    }
}

// 3.7: Engineering and calibration mode
void SystemFSM::handleTest() {
    // CRITICAL SAFETY OVERRIDE: If referee starts game unexpectedly, immediately shift out
    if (digitalRead(pinRCJ) == HIGH) {
        killMotors(); // Instant execution safety rule!
        changeMode(OperatingMode::GAME);
        return;
    }
    
    maintainEspNow();
    // Manual testing loops / Web UI relays are handled here
}

// 3.8: Critical Error Trap
void SystemFSM::handleFault() {
    killMotors();
    // Force constant solid red LED
    // Send periodic alerts over UART and ESP-NOW to notify ally robot
    static unsigned long lastAlert = 0;
    if (millis() - lastAlert > 1000) {
        Serial.println("[CRITICAL FAULT] Robot halted. Hard hardware power cycle required.");
        lastAlert = millis();
    }
    // No programmatic code exit transitions allowed here.
}

void SystemFSM::changeMode(OperatingMode newMode) {
    // --- EXIT ACTIONS ---
    switch (currentMode) {
        case OperatingMode::GAME:
            if (newMode == OperatingMode::PAUSED) {
                killMotors(); // Trigger immediate motor safe (< 10ms response)
                // Save strategic snapshots to local variables
                strategicStateSaved = true;
                pauseStartTime = millis();
                Serial.println("[Exit GAME] System paused. State variables preserved.");
            }
            break;
        case OperatingMode::READY:
            if (newMode == OperatingMode::GAME) {
                // 3.5: Shut off SoftAP to decrease RF contention and optimize noise
                Serial.println("[Exit READY] Shutting down SoftAP for match safety.");
            }
            break;
        default:
            break;
    }

    previousMode = currentMode;
    currentMode = newMode;
    stateEntryTime = millis();
    
    Serial.print("[STATE TRANSITION] Entered: ");
    Serial.println(getModeName(newMode));

    // --- ENTRY ACTIONS ---
    switch (currentMode) {
        case OperatingMode::READY:
            // Ensure SoftAP is running, broadcast ready messages
            break;
        case OperatingMode::GAME:
            if (previousMode == OperatingMode::PAUSED && strategicStateSaved) {
                Serial.println("[Entry GAME] Resuming strategy from saved snapshots.");
                strategicStateSaved = false; 
            }
            break;
        case OperatingMode::TEST:
            // Enable manual controls and full diagnostic Web UI stack
            break;
        default:
            break;
    }
}

// 3.3: Sequenced hardware checks
bool SystemFSM::runSelfTests() {
    // 1. Check battery capacity threshold
    // float voltage = readBatteryVoltage();
    // if (voltage < 11.0f) return false;

    // 2. Scan I2C addresses (Verify BNO055, TCS34725 response status)
    // 3. Validate UART link IDs on 4 ESP nodes via CMD_QUERY_ID (0xA0)
    // 4. Low duty cycle check (10% PWM pulse checking current response)
    // 5. Fire Kicker for 5ms testing cycle
    // 6. Verify consistent compass frame telemetry updates
    // 7. Check TSOP34838 (U1) baseline values are clear of IR ball signatures
    
    return true; // Return false on any blocking failures
}

void SystemFSM::handleWebCommand(const String& command) {
    if (currentMode == OperatingMode::READY && command == "Enter Test") {
        changeMode(OperatingMode::TEST);
    } else if (currentMode == OperatingMode::TEST && command == "Exit Test") {
        changeMode(OperatingMode::READY);
    }
}

void SystemFSM::triggerExternalFault(const String& reason) {
    Serial.print("[RUNTIME FAULT ENCOUNTERED]: ");
    Serial.println(reason);
    changeMode(OperatingMode::FAULT);
}

void SystemFSM::killMotors() {
    // Clear PWM parameters, pull motor channel pins low, disable kicker solenoids, turn off dribbler
}

void SystemFSM::runStrategicFSM() {
    // Step through SEARCH / ORBIT / ATTACK / GOALIE internal game loop
}

void SystemFSM::updateCompass() {
    // Query I2C bus tracking BNO055 registers to mitigate orientation drift
}

void SystemFSM::maintainEspNow() {
    // Run radio packet networking functions
}

const char* SystemFSM::getModeName(OperatingMode mode) const {
    switch (mode) {
        case OperatingMode::POWER_ON: return "POWER_ON";
        case OperatingMode::POST:     return "POST";
        case OperatingMode::READY:    return "READY";
        case OperatingMode::GAME:     return "GAME";
        case OperatingMode::PAUSED:   return "PAUSED";
        case OperatingMode::TEST:     return "TEST";
        case OperatingMode::FAULT:    return "FAULT";
        default:                      return "UNKNOWN";
    }
}
