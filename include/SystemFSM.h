#ifndef SYSTEM_FSM_H
#define SYSTEM_FSM_H

#include <Arduino.h>


/*
POWER_ON - init
POST - Power-On Self-Test
        checks:
        - battery above 11.0v
        - sensors react
        - get 4 esp ids
        - power motors on 10%: to check electrical continuity – if idle current exceeds the normal range
        - solenoid reacts to small pulse (5ms)
        - gyro returns stable angle
        - IRs
*/
enum OperatingMode {
    POWER_ON,
    POST,
    READY,
    GAME,
    PAUSED,
    TEST,
    FAULT
};

// Placeholder structure to save the strategic game state during a pause
struct StrategicStateSnapshot {
    int behaviorState = 0;
    float ballAngle = 0.0f;
    // Add other strategic data to persist here
};

class SystemFSM {
public:
    SystemFSM(uint8_t rcjPin = 9);
    
    void init();
    void update();
    
    // External events triggered by Web UI or USB communication
    void handleWebCommand(const String& command);
    void triggerExternalFault(const String& reason);
    
    // Getters for external architecture awareness
    OperatingMode getCurrentMode() const { return currentMode; }
    const char* getModeName(OperatingMode mode) const;

private:
    OperatingMode currentMode;
    OperatingMode previousMode;
    
    const uint8_t pinRCJ; // Pin 9
    unsigned long stateEntryTime;
    unsigned long pauseStartTime;
    bool strategicStateSaved;
    
    // Internal State Machine Handlers
    void handlePowerOn();
    void handlePOST();
    void handleReady();
    void handleGame();
    void handlePaused();
    void handleTest();
    void handleFault();
    
    // Transition Helper Actions
    void changeMode(OperatingMode newMode);
    bool runSelfTests();
    void killMotors();
    void runStrategicFSM();
    void updateCompass();
    void maintainEspNow();
};

#endif // SYSTEM_FSM_H
