#pragma once

#include <Arduino.h>


class SystemFSM {
    private:
        /*
        POWER_ON - init
            → POST
        POST - Power-On Self-Test;
                checks:
                - sensors react
                - get 4 esp ids
                - power motors on 30%: to check electrical continuity – if idle current exceeds the normal range
                - solenoid reacts to small pulse (5ms)
                - gyro returns stable angle
                - IRs return 0
            everything ok: → READY
            problem detected: → FAULT
        READY - stand-by;
            - motors 0%
            - ESP-NOW working and searching for other ESP
            - SoftAP working
            - PIN 9 (game module) monitored in high frequency (interupt driven); ready for GAME
            PIN 9 = HIGH → GAME
            "Enter Test" from Web UI → TEST
            * Error detected → FAULT
        GAME - active strategy
            - SoftAP off
            - ESP-NOW active for partner comm
            - PIN 9 monitored; ready for STOP
            - strategy FSM ATTACKER/ATTACK SUPPORT
            PIN 9 = LOW → PAUSED
            * penalty (out of lines / ball holding) → TBD
        * PAUSED - in between games/breaks/penalties
            - motors 0%
            - kicker off
            - dribbler 0%
            - internal strategy state, ballAngle are saved

        TEST - test state
            - SoftAP & Web UI active
            - manual control over motors, dribbler, kicker, color sensors
            - IR & gyro feedback
            PIN 9 = HIGH → GAME
        FAULT - loss of sensor, critical error, etc..
            manual restart required
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

    public:
        void init();
        void update();

};
