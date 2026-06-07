#include <Arduino.h>

class FSM {
public:
    enum State {
        POWER_ON,
        POST,
        READY,
        GAME,
        PAUSED,
        TEST,
        FAULT
    };

    State state;

    FSM() {
        state = PAUSED;
    }

    void begin() {
        Serial.println("FSM ready");
        Serial.println("Commands: idle, move, spin");
    }

    void update() {
        readSerialCommand();

        switch (state) {
            case PAUSED:
                Serial.println("State: PAUSED");
                break;

            case TEST:
                Serial.println("State: TEST");
                break;

            case READY:
                Serial.println("State: REASY");
                break;

            case GAME:
                Serial.println("State: GAME");
                break;
        }

        delay(500);
    }

private:
    void readSerialCommand() {
        if (Serial.available() == 0) {
            return;
        }

        String command = Serial.readStringUntil('\n');
        command.trim();        // removes spaces, \r, \n
        command.toLowerCase(); // makes command lowercase

        if (command == "pause") {
            state = PAUSED;
            Serial.println("Changed to PAUSED");
        }
        else if (command == "test") {
            state = TEST;
            Serial.println("Changed to TEST");
        }
        else if (command == "ready") {
            state = READY;
            Serial.println("Changed to READY");
        }
        else if (command == "game") {
            state = GAME;
            Serial.println("Changed to GAME");
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(command);
            Serial.println("Use: idle, move, spin");
        }
    }
};