#include <Arduino.h>

class FSM {
public:
    enum State {
        IDLE,
        MOVING,
        SPINNING
    };

    State state;

    FSM() {
        state = IDLE;
    }

    void begin() {
        Serial.println("FSM ready");
        Serial.println("Commands: idle, move, spin");
    }

    void update() {
        readSerialCommand();

        switch (state) {
            case IDLE:
                Serial.println("State: IDLE");
                break;

            case MOVING:
                Serial.println("State: MOVING");
                break;

            case SPINNING:
                Serial.println("State: SPINNING");
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

        if (command == "idle") {
            state = IDLE;
            Serial.println("Changed to IDLE");
        }
        else if (command == "move") {
            state = MOVING;
            Serial.println("Changed to MOVING");
        }
        else if (command == "spin") {
            state = SPINNING;
            Serial.println("Changed to SPINNING");
        }
        else {
            Serial.print("Unknown command: ");
            Serial.println(command);
            Serial.println("Use: idle, move, spin");
        }
    }
};