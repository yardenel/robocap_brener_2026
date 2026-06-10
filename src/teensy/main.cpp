#include <Arduino.h>

// ============================================================================
// MOTOR PINS
// ============================================================================

#define ENG1_DR1 13
#define ENG1_DR2 41
#define ENG1_SP  23

#define ENG2_DR1 40
#define ENG2_DR2 39
#define ENG2_SP  22

#define ENG3_DR1 38
#define ENG3_DR2 35
#define ENG3_SP  37

#define ENG4_DR1 34
#define ENG4_DR2 33
#define ENG4_SP  36

// ========================================
// Motor Control Function
// ========================================

void setMotor(int dir1, int dir2, int pwmPin, int speed) {
    /*
     * פונקציה לשליטה במנוע בודד
     *
     * dir1, dir2: פיני כיוון (DR1, DR2)
     * pwmPin: פין PWM למהירות
     * speed: -255 עד +255
     *   חיובי = קדימה
     *   שלילי = אחורה
     *   0 = עצירה
     */

    if (speed > 0) {
        // Forward - קדימה
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
        analogWrite(pwmPin, constrain(speed, 0, 255));
    } else if (speed < 0) {
        // Backward - אחורה
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        analogWrite(pwmPin, constrain(-speed, 0, 255));
    } else {
        // Brake - בלם (שני הפינים LOW)
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, LOW);
        analogWrite(pwmPin, 0);
    }
}

void setup() {
    Serial.println("starting");

    Serial.begin(115200);

    pinMode(ENG1_DR1, OUTPUT);
    pinMode(ENG1_DR2, OUTPUT);
    pinMode(ENG1_SP, OUTPUT);

    // Configure ENG2 (Right Front) pins
    pinMode(ENG2_DR1, OUTPUT);
    pinMode(ENG2_DR2, OUTPUT);
    pinMode(ENG2_SP, OUTPUT);

    // Configure ENG3 (Left Rear) pins
    pinMode(ENG3_DR1, OUTPUT); 
    pinMode(ENG3_DR2, OUTPUT);
    pinMode(ENG3_SP, OUTPUT);

    // Configure ENG4 (Right Rear) pins
    pinMode(ENG4_DR1, OUTPUT);
    pinMode(ENG4_DR2, OUTPUT);
    pinMode(ENG4_SP, OUTPUT);

    // Set PWM frequency to 20kHz (smooth, silent motor operation)
    analogWriteFrequency(ENG1_SP, 20000);
    analogWriteFrequency(ENG2_SP, 20000);
    analogWriteFrequency(ENG3_SP, 20000);
    analogWriteFrequency(ENG4_SP, 20000);

    Serial.println("finished");
}

void loop() {
    Serial.println("s");
    setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, 200);  // Front Left forward
    Serial.println("t");
}
