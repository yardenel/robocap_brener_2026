#include <Arduino.h>
// ========================================
// Mecanum Robot - RoboCup Junior
// Teensy 4.1 + 2x L298N Motor Drivers
// ========================================

// ========================================
// Motor Pin Definitions (After Rewiring)
// ========================================

// ENG1 - Left Front (LF)
#define ENG1_SP 23   // Speed (PWM) - Header pin 25
#define ENG1_DR1 13  // Direction 1 - Header pin 28
#define ENG1_DR2 41  // Direction 2 - Header pin 44

// ENG2 - Right Front (RF)
#define ENG2_SP 22   // Speed (PWM) - Header pin 26
#define ENG2_DR1 40  // Direction 1 - Header pin 32
#define ENG2_DR2 39  // Direction 2 - Header pin 35

// ENG3 - Left Rear (LR)
#define ENG3_SP 37   // Speed (PWM) - Header pin 30
#define ENG3_DR1 38  // Direction 1 - Header pin 27
#define ENG3_DR2 35  // Direction 2 - Header pin 29

// ENG4 - Right Rear (RR)
#define ENG4_SP 36   // Speed (PWM) - Header pin 31
#define ENG4_DR1 34  // Direction 1 - Header pin 33
#define ENG4_DR2 33  // Direction 2 - Header pin 45

// ========================================
// Setup Function
// ========================================

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait max 3 seconds

    Serial.println("====================================");
    Serial.println("Mecanum Robot - RoboCup Junior");
    Serial.println("Teensy 4.1 Initialization");
    Serial.println("====================================");

    // Configure ENG1 (Left Front) pins
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

    Serial.println("Motor pins configured!");
    Serial.println("PWM frequency set to 20kHz");
    Serial.println("====================================");
    Serial.println("Ready for competition!");
    Serial.println("====================================\n");

    delay(1000);
}

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

// ========================================
// Mecanum Drive Kinematics
// ========================================

void xDrive(float vx, float vy, float omega) {
    /*
     * X-Drive kinematics — omni wheels at 45° on a circular robot
     *
     * Wheel layout (top view, front = up):
     *
     *    FL /     \ FR
     *      /   ↑   \
     *     |    vy   |
     *     |  ←  vx  |
     *      \       /
     *    RL \     / RR
     *
     * Each wheel is angled 45° so it contributes equally to
     * both forward/strafe motion. At 45° the sin/cos factor
     * is the same (√2/2) for all wheels, so it cancels out
     * during normalization — the simplified form is:
     *
     *   FL = vy + vx + omega   (front-left,  spins NE/SW)
     *   FR = vy - vx - omega   (front-right, spins NW/SE)
     *   RL = vy - vx + omega   (rear-left,   spins NW/SE)
     *   RR = vy + vx - omega   (rear-right,  spins NE/SW)
     *
     * vx:    strafe right (+1.0) / left  (-1.0)
     * vy:    forward      (+1.0) / back  (-1.0)
     * omega: CCW          (+1.0) / CW    (-1.0)
     */

    float vFL = vy + vx + omega;
    float vFR = vy - vx - omega;
    float vRL = vy - vx + omega;
    float vRR = vy + vx - omega;

    // Normalize — keep all wheels within [-1, 1]
    float maxSpeed = max(max(abs(vFL), abs(vFR)), max(abs(vRL), abs(vRR)));
    if (maxSpeed > 1.0) {
        vFL /= maxSpeed;
        vFR /= maxSpeed;
        vRL /= maxSpeed;
        vRR /= maxSpeed;
    }

    int pwmFL = (int)(vFL * 255);
    int pwmFR = (int)(vFR * 255);
    int pwmRL = (int)(vRL * 255);
    int pwmRR = (int)(vRR * 255);

    // Dead-zone compensation (uncomment if needed)
    /*
    const int DZ = 50;
    if (abs(pwmFL) > 0 && abs(pwmFL) < DZ) pwmFL = (pwmFL > 0) ? DZ : -DZ;
    if (abs(pwmFR) > 0 && abs(pwmFR) < DZ) pwmFR = (pwmFR > 0) ? DZ : -DZ;
    if (abs(pwmRL) > 0 && abs(pwmRL) < DZ) pwmRL = (pwmRL > 0) ? DZ : -DZ;
    if (abs(pwmRR) > 0 && abs(pwmRR) < DZ) pwmRR = (pwmRR > 0) ? DZ : -DZ;
    */

    setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, pwmFL);  // Front Left
    setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, pwmFR);  // Front Right
    setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, pwmRL);  // Rear Left
    setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, pwmRR);  // Rear Right

    Serial.print("FL:"); Serial.print(pwmFL);
    Serial.print("\tFR:"); Serial.print(pwmFR);
    Serial.print("\tRL:"); Serial.print(pwmRL);
    Serial.print("\tRR:"); Serial.println(pwmRR);
}

// ========================================
// Test Loop - תוכנית בדיקה
// ========================================

void loop() {
    // מחק את הקוד הזה ותחליף בלוגיקת המשחק שלך!

    Serial.println("\n>>> Test 1: Forward (קדימה)");
    mecanumDrive(0, 0.5, 0);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 2: Backward (אחורה)");
    mecanumDrive(0, -0.5, 0);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 3: Strafe Right (ימינה)");
    mecanumDrive(0.5, 0, 0);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 4: Strafe Left (שמאלה)");
    mecanumDrive(-0.5, 0, 0);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 5: Rotate CW (סיבוב עם השעון)");
    mecanumDrive(0, 0, -0.5);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 6: Rotate CCW (סיבוב נגד השעון)");
    mecanumDrive(0, 0, 0.5);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 7: Diagonal Forward-Right (אלכסון קדימה-ימין)");
    mecanumDrive(0.5, 0.5, 0);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(1000);

    Serial.println("\n>>> Test 8: Diagonal Backward-Left (אלכסון אחורה-שמאל)");
    mecanumDrive(-0.5, -0.5, 0);
    delay(2000);

    Serial.println("\n>>> Stop (עצירה)");
    mecanumDrive(0, 0, 0);
    delay(3000);

    Serial.println("\n====================================");
    Serial.println("All tests complete! Repeating...");
    Serial.println("====================================\n");
}