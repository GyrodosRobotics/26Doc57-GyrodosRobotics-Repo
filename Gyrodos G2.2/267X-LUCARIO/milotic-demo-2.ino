// ═══════════════════════════════════════════════════════════════
//  26COD72 LUCARIO — Arduino Mega 2560
//  v1.2.0
//  Comms: RS422 on Serial1 | Claws: TMC2209 UART on Serial2
//  Sensors: ICM42688 (SPI) | MS5837 (I2C) | 2× ACS712 (analog)
//  Power Mgmt: auto-throttle, 2× relay shutdown, override modes
// ═══════════════════════════════════════════════════════════════
//
//  PIN MAPPING
//  ───────────────────────────────────────
//  0/1   Serial0 — USB debug
//  2–9   Thruster ESC signals (1–8)
//  11    Lights (PWM, direct Mega control — no relay)
//  16/17 Serial2 — TMC2209 shared UART bus
//  18/19 Serial1 — RS422 transceiver (topside link)
//  20/21 I2C (SDA/SCL) — MS5837 pressure sensor
//  24    TMC2209 claw-right EN
//  25    TMC2209 claw-left  EN
//  26    Claw servo right
//  27    Claw servo left
//  28    Continuous rotation servo
//  30    Relay LEFT  (claw-left servo + stepper power)
//  31    Relay RIGHT (claw-right servo + stepper power)
//  50    SPI MISO ─┐
//  51    SPI MOSI  ├─ ICM42688
//  52    SPI SCK  ─┤
//  53    SPI CS   ─┘
//  A0    ACS712 current sensor — thruster block
//  A1    ACS712 current sensor — stepper block
//
//  TMC2209 UART WIRING (shared bus):
//    Mega TX2 (16) ——[1kΩ]——┬—— UART pin (driver #0)
//    Mega RX2 (17) —————————┤
//                           └—— UART pin (driver #1)
//    Driver #0 (claw right): MS1=LOW  MS2=LOW   → addr 0
//    Driver #1 (claw left):  MS1=HIGH MS2=LOW   → addr 1
//
//  RELAY WIRING:
//    RELAY_ACTIVE level engages relay and CUTS power.
//    Left relay  (pin 30): cuts power to claw-left servo + stepper
//    Right relay (pin 31): cuts power to claw-right servo + stepper
//    If your relay module is active-LOW, change RELAY_ACTIVE below.
//
//  ACS712 WIRING:
//    A0 — output of ACS712 on thruster power rail (all 8 ESCs)
//    A1 — output of ACS712 between PDU and stepper controllers
//    Adjust ACS_*_SENS constants to match your variant (5A/20A/30A).
//
//  PACKET FORMAT (bidirectional on RS422)
//  ───────────────────────────────────────
//  [0xAA] [CMD] [LEN] [PAYLOAD × LEN] [CHECKSUM]
//
//  Checksum = (sum of all preceding bytes) & 0xFF
//
//  CMD 0x01 — Thruster (LEN = 8) [topside → sub]
//    8 bytes, each value offset by +100
//    (100 = neutral, 0 = full reverse, 200 = full forward)
//    NOTE: output is scaled down when auto-throttle is active.
//
//  CMD 0x02 — Actuator (LEN = 8) [topside → sub]
//    [0] Servo right  (degrees, 0–180)
//    [1] Servo left   (degrees, 0–180)
//    [2] Claw right   (0–200, 100 = hold)
//    [3] Claw left    (0–200, 100 = hold)
//    [4] Cont. servo  (0–200, 100 = stop)
//    [5] Lights       (0 = off, nonzero = on)
//    [6] Status       (0xFF = normal, 0x00 = emergency)
//    [7] Override      0x00 = forbid auto-shutdown (keep all online)
//                      0x88 = allow automatic power management
//                      0xFF = manual cut power (force all offline)
//
//  CMD 0x03 — Sensor telemetry (LEN = 16) [sub → topside]
//    All multi-byte fields are LITTLE-ENDIAN (LSB first).
//    [0-1]   Accel X    (int16_t,  milli-g       ±4000 typ)
//    [2-3]   Accel Y    (int16_t,  milli-g)
//    [4-5]   Accel Z    (int16_t,  milli-g)
//    [6-7]   Gyro  X    (int16_t,  deci-°/s      ±5000 typ)
//    [8-9]   Gyro  Y    (int16_t,  deci-°/s)
//    [10-11] Gyro  Z    (int16_t,  deci-°/s)
//    [12-13] Depth      (uint16_t, millimetres    0–65535)
//    [14]    Throttle % (uint8_t,  0–100, 0 = no throttle)
//    [15]    Shutdown    (uint8_t, bitmask — 0 = running, 1 = shut down)
//              bit 0: Claw L servo
//              bit 1: Claw L stepper
//              bit 2: Claw R servo
//              bit 3: Claw R stepper
//              bit 4: Continuous rotation servo
//              bit 5: Lights
//              bit 6: (unused)
//              bit 7: (unused)
//
//    Resolution: 0.001 g  |  0.1 °/s  |  1 mm
//
//  LIBRARIES REQUIRED
//  ───────────────────────────────────────
//  TMCStepper           — teemuatlut/TMCStepper
//  MS5837               — bluerobotics/BlueRobotics MS5837
//  ICM42688 (bfs)       — bolderflight/invensense-imu
//    ↳ API returns m/s² and rad/s — we convert below
// ═══════════════════════════════════════════════════════════════

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <TMCStepper.h>
#include "ICM42688.h"          // bolderflight/invensense-imu
#include "MS5837.h"            // bluerobotics

// ─────────────── PIN DEFINITIONS ───────────────

const uint8_t THRUSTER_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t NUM_THRUSTERS    = 8;

const uint8_t LIGHTS_PIN       = 11;

const uint8_t TMC_EN_RIGHT     = 24;
const uint8_t TMC_EN_LEFT      = 25;

const uint8_t SERVO_RIGHT_PIN  = 26;
const uint8_t SERVO_LEFT_PIN   = 27;
const uint8_t CONT_SERVO_PIN   = 28;

const uint8_t RELAY_LEFT_PIN   = 30;   // v1.2 — left claw power relay
const uint8_t RELAY_RIGHT_PIN  = 31;   // v1.2 — right claw power relay

const uint8_t ACS_THRUSTER_PIN = A0;   // v1.2 — ACS712 thruster block
const uint8_t ACS_STEPPER_PIN  = A1;   // v1.2 — ACS712 stepper block

const uint8_t IMU_CS_PIN       = 53;

// ─────────────── RELAY ACTIVE LEVEL ────────────
// HIGH = relay energised = power CUT to module.
// Change to LOW if your relay module is active-LOW.
const uint8_t RELAY_ACTIVE = HIGH;

// ─────────────── TMC2209 SETUP ─────────────────

#define R_SENSE 0.11f

TMC2209Stepper clawRightDriver(&Serial2, R_SENSE, 0);
TMC2209Stepper clawLeftDriver (&Serial2, R_SENSE, 1);

const int32_t CLAW_VMAX     = 20000;
const int16_t CLAW_DEADBAND = 5;

// ─────────────── ACS712 CURRENT SENSING ────────
//  Adjust sensitivity to match your ACS712 variant:
//    5A  → 0.185 V/A
//    20A → 0.100 V/A
//    30A → 0.066 V/A

const float   ACS_VREF          = 2.50f;   // Quiescent output at 0 A (Vcc / 2)
const float   ACS_THRUSTER_SENS = 0.066f;  // V/A — 30 A variant
const float   ACS_STEPPER_SENS  = 0.100f;  // V/A — 20 A variant
const uint8_t ACS_SAMPLES       = 8;       // Analog reads averaged per measurement

// ─────────────── CURRENT / POWER LIMITS ────────

const float THRUSTER_CURRENT_SOFT  = 20.0f;  // A — begin throttling
const float THRUSTER_CURRENT_HARD  = 25.0f;  // A — 100 % throttle (all stop)
const float STEPPER_CURRENT_TRIP   = 3.0f;   // A — trip relay, shut down claws
const float STEPPER_CURRENT_RESET  = 2.0f;   // A — clear trip (hysteresis)

// ─────────────── SENSOR OBJECTS ────────────────

bfs::Icm42688 imu(&SPI, IMU_CS_PIN);
MS5837        depthSensor;

bool imuReady   = false;
bool depthReady = false;

// ─────────────── SERVO / ESC OBJECTS ───────────

Servo thrusters[NUM_THRUSTERS];
Servo servoRight;
Servo servoLeft;
Servo contServo;

// ─────────────── PACKET PROTOCOL ───────────────

const uint8_t START_MARKER = 0xAA;
const uint8_t CMD_THRUSTER = 0x01;
const uint8_t CMD_ACTUATOR = 0x02;
const uint8_t CMD_SENSOR   = 0x03;

enum ParseState {
    WAIT_START,
    WAIT_CMD,
    WAIT_LENGTH,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM
};

ParseState parseState   = WAIT_START;
uint8_t    pktCmd       = 0;
uint8_t    pktLength    = 0;
uint8_t    payload[32];
uint8_t    payloadIndex = 0;
uint8_t    runningSum   = 0;

// ─────────────── TELEMETRY TIMING ──────────────

const uint32_t TELEM_INTERVAL_MS = 50;   // 20 Hz
uint32_t       lastTelemTime     = 0;

// ─────────────── POWER MANAGEMENT TIMING ───────

const uint32_t POWER_MGMT_INTERVAL_MS = 10;   // 100 Hz
uint32_t       lastPowerMgmtTime      = 0;

// ─────────────── SAFETY ────────────────────────

const uint16_t THRUSTER_STOP_US = 1500;
const uint32_t TIMEOUT_MS       = 500;
uint32_t       lastPacketTime   = 0;
bool           timedOut         = false;
bool           emergency        = false;

// ─────────────── POWER MANAGEMENT STATE ────────

uint8_t  overrideMode          = 0x88;  // Default: automatic
uint8_t  thrustThrottlePercent = 0;     // 0–100 → telemetry byte [14]
uint8_t  shutdownBitmask       = 0;     // Bitmask → telemetry byte [15]
bool     stepperTripped        = false; // Hysteresis latch

// Raw thruster commands — stored so throttle can be re-applied live
uint8_t  rawThrusterCmd[8] = {100, 100, 100, 100, 100, 100, 100, 100};

// ════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println(F("LUCARIO subsea controller v1.2.0 starting..."));

    Serial1.begin(115200);      // RS422 transceiver
    Serial2.begin(115200);      // TMC2209 UART bus

    // ── TMC2209 claw drivers ──
    pinMode(TMC_EN_RIGHT, OUTPUT);
    pinMode(TMC_EN_LEFT,  OUTPUT);
    digitalWrite(TMC_EN_RIGHT, LOW);   // Enabled (active-LOW)
    digitalWrite(TMC_EN_LEFT,  LOW);
    delay(100);

    initClawDriver(clawRightDriver, "right");
    initClawDriver(clawLeftDriver,  "left");

    // ── Relays — start de-energised (modules powered) ──
    pinMode(RELAY_LEFT_PIN,  OUTPUT);
    pinMode(RELAY_RIGHT_PIN, OUTPUT);
    digitalWrite(RELAY_LEFT_PIN,  !RELAY_ACTIVE);
    digitalWrite(RELAY_RIGHT_PIN, !RELAY_ACTIVE);

    // ── ACS712 analog inputs ──
    pinMode(ACS_THRUSTER_PIN, INPUT);
    pinMode(ACS_STEPPER_PIN,  INPUT);

    // ── ICM42688 IMU (SPI) ──
    SPI.begin();
    if (!imu.Begin()) {
        Serial.println(F("ICM42688 init FAILED — check wiring"));
    } else {
        imu.ConfigAccelRange(bfs::Icm42688::ACCEL_RANGE_4G);
        imu.ConfigGyroRange(bfs::Icm42688::GYRO_RANGE_500DPS);
        imuReady = true;
        Serial.println(F("ICM42688 — OK"));
    }

    // ── MS5837 depth sensor (I2C) ──
    Wire.begin();
    if (!depthSensor.init()) {
        Serial.println(F("MS5837 init FAILED — check wiring"));
    } else {
        depthSensor.setModel(MS5837::MS5837_30BA);
        depthSensor.setFluidDensity(1029);  // kg/m³ seawater
        depthReady = true;
        Serial.println(F("MS5837 — OK"));
    }

    // ── Thruster ESCs ──
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].attach(THRUSTER_PINS[i]);
        thrusters[i].writeMicroseconds(THRUSTER_STOP_US);
    }

    // ── Claw servos ──
    servoRight.attach(SERVO_RIGHT_PIN);
    servoLeft.attach(SERVO_LEFT_PIN);
    servoRight.write(90);
    servoLeft.write(90);

    // ── Continuous rotation servo ──
    contServo.attach(CONT_SERVO_PIN);
    contServo.writeMicroseconds(1500);

    // ── Lights ──
    pinMode(LIGHTS_PIN, OUTPUT);
    analogWrite(LIGHTS_PIN, 0);

    // ── ESC arming delay ──
    Serial.println(F("Arming ESCs..."));
    delay(2000);

    lastPacketTime    = millis();
    lastTelemTime     = millis();
    lastPowerMgmtTime = millis();
    Serial.println(F("Ready — awaiting packets on Serial1"));
}

void initClawDriver(TMC2209Stepper &driver, const char *name) {
    driver.begin();
    driver.toff(4);
    driver.rms_current(400);
    driver.microsteps(16);
    driver.pwm_autoscale(true);
    driver.en_spreadCycle(false);
    driver.VACTUAL(0);

    uint8_t version = driver.version();
    Serial.print(F("TMC2209 claw "));
    Serial.print(name);
    if (version == 0x21) {
        Serial.println(F(" — OK (v0x21)"));
    } else {
        Serial.print(F(" — WARNING: version 0x"));
        Serial.println(version, HEX);
    }
}

// ════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════

void loop() {
    // ── Receive commands from topside ──
    while (Serial1.available()) {
        parseByte(Serial1.read());
    }

    // ── Timeout safety ──
    if (millis() - lastPacketTime > TIMEOUT_MS) {
        if (!timedOut) {
            Serial.println(F("TIMEOUT — all stop"));
            allStop();
            timedOut = true;
        }
    }

    // ── Power management (100 Hz) ──
    if (millis() - lastPowerMgmtTime >= POWER_MGMT_INTERVAL_MS) {
        lastPowerMgmtTime = millis();
        updatePowerManagement();
    }

    // ── Periodic telemetry back to topside ──
    if (millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
        lastTelemTime = millis();
        sendTelemetry();
    }
}

// ════════════════════════════════════════════════
//  ACS712 CURRENT READING
// ════════════════════════════════════════════════

float readCurrent(uint8_t pin, float sensitivity) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < ACS_SAMPLES; i++) {
        sum += analogRead(pin);
    }
    float voltage = (sum / (float)ACS_SAMPLES) * (5.0f / 1023.0f);
    return fabs((voltage - ACS_VREF) / sensitivity);
}

// ════════════════════════════════════════════════
//  POWER MANAGEMENT — THROTTLE & SHUTDOWN
// ════════════════════════════════════════════════

void applyThrusters() {
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        int16_t deviation = (int16_t)rawThrusterCmd[i] - 100;
        deviation = (int16_t)((int32_t)deviation * (100 - (int16_t)thrustThrottlePercent) / 100);
        int32_t us = 1500 + (int32_t)deviation * 4;
        thrusters[i].writeMicroseconds(constrain(us, 1100, 1900));
    }
}

void updatePowerManagement() {
    // ── Thruster current → throttle (always active) ──
    float thrusterAmps = readCurrent(ACS_THRUSTER_PIN, ACS_THRUSTER_SENS);

    if (thrusterAmps >= THRUSTER_CURRENT_HARD) {
        thrustThrottlePercent = 100;
    } else if (thrusterAmps > THRUSTER_CURRENT_SOFT) {
        float ratio = (thrusterAmps - THRUSTER_CURRENT_SOFT)
                    / (THRUSTER_CURRENT_HARD - THRUSTER_CURRENT_SOFT);
        thrustThrottlePercent = (uint8_t)constrain((int32_t)(ratio * 100.0f), 1, 100);
    } else {
        thrustThrottlePercent = 0;
    }

    // Re-apply throttled thruster values immediately
    if (!emergency && !timedOut) {
        applyThrusters();
    }

    // ── Module shutdown logic (depends on override mode) ──
    float stepperAmps = readCurrent(ACS_STEPPER_PIN, ACS_STEPPER_SENS);
    uint8_t newMask = 0;

    if (overrideMode == 0x00) {
        // ── FORBID shutdown — keep everything online ──
        newMask = 0;
        stepperTripped = false;

    } else if (overrideMode == 0xFF) {
        // ── MANUAL CUT — force all modules offline ──
        newMask = 0x3F;   // Bits 0-5

    } else {
        // ── AUTOMATIC (0x88 or default) ──
        // Stepper overcurrent with hysteresis
        if (!stepperTripped && stepperAmps > STEPPER_CURRENT_TRIP) {
            stepperTripped = true;
            Serial.print(F("STEPPER OC — "));
            Serial.print(stepperAmps, 1);
            Serial.println(F(" A — claws shut down"));
        } else if (stepperTripped && stepperAmps < STEPPER_CURRENT_RESET) {
            stepperTripped = false;
            Serial.println(F("STEPPER current normal — claws restored"));
        }

        if (stepperTripped) {
            newMask |= 0x0F;  // Bits 0-3: both claw assemblies
        }
    }

    // ── Detect change for debug logging ──
    static uint8_t prevThrottle = 0;
    if (thrustThrottlePercent > 0 && prevThrottle == 0) {
        Serial.print(F("THROTTLE active: "));
        Serial.print(thrustThrottlePercent);
        Serial.println(F("%"));
    } else if (thrustThrottlePercent == 0 && prevThrottle > 0) {
        Serial.println(F("THROTTLE cleared"));
    }
    prevThrottle = thrustThrottlePercent;

    shutdownBitmask = newMask;

    // ── Apply relay hardware ──
    bool leftCut  = (shutdownBitmask & 0x03) != 0;
    bool rightCut = (shutdownBitmask & 0x0C) != 0;

    digitalWrite(RELAY_LEFT_PIN,  leftCut  ? RELAY_ACTIVE : !RELAY_ACTIVE);
    digitalWrite(RELAY_RIGHT_PIN, rightCut ? RELAY_ACTIVE : !RELAY_ACTIVE);

    // TMC2209 enable pins (active-LOW: LOW = enabled, HIGH = disabled)
    digitalWrite(TMC_EN_LEFT,  leftCut  ? HIGH : LOW);
    digitalWrite(TMC_EN_RIGHT, rightCut ? HIGH : LOW);

    if (leftCut)  clawLeftDriver.VACTUAL(0);
    if (rightCut) clawRightDriver.VACTUAL(0);

    // ── Software-only shutdowns (no relay) ──
    if (shutdownBitmask & 0x10) {  // Continuous rotation servo
        contServo.writeMicroseconds(1500);
    }
    if (shutdownBitmask & 0x20) {  // Lights
        analogWrite(LIGHTS_PIN, 0);
    }
}

// ════════════════════════════════════════════════
//  TELEMETRY — READ SENSORS & SEND 0x03 PACKET
// ════════════════════════════════════════════════

void packInt16(uint8_t *buf, uint8_t offset, int16_t val) {
    buf[offset]     = (uint8_t)(val & 0xFF);         // LSB
    buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);  // MSB
}

void packUint16(uint8_t *buf, uint8_t offset, uint16_t val) {
    buf[offset]     = (uint8_t)(val & 0xFF);
    buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
}

void sendTelemetry() {
    uint8_t telem[16];                            // v1.2: was 14

    // ── ICM42688 ──
    if (imuReady && imu.Read()) {
        float ax = imu.accel_x_mps2() / 9.80665f;
        float ay = imu.accel_y_mps2() / 9.80665f;
        float az = imu.accel_z_mps2() / 9.80665f;
        float gx = imu.gyro_x_radps() * 57.29578f;
        float gy = imu.gyro_y_radps() * 57.29578f;
        float gz = imu.gyro_z_radps() * 57.29578f;

        packInt16(telem, 0, (int16_t)constrain((int32_t)(ax * 1000.0f), -32768, 32767));
        packInt16(telem, 2, (int16_t)constrain((int32_t)(ay * 1000.0f), -32768, 32767));
        packInt16(telem, 4, (int16_t)constrain((int32_t)(az * 1000.0f), -32768, 32767));

        packInt16(telem, 6,  (int16_t)constrain((int32_t)(gx * 10.0f), -32768, 32767));
        packInt16(telem, 8,  (int16_t)constrain((int32_t)(gy * 10.0f), -32768, 32767));
        packInt16(telem, 10, (int16_t)constrain((int32_t)(gz * 10.0f), -32768, 32767));
    } else {
        memset(telem, 0, 12);
    }

    // ── MS5837 ──
    if (depthReady) {
        depthSensor.read();
        float depthM = depthSensor.depth();
        if (depthM < 0.0f) depthM = 0.0f;
        packUint16(telem, 12, (uint16_t)constrain((int32_t)(depthM * 1000.0f), 0, 65535));
    } else {
        packUint16(telem, 12, 0);
    }

    // ── Power management status (v1.2) ──
    telem[14] = thrustThrottlePercent;
    telem[15] = shutdownBitmask;

    sendPacket(CMD_SENSOR, telem, 16);            // v1.2: was 14
}

// Build [0xAA][CMD][LEN][payload...][CHECKSUM] and write to Serial1
void sendPacket(uint8_t cmd, const uint8_t *data, uint8_t len) {
    uint8_t cs = 0;

    Serial1.write(START_MARKER);  cs += START_MARKER;
    Serial1.write(cmd);           cs += cmd;
    Serial1.write(len);           cs += len;

    for (uint8_t i = 0; i < len; i++) {
        Serial1.write(data[i]);
        cs += data[i];
    }

    Serial1.write(cs);
}

// ════════════════════════════════════════════════
//  PACKET PARSER (incoming from topside)
// ════════════════════════════════════════════════

void parseByte(uint8_t b) {
    switch (parseState) {
        case WAIT_START:
            if (b == START_MARKER) {
                runningSum = b;
                parseState = WAIT_CMD;
            }
            break;

        case WAIT_CMD:
            pktCmd = b;
            runningSum += b;
            parseState = WAIT_LENGTH;
            break;

        case WAIT_LENGTH:
            pktLength = b;
            runningSum += b;
            if (pktLength == 0 || pktLength > sizeof(payload)) {
                parseState = WAIT_START;
            } else {
                payloadIndex = 0;
                parseState = WAIT_PAYLOAD;
            }
            break;

        case WAIT_PAYLOAD:
            payload[payloadIndex++] = b;
            runningSum += b;
            if (payloadIndex >= pktLength) {
                parseState = WAIT_CHECKSUM;
            }
            break;

        case WAIT_CHECKSUM: {
            if (runningSum == b) {
                lastPacketTime = millis();
                timedOut = false;
                handlePacket();
            } else {
                Serial.println(F("Checksum fail"));
            }
            parseState = WAIT_START;
            break;
        }
    }
}

// ════════════════════════════════════════════════
//  PACKET HANDLERS
// ════════════════════════════════════════════════

void handlePacket() {
    switch (pktCmd) {
        case CMD_THRUSTER: handleThrusterPacket(); break;
        case CMD_ACTUATOR: handleActuatorPacket(); break;
        default:
            Serial.print(F("Unknown cmd: 0x"));
            Serial.println(pktCmd, HEX);
            break;
    }
}

void handleThrusterPacket() {
    if (pktLength < NUM_THRUSTERS) return;
    if (emergency) return;

    // Store raw values so throttle can be re-applied between packets
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        rawThrusterCmd[i] = payload[i];
    }

    applyThrusters();
}

void handleActuatorPacket() {
    if (pktLength < 8) return;

    // ── Status check first (byte 6) ──
    if (payload[6] == 0x00) {
        if (!emergency) {
            Serial.println(F("EMERGENCY — all stop"));
            emergency = true;
        }
        allStop();
        return;
    }
    emergency = false;

    // ── Override mode (byte 7) ── v1.2
    uint8_t newOverride = payload[7];
    if (newOverride == 0x00 || newOverride == 0x88 || newOverride == 0xFF) {
        if (newOverride != overrideMode) {
            overrideMode = newOverride;
            Serial.print(F("Override mode → 0x"));
            Serial.println(overrideMode, HEX);
        }
    }
    // Invalid values are silently ignored; previous mode persists.

    // If manual cut is active, skip all actuator commands —
    // updatePowerManagement() handles the hardware.
    if (overrideMode == 0xFF) return;

    // ── Servo right (byte 0) — bit 2 ──
    if (!(shutdownBitmask & 0x04)) {
        servoRight.write(constrain(payload[0], 0, 180));
    }

    // ── Servo left (byte 1) — bit 0 ──
    if (!(shutdownBitmask & 0x01)) {
        servoLeft.write(constrain(payload[1], 0, 180));
    }

    // ── Claw right stepper (byte 2) — bit 3 ──
    if (!(shutdownBitmask & 0x08)) {
        int16_t rightInput = (int16_t)payload[2] - 100;
        if (abs(rightInput) < CLAW_DEADBAND) {
            clawRightDriver.VACTUAL(0);
        } else {
            clawRightDriver.VACTUAL(map(rightInput, -100, 100, -CLAW_VMAX, CLAW_VMAX));
        }
    }

    // ── Claw left stepper (byte 3) — bit 1 ──
    if (!(shutdownBitmask & 0x02)) {
        int16_t leftInput = (int16_t)payload[3] - 100;
        if (abs(leftInput) < CLAW_DEADBAND) {
            clawLeftDriver.VACTUAL(0);
        } else {
            clawLeftDriver.VACTUAL(map(leftInput, -100, 100, -CLAW_VMAX, CLAW_VMAX));
        }
    }

    // ── Continuous servo (byte 4) — bit 4 ──
    if (!(shutdownBitmask & 0x10)) {
        uint16_t contUs = 1000 + (uint16_t)payload[4] * 5;
        contServo.writeMicroseconds(constrain(contUs, 1000, 2000));
    }

    // ── Lights (byte 5) — bit 5 ──
    if (!(shutdownBitmask & 0x20)) {
        analogWrite(LIGHTS_PIN, (payload[5] > 0) ? 255 : 0);
    }
}

// ════════════════════════════════════════════════
//  SAFETY — ALL STOP
// ════════════════════════════════════════════════

void allStop() {
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].writeMicroseconds(THRUSTER_STOP_US);
        rawThrusterCmd[i] = 100;       // Reset stored commands to neutral
    }
    servoRight.write(90);
    servoLeft.write(90);
    contServo.writeMicroseconds(1500);
    analogWrite(LIGHTS_PIN, 0);
    clawRightDriver.VACTUAL(0);
    clawLeftDriver.VACTUAL(0);
}