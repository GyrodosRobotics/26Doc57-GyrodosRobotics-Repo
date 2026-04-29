// ═══════════════════════════════════════════════════════════════
//  26COD74 LUCARIO Float — Arduino Nano
//  v1.0.0
//  Float Magikarp buoyancy engine controller
//  Power: 9V battery via reed-switch activation circuit
//  Motor: DRV8871 H-bridge — piston buoyancy engine
//  Depth: MS5837 via UART computation board (same as 26Cod72)
//  Comms: HC-06 Bluetooth UART — post-dive data broadcast
//  Gyrodos Robotics
// ═══════════════════════════════════════════════════════════════
//
//  POWER ACTIVATION CIRCUIT
//  ────────────────────────
//
//  Battery(+) ──┬── NC relay (COM→NC) ── Reed Switch (NO) ── Arduino VIN
//               │
//               └── NO relay (COM→NO) ── Arduino VIN + all peripherals
//
//  Startup sequence:
//    1. Magnet placed near reed switch → reed closes → current flows
//       through NC relay + reed switch → Arduino boots
//    2. Arduino immediately energises NO relay → main power path active
//    3. Arduino de-energises NC relay (opens NC path) → reed switch
//       is now disconnected from circuit
//    4. Magnet can be safely removed — power sustained through NO relay
//
//  Shutdown:
//    Arduino de-energises NO relay → all power cut, system off.
//
//  PIN MAPPING (Arduino Nano)
//  ────────────────────────
//  D2      NO relay coil (HIGH = energised = power ON)
//  D3      NC relay coil (HIGH = energised = NC opens = reed path CUT)
//  D5      DRV8871 IN1 (PWM)
//  D6      DRV8871 IN2 (PWM)
//  D8      (available)
//  RX(D0)  HC-06 TX  (via level-shifted UART adaptor)
//  TX(D1)  HC-06 RX  (via level-shifted UART adaptor)
//  A4/A5   (I2C — unused, depth sensor is on UART)
//
//  UART allocation:
//    Serial  (D0/D1, 9600)    — HC-06 Bluetooth (default baud)
//    SoftwareSerial (D10/D11, 115200) — MS5837 computation board
//
//  DRV8871 CONTROL
//  ────────────────────────
//  IN1=PWM  IN2=LOW   → motor forward  (descend / push piston in)
//  IN1=LOW  IN2=PWM   → motor reverse  (ascend  / pull piston out)
//  IN1=LOW  IN2=LOW   → coast (motor off)
//  IN1=HIGH IN2=HIGH  → brake
//
//  DEPTH SENSOR UART FORMAT (from computation board)
//  ────────────────────────
//  "Depth:0.13m Temp:27.67C\n"   ~2 Hz at 115200 baud
//  Same parser as 26Cod72 (LUCARIO ROV Arduino Mega Serial3)
//
//  BLUETOOTH BROADCAST PACKET FORMAT
//  ────────────────────────
//  [0xAA] [0x42] [LEN] [PAYLOAD × LEN] [CHECKSUM]
//  Checksum = (sum of all preceding bytes) & 0xFF
//
//  Payload: array of depth records
//    Each record: int16_t depth_mm (little-endian) — 2 bytes
//    Plus 1 byte header: dive_id (1 or 2)
//    Plus 2 bytes: record_index uint16_t (little-endian)
//    Total per record: 5 bytes
//
// ═══════════════════════════════════════════════════════════════

#include <SoftwareSerial.h>

// ─────────────── PIN DEFINITIONS ───────────────
const uint8_t RELAY_NO_PIN   = 2;    // NO relay — main power latch
const uint8_t RELAY_NC_PIN   = 3;    // NC relay — reed switch path
const uint8_t MOTOR_IN1_PIN  = 5;    // DRV8871 IN1 (PWM)
const uint8_t MOTOR_IN2_PIN  = 6;    // DRV8871 IN2 (PWM)
const uint8_t DEPTH_RX_PIN   = 10;   // SoftwareSerial RX from depth board
const uint8_t DEPTH_TX_PIN   = 11;   // SoftwareSerial TX (unused but required)

// ─────────────── MOTOR SPEED ───────────────────
const uint8_t DESCEND_SPEED  = 180;  // PWM 0–255, tune to buoyancy engine
const uint8_t ASCEND_SPEED   = 180;
const uint8_t HOLD_SPEED     = 0;    // Coast — rely on neutral buoyancy

// ─────────────── DEPTH TARGETS (mm) ────────────
const int16_t DIVE1_BOTTOM   = 1000;   // 1.00 m
const int16_t DIVE1_TOP      = 400;    // 0.40 m
const int16_t DIVE2_BOTTOM   = 1000;   // 1.00 m
const int16_t DIVE2_TOP      = 50;     // surface (~5 cm tolerance)
const int16_t DEPTH_TOLERANCE = 30;    // ±30 mm acceptable

// ─────────────── RECORDING ─────────────────────
const uint32_t RECORD_INTERVAL_MS = 5000;   // 5 seconds
const uint16_t MAX_RECORDS        = 120;    // ~10 min of recording

struct DepthRecord {
    uint8_t  dive_id;       // 1 or 2
    uint16_t record_index;  // sequential
    int16_t  depth_mm;      // millimetres
};

DepthRecord records[MAX_RECORDS];
uint16_t    recordCount = 0;
uint16_t    globalIndex = 0;

// ─────────────── DEPTH SENSOR UART ─────────────
SoftwareSerial depthSerial(DEPTH_RX_PIN, DEPTH_TX_PIN);

float  parsedDepthM        = 0.0f;
float  parsedTempC         = 0.0f;
float  depthZeroOffset     = 0.0f;
bool   depthZeroed         = false;
char   depthLineBuf[64];
uint8_t depthLineIdx       = 0;

// Depth zero-cal state (module scope — resettable, per 26Cod72 review)
uint8_t depthZeroCount     = 0;
float   depthZeroSum       = 0.0f;

// ─────────────── BLUETOOTH PACKET ──────────────
const uint8_t BT_START     = 0xAA;
const uint8_t BT_CMD       = 0x42;   // 'B' for buoyancy / float ID

// ─────────────── STATE MACHINE ─────────────────
enum FloatState {
    STATE_BOOT,
    STATE_ZERO_CAL,
    STATE_DIVE1_DESCEND,
    STATE_DIVE1_ASCEND,
    STATE_DIVE2_DESCEND,
    STATE_DIVE2_ASCEND,
    STATE_BROADCAST,
    STATE_SHUTDOWN,
};

FloatState currentState    = STATE_BOOT;
uint32_t  lastRecordTime   = 0;
uint32_t  stateEntryTime   = 0;
uint8_t   currentDive      = 0;

// ─────────────── FORWARD DECLARATIONS ──────────
void motorDescend(uint8_t speed);
void motorAscend(uint8_t speed);
void motorStop();
void motorBrake();
void pollDepthSensor();
void parseDepthLine();
void recordDepth(uint8_t diveId);
void broadcastData();
void sendBtPacket(const uint8_t *data, uint8_t len);
void powerShutdown();
int16_t currentDepthMm();

// ════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════
void setup() {
    // ── CRITICAL: Latch power immediately ──
    pinMode(RELAY_NO_PIN, OUTPUT);
    pinMode(RELAY_NC_PIN, OUTPUT);

    // Step 1: Energise NO relay — establish main power path
    digitalWrite(RELAY_NO_PIN, HIGH);
    delay(2000);  // Let relay settle

    // Step 2: Energise NC relay — this OPENS the NC contact,
    //         disconnecting the reed switch path
    digitalWrite(RELAY_NC_PIN, HIGH);
    delay(2000);
    // Magnet can now be safely removed — power is sustained
    // through the NO relay path only.

    // ── Motor pins ──
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    motorStop();

    // ── Serial: Bluetooth on hardware UART ──
    Serial.begin(9600);     // HC-06 default baud
    Serial.println(F("LUCARIO Float v1.0.0 — Magikarp"));
    Serial.println(F("Power latched. Initialising..."));

    // ── Depth sensor UART ──
    depthSerial.begin(115200);

    // ── Wait for depth sensor to start sending ──
    Serial.println(F("Waiting for depth sensor..."));
    uint32_t waitStart = millis();
    bool gotReading = false;
    while (millis() - waitStart < 5000) {
        pollDepthSensor();
        if (depthLineIdx == 0 && parsedDepthM != 0.0f) {
            gotReading = true;
            break;
        }
        // Also accept if we've parsed at least one line
        if (depthZeroCount > 0) {
            gotReading = true;
            break;
        }
    }

    if (!gotReading) {
        Serial.println(F("WARNING: No depth data received — proceeding anyway"));
    }

    // ── Begin zero-calibration phase ──
    currentState  = STATE_ZERO_CAL;
    stateEntryTime = millis();
    lastRecordTime = millis();

    Serial.println(F("Zero-calibrating depth sensor at surface..."));
}

// ════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════
void loop() {
    // Always poll depth sensor
    pollDepthSensor();

    uint32_t now = millis();

    switch (currentState) {

        // ────────────────────────────────────
        case STATE_ZERO_CAL: {
            // Collect 6 readings (~3 seconds at 2 Hz) for surface zero
            // The parseDepthLine() function handles accumulation
            if (depthZeroed) {
                Serial.print(F("Depth zeroed: offset="));
                Serial.print(depthZeroOffset, 3);
                Serial.println(F("m"));
                Serial.println(F("=== DIVE 1: Descending to 1.00m ==="));
                currentState   = STATE_DIVE1_DESCEND;
                currentDive    = 1;
                stateEntryTime = now;
                lastRecordTime = now;
                motorDescend(DESCEND_SPEED);
            }
            // Timeout: if no zero after 10 seconds, force it
            if (!depthZeroed && (now - stateEntryTime > 10000)) {
                depthZeroOffset = parsedDepthM;
                depthZeroed = true;
                Serial.println(F("Zero-cal timeout — using current reading"));
            }
            break;
        }

        // ────────────────────────────────────
        case STATE_DIVE1_DESCEND: {
            // Record every 5 seconds
            if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
                recordDepth(1);
                lastRecordTime = now;
            }
            // Check target
            if (currentDepthMm() >= DIVE1_BOTTOM - DEPTH_TOLERANCE) {
                motorStop();
                delay(500);  // Settle
                Serial.println(F("=== DIVE 1: Ascending to 0.40m ==="));
                currentState   = STATE_DIVE1_ASCEND;
                stateEntryTime = now;
                motorAscend(ASCEND_SPEED);
            }
            break;
        }

        // ────────────────────────────────────
        case STATE_DIVE1_ASCEND: {
            if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
                recordDepth(1);
                lastRecordTime = now;
            }
            if (currentDepthMm() <= DIVE1_TOP + DEPTH_TOLERANCE) {
                motorStop();
                delay(500);
                Serial.println(F("=== DIVE 2: Descending to 1.00m ==="));
                currentState   = STATE_DIVE2_DESCEND;
                currentDive    = 2;
                stateEntryTime = now;
                motorDescend(DESCEND_SPEED);
            }
            break;
        }

        // ────────────────────────────────────
        case STATE_DIVE2_DESCEND: {
            if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
                recordDepth(2);
                lastRecordTime = now;
            }
            if (currentDepthMm() >= DIVE2_BOTTOM - DEPTH_TOLERANCE) {
                motorStop();
                delay(500);
                Serial.println(F("=== DIVE 2: Ascending to surface ==="));
                currentState   = STATE_DIVE2_ASCEND;
                stateEntryTime = now;
                motorAscend(ASCEND_SPEED);
            }
            break;
        }

        // ────────────────────────────────────
        case STATE_DIVE2_ASCEND: {
            if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
                recordDepth(2);
                lastRecordTime = now;
            }
            if (currentDepthMm() <= DIVE2_TOP + DEPTH_TOLERANCE) {
                motorStop();
                // Final record at surface
                recordDepth(2);
                Serial.println(F("=== DIVES COMPLETE ==="));
                Serial.print(F("Total records: "));
                Serial.println(recordCount);
                currentState   = STATE_BROADCAST;
                stateEntryTime = now;
            }
            break;
        }

        // ────────────────────────────────────
        case STATE_BROADCAST: {
            motorStop();  // Safety — ensure motor is off
            Serial.println(F("=== BROADCASTING DATA ==="));
            broadcastData();
            Serial.println(F("=== BROADCAST COMPLETE ==="));
            currentState   = STATE_SHUTDOWN;
            stateEntryTime = now;
            break;
        }

        // ────────────────────────────────────
        case STATE_SHUTDOWN: {
            // Wait 2 seconds for final Bluetooth buffer flush
            if (now - stateEntryTime > 2000) {
                Serial.println(F("Shutting down — releasing power latch"));
                powerShutdown();
                // If shutdown fails (relay stuck), just idle
                while (1) { delay(1000); }
            }
            break;
        }
    }

    // ── Safety timeout: if any dive phase takes >120 seconds,
    //    abort and surface ──
    if (currentState >= STATE_DIVE1_DESCEND &&
        currentState <= STATE_DIVE2_ASCEND) {
        if (now - stateEntryTime > 120000UL) {
            Serial.println(F("SAFETY TIMEOUT — aborting, ascending"));
            motorAscend(ASCEND_SPEED);
            delay(15000);  // Ascend for 15 seconds
            motorStop();
            currentState   = STATE_BROADCAST;
            stateEntryTime = now;
        }
    }
}

// ════════════════════════════════════════════════
//  MOTOR CONTROL — DRV8871
// ════════════════════════════════════════════════
void motorDescend(uint8_t speed) {
    // IN1=PWM, IN2=LOW → forward (piston pushes water out → sinks)
    analogWrite(MOTOR_IN1_PIN, speed);
    analogWrite(MOTOR_IN2_PIN, 0);
}

void motorAscend(uint8_t speed) {
    // IN1=LOW, IN2=PWM → reverse (piston pulls water in → rises)
    analogWrite(MOTOR_IN1_PIN, 0);
    analogWrite(MOTOR_IN2_PIN, speed);
}

void motorStop() {
    // Coast
    analogWrite(MOTOR_IN1_PIN, 0);
    analogWrite(MOTOR_IN2_PIN, 0);
}

void motorBrake() {
    // Active brake
    analogWrite(MOTOR_IN1_PIN, 255);
    analogWrite(MOTOR_IN2_PIN, 255);
}

// ════════════════════════════════════════════════
//  DEPTH SENSOR UART PARSER
//  Reused from 26Cod72 (LUCARIO ROV Mega)
//  Format: "Depth:0.13m Temp:27.67C\n"
// ════════════════════════════════════════════════
void pollDepthSensor() {
    while (depthSerial.available()) {
        char c = depthSerial.read();
        if (c == '\n' || c == '\r') {
            if (depthLineIdx > 0) {
                depthLineBuf[depthLineIdx] = '\0';
                parseDepthLine();
                depthLineIdx = 0;
            }
        } else {
            if (depthLineIdx < sizeof(depthLineBuf) - 1) {
                depthLineBuf[depthLineIdx++] = c;
            } else {
                depthLineIdx = 0;   // Overflow — discard
            }
        }
    }
}

void parseDepthLine() {
    // ── Parse depth ──
    char *dp = strstr(depthLineBuf, "Depth:");
    if (dp) {
        float rawDepth = atof(dp + 6);

        // Auto-zero: average first 6 readings as surface reference
        if (!depthZeroed) {
            depthZeroSum += rawDepth;
            depthZeroCount++;
            if (depthZeroCount >= 6) {
                depthZeroOffset = depthZeroSum / depthZeroCount;
                depthZeroed = true;
            }
        }

        parsedDepthM = rawDepth - depthZeroOffset;
        if (parsedDepthM < 0.0f) parsedDepthM = 0.0f;
    }

    // ── Parse temperature ──
    char *tp = strstr(depthLineBuf, "Temp:");
    if (tp) {
        parsedTempC = atof(tp + 5);
    }
}

int16_t currentDepthMm() {
    return (int16_t)constrain((int32_t)(parsedDepthM * 1000.0f), 0, 32767);
}

// ════════════════════════════════════════════════
//  DEPTH RECORDING
// ════════════════════════════════════════════════
void recordDepth(uint8_t diveId) {
    if (recordCount >= MAX_RECORDS) return;

    records[recordCount].dive_id      = diveId;
    records[recordCount].record_index = globalIndex++;
    records[recordCount].depth_mm     = currentDepthMm();

    Serial.print(F("REC "));
    Serial.print(recordCount);
    Serial.print(F("  dive="));
    Serial.print(diveId);
    Serial.print(F("  depth="));
    Serial.print(records[recordCount].depth_mm);
    Serial.println(F("mm"));

    recordCount++;
}

// ════════════════════════════════════════════════
//  BLUETOOTH DATA BROADCAST
// ════════════════════════════════════════════════
//
//  Packet format (sent over HC-06 → Serial):
//    [0xAA] [0x42] [LEN] [PAYLOAD] [CHECKSUM]
//
//  We send in chunks of up to 10 records per packet
//  to avoid overwhelming the HC-06 buffer (typically 64–128 bytes).
//
//  Each record in payload (5 bytes):
//    [0]     dive_id       uint8_t
//    [1-2]   record_index  uint16_t LE
//    [3-4]   depth_mm      int16_t  LE
//
//  Final packet with LEN=0 signals end of transmission.
//
// ════════════════════════════════════════════════

const uint8_t BT_RECORDS_PER_PACKET = 10;
const uint8_t BT_RECORD_SIZE        = 5;

void broadcastData() {
    // Send data in chunks
    uint16_t sent = 0;

    while (sent < recordCount) {
        uint8_t chunkSize = min((uint8_t)(recordCount - sent),
                                BT_RECORDS_PER_PACKET);
        uint8_t payloadLen = chunkSize * BT_RECORD_SIZE;
        uint8_t buf[BT_RECORDS_PER_PACKET * BT_RECORD_SIZE];

        for (uint8_t i = 0; i < chunkSize; i++) {
            uint16_t ri = sent + i;
            uint8_t offset = i * BT_RECORD_SIZE;

            buf[offset]     = records[ri].dive_id;
            buf[offset + 1] = (uint8_t)(records[ri].record_index & 0xFF);
            buf[offset + 2] = (uint8_t)((records[ri].record_index >> 8) & 0xFF);
            buf[offset + 3] = (uint8_t)(records[ri].depth_mm & 0xFF);
            buf[offset + 4] = (uint8_t)((records[ri].depth_mm >> 8) & 0xFF);
        }

        sendBtPacket(buf, payloadLen);
        sent += chunkSize;

        Serial.print(F("BT sent "));
        Serial.print(sent);
        Serial.print(F("/"));
        Serial.println(recordCount);

        // Give HC-06 time to transmit (~1ms per byte at 9600 baud)
        delay(100);
    }

    // ── End-of-transmission packet (LEN=0) ──
    sendBtPacket(nullptr, 0);
    Serial.println(F("BT end-of-transmission sent"));
}

void sendBtPacket(const uint8_t *data, uint8_t len) {
    uint8_t cs = 0;

    Serial.write(BT_START);   cs += BT_START;
    Serial.write(BT_CMD);     cs += BT_CMD;
    Serial.write(len);        cs += len;

    for (uint8_t i = 0; i < len; i++) {
        Serial.write(data[i]);
        cs += data[i];
    }

    Serial.write(cs);  // Checksum
}

// ════════════════════════════════════════════════
//  POWER CONTROL
// ════════════════════════════════════════════════
void powerShutdown() {
    // Ensure motor is stopped
    motorStop();

    // Small delay for safety
    delay(100);

    // De-energise NO relay → cuts main power path
    // System will lose power after this line executes.
    digitalWrite(RELAY_NO_PIN, LOW);

    // If we reach here, the relay didn't cut power.
    // De-energise NC relay too as fallback.
    digitalWrite(RELAY_NC_PIN, LOW);
}
