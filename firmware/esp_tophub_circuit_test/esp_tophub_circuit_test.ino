// ============================================================================
// PROJECT DOULA — BAREBONES U2 (ESP_TOPHUB) CIRCUIT TEST SKETCH
// Target MCU : ESP32-S NodeMCU V1.1 (Arduino-ESP32 v2.x / v3.x)
// Purpose    : Minimal open-loop bench test for:
//                - Rhino MD20Amp Motor Driver (GPIO25 PWM, GPIO26 DIR)
//                - 30A Brake Chopper MOSFET   (GPIO27 PWM)
//                - ENC1 Quadrature Encoder    (GPIO32 A, GPIO33 B)
//                - VBUS Voltage Divider       (GPIO34 ADC1_CH6)
//                - ACS712-30A Current Sensor  (GPIO35 ADC1_CH7)
//
// Serial Monitor: 115200 baud (Newline or Both NL & CR)
// ============================================================================

#include <Arduino.h>
#include <driver/pcnt.h>

// ----------------------------------------------------------------------------
// 1. HARDWARE PINS (Matches U2 ESP_TOPHUB Wiring Guide)
// ----------------------------------------------------------------------------
static const int PIN_MD_PWM       = 25; // Rhino MD20Amp PWM (20 kHz)
static const int PIN_MD_DIR       = 26; // Rhino MD20Amp DIR
static const int PIN_CHOP_PWM     = 27; // 30A Brake Chopper MOSFET (5 kHz)
static const int PIN_ENC1_A       = 32; // ENC1 Out A (INPUT_PULLUP)
static const int PIN_ENC1_B       = 33; // ENC1 Out B (INPUT_PULLUP)
static const int PIN_VBUS_SENSE   = 34; // ADC1_CH6: R11/R12 Bus Voltage Divider
static const int PIN_ACS712_SENSE = 35; // ADC1_CH7: R13/R14 ACS712-30A Divider

// ----------------------------------------------------------------------------
// 2. SAFETY CAPS & CONSTANTS (Edit these if needed)
// ----------------------------------------------------------------------------
static const int   MAX_MOTOR_PWM_PCT = 100;      // Safety cap for motor PWM (0..100%)
static const int   MAX_CHOP_PWM_PCT  = 50;      // Safety cap for chopper PWM (0..100%)

static const int   MD_PWM_FREQ_HZ    = 20000;   // 20 kHz motor PWM
static const int   CHOP_PWM_FREQ_HZ  = 5000;    // 5 kHz chopper PWM
static const int   PWM_RES_BITS      = 10;      // 10-bit resolution (0..1023)
static const int   PWM_MAX_DUTY      = (1 << PWM_RES_BITS) - 1;

static const int   LEDC_CH_MOTOR     = 0;       // Used on Arduino-ESP32 v2.x
static const int   LEDC_CH_CHOPPER   = 1;       // Used on Arduino-ESP32 v2.x

static const float VBUS_DIV_MULT     = 16.942f; // (73.333k + 4.6k) / 4.6k
static const float ACS712_MV_PER_AMP = 33.0f;   // 66 mV/A * 0.50 (22k/22k divider)

// ----------------------------------------------------------------------------
// 3. GLOBAL STATE
// ----------------------------------------------------------------------------
static int     g_motor_pwm_pct  = 0;
static int     g_motor_dir      = 1;      // 1 = Forward (HIGH), 0 = Reverse (LOW)
static int     g_chop_pwm_pct   = 0;
static bool    g_live_print     = true;
static float   g_acs_zero_mv    = 1250.0f;
static int64_t g_enc_counts     = 0;
static int16_t g_prev_pcnt_raw  = 0;

static char    g_cmd_buf[64];
static uint8_t g_cmd_len        = 0;

// ----------------------------------------------------------------------------
// 4. PWM HELPERS (Arduino-ESP32 v2.x & v3.x compatible)
// ----------------------------------------------------------------------------
static void initPWM() {
    pinMode(PIN_MD_DIR, OUTPUT);
    digitalWrite(PIN_MD_DIR, HIGH);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_MD_PWM, MD_PWM_FREQ_HZ, PWM_RES_BITS);
    ledcWrite(PIN_MD_PWM, 0);
    ledcAttach(PIN_CHOP_PWM, CHOP_PWM_FREQ_HZ, PWM_RES_BITS);
    ledcWrite(PIN_CHOP_PWM, 0);
#else
    ledcSetup(LEDC_CH_MOTOR, MD_PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_MD_PWM, LEDC_CH_MOTOR);
    ledcWrite(LEDC_CH_MOTOR, 0);

    ledcSetup(LEDC_CH_CHOPPER, CHOP_PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_CHOP_PWM, LEDC_CH_CHOPPER);
    ledcWrite(LEDC_CH_CHOPPER, 0);
#endif
}

static void setMotorOutput(int dir, int pct) {
    pct = constrain(pct, 0, MAX_MOTOR_PWM_PCT);
    g_motor_dir     = (dir != 0) ? 1 : 0;
    g_motor_pwm_pct = pct;

    digitalWrite(PIN_MD_DIR, g_motor_dir ? HIGH : LOW);
    uint32_t duty = (uint32_t)((pct * PWM_MAX_DUTY) / 100);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_MD_PWM, duty);
#else
    ledcWrite(LEDC_CH_MOTOR, duty);
#endif
}

static void setChopperOutput(int pct) {
    pct = constrain(pct, 0, MAX_CHOP_PWM_PCT);
    g_chop_pwm_pct = pct;

    uint32_t duty = (uint32_t)((pct * PWM_MAX_DUTY) / 100);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_CHOP_PWM, duty);
#else
    ledcWrite(LEDC_CH_CHOPPER, duty);
#endif
}

static void stopAllOutputs() {
    setMotorOutput(g_motor_dir, 0);
    setChopperOutput(0);
}

// ----------------------------------------------------------------------------
// 5. HARDWARE PCNT ENCODER HELPERS (GPIO32 / GPIO33)
// ----------------------------------------------------------------------------
static void initEncoderPCNT() {
    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);

    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = PIN_ENC1_A;
    ch0.ctrl_gpio_num  = PIN_ENC1_B;
    ch0.unit           = PCNT_UNIT_0;
    ch0.channel        = PCNT_CHANNEL_0;
    ch0.pos_mode       = PCNT_COUNT_INC;
    ch0.neg_mode       = PCNT_COUNT_DEC;
    ch0.lctrl_mode     = PCNT_MODE_REVERSE;
    ch0.hctrl_mode     = PCNT_MODE_KEEP;
    ch0.counter_h_lim  = 30000;
    ch0.counter_l_lim  = -30000;
    pcnt_unit_config(&ch0);

    pcnt_config_t ch1 = {};
    ch1.pulse_gpio_num = PIN_ENC1_B;
    ch1.ctrl_gpio_num  = PIN_ENC1_A;
    ch1.unit           = PCNT_UNIT_0;
    ch1.channel        = PCNT_CHANNEL_1;
    ch1.pos_mode       = PCNT_COUNT_DEC;
    ch1.neg_mode       = PCNT_COUNT_INC;
    ch1.lctrl_mode     = PCNT_MODE_REVERSE;
    ch1.hctrl_mode     = PCNT_MODE_KEEP;
    ch1.counter_h_lim  = 30000;
    ch1.counter_l_lim  = -30000;
    pcnt_unit_config(&ch1);

    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    g_prev_pcnt_raw = 0;
    g_enc_counts    = 0;
}

static int64_t readEncoderCounts() {
    int16_t raw = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw);
    int16_t delta = raw - g_prev_pcnt_raw;
    g_prev_pcnt_raw = raw;
    g_enc_counts += (int64_t)delta;

    if (raw > 25000 || raw < -25000) {
        pcnt_counter_clear(PCNT_UNIT_0);
        g_prev_pcnt_raw = 0;
    }
    return g_enc_counts;
}

static void zeroEncoder() {
    pcnt_counter_clear(PCNT_UNIT_0);
    g_prev_pcnt_raw = 0;
    g_enc_counts    = 0;
}

// ----------------------------------------------------------------------------
// 6. ANALOG SENSOR HELPERS (VBUS & ACS712)
// ----------------------------------------------------------------------------
static float readAveragedMilliVolts(int pin, int samples = 16) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; ++i) {
        sum += analogReadMilliVolts(pin);
    }
    return (float)sum / (float)samples;
}

static void zeroACS712() {
    g_acs_zero_mv = readAveragedMilliVolts(PIN_ACS712_SENSE, 64);
}

static void printTelemetry() {
    float vbus_mv = readAveragedMilliVolts(PIN_VBUS_SENSE, 16);
    float vbus_v  = (vbus_mv / 1000.0f) * VBUS_DIV_MULT;

    float acs_mv  = readAveragedMilliVolts(PIN_ACS712_SENSE, 16);
    float amps    = (acs_mv - g_acs_zero_mv) / ACS712_MV_PER_AMP;

    int64_t enc   = readEncoderCounts();

    Serial.printf("VBUS: %5.2f V | ACS712: %6.1f mV (%+6.2f A) | ENC1: %8lld | MOT: %s %2d%% | CHOP: %2d%%\n",
                  vbus_v,
                  acs_mv,
                  amps,
                  (long long)enc,
                  (g_motor_pwm_pct == 0) ? "STOP" : (g_motor_dir ? "FWD " : "REV "),
                  g_motor_pwm_pct,
                  g_chop_pwm_pct);
}

// ----------------------------------------------------------------------------
// 7. AUTO BENCH TEST SEQUENCE (Abortable anytime via Serial input)
// ----------------------------------------------------------------------------
static bool waitWithAbort(uint32_t duration_ms) {
    uint32_t start = millis();
    uint32_t last_print = 0;
    while (millis() - start < duration_ms) {
        if (Serial.available() > 0) {
            while (Serial.available() > 0) Serial.read();
            stopAllOutputs();
            Serial.println("[TEST] Aborted by Serial input -> ALL OUTPUTS STOPPED.");
            return false;
        }
        if (millis() - last_print >= 250) {
            last_print = millis();
            printTelemetry();
        }
        delay(10);
    }
    return true;
}

static void runAutoBenchTest() {
    Serial.println("\n[TEST] Starting safe auto bench test (send any char to abort)...");

    Serial.println("[TEST] 1/5 Forward 20% for 2s");
    setMotorOutput(1, 20);
    if (!waitWithAbort(2000)) return;

    Serial.println("[TEST] 2/5 Stop for 1s");
    setMotorOutput(1, 0);
    if (!waitWithAbort(1000)) return;

    Serial.println("[TEST] 3/5 Reverse 20% for 2s");
    setMotorOutput(0, 20);
    if (!waitWithAbort(2000)) return;

    Serial.println("[TEST] 4/5 Stop for 1s");
    setMotorOutput(0, 0);
    if (!waitWithAbort(1000)) return;

    Serial.println("[TEST] 5/5 Chopper 20% for 1s");
    setChopperOutput(20);
    if (!waitWithAbort(1000)) return;

    stopAllOutputs();
    Serial.println("[TEST] Complete! All outputs stopped.\n");
}

// ----------------------------------------------------------------------------
// 8. SERIAL COMMAND INTERFACE
// ----------------------------------------------------------------------------
static void printHelp() {
    Serial.println("\n=== U2 (ESP_TOPHUB) BAREBONES CIRCUIT TEST ===");
    Serial.printf("Caps: Motor <= %d%%, Chopper <= %d%%\n", MAX_MOTOR_PWM_PCT, MAX_CHOP_PWM_PCT);
    Serial.println("Commands:");
    Serial.println("  f <pct>   : Run motor FORWARD at <pct>% PWM (e.g. 'f 20')");
    Serial.println("  r <pct>   : Run motor REVERSE at <pct>% PWM (e.g. 'r 20')");
    Serial.println("  s / stop  : STOP motor and chopper (0% PWM)");
    Serial.println("  c <pct>   : Set Brake Chopper PWM to <pct>% (e.g. 'c 20', 'c 0')");
    Serial.println("  z         : Zero encoder count & re-zero ACS712 (must be stopped)");
    Serial.println("  test      : Run short auto bench test (Fwd 20%, Rev 20%, Chop 20%)");
    Serial.println("  p         : Toggle live telemetry print (every 500 ms)");
    Serial.println("  h / ?     : Show this help menu\n");
}

static void handleCommand(char *line) {
    // Trim leading whitespace
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    if (strcmp(line, "s") == 0 || strcmp(line, "stop") == 0) {
        stopAllOutputs();
        Serial.println("[CMD] STOP -> Motor 0%, Chopper 0%");
    } else if (strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
        printHelp();
    } else if (strcmp(line, "p") == 0) {
        g_live_print = !g_live_print;
        Serial.printf("[CMD] Live telemetry print: %s\n", g_live_print ? "ON" : "OFF");
    } else if (strcmp(line, "z") == 0) {
        zeroEncoder();
        if (g_motor_pwm_pct == 0) {
            zeroACS712();
            Serial.printf("[CMD] Zeroed ENC1 and ACS712 (0A = %.1f mV)\n", g_acs_zero_mv);
        } else {
            Serial.println("[CMD] Zeroed ENC1 (stop motor first to re-zero ACS712)");
        }
    } else if (strcmp(line, "test") == 0) {
        runAutoBenchTest();
    } else if (line[0] == 'f' && (line[1] == ' ' || line[1] == '\0')) {
        int pct = (line[1] == '\0') ? 20 : atoi(&line[2]);
        setMotorOutput(1, pct);
        Serial.printf("[CMD] Motor FORWARD at %d%%\n", g_motor_pwm_pct);
    } else if (line[0] == 'r' && (line[1] == ' ' || line[1] == '\0')) {
        int pct = (line[1] == '\0') ? 20 : atoi(&line[2]);
        setMotorOutput(0, pct);
        Serial.printf("[CMD] Motor REVERSE at %d%%\n", g_motor_pwm_pct);
    } else if (line[0] == 'c' && (line[1] == ' ' || line[1] == '\0')) {
        int pct = (line[1] == '\0') ? 0 : atoi(&line[2]);
        setChopperOutput(pct);
        Serial.printf("[CMD] Chopper PWM at %d%%\n", g_chop_pwm_pct);
    } else {
        Serial.printf("[CMD] Unknown command '%s'. Type 'h' for help.\n", line);
    }
}

static void pollSerial() {
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            g_cmd_buf[g_cmd_len] = '\0';
            handleCommand(g_cmd_buf);
            g_cmd_len = 0;
        } else if (g_cmd_len < sizeof(g_cmd_buf) - 1) {
            g_cmd_buf[g_cmd_len++] = (char)tolower((unsigned char)ch);
        }
    }
}

// ----------------------------------------------------------------------------
// 9. SETUP & LOOP
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);

    initPWM();
    stopAllOutputs();
    initEncoderPCNT();

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBUS_SENSE, ADC_11db);
    analogSetPinAttenuation(PIN_ACS712_SENSE, ADC_11db);

    delay(100);
    zeroACS712();

    printHelp();
    Serial.printf("ACS712 0A baseline captured: %.1f mV\n", g_acs_zero_mv);
}

void loop() {
    pollSerial();

    static uint32_t last_print_ms = 0;
    if (g_live_print && (millis() - last_print_ms >= 500)) {
        last_print_ms = millis();
        printTelemetry();
    }
}
