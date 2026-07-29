#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "pwm_capture.h"

// TODO: keep this in sync with DEFAULT_LAT_DEG/DEFAULT_LON_DEG in hil_config.py
constexpr float DEFAULT_LAT_DEG = 43.4723f;
constexpr float DEFAULT_LON_DEG = -80.5449f;

// INA228 register addresses used by ZeroPilot
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_ADC_CONFIG = 0x01;
constexpr uint8_t REG_SHUNT_CAL = 0x02;
constexpr uint8_t REG_VBUS = 0x05;
constexpr uint8_t REG_CURRENT = 0x07;
constexpr uint8_t REG_POWER = 0x08;
constexpr uint8_t REG_ENERGY = 0x09;
constexpr uint8_t REG_CHARGE = 0x0A;

// from power_module.hpp: INA228_ADDR = 0b1000101 = 0x45, not 0x40
constexpr uint8_t INA228_ADDR = 0x45;
constexpr uint8_t MAX_REGISTER_BYTES = 5;

#define FAKE_VOLTAGE 12.4f
#define FAKE_CURRENT 1.8f
#define FAKE_POWER (FAKE_VOLTAGE * FAKE_CURRENT)

// from power_module.hpp: exact LSB constants, copied directly so encoding matches ZP's decoding
#define CURRENT_LSB (32.0f / (1 << 19))
#define VBUS_LSB 195.3125e-6f
#define POWER_LSB (3.2f * CURRENT_LSB)
#define ENERGY_LSB (16 * 3.2f * CURRENT_LSB)
#define CHARGE_LSB CURRENT_LSB

// UART packet constants, must match hil_config.py / uart_bridge.py
#define START 0xAA
#define END 0x55
#define MSG_AIRCRAFT_STATE 0x01
#define AIRCRAFT_STATE_PAYLOAD_LEN 41 // type byte + 10 big-endian int32 values
#define PKT_LEN (1 + AIRCRAFT_STATE_PAYLOAD_LEN + 1 + 1)

// GPS serial output to ZeroPilot.
// TODO: verify these pins against the actual wiring/schematic.
// Classic ESP32 boards often use GPIO17(TX)/GPIO16(RX) for an extra UART, but ESP32-S3 pins are board-specific.
#define GPS_TX_PIN 17 // ESP32 TX -> ZeroPilot GPS RX
#define GPS_RX_PIN 16 // ESP32 RX <- ZeroPilot GPS TX, optional unless ZP expects bidirectional GPS
#define GPS_BAUD 9600
#define GPS_PERIOD_MS 200 // 5 Hz GPS spoof output

HardwareSerial GPS(1);

struct AircraftState
{
    float altitude_ft;
    float airspeed_kts;
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float latitude_deg;
    float longitude_deg;
    float g_force;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
};

static AircraftState g_state = {
    2000.0f,   // altitude_ft
    55.0f,     // airspeed_kts
    0.0f,      // pitch_deg
    0.0f,      // roll_deg
    0.0f,      // heading_deg
    DEFAULT_LAT_DEG,  // latitude_deg, default near Waterloo
    DEFAULT_LON_DEG, // longitude_deg
    1.0f,      // g_force
    0.0f,      // roll_rate_rad_s
    0.0f       // pitch_rate_rad_s
};

static bool g_haveState = false;

// from power_module.cpp: ZP reads charge and energy as accumulated values, so we accumulate over time
static float fakeCharge = 0.0f;
static float fakeEnergy = 0.0f;

// Register table stores configuration values written and read by ZeroPilot.
// Measurement registers are generated dynamically in onRequest().
static uint8_t registers[256][MAX_REGISTER_BYTES] = {};
static uint8_t registerLength[256] = {};
static volatile uint8_t currentRegister = REG_CONFIG;

void setRegisterBytes(uint8_t reg, const uint8_t *data, uint8_t length)
{
    if (length > MAX_REGISTER_BYTES)
    {
        length = MAX_REGISTER_BYTES;
    }

    registerLength[reg] = length;
    for (uint8_t i = 0; i < length; i++)
    {
        registers[reg][i] = data[i];
    }
}

void setupFakeINA228Registers()
{
    // Default values used until ZeroPilot writes its own configuration.
    const uint8_t config[2] = {0x80, 0x00};
    const uint8_t adcConfig[2] = {0xF0, 0x02};
    const uint8_t shuntCal[2] = {0x00, 0x00};

    setRegisterBytes(REG_CONFIG, config, sizeof(config));
    setRegisterBytes(REG_ADC_CONFIG, adcConfig, sizeof(adcConfig));
    setRegisterBytes(REG_SHUNT_CAL, shuntCal, sizeof(shuntCal));
}

// from power_module.hpp: VBUS, CURRENT, POWER are 24 bit (3 bytes), ENERGY and CHARGE are 40 bit (5 bytes)
void packU24(uint8_t *buf, uint32_t val)
{
    buf[0] = (val >> 16) & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = val & 0xFF;
}

void packU40(uint8_t *buf, uint64_t val)
{
    buf[0] = (val >> 32) & 0xFF;
    buf[1] = (val >> 24) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 8) & 0xFF;
    buf[4] = val & 0xFF;
}

// Fires when ZeroPilot writes a register address, optionally followed by data.
void onReceive(int numBytes)
{
    if (numBytes <= 0 || !Wire.available())
    {
        return;
    }

    const uint8_t reg = Wire.read();
    currentRegister = reg;

    uint8_t data[MAX_REGISTER_BYTES];
    uint8_t count = 0;

    while (Wire.available() && count < MAX_REGISTER_BYTES)
    {
        data[count++] = Wire.read();
    }

    // Drain any unexpected extra bytes so the next transaction starts cleanly.
    while (Wire.available())
    {
        Wire.read();
    }

    if (count > 0)
    {
        setRegisterBytes(reg, data, count);
    }
}

void onRequest()
{
    uint8_t buf[5] = {0};
    const uint8_t reg = currentRegister;

    // from power_module.cpp: ZP reads in this order via DMA callback chain: VBUS -> CURRENT -> POWER -> ENERGY -> CHARGE
    // from power_module.cpp: readData() reverses this encoding, so we must encode with same LSBs and bit shifts
    switch (reg)
    {
    case REG_VBUS:
    { // from power_module.hpp: REG_VBUS = {0x05, 3 bytes}, 24 bit unsigned, left shifted 4
        uint32_t raw = ((uint32_t)(FAKE_VOLTAGE / VBUS_LSB)) << 4;
        packU24(buf, raw);
        Wire.write(buf, 3);
        break;
    }
    case REG_CURRENT:
    { // from power_module.hpp: REG_CURRENT = {0x07, 3 bytes}, 24 bit signed, left shifted 4
        int32_t raw = ((int32_t)(FAKE_CURRENT / CURRENT_LSB)) << 4;
        packU24(buf, (uint32_t)raw);
        Wire.write(buf, 3);
        break;
    }
    case REG_POWER:
    { // from power_module.hpp: REG_POWER = {0x08, 3 bytes}, 24 bit unsigned, no shift
        uint32_t raw = (uint32_t)(FAKE_POWER / POWER_LSB);
        packU24(buf, raw);
        Wire.write(buf, 3);
        break;
    }
    case REG_ENERGY:
    { // from power_module.hpp: REG_ENERGY = {0x09, 5 bytes}, 40 bit unsigned
        uint64_t raw = (uint64_t)(fakeEnergy / ENERGY_LSB);
        packU40(buf, raw);
        Wire.write(buf, 5);
        break;
    }
    case REG_CHARGE:
    { // from power_module.hpp: REG_CHARGE = {0x0A, 5 bytes}, 40 bit signed
        int64_t raw = (int64_t)(fakeCharge / CHARGE_LSB);
        packU40(buf, (uint64_t)raw);
        Wire.write(buf, 5);
        break;
    }
    default:
    {
        const uint8_t length = registerLength[reg];
        if (length > 0)
        {
            Wire.write(registers[reg], length);
        }
        else
        {
            Wire.write((uint8_t)0x00);
        }
        break;
    }
    }
}

int32_t readI32BE(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) |
                     ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) |
                     ((uint32_t)p[3]));
}

void parseAircraftStatePayload(const uint8_t *payload)
{
    if (payload[0] != MSG_AIRCRAFT_STATE)
    {
        Serial.print("[ESP32] unknown msg type: ");
        Serial.println(payload[0], HEX);
        return;
    }

    g_state.altitude_ft = readI32BE(payload + 1) / 100.0f;
    g_state.airspeed_kts = readI32BE(payload + 5) / 100.0f;
    g_state.pitch_deg = readI32BE(payload + 9) / 100.0f;
    g_state.roll_deg = readI32BE(payload + 13) / 100.0f;
    g_state.heading_deg = readI32BE(payload + 17) / 100.0f;
    g_state.latitude_deg = readI32BE(payload + 21) / 10000000.0f;
    g_state.longitude_deg = readI32BE(payload + 25) / 10000000.0f;
    g_state.g_force = readI32BE(payload + 29) / 1000.0f;
    g_state.roll_rate_rad_s = readI32BE(payload + 33) / 1000.0f;
    g_state.pitch_rate_rad_s = readI32BE(payload + 37) / 1000.0f;
    g_haveState = true;

    // Serial.print("[ESP32] state alt_ft=");
    // Serial.print(g_state.altitude_ft, 2);
    // Serial.print(" lat=");
    // Serial.print(g_state.latitude_deg, 7);
    // Serial.print(" lon=");
    // Serial.print(g_state.longitude_deg, 7);
    // Serial.print(" spd_kts=");
    // Serial.println(g_state.airspeed_kts, 2);
}

uint8_t nmeaChecksum(const char *body)
{
    uint8_t checksum = 0;
    while (*body)
    {
        checksum ^= (uint8_t)(*body++);
    }
    return checksum;
}

float utcTimeFromMillis()
{
    uint32_t ms = millis();
    uint32_t totalSeconds = (ms / 1000) % 86400;
    uint32_t hh = totalSeconds / 3600;
    uint32_t mm = (totalSeconds % 3600) / 60;
    float ss = (float)(totalSeconds % 60) + (float)(ms % 1000) / 1000.0f;
    return (float)(hh * 10000 + mm * 100) + ss;
}

void decimalDegToNmea(float deg, bool isLat, char *out, size_t outLen, char *hemi)
{
    *hemi = deg >= 0.0f ? (isLat ? 'N' : 'E') : (isLat ? 'S' : 'W');

    float absDeg = fabsf(deg);
    int wholeDeg = (int)absDeg;
    float minutes = (absDeg - (float)wholeDeg) * 60.0f;

    if (isLat)
    {
        snprintf(out, outLen, "%02d%08.5f", wholeDeg, minutes);
    }
    else
    {
        snprintf(out, outLen, "%03d%08.5f", wholeDeg, minutes);
    }
}

void sendNmeaSentence(const char *body)
{
    char sentence[160];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, nmeaChecksum(body));
    GPS.print(sentence);
}

void sendGpsNmea()
{
    char lat[16], lon[16];
    char ns, ew;

    // debug
    GPS.print("[DEBUG] raw lat="); GPS.println(g_state.latitude_deg, 7);
    GPS.print("[DEBUG] raw lon="); GPS.println(g_state.longitude_deg, 7);

    decimalDegToNmea(g_state.latitude_deg, true, lat, sizeof(lat), &ns);
    decimalDegToNmea(g_state.longitude_deg, false, lon, sizeof(lon), &ew);

    float utcTime = utcTimeFromMillis();
    float altitudeM = g_state.altitude_ft * 0.3048f;

    char body[140];

    // RMC: time, validity, lat/lon, speed in knots, course/heading, date.
    snprintf(
        body,
        sizeof(body),
        "GPRMC,%09.2f,A,%s,%c,%s,%c,%.3f,%.2f,010126,,,A",
        utcTime,
        lat,
        ns,
        lon,
        ew,
        g_state.airspeed_kts,
        g_state.heading_deg);
    sendNmeaSentence(body);

    // GGA: time, lat/lon, fix quality, satellite count, HDOP, altitude in meters.
    snprintf(
        body,
        sizeof(body),
        "GPGGA,%09.2f,%s,%c,%s,%c,1,12,1.01,%.1f,M,48.0,M,,",
        utcTime,
        lat,
        ns,
        lon,
        ew,
        altitudeM);
    sendNmeaSentence(body);
}

void readPiUartPackets()
{
    static uint8_t buf[PKT_LEN];
    static int idx = 0;
    static bool synced = false;

    while (Serial.available())
    {
        uint8_t b = Serial.read();
        if (!synced)
        {
            if (b == START)
            {
                idx = 0;
                buf[idx++] = b;
                synced = true;
            }
            continue;
        }

        buf[idx++] = b;
        if (idx == PKT_LEN)
        {
            synced = false;

            if (buf[PKT_LEN - 1] != END)
            {
                Serial.println("[ESP32] dropped packet: bad end byte");
                return;
            }

            uint8_t crc = 0;
            for (int i = 1; i < PKT_LEN - 2; i++)
            {
                crc ^= buf[i];
            }

            if (crc != buf[PKT_LEN - 2])
            {
                Serial.println("[ESP32] dropped packet: bad crc");
                return;
            }

            parseAircraftStatePayload(buf + 1);
            // for checking esp32 -> pi
            // Serial.write(buf, PKT_LEN);  // echo the validated packet straight back to the pi
        }
        else if (idx >= PKT_LEN)
        {
            synced = false;
            idx = 0;
        }
    }
}

void setup()
{
    Serial.begin(115200); // Pi <-> ESP32 packet UART over USB for now

    GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    setupFakeINA228Registers();
    Wire.begin(INA228_ADDR); // I2C slave, default pins GPIO21(SDA) GPIO22(SCL)
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    setupPwmCapture();

    Serial.println("[ESP32] HIL peripheral emulator ready");
    Serial.print("[ESP32] GPS spoof UART baud=");
    Serial.print(GPS_BAUD);
    Serial.print(" tx=");
    Serial.print(GPS_TX_PIN);
    Serial.print(" rx=");
    Serial.println(GPS_RX_PIN);
}

void loop()
{
    readPiUartPackets();
    sendPwmCapturePacket();

    static uint32_t lastGpsMs = 0;
    uint32_t now = millis();
    if (now - lastGpsMs >= GPS_PERIOD_MS)
    {
        lastGpsMs = now;
        sendGpsNmea();
    }

    // from power_module_iface.hpp: charge and energy are accumulated fields, accumulate here so ZP sees realistic values
    fakeCharge += FAKE_CURRENT * (10.0f / 1000.0f); // amps * dt_sec
    fakeEnergy += FAKE_POWER * (10.0f / 1000.0f);   // watts * dt_sec

    delay(10); // 100Hz loop
}
