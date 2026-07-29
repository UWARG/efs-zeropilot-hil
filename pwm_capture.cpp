/*
packet format (matches hil_config.py's PWM_CAPTURE_FORMAT = ">B5H"):

[0xAA][0x02][type byte][5 x uint16, big endian][CRC8][0x55] (14 bytes total)
CRC8 is an XOR over the type byte + the 10 data bytes
*/

#include "pwm_capture.h"

// TODO: pin 25->elevator, 4->aileron, 5->rudder, 6->flap, 7->throttle when physically connect to ZP's 5 PWM outputs
static const uint8_t PWM_CAPTURE_PINS[PWM_CAPTURE_NUM_CHANNELS] = {15, 4, 5, 6, 7};

// order matches hil_config.py's PWM_CHANNEL_NAMES:
// elevator, aileron, rudder, flap, throttle

// shared between the ISR and the main loop
static volatile uint16_t g_pulseWidthUs[PWM_CAPTURE_NUM_CHANNELS] = {0};
static volatile uint32_t g_risingEdgeUs[PWM_CAPTURE_NUM_CHANNELS] = {0};

// one ISR per pin, since attachInterrupt doesn't tell you which pin fired, 
// wrapping a shared handler with a channel index avoids five nearly identical functions
// IRAM_ATTR keeps this in fast RAM since ISRs can't safely run from flash on the ESP32
template <int CH>
void IRAM_ATTR onPwmEdge() {
    const uint32_t now = micros();
    if (digitalRead(PWM_CAPTURE_PINS[CH]) == HIGH) {
        // rising edge: pulse starting
        g_risingEdgeUs[CH] = now;
    } else {
        // falling edge: pulse finished, compute width
        uint32_t width = now - g_risingEdgeUs[CH];
        // reject anything outside a sane servo range
        if (width >= 800 && width <= 2200) {
            g_pulseWidthUs[CH] = (uint16_t)width;
        }
    }
}

// attachInterrupt needs a plain function pointer per pin, so we instantiate
// the template once per channel index rather than trying to pass the index at runtime
static void (*const kEdgeHandlers[PWM_CAPTURE_NUM_CHANNELS])() = {
    onPwmEdge<0>, onPwmEdge<1>, onPwmEdge<2>, onPwmEdge<3>, onPwmEdge<4>,
};

void setupPwmCapture() {
    for (int i = 0; i < PWM_CAPTURE_NUM_CHANNELS; i++) {
        pinMode(PWM_CAPTURE_PINS[i], INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(PWM_CAPTURE_PINS[i]), kEdgeHandlers[i], CHANGE);
    }
}

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

void sendPwmCapturePacket() {
    // payload = type byte + 5 x uint16 big endian, matching
    // hil_config.py's PWM_CAPTURE_FORMAT = ">B5H"
    uint8_t payload[1 + PWM_CAPTURE_NUM_CHANNELS * 2];
    payload[0] = PWM_CAPTURE_PACKET_TYPE;

    for (int i = 0; i < PWM_CAPTURE_NUM_CHANNELS; i++) {
        // snapshot the volatile value once so it can't change mid encode
        uint16_t width = g_pulseWidthUs[i];
        payload[1 + i * 2] = (width >> 8) & 0xFF; // big endian high byte
        payload[1 + i * 2 + 1] = width & 0xFF; // big endian low byte
    }

    uint8_t crc = crc8(payload, sizeof(payload));

    Serial.write(0xAA);
    Serial.write(payload, sizeof(payload));
    Serial.write(crc);
    Serial.write(0x55);
}
