/*
ESP32 version of pwm_lgpio.py (for Pi)
captures ZP's 5 PWM output lines and packs them into a 0x02 packet for the Pi
*/

#ifndef PWM_CAPTURE_H
#define PWM_CAPTURE_H

#include <Arduino.h>

// matches hil_config.py and esp32_link.py
#define PWM_CAPTURE_PACKET_TYPE 0x02
#define PWM_CAPTURE_NUM_CHANNELS 5

// configures the 5 input pins and attaches the edge interrupts that do the actual pulse timing
void setupPwmCapture();

// builds and writes one 0x02 packet to Serial using the latest captured widths every loop
void sendPwmCapturePacket();

#endif
