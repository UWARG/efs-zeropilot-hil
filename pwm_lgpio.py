import time
import lgpio
import socket
import struct
import sys

GPIO_PINS = [18, 27, 24, 25, 17] 
GPIO_CHIP_ID = 4

DEST_IP = "127.0.0.1"
DEST_PORT = 50005

SIGNAL_TIMEOUT = 0.1

high_ticks = {pin: None for pin in GPIO_PINS}
pulse_widths = {pin: 0.0 for pin in GPIO_PINS}
last_edge_times = {pin: time.time() for pin in GPIO_PINS}


def cbf(chip, gpio, level, timestamp):
    global high_ticks, pulse_widths, last_edge_times
    
    last_edge_times[gpio] = time.time()
    
    if level == 1: # Rising Edge
        high_ticks[gpio] = timestamp
    elif level == 0: # Falling Edge
        if high_ticks[gpio] is not None:
            diff_ns = timestamp - high_ticks[gpio]
            pulse_widths[gpio] = diff_ns / 1000000.0

try:
    h = lgpio.gpiochip_open(GPIO_CHIP_ID)
except lgpio.error as e:
    print(f"Failed to open GPIO Chip {GPIO_CHIP_ID}.")
    sys.exit(1)

for pin in GPIO_PINS:
    try:
        lgpio.gpio_claim_input(h, pin)
        lgpio.gpio_claim_alert(h, pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_DOWN)
        lgpio.callback(h, pin, lgpio.BOTH_EDGES, cbf)
    except lgpio.error as e:
        print(f"Error claiming Pin {pin}: {e}")


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"--- PWM DRIVER RUNNING ---")
print(f"Sending 5 channels to Simulink: {DEST_PORT}")

try:
    while True:
        output_vector = []
        current_time = time.time()

        for pin in GPIO_PINS:
            if (current_time - last_edge_times[pin]) > SIGNAL_TIMEOUT:
                val = 0.0
            else:
                val = pulse_widths[pin]
            output_vector.append(val)

        data = struct.pack('ddddd', *output_vector)
        
        sock.sendto(data, (DEST_IP, DEST_PORT))
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    lgpio.gpio_cancel_call(h)
    lgpio.gpiochip_close(h)
    sock.close()
