
import subprocess
import os

def run_sync_test():
    ino_path = "YASL_Consolidated.ino"
    with open(ino_path, 'r') as f:
        content = f.read()

    wrapper = f"""
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

// Content of .ino file
{content}

int main() {{
    sim.vcc = 5.0;
    sim.batteryV = 3.3;
    sim.solarOCV = 18.0;

    setup();

    Serial.println("[SYNCHRONOUS PWM TEST]");

    // Day/Night debounce is 60s. To skip it in test, we force state.
    sys.isDark = false;
    current_charge_stage = 'B';

    // Run long enough for MPPT to ramp up
    for(int i=0; i<200; i++) {{
        update_sim();
        loop();
        if (OCR1A > 100) break;
    }}

    Serial.print("Duty A (OCR1A): "); Serial.println(OCR1A);
    Serial.print("Duty B (OCR1B): "); Serial.println(OCR1B);

    // In Phase Correct mode (OC1B inverted), OCR1B must be > OCR1A for deadtime
    if (OCR1A > 100 && OCR1B > OCR1A) {{
        Serial.println("SYNC SIGNAL ACTIVE WITH DEADTIME");
    }} else {{
        Serial.println("SYNC SIGNAL FAILURE");
    }}

    return 0;
}}
"""
    with open('test_env/sync_test.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/sync_test.cpp", "test_env/Arduino.cpp", "-o", "test_env/sync_test_bin"]
    subprocess.run(cmd, check=True)
    result = subprocess.run(["./test_env/sync_test_bin"], capture_output=True, text=True)
    print(result.stdout)

    return "SYNC SIGNAL ACTIVE WITH DEADTIME" in result.stdout

if __name__ == "__main__":
    if run_sync_test():
        print("PASSED")
    else:
        print("FAILED")
        exit(1)
