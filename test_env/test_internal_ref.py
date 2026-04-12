
import subprocess
import os

def run_internal_ref_test():
    ino_path = "YASL_Consolidated.ino"
    with open(ino_path, 'r') as f:
        content = f.read()

    # Enable internal reference for this test
    content = content.replace("#define USE_INTERNAL_1V1_REF    false", "#define USE_INTERNAL_1V1_REF    true")

    wrapper = f"""
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

// Content of .ino file
{content}

int main() {{
    setup();

    Serial.println("[INTERNAL REF TEST]");

    // In INTERNAL mode, ADC uses 1.1V ref.
    // Battery Divider is 5.54.
    // Real Vbat = 3.7V -> Vpin = 3.7 / 5.54 = 0.667V.
    // ADC = 0.667 * 1023 / 1.1 = 620.

    sim.vcc = 4.0; // Vcc shouldn't matter now
    sim.batteryV = 3.7;
    readSensors();
    Serial.print("Vcc: 4.0, Real Vbat: 3.7, Reported: "); Serial.println(sys.batV);

    sim.vcc = 3.3;
    sim.batteryV = 3.0;
    readSensors();
    Serial.print("Vcc: 3.3, Real Vbat: 3.0, Reported: "); Serial.println(sys.batV);

    if (abs(sys.batV - 3.0) < 0.1) {{
        Serial.println("INTERNAL REF SUCCESS");
    }} else {{
        Serial.println("INTERNAL REF FAIL");
    }}

    return 0;
}}
"""
    with open('test_env/internal_test.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/internal_test.cpp", "test_env/Arduino.cpp", "-o", "test_env/internal_test_bin"]
    subprocess.run(cmd, check=True)
    result = subprocess.run(["./test_env/internal_test_bin"], capture_output=True, text=True)
    print(result.stdout)

    return "INTERNAL REF SUCCESS" in result.stdout

if __name__ == "__main__":
    if run_internal_ref_test():
        print("PASSED")
    else:
        print("FAILED")
        exit(1)
