
import subprocess
import os

def run_vcc_dependency_test():
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
    setup();

    Serial.println("[VCC DEPENDENCY TEST]");

    // Scenario 1: Fixed 5V supply (Baseline)
    sim.vcc = 5.0;
    sim.batteryV = 3.7;
    // Advance time to ensure Vcc check triggers
    delay(6000);
    readSensors();
    Serial.print("Vcc: 5.0, Real Vbat: 3.7, Reported: "); Serial.println(sys.batV);

    // Scenario 2: Powered from Li-ion (e.g. 3.3V)
    sim.vcc = 3.3;
    sim.batteryV = 3.3; // System powered from the battery it measures
    delay(6000);
    readSensors();
    Serial.print("Vcc: 3.3, Real Vbat: 3.3, Reported: "); Serial.println(sys.batV);

    if (abs(sys.batV - 3.3) < 0.1) {{
        Serial.println("VCC COMPENSATION SUCCESS");
    }} else {{
        Serial.println("VCC COMPENSATION FAIL");
    }}

    return 0;
}}
"""
    with open('test_env/vcc_test.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/vcc_test.cpp", "test_env/Arduino.cpp", "-o", "test_env/vcc_test_bin"]
    subprocess.run(cmd, check=True)
    result = subprocess.run(["./test_env/vcc_test_bin"], capture_output=True, text=True)
    print(result.stdout)

    return "VCC COMPENSATION SUCCESS" in result.stdout

if __name__ == "__main__":
    if run_vcc_dependency_test():
        print("PASSED")
    else:
        print("FAILED")
        exit(1)
