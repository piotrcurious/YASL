
import subprocess
import os
import json

def run_low_light_sim():
    ino_path = "YASL_Consolidated.ino"
    with open(ino_path, 'r') as f:
        content = f.read()

    wrapper = f"""
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "Adafruit_INA219.h"

// Content of .ino file
{content}

int main() {{
    sim.ina219_ok = false; // Test sensorless as it's harder in low light
    setup();

    // Stabilize to Daylight
    sim.solarOCV = 18.0;
    Serial.println("[STABILIZING TO DAYLIGHT]");
    for(int i=0; i<800; i++) {{ update_sim(); loop(); }}
    Serial.print("Initial Dark State: "); Serial.println(sys.isDark);

    // Low light sweep
    Serial.println("[START LOW LIGHT SWEEP]");
    for (float ocv = 4.0; ocv <= 8.0; ocv += 0.5) {{
        sim.solarOCV = ocv;
        // Run several loops to allow MPPT to track
        for(int i=0; i<100; i++) {{
            update_sim();
            loop();
        }}
        Serial.print("OCV: "); Serial.print(ocv);
        Serial.print(" Mode: "); Serial.print(sys.chargeMode);
        Serial.print(" SolarV: "); Serial.print(sys.solarV);
        Serial.print(" SolarP: "); Serial.print(sys.solarP_mW);
        Serial.print(" Duty: "); Serial.println(sys.mpptPWM);
    }}

    // Dusk transition with debounce check
    Serial.println("[START DUSK TRANSITION]");
    sim.solarOCV = 1.0;
    for(int i=0; i<1000; i++) {{
        update_sim();
        loop();
        if (i % 200 == 0) {{
            Serial.print("T+"); Serial.print(i*0.1);
            Serial.print("s Mode: "); Serial.print(sys.chargeMode);
            Serial.print(" isDark: "); Serial.println(sys.isDark);
        }}
    }}

    return 0;
}}
"""
    with open('test_env/low_light_sim.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/low_light_sim.cpp", "test_env/Arduino.cpp", "-o", "test_env/low_light_sim_bin"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr)
        return False

    result = subprocess.run(["./test_env/low_light_sim_bin"], capture_output=True, text=True)
    print(result.stdout)

    return True

if __name__ == "__main__":
    run_low_light_sim()
