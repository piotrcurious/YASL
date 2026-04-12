
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
    // Debounce is 60s, so we need > 600 loops of 100ms.
    // Main loop has a 50ms delay + whatever update_sim does.
    // Actually, loop() in .ino has a 50ms delay. update_sim advances 100ms.
    // So each loop iteration is effectively 150ms of sim time?
    // No, delay(50) advances current_time_ms by 50. update_sim advances by 100.
    // So each iteration is 150ms. 60s / 0.15s = 400 iterations.
    for(int i=0; i<500; i++) {{ update_sim(); loop(); }}
    Serial.print("Initial Dark State: "); Serial.println(sys.isDark);
    Serial.print("Initial Mode: "); Serial.println(sys.chargeMode);

    // Low light sweep
    Serial.println("[START LOW LIGHT SWEEP]");
    for (float ocv = 3.0; ocv <= 6.0; ocv += 0.5) {{
        sim.solarOCV = ocv;
        // Run several loops to allow MPPT to track
        for(int i=0; i<200; i++) {{
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
    // OCV 1.0 is below SOLAR_DARK_V (2.0). Debounce should trigger after 60s.
    // 60s / 0.15s = 400 iterations.
    for(int i=0; i<600; i++) {{
        update_sim();
        loop();
        if (i % 100 == 0) {{
            Serial.print("T+"); Serial.print(i*0.15);
            Serial.print("s Mode: "); Serial.print(sys.chargeMode);
            Serial.print(" isDark: "); Serial.println(sys.isDark);
        }}
        if (sys.isDark && i > 400) break; // Optimization
    }}
    Serial.print("Final isDark: "); Serial.println(sys.isDark);

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
