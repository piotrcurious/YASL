
import subprocess, os, re

def inspect_run():
    ino_path = "YASL_Consolidated.ino"
    test_env_path = "test_env"
    mode_name = "Diode"
    low_spec_val = "false"

    with open(ino_path, 'r') as f: content = f.read()
    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE)
    prototypes = []
    seen = set(["setup", "loop", "ISR", "main"])
    for m in matches:
        if m.group(2) not in seen:
            prototypes.append(f"{m.group(1)} {m.group(2)}({m.group(3)});")
            seen.add(m.group(2))
    prototypes_str = "\n".join(prototypes)

    wrapper = f"""
#include <iostream>
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "Adafruit_INA219.h"
{prototypes_str}
{content}
int main() {{
    sim.ina219_ok = true;
    sim.batteryCapAH = 10.0;
    sim.sync_mode = false;
    sim.low_spec_inductor = {low_spec_val};
    setup();
    config.batMaxV = 20.0; config.batMinV = 10.0; config.batFloatV = 13.5;
    sys.batV = 12.0; sim.batteryOCV = 12.0;
    sys.isDark = false; current_charge_stage = 'B';
    ina219_present = true;

    const int RESOLUTION = 5;
    for (int i=0; i<RESOLUTION; i++) {{
        float ocv = 15.0f + (float)i * (25.0f - 15.0f) / (float)(RESOLUTION - 1);
        sim.solarOCV = ocv; sim.solarCurrentMA = 5000.0;
        sim.solarCurrentMA_actual = 0;
        if (ocv > 12.5f) sys.mpptPWM = (int)(12.5f / ocv * (float)ICR1);
        else sys.mpptPWM = 0;
        OCR1A = sys.mpptPWM;
        for(int j=0; j<1000; j++) {{ update_sim(); loop(); }}
        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (ICR1 > 0) ? ((float)OCR1A / (float)ICR1) : 0;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;
        std::cout << "DATA," << "{mode_name}_" << "High" << ",V_SWEEP," << ocv << "," << pin << "," << pout << std::endl;
    }}
    return 0;
}}
"""
    cpp = "test_env/inspect.cpp"
    bin = "test_env/inspect_bin"
    with open(cpp, 'w') as f: f.write(wrapper)
    subprocess.run(["g++", "-fpermissive", "-Itest_env", cpp, "test_env/Arduino.cpp", "-o", bin], check=True)
    res = subprocess.run([bin], capture_output=True, text=True)
    print(res.stdout)

if __name__ == "__main__":
    inspect_run()
