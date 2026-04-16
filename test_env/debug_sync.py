
import subprocess, os, re

def debug_sync():
    ino_path = "v2_4_sync/YASL_Consolidated.ino"
    test_env_path = "test_env"

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
#include <iomanip>
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "Adafruit_INA219.h"
{prototypes_str}
{content}
int main() {{
    sim.ina219_ok = true;
    sim.sync_mode = true;
    sim.low_spec_inductor = false;
    sim.batteryOCV = 12.0;
    setup();
    config.batMaxV = 20.0; config.batMinV = 10.0; config.batFloatV = 13.5;
    sys.batV = 12.0;
    sys.isDark = false;
    current_charge_stage = 'B';

    sim.solarOCV = 20.0; sim.solarCurrentMA = 5000.0;

    std::cout << "Starting Sync MPPT debug..." << std::endl;
    std::cout << "Iter | PWM | Vsol | Isoli | isDark | Mode | Power" << std::endl;

    for(int j=0; j<2000; j++) {{
        update_sim();
        loop();
        if (j % 100 == 0) {{
            float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
            std::cout << std::setw(4) << j << " | "
                      << std::setw(3) << OCR1A << " | "
                      << std::fixed << std::setprecision(2) << sim.solarBusV << " | "
                      << std::setw(5) << (int)sim.solarCurrentMA_actual << " | "
                      << (sys.isDark ? "Y" : "N") << " | "
                      << sys.chargeMode << " | "
                      << pin << std::endl;
        }}
    }}
    return 0;
}}
"""
    cpp = "test_env/debug_sync.cpp"
    bin = "test_env/debug_sync_bin"
    with open(cpp, 'w') as f: f.write(wrapper)
    subprocess.run(["g++", "-fpermissive", "-Itest_env", cpp, "test_env/Arduino.cpp", "-o", bin], check=True)
    subprocess.run([bin])

if __name__ == "__main__":
    debug_sync()
