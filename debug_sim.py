
import subprocess, os, re

def run_test(ino_path, test_env_path, mode_name, low_spec=False):
    with open(ino_path, 'r') as f: content = f.read()
    # Remove any existing main if it exists (it shouldn't in .ino but just in case)
    content = re.sub(r'int\s+main\s*\(.*?\)\s*\{.*?\}', '', content, flags=re.DOTALL)

    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE)
    prototypes = []
    seen = set(["setup", "loop", "ISR", "main"])
    for m in matches:
        if m.group(2) not in seen:
            prototypes.append(f"{m.group(1)} {m.group(2)}({m.group(3)});")
            seen.add(m.group(2))
    prototypes_str = "\n".join(prototypes)
    low_spec_val = "true" if low_spec else "false"

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
    sim.R_conv_base = 0.2;
    sim.tempC = 25.0;
    sim.sync_mode = ("{mode_name}" == "Sync");
    sim.low_spec_inductor = {low_spec_val};
    setup();
    config.batMaxV = 14.4; config.batMinV = 10.0; config.batFloatV = 13.5;
    sys.batV = 12.0; sim.batteryOCV = 12.0;
    sys.isDark = false; current_charge_stage = 'B';
    ina219_present = true;
    float ocv_sweep[] = {{14.0, 15.0, 16.0, 17.0, 18.0, 20.0, 22.0, 24.0}};
    for (float ocv : ocv_sweep) {{
        sim.solarOCV = ocv; sim.solarCurrentMA = 5000.0;
        sys.mpptPWM = 500;
        for(int i=0; i<3000; i++) {{ update_sim(); loop(); }}
        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (ICR1 > 0) ? ((float)OCR1A / (float)ICR1) : 0;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;
        std::cout << "DATA," << "{mode_name}_" << (sim.low_spec_inductor ? "Low" : "High") << ",V_SWEEP," << ocv << "," << pin << "," << pout << std::endl;
    }}
    return 0;
}}
"""
    cpp = os.path.join("test_env", "debug_runner.cpp")
    bin = os.path.join("test_env", "debug_runner_bin")
    with open(cpp, 'w') as f: f.write(wrapper)
    res = subprocess.run(["g++", "-fpermissive", f"-I{test_env_path}", cpp, os.path.join(test_env_path, "Arduino.cpp"), "-o", bin], capture_output=True, text=True)
    if res.returncode != 0:
        print("Compile Error:")
        print(res.stderr)
        return
    result = subprocess.run([bin], capture_output=True, text=True)
    print("Execution Output:")
    print(result.stdout)
    print("Execution Error:")
    print(result.stderr)

run_test("YASL_Consolidated.ino", "test_env", "Diode", False)
