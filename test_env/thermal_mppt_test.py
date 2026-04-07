import os
import subprocess
import re

def run_thermal_mppt_test():
    print("Testing SENSORLESS MPPT with THERMAL DRIFT...")

    with open('YASL_Consolidated.ino', 'r') as f:
        content = f.read()

    # Simple prototype extraction
    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE | re.DOTALL)
    prototypes_str = ""
    for match in matches:
        ret_type = match.group(1)
        name = match.group(2)
        params = match.group(3)
        if name in ["setup", "loop", "ISR"]: continue
        params = re.sub(r'//.*', '', params)
        params = params.replace('\n', ' ')
        prototypes_str += f"{ret_type} {name}({params});\n"

    simulation_main = """
int main() {
    sim.ina219_ok = false; // Force sensorless mode
    sim.R_conv_base = 0.2f;
    sim.tempC = 25.0f;
    sim.batteryV = 3.5f;
    sim.solarOCV = 18.0f;

    setup();

    std::cout << "\\n[SCENARIO] Sensorless MPPT start" << std::endl;
    for(int i = 0; i < 50; ++i) {
        loop();
        update_sim();
    }

    std::cout << "\\n[SCENARIO] Thermal Drift (Heating up to 75C)" << std::endl;
    sim.tempC = 75.0f;
    for(int i = 0; i < 100; ++i) {
        loop();
        update_sim();
    }

    std::cout << "\\n[SCENARIO] Triggering Calibration" << std::endl;
    Serial.sim_input("k\\n");
    for(int i = 0; i < 20; ++i) {
        loop();
        update_sim();
    }

    std::cout << "\\n[SCENARIO] Re-evaluating MPPT performance" << std::endl;
    for(int i = 0; i < 50; ++i) {
        loop();
        update_sim();
    }

    return 0;
}
"""

    wrapper = f"""
#include "Arduino.h"
#include "avr/interrupt.h"

{prototypes_str}

// Content of .ino file
{content}

{simulation_main}
"""
    with open('test_env/thermal_mppt_sim.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/thermal_mppt_sim.cpp", "test_env/Arduino.cpp", "-o", "test_env/thermal_bin"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("Compilation FAILED")
            print(result.stderr)
            return False

        result = subprocess.run(["./test_env/thermal_bin"], capture_output=True, text=True, timeout=60)
        print(result.stdout)
        return result.returncode == 0
    finally:
        # Cleanup
        if os.path.exists("test_env/thermal_bin"): os.remove("test_env/thermal_bin")
        #if os.path.exists("test_env/thermal_mppt_sim.cpp"): os.remove("test_env/thermal_mppt_sim.cpp")

if __name__ == "__main__":
    run_thermal_mppt_test()
