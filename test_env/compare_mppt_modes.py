import os
import subprocess
import re

def run_comparison():
    print("Comparing MPPT Modes: SENSED vs INFERRED...")

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
#include <vector>
#include <numeric>

void run_case(bool use_ina, const char* label) {
    sim.ina219_ok = use_ina;
    sim.tempC = 25.0f;
    sim.batteryV = 3.2f; // Low bat to stay in Bulk/MPPT
    sim.solarOCV = 18.0f;

    setup();
    std::cout << "\\n--- START CASE: " << label << " ---" << std::endl;

    std::vector<float> powers;
    float total_harvest = 0;

    for(int i = 0; i < 150; ++i) {
        loop();
        update_sim();
        if (i > 50) { // Let it stabilize
            powers.push_back(sim.solarCurrentMA * sim.solarBusV / 1000.0f);
        }
    }

    float avg_p = std::accumulate(powers.begin(), powers.end(), 0.0f) / powers.size();
    float max_p = *std::max_element(powers.begin(), powers.end());
    float min_p = *std::min_element(powers.begin(), powers.end());

    std::cout << "[RESULT] " << label << " Avg Power: " << avg_p << "W" << std::endl;
    std::cout << "[RESULT] " << label << " Ripple: " << (max_p - min_p) << "W" << std::endl;
    std::cout << "[RESULT] " << label << " Final Vsolar: " << sim.solarBusV << "V" << std::endl;
    std::cout << "[RESULT] " << label << " Final Duty: " << (float)OCR1A/1023.0f << std::endl;
}

int main() {
    run_case(true, "INA219 SENSED");
    run_case(false, "MODEL INFERRED");
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
    with open('test_env/compare_sim.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/compare_sim.cpp", "test_env/Arduino.cpp", "-o", "test_env/compare_bin"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("Compilation FAILED")
            print(result.stderr)
            return False

        result = subprocess.run(["./test_env/compare_bin"], capture_output=True, text=True, timeout=60)
        print(result.stdout)
        return result.returncode == 0
    finally:
        # Cleanup
        if os.path.exists("test_env/compare_bin"): os.remove("test_env/compare_bin")
        if os.path.exists("test_env/compare_sim.cpp"): os.remove("test_env/compare_sim.cpp")

if __name__ == "__main__":
    run_comparison()
