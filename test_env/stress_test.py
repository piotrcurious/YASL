import os
import subprocess
import re

def run_stress_test(ino_path):
    print(f"Running STRESS TEST for {ino_path}...")

    with open(ino_path, 'r') as f:
        content = f.read()

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
    setup();

    auto check_motion = []() {
        if (sim.motion && (EIMSK & (1 << INT0))) {
            INT0_handler();
        }
    };

    // Scenario A: High-frequency motion triggers
    std::cout << "\\n[STRESS] Rapid motion triggers (Every 2s)" << std::endl;
    sim.solarBusV = 0.5f;
    for(int i = 0; i < 50; ++i) {
        sim.motion = (i % 20 == 0); // Trigger PIR periodically
        check_motion();
        loop();
        update_sim();
    }

    // Scenario B: Rapid solar fluctuations (Cloudy day simulation)
    std::cout << "\\n[STRESS] Rapid solar fluctuations (MPPT stress)" << std::endl;
    sim.batteryV = 3.6f;
    for(int i = 0; i < 100; ++i) {
        sim.solarBusV = 10.0f + (float)(rand() % 10); // 10V to 20V random
        sim.solarCurrentMA = 100.0f + (float)(rand() % 900);
        loop();
        update_sim();
    }

    // Scenario C: Brown-out / Threshold oscillation
    std::cout << "\\n[STRESS] Battery threshold oscillation (LVD stress)" << std::endl;
    sim.solarBusV = 0.5f;
    sim.solarCurrentMA = 0.0f;
    sim.batteryV = 3.01f; // Just above min
    sim.harvestedMAH = 0;
    sim.consumedMAH = 0;
    for(int i = 0; i < 50; ++i) {
        loop();
        update_sim();
    }
    std::cout << "[SIM] Total Consumed: " << sim.consumedMAH << " mAh" << std::endl;

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
    with open('test_env/main_stress.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/main_stress.cpp", "test_env/Arduino.cpp", "-o", "test_env/stress_bin"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("Compilation FAILED")
            print(result.stderr)
            return False

        result = subprocess.run(["./test_env/stress_bin"], capture_output=True, text=True, timeout=30)
        print(result.stdout)
        return result.returncode == 0
    finally:
        # Cleanup
        if os.path.exists("test_env/stress_bin"): os.remove("test_env/stress_bin")
        if os.path.exists("test_env/main_stress.cpp"): os.remove("test_env/main_stress.cpp")

if __name__ == "__main__":
    run_stress_test("YASL_Consolidated.ino")
