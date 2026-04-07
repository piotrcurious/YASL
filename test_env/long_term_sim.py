import os
import subprocess
import re

def run_long_term_sim(ino_path):
    print(f"Running LONG-TERM SIMULATION for {ino_path}...")

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

    # Simulation main for 10-day cycle
    simulation_main = """
int main() {
    setup();

    auto check_motion = []() {
        if (sim.motion && (EIMSK & (1 << INT0))) {
            INT0_handler();
        }
    };

    for(int day = 1; day <= 10; ++day) {
        std::cout << "\\n--- DAY " << day << " ---" << std::endl;

        // Daytime (12 hours simulated in 500 steps)
        std::cout << "[SIM] Daytime charging..." << std::endl;
        sim.solarOCV = 18.0f + (float)(rand() % 4); // variable solar quality
        sim.motion = false;
        for(int i = 0; i < 500; ++i) { loop(); update_sim(); }

        // Nighttime (12 hours simulated in 500 steps)
        std::cout << "[SIM] Nighttime operation..." << std::endl;
        sim.solarOCV = 0.5f;
        for(int i = 0; i < 500; ++i) {
            // Random motion events
            sim.motion = (rand() % 50 == 0);
            if (sim.motion) check_motion();
            loop();
            update_sim();
        }

        // End of day report
        Serial.sim_input("d");
        loop();
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
    with open('test_env/main_longterm.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/main_longterm.cpp", "test_env/Arduino.cpp", "-o", "test_env/longterm_bin"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("Compilation FAILED")
            print(result.stderr)
            return False

        result = subprocess.run(["./test_env/longterm_bin"], capture_output=True, text=True, timeout=60)
        print(result.stdout)
        return result.returncode == 0
    finally:
        # Cleanup
        if os.path.exists("test_env/longterm_bin"): os.remove("test_env/longterm_bin")
        if os.path.exists("test_env/main_longterm.cpp"): os.remove("test_env/main_longterm.cpp")

if __name__ == "__main__":
    run_long_term_sim("YASL_Consolidated.ino")
