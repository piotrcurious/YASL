import os
import subprocess
import re

def run_scenario(ino_path):
    print(f"Simulating scenario for {ino_path}...")

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

    # Custom simulation main with assertions/checks
    simulation_main = """
int main() {
    setup();

    // Helper to check for motion and trigger ISR
    auto check_motion = []() {
        if (sim.motion && (EIMSK & (1 << INT0))) {
            std::cout << "[SIM] Triggering INT0_vect (Motion)" << std::endl;
            INT0_handler();
        }
    };

    // Scenario 1: DAY (Charging)
    std::cout << "\\n[SCENARIO] DAY - Charging test" << std::endl;
    sim.solarBusV = 18.0f;
    sim.batteryV = 3.5f;
    sim.motion = false;
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    // Scenario 2: NIGHT (Motion)
    std::cout << "\\n[SCENARIO] NIGHT - Motion detection test" << std::endl;
    sim.solarBusV = 0.5f;
    sim.motion = true;
    for(int i = 0; i < 10; ++i) { check_motion(); loop(); update_sim(); }

    // Scenario 3: NIGHT (Idle -> Sleep)
    std::cout << "\\n[SCENARIO] SLEEP - Inactivity test" << std::endl;
    sim.motion = false;
    // We need to advance time past config.motionTimeout (15s) + 5s buffer
    for(int i = 0; i < 200; ++i) {
        loop();
        update_sim();
    }

    // Scenario 4: Sensor Failure
    std::cout << "\\n[SCENARIO] FAILURE - INA219 failure test" << std::endl;
    sim.ina219_ok = false;
    setup(); // Re-init to trigger failure logic
    sim.solarBusV = 15.0f;
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    // Scenario 5: Low Battery Recovery
    std::cout << "\\n[SCENARIO] RECOVERY - Low battery test" << std::endl;
    sim.ina219_ok = true;
    setup();
    sim.batteryV = 2.9f; // Below MIN
    sim.solarBusV = 0.0f;
    sim.solarCurrentMA = 0.0f;
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Starting solar charging..." << std::endl;
    sim.solarBusV = 18.0f;
    sim.solarCurrentMA = 1000.0f; // High charge current
    for(int i = 0; i < 50; ++i) { loop(); update_sim(); }

    // Scenario 6: Serial Commands & persistence
    std::cout << "\\n[SCENARIO] INTERACTIVE - Serial commands test" << std::endl;
    Serial.sim_input("dcm"); // Diagnostics, Config, Manual Motion
    for(int i = 0; i < 5; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Updating parameter (Timeout -> 60000ms)" << std::endl;
    Serial.sim_input("sT60000");
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Soft Reset (calling setup)" << std::endl;
    setup();
    Serial.sim_input("c"); // Verify config
    for(int i = 0; i < 5; ++i) { loop(); update_sim(); }

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
    with open('test_env/main_sim.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/main_sim.cpp", "test_env/Arduino.cpp", "-o", "test_env/sim_bin"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print("Compilation FAILED")
            print(result.stderr)
            return False

        result = subprocess.run(["./test_env/sim_bin"], capture_output=True, text=True, timeout=15)
        print(result.stdout)
        return result.returncode == 0
    finally:
        # Cleanup
        if os.path.exists("test_env/sim_bin"): os.remove("test_env/sim_bin")
        if os.path.exists("test_env/main_sim.cpp"): os.remove("test_env/main_sim.cpp")

if __name__ == "__main__":
    run_scenario("YASL_Consolidated.ino")
