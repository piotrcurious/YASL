import os
import subprocess
import re

def run_scenario(ino_path):
    print(f"Simulating scenario for {ino_path}...")

    with open(ino_path, 'r') as f:
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

    // Scenario 1: DAY - MPPT (Bulk)
    std::cout << "\\n[SCENARIO] DAY - Bulk Charging (MPPT)" << std::endl;
    sim.solarOCV = 18.0f;
    sim.batteryV = 3.5f;
    sim.motion = false;
    for(int i = 0; i < 20; ++i) { loop(); update_sim(); }

    // Scenario 2: Absorption Stage
    std::cout << "\\n[SCENARIO] ABSORPTION - Reaching CV limit" << std::endl;
    sim.batteryV = 4.14f; // Near limit
    Serial.sim_input("SB\\n"); // Ensure we start from Bulk if needed
    for(int i = 0; i < 20; ++i) { loop(); update_sim(); }

    // Force Absorption and check tail current transition
    std::cout << "\\n[SCENARIO] TAIL CURRENT - Transition to Float" << std::endl;
    Serial.sim_input("SA\\n"); // Force Absorption
    for(int i = 0; i < 5; ++i) { loop(); update_sim(); }
    sim.solarCurrentMA = 20.0f; // Below 50mA tail current
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    // Scenario 3: NIGHT - Motion detection
    std::cout << "\\n[SCENARIO] NIGHT - Motion detection test" << std::endl;
    sim.solarOCV = 1.0f; // Dark
    sim.motion = true;
    for(int i = 0; i < 10; ++i) { check_motion(); loop(); update_sim(); }

    // Scenario 4: LVD (Low Voltage Disconnect)
    std::cout << "\\n[SCENARIO] LVD - Battery low test" << std::endl;
    sim.batteryV = 2.9f; // Below 3.0V
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    // Recovery
    std::cout << "[SIM] Battery recovery..." << std::endl;
    sim.batteryV = 4.0f; // High enough to stop sleeping
    for(int i = 0; i < 10; ++i) { loop(); update_sim(); }

    // Scenario 5: Serial Commands & persistence
    std::cout << "\\n[SCENARIO] SERIAL - Config and Diagnostics" << std::endl;
    sim.solarOCV = 18.0f; // Day
    sim.batteryV = 3.8f;
    sim.motion = false;
    update_sim(); // Propagate solarOCV to solarBusV

    for(int i = 0; i < 5; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Sending 'd' (Diagnostics)" << std::endl;
    Serial.sim_input("d\\n");
    for(int i = 0; i < 3; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Sending 'c' (Config)" << std::endl;
    Serial.sim_input("c\\n");
    for(int i = 0; i < 3; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Sending 'sM4.35' (Update MaxV)" << std::endl;
    Serial.sim_input("sM4.35\\n");
    for(int i = 0; i < 5; ++i) { loop(); update_sim(); }

    std::cout << "[SIM] Soft Reset (calling setup)" << std::endl;
    setup(); // Should reload config from mock EEPROM
    Serial.sim_input("c\\n"); // Verify config persisted
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

        result = subprocess.run(["./test_env/sim_bin"], capture_output=True, text=True, timeout=30)
        print(result.stdout)
        return result.returncode == 0
    finally:
        # Cleanup
        if os.path.exists("test_env/sim_bin"): os.remove("test_env/sim_bin")
        if os.path.exists("test_env/main_sim.cpp"): os.remove("test_env/main_sim.cpp")

if __name__ == "__main__":
    run_scenario("YASL_Consolidated.ino")
