import subprocess
import os
import re

def setup_arduino_cpp(path, sync_mode):
    with open(path, 'r') as f:
        content = f.read()
    sync_val = "true" if sync_mode else "false"
    physics_code = """
void update_sim() {
    float duty = (float)OCR1A / 1023.0f;
    float R_conv = 0.05f;
    float R_sync = 0.02f;
    float V_diode = 0.5f;
    float Vbat = 3.5f;
    float Isc = sim.solarOCV * 250.0f;
    if (duty < 0.01f) {
        sim.solarBusV = sim.solarOCV;
        sim.solarCurrentMA = 0;
    } else {
        float Iout_est = 2000.0f;
        for(int i=0; i<15; i++) {
            float V_drop_idle = sim.sync_mode ? ((Iout_est/1000.0f) * R_sync * (1.0f - duty)) : (V_diode * (1.0f - duty));
            float V_drop_active = (Iout_est/1000.0f) * R_conv * duty;
            sim.solarBusV = (Vbat + V_drop_idle + V_drop_active) / duty;
            if (sim.solarBusV > sim.solarOCV) sim.solarBusV = sim.solarOCV;
            sim.solarCurrentMA = Isc * (1.0f - exp(sim.solarBusV - sim.solarOCV));
            if (sim.solarCurrentMA < 0) sim.solarCurrentMA = 0;
            Iout_est = sim.solarCurrentMA / duty;
        }
    }
    sim.batteryV = Vbat;
    sim.solarShuntV = sim.solarCurrentMA * 0.01;
    current_time_ms += 100;
}
"""
    content = re.sub(r'SimSensors sim = \{ .* \};', f'SimSensors sim = {{ 12.0f, 18.0f, 0.0f, 100.0f, 3.5f, 1000.0f, 10.0f, 5.0f, 0.0, 0.0, 25.0f, 0.05f, {sync_val}, false, true }};', content)
    content = re.sub(r'void update_sim\(\) \{.*?\}\n\nvoid delay', physics_code + "\nvoid delay", content, flags=re.DOTALL)
    with open(path, 'w') as f: f.write(content)

def run_test(ino_path, test_env_path, mode_name, sync_mode):
    setup_arduino_cpp(os.path.join(test_env_path, "Arduino.cpp"), sync_mode)
    with open(ino_path, 'r') as f: content = f.read()
    content = content.replace("Serial.print", "//Serial.print")
    wrapper = f"""
#include <iostream>
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
{content}
int main() {{
    setup();
    ina219_present = true;
    sys.isDark = false;
    current_charge_stage = 'B';
    float ocv_sweep[] = {{10.0, 14.0, 18.0, 22.0, 26.0, 30.0}};
    for (float ocv : ocv_sweep) {{
        sim.solarOCV = ocv;
        for(int i=0; i<3000; i++) {{ update_sim(); loop(); }}
        float input_w = (sim.solarBusV * sim.solarCurrentMA) / 1000.0;
        float duty = (float)OCR1A / 1023.0f;
        float bat_i = (duty > 0.01) ? (sim.solarCurrentMA / duty) : 0;
        float harvest_w = (bat_i * sim.batteryV) / 1000.0;
        std::cout << "DATA," << "{mode_name}," << input_w << "," << harvest_w << std::endl;
    }}
    return 0;
}}
"""
    cpp_path = os.path.join(test_env_path, 'comp_test.cpp')
    bin_path = os.path.join(test_env_path, 'comp_test_bin')
    with open(cpp_path, 'w') as f: f.write(wrapper)
    subprocess.run(["g++", "-fpermissive", f"-I{test_env_path}", cpp_path, os.path.join(test_env_path, "Arduino.cpp"), "-o", bin_path], check=True)
    result = subprocess.run([bin_path], capture_output=True, text=True)
    return result.stdout

if __name__ == "__main__":
    results = []
    # Both versions will use the root test_env for compilation
    # But we point to the different .ino files
    results.extend([l for l in run_test("YASL_Consolidated.ino", "test_env", "Diode", False).split('\n') if l.startswith("DATA,")])
    results.extend([l for l in run_test("v2_4_sync/YASL_Consolidated.ino", "test_env", "Sync", True).split('\n') if l.startswith("DATA,")])
    with open("efficiency_results.csv", 'w') as f:
        f.write("Mode,InputPower,HarvestedPower\n")
        for r in results: f.write(r[5:] + "\n")
    print("Done.")
