
import subprocess, os, re, math

def run_test(ino_path, test_env_path, mode_name):
    with open(ino_path, 'r') as f: content = f.read()

    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE)
    prototypes = []
    for m in matches:
        if m.group(2) not in ["setup", "loop", "ISR"]:
            prototypes.append(f"{m.group(1)} {m.group(2)}({m.group(3)});")
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
    sim.R_conv_base = 0.2;
    sim.tempC = 25.0;
    sim.sync_mode = ("{mode_name}" == "Sync");

    setup();
    config.batMaxV = 14.4;
    config.batMinV = 10.0;
    config.batFloatV = 13.5;
    sys.batV = 12.0;
    sim.batteryV = 12.0;
    sys.isDark = false;
    current_charge_stage = 'B';
    ina219_present = true;

    float ocv_sweep[] = {{13.0, 14.0, 15.0, 17.0, 20.0, 24.0}};
    for (float ocv : ocv_sweep) {{
        sim.solarOCV = ocv; sim.solarCurrentMA = 5000.0; sim.batteryV = 12.0;
        sys.mpptPWM = 500;
        for(int i=0; i<3000; i++) {{ update_sim(); loop(); }}

        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (float)OCR1A/1023.0f;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;

        std::cerr << "DATA," << "{mode_name},V_SWEEP," << ocv << "," << pin << "," << pout << "," << (float)OCR1A << "," << (float)OCR1B << std::endl;
        std::cout << "DATA," << "{mode_name},V_SWEEP," << ocv << "," << pin << "," << pout << "," << (float)OCR1A << "," << (float)OCR1B << std::endl;
    }}

    for (float isc = 500.0; isc <= 6000.0; isc += 1000.0) {{
        sim.solarOCV = 20.0; sim.solarCurrentMA = isc; sim.batteryV = 12.0;
        for(int i=0; i<3000; i++) {{ update_sim(); loop(); }}

        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (float)OCR1A/1023.0f;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;

        std::cout << "DATA," << "{mode_name},P_SWEEP," << isc << "," << pin << "," << pout << "," << (float)OCR1A << "," << (float)OCR1B << std::endl;
    }}
    return 0;
}}
"""
    cpp = os.path.join("test_env", "test_runner.cpp")
    bin = os.path.join("test_env", "test_runner_bin")
    with open(cpp, 'w') as f: f.write(wrapper)

    cmd = ["g++", "-fpermissive", f"-I{test_env_path}", cpp, os.path.join(test_env_path, "Arduino.cpp"), "-o", bin]
    subprocess.run(cmd, check=True)
    result = subprocess.run([bin], capture_output=True, text=True)

    if os.path.exists(cpp): os.remove(cpp)
    if os.path.exists(bin): os.remove(bin)
    return result.stdout

def generate_svg(data, filename, title, xlabel, x_key='X'):
    width, height, margin = 800, 500, 60
    diode = [d for d in data if d['Mode'] == 'Diode' and d['Pin'] > 0.1]
    sync = [d for d in data if d['Mode'] == 'Sync' and d['Pin'] > 0.1]

    all_x = [d[x_key] for d in diode + sync]
    if not all_x: return
    mx, mn = max(all_x), min(all_x)

    def trans(x, y):
        tx = margin + (x - mn) / (mx - mn + 0.001) * (width - 2 * margin)
        ty = height - margin - (y / 100.0) * (height - 2 * margin)
        return tx, ty

    svg = [f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">']
    svg.append('<rect width="100%" height="100%" fill="#fff"/>')
    svg.append(f'<text x="{width//2}" y="30" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold">{title}</text>')

    for i in range(11):
        y = i * 10
        _, ty = trans(mn, y)
        svg.append(f'<line x1="{margin}" y1="{ty}" x2="{width-margin}" y2="{ty}" stroke="#eee" stroke-width="1"/>')
        svg.append(f'<text x="{margin-10}" y="{ty+5}" text-anchor="end" font-family="Arial" font-size="12">{y}%</text>')

    svg.append(f'<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<text x="{width//2}" y="{height-10}" text-anchor="middle" font-family="Arial" font-size="14">{xlabel}</text>')

    for label, dlist, color in [("Diode Rectification", diode, "red"), ("Synchronous Rectification", sync, "blue")]:
        if not dlist: continue
        dlist.sort(key=lambda x: x[x_key])
        pts = []
        for d in dlist:
            pts.append(f"{trans(d[x_key], d['Eff'])[0]},{trans(d[x_key], d['Eff'])[1]}")
        svg.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round"/>')

        ly = margin + 20 + (0 if color == "red" else 30)
        svg.append(f'<line x1="{width-200}" y1="{ly}" x2="{width-170}" y2="{ly}" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<text x="{width-160}" y="{ly+5}" font-family="Arial" font-size="12">{label}</text>')

    svg.append('</svg>')
    with open(filename, 'w') as f: f.write("\n".join(svg))

if __name__ == "__main__":
    print("Running benchmarks...")
    results = []

    out_diode = run_test("YASL_Consolidated.ino", "test_env", "Diode")
    results.extend([l for l in out_diode.split('\n') if l.startswith("DATA,")])

    out_sync = run_test("v2_4_sync/YASL_Consolidated.ino", "test_env", "Sync")
    results.extend([l for l in out_sync.split('\n') if l.startswith("DATA,")])

    parsed = []
    for r in results:
        parts = r.split(',')
        if len(parts) < 7: continue
        pin, pout = float(parts[4]), float(parts[5])
        eff = (pout / pin * 100.0) if pin > 0.05 else 0
        parsed.append({
            'Mode': parts[1],
            'Sweep': parts[2],
            'X': float(parts[3]),
            'Pin': pin,
            'Eff': min(eff, 99.5)
        })

    generate_svg([d for d in parsed if d['Sweep'] == 'V_SWEEP'], 'efficiency_vs_voltage.svg', 'Efficiency vs Solar Voltage', 'Solar OCV (V)')
    generate_svg([d for d in parsed if d['Sweep'] == 'P_SWEEP'], 'efficiency_vs_power.svg', 'Efficiency vs Input Power', 'Input Power (W)', x_key='Pin')

    with open("efficiency_report.md", "w") as f:
        f.write("# MPPT Efficiency Benchmarking Report\n\n")
        f.write("Comparing Diode Rectification (v2.3.1) vs Synchronous Rectification (v2.4.0).\n\n")
        f.write("## 1. Efficiency vs Solar Voltage\n")
        f.write("![Efficiency vs Voltage](efficiency_vs_voltage.svg)\n\n")
        f.write("## 2. Efficiency vs Input Power\n")
        f.write("![Efficiency vs Power](efficiency_vs_power.svg)\n\n")

        f.write("## 3. Comparative Analysis\n\n")
        f.write("| Configuration | Peak Efficiency | Startup OCV | 5W Efficiency | 20W Efficiency |\n")
        f.write("| :--- | :---: | :---: | :---: | :---: |\n")

        for mode in ["Diode", "Sync"]:
            v_data = [d for d in parsed if d['Mode'] == mode and d['Sweep'] == 'V_SWEEP' and d['Eff'] > 5]
            p_data = [d for d in parsed if d['Mode'] == mode and d['Sweep'] == 'P_SWEEP' and d['Pin'] > 0.1]
            p_data.sort(key=lambda x: x['Pin'])

            peak = max([d['Eff'] for d in p_data]) if p_data else 0
            start = min([d['X'] for d in v_data]) if v_data else 0

            eff_5w = 0
            for d in p_data:
                if d['Pin'] >= 5.0:
                    eff_5w = d['Eff']; break

            eff_20w = 0
            for d in p_data:
                if d['Pin'] >= 20.0:
                    eff_20w = d['Eff']; break

            f.write(f"| {mode} | {peak:.1f}% | {start:.1f}V | {eff_5w:.1f}% | {eff_20w:.1f}% |\n")

    print("Done. Report generated in efficiency_report.md")
