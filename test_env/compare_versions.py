
import subprocess, os, re, math

def run_test(ino_path, test_env_path, mode_name, low_spec=False):
    with open(ino_path, 'r') as f: content = f.read()

    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE)
    prototypes = []
    for m in matches:
        if m.group(2) not in ["setup", "loop", "ISR"]:
            prototypes.append(f"{m.group(1)} {m.group(2)}({m.group(3)});")
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
    config.batMaxV = 14.4;
    config.batMinV = 10.0;
    config.batFloatV = 13.5;
    sys.batV = 12.0;
    sim.batteryV = 12.0;
    sys.isDark = false;
    current_charge_stage = 'B';
    ina219_present = true;

    float ocv_sweep[] = {{14.0, 15.0, 16.0, 17.0, 18.0, 20.0, 22.0, 24.0}};
    for (float ocv : ocv_sweep) {{
        sim.solarOCV = ocv; sim.solarCurrentMA = 5000.0; sim.batteryV = 12.0;
        sys.mpptPWM = 500;
        for(int i=0; i<3000; i++) {{ update_sim(); loop(); }}

        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (float)OCR1A/1023.0f;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;

        std::cout << "DATA," << "{mode_name}_" << (sim.low_spec_inductor ? "Low" : "High") << ",V_SWEEP," << ocv << "," << pin << "," << pout << std::endl;
    }}

    for (float isc = 500.0; isc <= 6000.0; isc += 1500.0) {{
        sim.solarOCV = 20.0; sim.solarCurrentMA = isc; sim.batteryV = 12.0;
        for(int i=0; i<3000; i++) {{ update_sim(); loop(); }}

        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (float)OCR1A/1023.0f;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;

        std::cout << "DATA," << "{mode_name}_" << (sim.low_spec_inductor ? "Low" : "High") << ",P_SWEEP," << isc << "," << pin << "," << pout << std::endl;
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
    width, height, margin_l, margin_r, margin_t, margin_b = 800, 500, 80, 50, 60, 80
    modes = sorted(list(set([d['Mode'] for d in data])))
    colors = {"Diode_High": "red", "Sync_High": "blue", "Diode_Low": "orange", "Sync_Low": "cyan"}
    labels = {"Diode_High": "Diode (High-Spec L)", "Sync_High": "Sync (High-Spec L)", "Diode_Low": "Diode (Low-Spec L)", "Sync_Low": "Sync (Low-Spec L)"}

    all_x = [d[x_key] for d in data if d['Pin'] > 0.05]
    if not all_x: return
    mx, mn = max(all_x), min(all_x)

    def trans(x, y):
        tx = margin_l + (x - mn) / (mx - mn + 0.001) * (width - margin_l - margin_r)
        ty = height - margin_b - (y / 100.0) * (height - margin_t - margin_b)
        return tx, ty

    svg = [f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">']
    svg.append('<rect width="100%" height="100%" fill="#fff"/>')
    svg.append(f'<text x="{width//2}" y="30" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold">{title}</text>')

    # Y-Axis labels (Efficiency %)
    for i in range(11):
        y = i * 10
        tx, ty = trans(mn, y)
        svg.append(f'<line x1="{margin_l}" y1="{ty}" x2="{width-margin_r}" y2="{ty}" stroke="#eee" stroke-width="1"/>')
        svg.append(f'<text x="{margin_l-10}" y="{ty+5}" text-anchor="end" font-family="Arial" font-size="12">{y}%</text>')

    # X-Axis labels & ticks
    num_ticks = 8
    for i in range(num_ticks):
        xv = mn + i * (mx - mn) / (num_ticks - 1)
        tx, ty = trans(xv, 0)
        svg.append(f'<line x1="{tx}" y1="{height-margin_b}" x2="{tx}" y2="{height-margin_b+5}" stroke="#333" stroke-width="2"/>')
        svg.append(f'<text x="{tx}" y="{height-margin_b+25}" text-anchor="middle" font-family="Arial" font-size="12" fill="#333">{xv:.1f}</text>')

    # Axis Lines
    svg.append(f'<line x1="{margin_l}" y1="{height-margin_b}" x2="{width-margin_r}" y2="{height-margin_b}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<line x1="{margin_l}" y1="{margin_t}" x2="{margin_l}" y2="{height-margin_b}" stroke="#333" stroke-width="2"/>')

    # X-Axis Title
    svg.append(f'<text x="{width//2}" y="{height-15}" text-anchor="middle" font-family="Arial" font-size="14" font-weight="bold">{xlabel}</text>')
    # Y-Axis Title
    svg.append(f'<text x="20" y="{height//2}" text-anchor="middle" font-family="Arial" font-size="14" font-weight="bold" transform="rotate(-90, 20, {height//2})">Efficiency (%)</text>')

    for mode in modes:
        dlist = [d for d in data if d['Mode'] == mode and d['Pin'] > 0.05]
        if not dlist: continue
        dlist.sort(key=lambda x: x[x_key])
        color = colors.get(mode, "black")
        pts = []
        for d in dlist:
            pts.append(f"{trans(d[x_key], d['Eff'])[0]},{trans(d[x_key], d['Eff'])[1]}")
        svg.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round"/>')
        for pt in pts:
            px, py = pt.split(',')
            svg.append(f'<circle cx="{px}" cy="{py}" r="4" fill="{color}"/>')

    # Legend
    for i, mode in enumerate(modes):
        ly = margin_t + 20 + (i * 25)
        color = colors.get(mode, "black")
        svg.append(f'<line x1="{width-200}" y1="{ly}" x2="{width-170}" y2="{ly}" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<text x="{width-160}" y="{ly+5}" font-family="Arial" font-size="12">{labels.get(mode, mode)}</text>')

    svg.append('</svg>')
    with open(filename, 'w') as f: f.write("\n".join(svg))

if __name__ == "__main__":
    print("Running benchmarks...")
    results = []

    for mode in ["Diode", "Sync"]:
        for spec in [False, True]:
            print(f"  Testing {mode} mode with {'Low' if spec else 'High'}-Spec Inductor...")
            out = run_test("YASL_Consolidated.ino" if mode == "Diode" else "v2_4_sync/YASL_Consolidated.ino", "test_env", mode, spec)
            results.extend([l for l in out.split('\n') if l.startswith("DATA,")])

    parsed = []
    for r in results:
        parts = r.split(',')
        if len(parts) < 6: continue
        pin, pout = float(parts[4]), float(parts[5])
        eff = (pout / pin * 100.0) if pin > 0.1 else 0
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
        f.write("Comparative analysis of Diode (v2.3.1) vs Synchronous (v2.4.0) rectification across multiple hardware grades.\n\n")
        f.write("## 1. Efficiency vs Solar Voltage\n")
        f.write("![Efficiency vs Voltage](efficiency_vs_voltage.svg)\n\n")
        f.write("## 2. Efficiency vs Input Power\n")
        f.write("![Efficiency vs Power](efficiency_vs_power.svg)\n\n")

        f.write("## 3. Hardware Comparison Summary\n\n")
        f.write("| Configuration | Inductor Spec | Peak Efficiency | 20W Efficiency |\n")
        f.write("| :--- | :--- | :---: | :---: |\n")

        modes = sorted(list(set([d['Mode'] for d in parsed])))
        for mode in modes:
            p_data = [d for d in parsed if d['Mode'] == mode and d['Sweep'] == 'P_SWEEP' and d['Pin'] > 0.1]
            p_data.sort(key=lambda x: x['Pin'])
            peak = max([d['Eff'] for d in p_data]) if p_data else 0
            eff_20w = 0
            for d in p_data:
                if d['Pin'] >= 18.0:
                    eff_20w = d['Eff']; break

            m_parts = mode.split('_')
            m_name = m_parts[0]
            l_name = m_parts[1]
            f.write(f"| {m_name} | {l_name}-Spec | {peak:.1f}% | {eff_20w:.1f}% |\n")

    print("Done. Report generated in efficiency_report.md")
