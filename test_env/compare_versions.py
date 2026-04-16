
import subprocess, os, re, math

def run_test(ino_path, test_env_path, mode_name, low_spec=False):
    with open(ino_path, 'r') as f: content = f.read()
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

void sync_sys_vars() {{
    sys.batV = sim.batteryV;
    sys.solarV = sim.solarBusV;
    sys.isDark = (sim.solarOCV < 5.0f);
}}

int main() {{
    sim.ina219_ok = true;
    sim.batteryCapAH = 10.0;
    sim.sync_mode = ("{mode_name}" == "Sync");
    sim.low_spec_inductor = {low_spec_val};
    sim.batteryOCV = 12.0;

    setup();
    config.batMaxV = 25.0; config.batMinV = 10.0; config.batFloatV = 13.5;
    sys.batV = 12.0;
    sys.isDark = false; current_charge_stage = 'B';
    ina219_present = true;

    const int RESOLUTION = 32;

    // V_SWEEP
    for (int i=0; i<RESOLUTION; i++) {{
        float ocv = (float)i * 25.0f / (float)(RESOLUTION - 1);
        sim.solarOCV = ocv; sim.solarCurrentMA = 5000.0;
        sys.solarV = ocv; sys.batV = 12.0;

        if (ocv > 12.5f) {{
            sys.mpptPWM = (int)(13.5f / ocv * (float)ICR1);
            OCR1A = sys.mpptPWM;
        }} else {{
            sys.mpptPWM = 0;
            OCR1A = 0;
        }}

        for(int j=0; j<1500; j++) {{
            update_sim();
            sync_sys_vars();
            loop();
        }}

        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (ICR1 > 0) ? ((float)OCR1A / (float)ICR1) : 0;
        float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
        float pout = (sim.batteryV * solarOutMA) / 1000.0;
        std::cout << "DATA," << "{mode_name}_" << (sim.low_spec_inductor ? "Low" : "High") << ",V_SWEEP," << ocv << "," << pin << "," << pout << std::endl;
    }}

    // P_SWEEP
    for (int i=0; i<RESOLUTION; i++) {{
        float isc = 100.0f + (float)i * (8000.0f - 100.0f) / (float)(RESOLUTION - 1);
        sim.solarOCV = 20.0; sim.solarCurrentMA = isc;
        sys.solarV = 20.0; sys.batV = 12.0;
        sys.mpptPWM = (int)(13.5f / 20.0f * (float)ICR1);
        OCR1A = sys.mpptPWM;

        for(int j=0; j<1500; j++) {{
            update_sim();
            sync_sys_vars();
            loop();
        }}

        float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
        float duty = (ICR1 > 0) ? ((float)OCR1A / (float)ICR1) : 0;
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
    subprocess.run(["g++", "-fpermissive", f"-I{test_env_path}", cpp, os.path.join(test_env_path, "Arduino.cpp"), "-o", bin], check=True)
    result = subprocess.run([bin], capture_output=True, text=True)
    if os.path.exists(cpp): os.remove(cpp)
    if os.path.exists(bin): os.remove(bin)
    return result.stdout

def generate_svg(data, filename, title, xlabel, x_key='X'):
    width, height, margin_l, margin_r, margin_t, margin_b = 800, 500, 80, 50, 60, 100
    modes = sorted(list(set([d['Mode'] for d in data])))
    colors = {"Diode_High": "red", "Sync_High": "blue", "Diode_Low": "orange", "Sync_Low": "cyan"}
    labels = {"Diode_High": "Diode (High-Spec L)", "Sync_High": "Sync (High-Spec L)", "Diode_Low": "Diode (Low-Spec L)", "Sync_Low": "Sync (Low-Spec L)"}

    all_x = [d[x_key] for d in data]
    if not all_x: return
    mx, mn = max(all_x), min(all_x)

    def trans(x, y):
        tx = margin_l + (x - mn) / (mx - mn + 0.001) * (width - margin_l - margin_r)
        ty = height - margin_b - (y / 100.0) * (height - margin_t - margin_b)
        return tx, ty

    svg = [f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">']
    svg.append('<rect width="100%" height="100%" fill="#fff"/>')
    svg.append(f'<text x="{width//2}" y="30" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold">{title}</text>')

    for i in range(11):
        y = i * 10
        tx, ty = trans(mn, y)
        svg.append(f'<line x1="{margin_l}" y1="{ty}" x2="{width-margin_r}" y2="{ty}" stroke="#eee" stroke-width="1"/>')
        svg.append(f'<text x="{margin_l-10}" y="{ty+5}" text-anchor="end" font-family="Arial" font-size="12">{y}%</text>')

    num_ticks = 10
    for i in range(num_ticks):
        xv = mn + i * (mx - mn) / (num_ticks - 1)
        tx, ty = trans(xv, 0)
        svg.append(f'<line x1="{tx}" y1="{height-margin_b}" x2="{tx}" y2="{height-margin_b+5}" stroke="#333" stroke-width="2"/>')
        svg.append(f'<text x="{tx}" y="{height-margin_b+25}" text-anchor="middle" font-family="Arial" font-size="12" fill="#333">{xv:.1f}</text>')

    svg.append(f'<line x1="{margin_l}" y1="{height-margin_b}" x2="{width-margin_r}" y2="{height-margin_b}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<line x1="{margin_l}" y1="{margin_t}" x2="{margin_l}" y2="{height-margin_b}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<text x="{width//2}" y="{height-20}" text-anchor="middle" font-family="Arial" font-size="14" font-weight="bold">{xlabel}</text>')

    for mode in modes:
        dlist = [d for d in data if d['Mode'] == mode]
        if not dlist: continue
        dlist.sort(key=lambda x: x[x_key])
        color = colors.get(mode, "black")

        plot_pts = [d for d in dlist if d['Eff'] > 5.0]
        if not plot_pts: continue

        pts_str = " ".join([f"{trans(d[x_key], d['Eff'])[0]},{trans(d[x_key], d['Eff'])[1]}" for d in plot_pts])
        svg.append(f'<polyline points="{pts_str}" fill="none" stroke="{color}" stroke-width="3" stroke-linejoin="round"/>')
        for d in plot_pts:
            px, py = trans(d[x_key], d['Eff'])
            svg.append(f'<circle cx="{px}" cy="{py}" r="3" fill="{color}"/>')

    for i, mode in enumerate(modes):
        ly = margin_t + 20 + (i * 25)
        color = colors.get(mode, "black")
        label = labels.get(mode, mode)
        svg.append(f'<line x1="{width-220}" y1="{ly}" x2="{width-190}" y2="{ly}" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<text x="{width-180}" y="{ly+5}" font-family="Arial" font-size="12">{label}</text>')

    svg.append('</svg>')
    with open(filename, 'w') as f: f.write("\n".join(svg))

if __name__ == "__main__":
    results = []
    for mode in ["Diode", "Sync"]:
        for spec in [False, True]:
            print(f"Testing {mode} mode with {'Low' if spec else 'High'}-Spec Inductor...")
            out = run_test("YASL_Consolidated.ino" if mode == "Diode" else "v2_4_sync/YASL_Consolidated.ino", "test_env", mode, spec)
            lines = [l for l in out.split('\n') if l.startswith("DATA,")]
            results.extend(lines)

    parsed = []
    for r in results:
        parts = [p.strip() for p in r.split(',')]
        if len(parts) < 6: continue
        try:
            m_name = parts[1]
            swp = parts[2]
            xv = float(parts[3])
            pn = float(parts[4])
            po = float(parts[5])
            ef = (po / pn * 100.0) if pn > 1.0 else 0.0
            if ef > 100.1: ef = 0.0
            parsed.append({'Mode': m_name, 'Sweep': swp, 'X': xv, 'Pin': pn, 'Eff': min(ef, 99.9)})
        except: continue

    generate_svg([d for d in parsed if d['Sweep'] == 'V_SWEEP'], 'efficiency_vs_voltage.svg', 'Efficiency vs Solar Voltage (High Res)', 'Solar OCV (V)')
    generate_svg([d for d in parsed if d['Sweep'] == 'P_SWEEP'], 'efficiency_vs_power.svg', 'Efficiency vs Input Power (High Res)', 'Input Power (W)', x_key='Pin')

    with open("efficiency_report.md", "w") as f:
        f.write("# MPPT Efficiency Benchmarking Report (High Res)\n\n")
        f.write("Analysis of Synchronous vs Asynchronous (Diode) rectification. Solar sweep from 0V to 25V.\n\n")
        f.write("## 1. Efficiency vs Solar Voltage\n![Efficiency vs Voltage](efficiency_vs_voltage.svg)\n\n")
        f.write("## 2. Efficiency vs Input Power\n![Efficiency vs Power](efficiency_vs_power.svg)\n\n")
        f.write("## 3. Hardware Comparison Summary\n\n| Configuration | Inductor Spec | Peak Efficiency | 40W Efficiency |\n| :--- | :--- | :---: | :---: |\n")

        modes = sorted(list(set([d['Mode'] for d in parsed])))
        for mode in modes:
            m_data = [d for d in parsed if d['Mode'] == mode and d['Pin'] > 5.0]
            peak = max([d['Eff'] for d in m_data]) if m_data else 0
            eff_40w = 0
            p_data = [d for d in m_data if d['Sweep'] == 'P_SWEEP']
            p_data.sort(key=lambda x: x['Pin'])
            for d in p_data:
                if d['Pin'] >= 38.0:
                    eff_40w = d['Eff']
                    break
            m_parts = mode.split('_')
            f.write(f"| {m_parts[0]} | {m_parts[1]} | {peak:.1f}% | {eff_40w:.1f}% |\n")
    print("Done.")
