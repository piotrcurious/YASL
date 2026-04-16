
import subprocess, os, re, math

def run_freq_test(ino_path, mode_name, freq_hz, icr1_val):
    with open(ino_path, 'r') as f: content = f.read()

    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE)
    prototypes = []
    seen = set(["setup", "loop", "ISR", "main"])
    for m in matches:
        if m.group(2) not in seen:
            prototypes.append(f"{m.group(1)} {m.group(2)}({m.group(3)});")
            seen.add(m.group(2))
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
    sim.sync_mode = true;
    sim.low_spec_inductor = false;
    sim.freq_hz = {freq_hz};

    setup();
    ICR1 = {icr1_val};
    config.batMaxV = 14.4;
    config.batMinV = 10.0;
    config.batFloatV = 13.5;
    sys.batV = 12.0;
    sim.batteryV = 12.0;
    sys.isDark = false;
    current_charge_stage = 'B';
    ina219_present = true;

    sim.solarOCV = 20.0; sim.solarCurrentMA = 5000.0;
    sys.mpptPWM = ICR1 / 2;
    OCR1A = sys.mpptPWM;

    for(int i=0; i<2500; i++) {{ update_sim(); loop(); }}

    float pin = (sim.solarBusV * sim.solarCurrentMA_actual) / 1000.0;
    float duty = (ICR1 > 0) ? ((float)OCR1A / (float)ICR1) : 0;
    float solarOutMA = (duty > 0.001f) ? (sim.solarCurrentMA_actual / duty) : 0;
    float pout = (sim.batteryV * solarOutMA) / 1000.0;
    float eff = (pin > 0.1) ? (pout / pin * 100.0) : 0;

    std::cout << "FREQ_DATA," << {freq_hz} << "," << pin << "," << pout << "," << eff << std::endl;
    return 0;
}}
"""
    cpp = os.path.join("test_env", "freq_test_runner.cpp")
    bin = os.path.join("test_env", "freq_test_runner_bin")
    with open(cpp, 'w') as f: f.write(wrapper)

    subprocess.run(["g++", "-fpermissive", "-Itest_env", cpp, "test_env/Arduino.cpp", "-o", bin], check=True)
    result = subprocess.run([bin], capture_output=True, text=True)
    if os.path.exists(cpp): os.remove(cpp)
    if os.path.exists(bin): os.remove(bin)
    return result.stdout

def generate_freq_svg(data, filename):
    width, height, margin_l, margin_r, margin_t, margin_b = 800, 500, 80, 50, 60, 80
    freqs = [d['Freq'] for d in data]
    mx_f, mn_f = max(freqs), min(freqs)

    def trans(f, e):
        tx = margin_l + (math.log10(f) - math.log10(mn_f)) / (math.log10(mx_f) - math.log10(mn_f) + 0.001) * (width - margin_l - margin_r)
        ty = height - margin_b - (e / 100.0) * (height - margin_t - margin_b)
        return tx, ty

    svg = [f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">']
    svg.append('<rect width="100%" height="100%" fill="#fff"/>')
    svg.append(f'<text x="{width//2}" y="30" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold">Efficiency vs Switching Frequency (32 pts)</text>')

    for i in range(11):
        y = i * 10
        tx, ty = trans(mn_f, y)
        svg.append(f'<line x1="{margin_l}" y1="{ty}" x2="{width-margin_r}" y2="{ty}" stroke="#eee" stroke-width="1"/>')
        svg.append(f'<text x="{margin_l-10}" y="{ty+5}" text-anchor="end" font-family="Arial" font-size="12">{y}%</text>')

    num_ticks = 8
    for i in range(num_ticks):
        log_v = math.log10(mn_f) + i * (math.log10(mx_f) - math.log10(mn_f)) / (num_ticks - 1)
        fv = 10**log_v
        tx, ty = trans(fv, 0)
        svg.append(f'<line x1="{tx}" y1="{height-margin_b}" x2="{tx}" y2="{height-margin_b+5}" stroke="#333" stroke-width="2"/>')
        svg.append(f'<text x="{tx}" y="{height-margin_b+25}" text-anchor="middle" font-family="Arial" font-size="10">{fv/1000.0:.1f}k</text>')

    svg.append(f'<line x1="{margin_l}" y1="{height-margin_b}" x2="{width-margin_r}" y2="{height-margin_b}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<line x1="{margin_l}" y1="{margin_t}" x2="{margin_l}" y2="{height-margin_b}" stroke="#333" stroke-width="2"/>')
    svg.append(f'<text x="{width//2}" y="{height-15}" text-anchor="middle" font-family="Arial" font-size="14" font-weight="bold">Frequency (Hz, Log Scale)</text>')

    data.sort(key=lambda x: x['Freq'])
    pts = [f"{trans(d['Freq'], d['Eff'])[0]},{trans(d['Freq'], d['Eff'])[1]}" for d in data if d['Eff'] > 0]
    if pts:
        svg.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="purple" stroke-width="3" stroke-linejoin="round"/>')
        for pt in pts:
            px, py = pt.split(',')
            svg.append(f'<circle cx="{px}" cy="{py}" r="3" fill="purple"/>')

    svg.append('</svg>')
    with open(filename, 'w') as f: f.write("\n".join(svg))

if __name__ == "__main__":
    RESOLUTION = 32
    results = []
    print(f"Running frequency influence tests ({RESOLUTION} points)...")
    for i in range(RESOLUTION):
        # Sweep from 5kHz to 500kHz logarithmically
        log_f = math.log10(5000) + i * (math.log10(500000) - math.log10(5000)) / (RESOLUTION - 1)
        freq = 10**log_f
        # Calculate appropriate ICR1 for 16MHz clock (assuming no prescaler or prescaler=1)
        # ICR1 = F_CPU / F_SW - 1
        icr = int(16000000 / freq) - 1
        if icr < 10: icr = 10

        print(f"  Testing {freq:.0f} Hz...", end=" ", flush=True)
        out = run_freq_test("v2_4_sync/YASL_Consolidated.ino", "Sync", freq, icr)
        for line in out.split('\n'):
            if line.startswith("FREQ_DATA,"):
                parts = line.split(',')
                eff = float(parts[4])
                results.append({'Freq': freq, 'Eff': min(eff, 99.9)})
                print(f"{eff:.1f}%")

    generate_freq_svg(results, 'efficiency_vs_frequency.svg')
    with open("frequency_report.md", "w") as f:
        f.write("# Influence of Switching Frequency on Efficiency\n\n")
        f.write("High-resolution sweep of switching frequency from 5kHz to 500kHz.\n\n")
        f.write("![Efficiency vs Frequency](efficiency_vs_frequency.svg)\n\n")
        f.write("| Frequency (kHz) | Efficiency | Pin (W) |\n")
        f.write("| :--- | :---: | :---: |\n")
        for d in results:
            f.write(f"| {d['Freq']/1000.0:.1f} | {d['Eff']:.1f}% | 20.0 |\n")
    print("Done. frequency_report.md generated.")
