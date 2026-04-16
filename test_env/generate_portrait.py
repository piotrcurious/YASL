
import math

def generate_phase_portrait():
    width, height, margin = 800, 500, 80
    svg = [f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">']
    svg.append('<rect width="100%" height="100%" fill="#fff"/>')

    # Axes
    cx, cy = width // 2, height // 2
    svg.append(f'<line x1="{margin}" y1="{cy}" x2="{width-margin}" y2="{cy}" stroke="#333" stroke-width="2"/>') # V axis
    svg.append(f'<line x1="{cx}" y1="{margin}" x2="{cx}" y2="{height-margin}" stroke="#333" stroke-width="2"/>') # S axis

    # Labels
    svg.append(f'<text x="{width-margin}" y="{cy+25}" text-anchor="end" font-family="Arial" font-size="14" font-weight="bold">Voltage (V)</text>')
    svg.append(f'<text x="{cx+10}" y="{margin+10}" text-anchor="start" font-family="Arial" font-size="14" font-weight="bold">Sliding Surface S = dP/dV</text>')

    # Power Curve Sketch (Conceptual)
    pts = []
    for i in range(-200, 201):
        x = i * 1.5
        # dP/dV = -k * (V - Vmpp)
        y = -0.8 * x
        pts.append(f"{cx + x},{cy + y}")

    # The Sliding Surface Line (S=0) is the X-axis.
    # But we want to show the trajectory.

    # Trajectory 1: Start from Left (Low V)
    t1 = []
    for i in range(150, 0, -10):
        vx = -i * 2
        sy = -0.5 * vx + 20 * math.sin(i/10.0)
        t1.append(f"{cx+vx},{cy-sy}")

    svg.append(f'<polyline points="{" ".join(t1)}" fill="none" stroke="blue" stroke-width="2" stroke-dasharray="5,5"/>')
    # Arrow for T1
    svg.append(f'<polygon points="{cx-20},{cy-5} {cx},{cy} {cx-20},{cy+5}" fill="blue"/>')
    svg.append(f'<text x="{cx-150}" y="{cy-80}" fill="blue" font-family="Arial" font-size="12">Startup Trajectory</text>')

    # Trajectory 2: Oscillating around MPP
    t2 = []
    for i in range(0, 100):
        angle = i * 0.5
        r = 30 * math.exp(-0.05 * i)
        vx = r * math.cos(angle)
        sy = r * math.sin(angle)
        t2.append(f"{cx+vx},{cy-sy}")
    svg.append(f'<polyline points="{" ".join(t2)}" fill="red" stroke-width="2" fill-opacity="0"/>')
    svg.append(f'<circle cx="{cx}" cy="{cy}" r="5" fill="green"/>')
    svg.append(f'<text x="{cx+10}" y="{cy-10}" fill="green" font-family="Arial" font-size="12" font-weight="bold">MPP (S=0)</text>')

    # Regions
    svg.append(f'<text x="{margin+50}" y="{cy-50}" fill="#888" font-family="Arial" font-size="12">Region: Increase Duty</text>')
    svg.append(f'<text x="{width-margin-150}" y="{cy+50}" fill="#888" font-family="Arial" font-size="12">Region: Decrease Duty</text>')

    svg.append('</svg>')
    with open('smc_phase_portrait.svg', 'w') as f: f.write("\n".join(svg))

if __name__ == "__main__":
    generate_phase_portrait()
