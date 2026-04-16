To change the MPPT switching frequency in this code, you need to modify the **Timer1 configuration** located in the restoreHardware() function.
The current setup uses **Mode 10 (Phase Correct PWM)** with a TOP value of 1023.
### 1. The Math
The frequency (f_{pwm}) for Phase Correct PWM on an ATmega328P is calculated as:

 * **Current Settings:** f_{clk} = 16\text{ MHz}, N = 1 (no prescaler), TOP = 1023.
 * **Current Frequency:** 16,000,000 / (2 \cdot 1 \cdot 1023) \approx \mathbf{7.8\text{ kHz}}.
### 2. How to Change It
You have two primary levers to pull: the **Prescaler (N)** and the **ICR1 (TOP)**.
#### Option A: Change the Prescaler (Big Jumps)
In restoreHardware(), look for the line:
TCCR1B = _BV(WGM13) | _BV(CS10);
Changing the CS bits changes the frequency drastically:
 * _BV(CS10): Prescaler 1 (7.8\text{ kHz}) — **Current**
 * _BV(CS11): Prescaler 8 (976\text{ Hz})
 * _BV(CS11) | _BV(CS10): Prescaler 64 (122\text{ Hz})
#### Option B: Change ICR1 (Fine Tuning)
Changing ICR1 changes the resolution of your PWM. If you lower ICR1, the frequency goes up, but your "steps" of control become coarser.
To double the frequency to **15.6\text{ kHz}**, change these lines in restoreHardware():
```cpp
ICR1 = 511; // Lowering TOP increases frequency

```
**Important:** If you change ICR1, you must also update the #define MPPT_PWM_MAX_RES at the top of the code to match the new value (e.g., 511), otherwise the MPPT logic will try to write values (up to 1023) that exceed the new ceiling.
### Summary Table (at 16MHz Clock)
| Target Frequency | Prescaler (N) | ICR1 (TOP) | Resolution | TCCR1B bits |
|---|---|---|---|---|
| **7.8\text{ kHz}** | 1 | 1023 | 10-bit | CS10 (Default) |
| **15.6\text{ kHz}** | 1 | 511 | 9-bit | CS10 |
| **31.2\text{ kHz}** | 1 | 255 | 8-bit | CS10 |
| **20.0\text{ kHz}** | 1 | 400 | Custom | CS10 |
### Warning: Hardware Efficiency
MPPT converters usually perform best between **20\text{ kHz} and 50\text{ kHz}** to keep the inductor size small and reduce audible noise.
 * **Too Low (<10\text{ kHz}):** You may hear a high-pitched whine, and your inductor might saturate.
 * **Too High (>100\text{ kHz}):** The MOSFET switching losses increase, and the ATmega might struggle to run the SMC logic fast enough to keep up.
Would you like me to recalculate the register values for a specific target frequency?
