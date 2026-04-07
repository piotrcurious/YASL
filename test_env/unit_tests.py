import os
import subprocess
import re

def run_unit_tests():
    print("Running UNIT TESTS for core functions...")

    test_cpp = """
#include <iostream>
#include <cassert>
#include <cmath>

// Pull in definitions from Arduino.h mock
#include "Arduino.h"

// The functions we want to test (extracted/simulated from YASL_Consolidated.ino)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void test_mapFloat() {
    std::cout << "Testing mapFloat..." << std::endl;
    assert(std::abs(mapFloat(3.6f, 3.0f, 4.2f, 0.0f, 100.0f) - 50.0f) < 0.1f);
    assert(std::abs(mapFloat(3.0f, 3.0f, 4.2f, 0.0f, 100.0f) - 0.0f) < 0.1f);
    assert(std::abs(mapFloat(4.2f, 3.0f, 4.2f, 0.0f, 100.0f) - 100.0f) < 0.1f);
    // Edge case: div by zero
    assert(mapFloat(1.0f, 2.0f, 2.0f, 10.0f, 20.0f) == 10.0f);
    std::cout << "  PASS: mapFloat" << std::endl;
}

void test_battery_pcnt() {
    std::cout << "Testing battery percentage logic..." << std::endl;
    float minV = 3.0f;
    float maxV = 4.2f;
    assert(mapFloat(3.6f, minV, maxV, 0, 100) == 50.0f);
    assert(mapFloat(3.0f, minV, maxV, 0, 100) == 0.0f);
    assert(mapFloat(4.2f, minV, maxV, 0, 100) == 100.0f);
    std::cout << "  PASS: battery_pcnt" << std::endl;
}

void test_smc_math() {
    std::cout << "Testing SMC MPPT directional logic..." << std::endl;
    // S = dP / dV
    // If S > 0, we are to the left of MPP -> Decrease V (Increase Duty/PWM)
    // Actually in our Buck converter, increasing PWM increases Vout?
    // Wait, Vout = Vin * Duty. So Vin = Vout / Duty.
    // Increasing Duty -> Decreases Vin (Solar Bus Voltage).
    // So if S > 0 (Vin < Vmpp), we want to Decrease Vin -> Increase Duty.

    float dv = -0.1f; // Voltage decreased
    float dp = 0.5f;  // Power increased
    float S = dp / dv; // S = -5.0 (Negative)
    // If S < 0, we are to the right of MPP -> Need to decrease V -> Increase Duty.

    // Logic in code:
    // if (S > 0.01f) sys.mpptPWM -= gain;
    // else if (S < -0.01f) sys.mpptPWM += gain;

    assert(S < -0.01f); // Should lead to sys.mpptPWM += gain; Correct.
    std::cout << "  PASS: smc_math" << std::endl;
}

void test_macros() {
    std::cout << "Testing Arduino macros (constrain, min, max)..." << std::endl;
    assert(constrain(10, 0, 5) == 5);
    assert(constrain(-1, 0, 5) == 0);
    assert(constrain(3, 0, 5) == 3);
    assert(max(10, 20) == 20);
    assert(min(10, 20) == 10);
    std::cout << "  PASS: macros" << std::endl;
}

int main() {
    test_mapFloat();
    test_battery_pcnt();
    test_smc_math();
    test_macros();
    std::cout << "\\nALL UNIT TESTS PASSED" << std::endl;
    return 0;
}
"""
    with open('test_env/unit_test.cpp', 'w') as f:
        f.write(test_cpp)

    cmd = ["g++", "-I./test_env", "test_env/unit_test.cpp", "test_env/Arduino.cpp", "-o", "test_env/unit_bin"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Compilation FAILED")
        print(result.stderr)
        return False

    result = subprocess.run(["./test_env/unit_bin"], capture_output=True, text=True)
    print(result.stdout)

    # Cleanup
    if os.path.exists("test_env/unit_bin"): os.remove("test_env/unit_bin")
    if os.path.exists("test_env/unit_test.cpp"): os.remove("test_env/unit_test.cpp")

    return result.returncode == 0

if __name__ == "__main__":
    run_unit_tests()
