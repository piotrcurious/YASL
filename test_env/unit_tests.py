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
    assert(std::abs(mapFloat(3.6, 3.0, 4.2, 0.0, 100.0) - 50.0) < 0.1);
    assert(std::abs(mapFloat(3.0, 3.0, 4.2, 0.0, 100.0) - 0.0) < 0.1);
    assert(std::abs(mapFloat(4.2, 3.0, 4.2, 0.0, 100.0) - 100.0) < 0.1);
    // Edge case: div by zero
    assert(mapFloat(1.0, 2.0, 2.0, 10.0, 20.0) == 10.0);
    std::cout << "  PASS: mapFloat" << std::endl;
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
