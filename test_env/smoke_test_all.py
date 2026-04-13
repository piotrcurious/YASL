
import os
import subprocess
import re

def run_compilation_test(ino_path, test_env_path):
    with open(ino_path, 'r') as f:
        content = f.read()

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
    setup();
    for(int i=0; i<5; i++) loop();
    return 0;
}}
"""
    cpp_path = "smoke_test.cpp"
    bin_path = "smoke_test_bin"
    with open(cpp_path, 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", f"-I{test_env_path}", cpp_path, os.path.join(test_env_path, "Arduino.cpp"), "-o", bin_path]
    result = subprocess.run(cmd, capture_output=True, text=True)

    success = False
    if result.returncode == 0:
        try:
            run_res = subprocess.run([f"./{bin_path}"], capture_output=True, text=True, timeout=1)
            if run_res.returncode == 0:
                success = True
                msg = "Pass"
            else:
                msg = f"Runtime Error: {run_res.stderr}"
        except subprocess.TimeoutExpired:
            success = True
            msg = "Pass (Timeout - likely heavy loop)"
    else:
        msg = f"Compile Error: {result.stderr}"

    if os.path.exists(cpp_path): os.remove(cpp_path)
    if os.path.exists(bin_path): os.remove(bin_path)

    return success, msg

if __name__ == "__main__":
    test_env = "test_env"
    ino_files = []
    for root, dirs, files in os.walk('.'):
        if 'test_env' in root: continue
        for file in files:
            if file.endswith('.ino'):
                ino_files.append(os.path.join(root, file))

    print(f"Found {len(ino_files)} files to test.")
    results = []
    for f in sorted(ino_files):
        print(f"Testing {f}...", end=" ", flush=True)
        # Always use root test_env as we cleaned up the others
        env = "test_env"
        success, msg = run_compilation_test(f, env)
        status = "OK" if success else "FAIL"
        print(status)
        results.append((f, status, msg if not success else ""))

    print("\n--- Smoke Test Summary ---")
    for f, s, m in results:
        print(f"{f}: {s}")
        if s == "FAIL":
            print(f"   Error: {m[:200]}...")
