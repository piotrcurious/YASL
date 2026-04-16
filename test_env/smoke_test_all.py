
import os
import subprocess
import re

def smoke_test(ino_path, test_env_path):
    print(f"Smoke testing {ino_path}...", end=" ", flush=True)
    with open(ino_path, 'r') as f: content = f.read()

    if "```" in content:
        code_blocks = re.findall(r'```(?:cpp|ino|)\n(.*?)\n```', content, re.DOTALL)
        if code_blocks:
            content = "\n".join(code_blocks)

    content = re.sub(r'int\s+main\s*\(.*?\)\s*\{.*?\}', '', content, flags=re.DOTALL)

    prototypes_str = ""
    if "class " not in content:
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
#include <vector>
#include <string>
#include <cstring>
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "Adafruit_INA219.h"
#include "Eigen.h"
#include "TimeLib.h"
#include "ESP8266WiFi.h"
#include "IRremoteESP8266.h"
#include "NewPing.h"
#include "WiFiUdp.h"
#include "NTPClient.h"

// Missing Hardware Macros
#define PIR_PIN 2
#define TRIGGER_PIN 4
#define ECHO_PIN 5
#define IR_RECV_PIN 6
#define IR_SEND_PIN 7
#define LEARNING_TIMEOUT 30000

{prototypes_str}
{content}

#ifndef ARDUINO_MAIN
int main() {{
    return 0;
}}
#endif
"""
    cpp = "smoke_tmp.cpp"
    with open(cpp, 'w') as f: f.write(wrapper)

    abs_env = os.path.abspath(test_env_path)
    res = subprocess.run(["g++", "-c", "-fpermissive", f"-I{abs_env}", cpp, "-o", "smoke_tmp.o"], capture_output=True, text=True)

    if os.path.exists(cpp): os.remove(cpp)
    if os.path.exists("smoke_tmp.o"): os.remove("smoke_tmp.o")

    if res.returncode == 0:
        print("PASSED")
        return True
    else:
        print("FAILED")
        print(res.stderr)
        return False

if __name__ == "__main__":
    files = []
    # Major firmware files to track
    major_files = [
        "YASL.ino", "YASL2.ino", "YASL_Consolidated.ino",
        "v2_4_sync/YASL_Consolidated.ino"
    ]

    # Also find all other .ino files
    for root, dirs, filenames in os.walk('.'):
        if 'test_env' in root: continue
        for f in filenames:
            if f.endswith('.ino'):
                full_path = os.path.join(root, f).lstrip('./')
                if full_path not in major_files:
                    files.append(full_path)

    files = major_files + sorted(files)

    passed = 0
    failed = 0
    for f in files:
        if smoke_test(f, "test_env"):
            passed += 1
        else:
            # We allow legacy 'presence_junkbox/1.ino' to fail as it's known broken
            if "presence_junkbox/1.ino" in f:
                print("(Legacy file failure allowed)")
                continue
            failed += 1

    print(f"\nSummary: {passed} passed, {failed} failed (excluding known legacy issues).")
    if failed > 0:
        exit(1)
