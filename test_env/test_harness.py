import os
import subprocess
import re

def run_test(ino_path):
    print(f"Testing {ino_path}...")

    with open(ino_path, 'r') as f:
        content = f.read()

    matches = re.finditer(r'^\s*(void|int|float|double|long|byte|boolean)\s+(\w+)\s*\(([^)]*)\)\s*\{', content, re.MULTILINE | re.DOTALL)
    prototypes_str = ""
    for match in matches:
        ret_type = match.group(1)
        name = match.group(2)
        params = match.group(3)
        if name in ["setup", "loop", "ISR"]: continue
        params = re.sub(r'//.*', '', params)
        params = params.replace('\n', ' ')
        prototypes_str += f"{ret_type} {name}({params});\n"

    has_setup = "void setup()" in content
    has_loop = "void loop()" in content

    wrapper = f"""
#include "Arduino.h"
#include "avr/interrupt.h"

{prototypes_str}

// Content of .ino file
{content}

int main() {{
    {"setup();" if has_setup else ""}
    for(int i = 0; i < 10; ++i) {{
        {"loop();" if has_loop else ""}
    }}
    return 0;
}}
"""
    with open('test_env/main.cpp', 'w') as f:
        f.write(wrapper)

    cmd = ["g++", "-fpermissive", "-I./test_env", "test_env/main.cpp", "test_env/Arduino.cpp", "-o", "test_env/test_bin"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            return False, result.stderr
    except Exception as e:
        return False, str(e)

    try:
        try:
            result = subprocess.run(["./test_env/test_bin"], capture_output=True, text=True, timeout=2)
            success = (result.returncode == 0)
            output = result.stdout if success else result.stderr
        except subprocess.TimeoutExpired:
            success, output = False, "Timed out"
        except Exception as e:
            success, output = False, str(e)
    finally:
        # Cleanup
        if os.path.exists("test_env/test_bin"): os.remove("test_env/test_bin")
        if os.path.exists("test_env/main.cpp"): os.remove("test_env/main.cpp")

    return success, output

if __name__ == "__main__":
    ino_files = []
    for root, dirs, files in os.walk('.'):
        if 'test_env' in root: continue
        for file in files:
            if file.endswith('.ino'):
                ino_files.append(os.path.join(root, file))

    results = {}
    for ino in sorted(ino_files):
        success, output = run_test(ino)
        results[ino] = (success, output)
        if success:
            print(f"{ino}: PASS")
        else:
            print(f"{ino}: FAIL")

    print("\n\nSUMMARY")
    print("="*20)
    for ino, (success, output) in results.items():
        status = "PASS" if success else "FAIL"
        print(f"{ino}: {status}")
