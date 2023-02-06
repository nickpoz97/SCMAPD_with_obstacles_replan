import subprocess
import argparse
import os
import sys
import shutil

parser = argparse.ArgumentParser(description="build SCMAPD solver")
parser.add_argument("build_path", type=str, help="path where temp build files will be placed")
parser.add_argument("bin_path", type=str, help="path where exe files will be placed")
args = parser.parse_args()

build_path = args.build_path
bin_path = args.bin_path

assert(not os.path.exists(build_path))
os.makedirs(build_path)

# force MinGW on windows
generator = "-G \"MinGW Makefiles\"" if sys.platform == "win32" else ""

subprocess.run(f"cmake -S . -B {build_path} -DCMAKE_BUILD_TYPE=Release {generator}", check=True, shell=True)
print("########### Configuration complete ###########")
subprocess.run(f"cmake --build {build_path} -j 8 --config Release", check=True, shell=True)

target_extension = ".exe" if sys.platform == "win32" else ""
target_name = "cmapd" + target_extension

shutil.move(os.path.join(build_path, target_name), bin_path)
