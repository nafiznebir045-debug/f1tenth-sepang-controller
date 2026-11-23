#!/usr/bin/env python3
"""
Automated ROS 2 Bag Recording Script for F1Tenth Sepang.
Records all topics (-a) into 'f1tenth_run_01' for 1 complete clean lap (~36s),
then gracefully terminates the bag recording with SIGINT so metadata and DB are cleanly closed.
"""
import os
import signal
import subprocess
import time
import sys

WORKSPACE_DIR = "/home/admin1/Downloads/nebirAssaignment-main"
BAG_FOLDER = "ROS 2 Bag"
BAG_NAME = "f1tenth_run_01"
OUTPUT_BAG_DIR = os.path.join(WORKSPACE_DIR, BAG_FOLDER)
OUTPUT_BAG_PATH = os.path.join(OUTPUT_BAG_DIR, BAG_NAME)

env = os.environ.copy()
env["ROS_DOMAIN_ID"] = "42"
env["ROS_LOCALHOST_ONLY"] = "1"
env["DISPLAY"] = env.get("DISPLAY", ":0")

# Source command prefix
source_cmd = "source /opt/ros/humble/setup.bash && source /home/admin1/Downloads/nebirAssaignment-main/install/setup.bash"

print("[*] Cleaning up old nodes...")
subprocess.run("pkill -9 -f 'gym_bridge|gap_node|rviz2|map_server|ros2 bag' || true", shell=True)
time.sleep(1)

# Remove existing bag folder if it exists
if os.path.exists(OUTPUT_BAG_PATH):
    print(f"[*] Removing previous {OUTPUT_BAG_PATH}...")
    subprocess.run(f"rm -rf {OUTPUT_BAG_PATH}", shell=True)

print("[1/3] Launching F1Tenth Simulator Bridge...")
sim_proc = subprocess.Popen(
    f"{source_cmd} && ros2 launch f1tenth_gym_ros gym_bridge_launch.py",
    shell=True,
    executable="/bin/bash",
    env=env,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    preexec_fn=os.setsid
)
time.sleep(4.0)

print("[2/3] Launching Follow-the-Gap Controller...")
ctrl_proc = subprocess.Popen(
    f"{source_cmd} && ros2 run controllers gap_node",
    shell=True,
    executable="/bin/bash",
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid,
    text=True
)
time.sleep(2.0)

os.makedirs(OUTPUT_BAG_DIR, exist_ok=True)

print(f"[3/3] Starting ros2 bag record -a -o '{BAG_NAME}' inside '{BAG_FOLDER}'...")
bag_proc = subprocess.Popen(
    f"{source_cmd} && cd '{OUTPUT_BAG_DIR}' && ros2 bag record -a -o '{BAG_NAME}'",
    shell=True,
    executable="/bin/bash",
    env=env,
    preexec_fn=os.setsid
)

RECORD_DURATION = 38  # 38 seconds ensures 1 full completed lap at 6x speed + finish line crossing
print(f"[*] Recording for {RECORD_DURATION} seconds (1 complete lap)...")
start_t = time.time()
while time.time() - start_t < RECORD_DURATION:
    remaining = int(RECORD_DURATION - (time.time() - start_t))
    print(f"    Recording... {remaining}s remaining", end="\r", flush=True)
    time.sleep(2)

print("\n[*] Stopping ros2 bag record gracefully (SIGINT)...")
os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
try:
    bag_proc.wait(timeout=5)
except subprocess.TimeoutExpired:
    os.killpg(os.getpgid(bag_proc.pid), signal.SIGTERM)

print("[*] Stopping controller and simulator...")
try:
    os.killpg(os.getpgid(ctrl_proc.pid), signal.SIGTERM)
    os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
except Exception:
    pass

subprocess.run("pkill -9 -f 'gym_bridge|gap_node|rviz2|map_server' || true", shell=True)
time.sleep(2)

print(f"\n[✓] Recording complete! Inspecting '{BAG_FOLDER}/{BAG_NAME}'...")
res = subprocess.run(
    f"{source_cmd} && cd '{WORKSPACE_DIR}' && ros2 bag info '{BAG_FOLDER}/{BAG_NAME}'",
    shell=True,
    executable="/bin/bash",
    env=env,
    capture_output=True,
    text=True
)
print(res.stdout)
if res.stderr:
    print("Stderr:", res.stderr)
