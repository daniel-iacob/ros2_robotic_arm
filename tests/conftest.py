import atexit
import signal
import subprocess
import time

import pytest

STARTUP_TIMEOUT = 90
SIM_LOG = "/tmp/sim_test.log"


def _kill_sim_processes():
    """Kill all sim-related processes."""
    subprocess.run(["pkill", "-f", "motion_server"], capture_output=True)
    subprocess.run(["pkill", "-f", "scene_manager"], capture_output=True)
    subprocess.run(["pkill", "-f", "move_group"], capture_output=True)
    subprocess.run(["pkill", "-f", "ros2_control_node"], capture_output=True)
    subprocess.run(["pkill", "-f", "robot_state_publisher"], capture_output=True)
    subprocess.run(["pkill", "-f", "rviz2"], capture_output=True)
    subprocess.run(["pkill", "-f", "arm_system.launch.py"], capture_output=True)


def _wait_for_ready(proc, timeout=STARTUP_TIMEOUT):
    """Poll list-objects until the action server responds."""
    time.sleep(10)
    deadline = time.monotonic() + timeout
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        elapsed = int(time.monotonic() - (deadline - timeout))

        if proc.poll() is not None:
            print(f"  [poll #{attempt}] launch process DIED (exit={proc.returncode})")
            print(f"  Check {SIM_LOG} for details")
            return False

        try:
            result = subprocess.run(
                ["ros2", "run", "robotic_arm_bringup", "arm", "list-objects"],
                capture_output=True, text=True, timeout=12,
            )
            stdout = result.stdout.strip()
            stderr = result.stderr.strip()
            print(f"  [poll #{attempt} @ {elapsed}s] exit={result.returncode}")
            if stdout:
                print(f"    stdout: {stdout[:200]}")
            if stderr:
                print(f"    stderr: {stderr[:200]}")
            if result.returncode == 0:
                return True
        except subprocess.TimeoutExpired:
            print(f"  [poll #{attempt} @ {elapsed}s] TIMEOUT (12s)")
        time.sleep(5)
    return False


@pytest.fixture(scope="session", autouse=True)
def sim():
    """Launch a fresh sim, wait for ready, teardown."""
    _kill_sim_processes()
    time.sleep(2)

    print(f"\n[sim] Launching sim... (log: {SIM_LOG})")
    log = open(SIM_LOG, "w")
    proc = subprocess.Popen(
        ["ros2", "launch", "robotic_arm_bringup", "arm_system.launch.py"],
        stdout=log, stderr=log,
    )

    # Register cleanup so sim dies even on Ctrl+C / crash
    atexit.register(_kill_sim_processes)

    if not _wait_for_ready(proc):
        proc.kill()
        log.close()
        pytest.fail(f"Sim did not come up within {STARTUP_TIMEOUT}s — see {SIM_LOG}")

    print("[sim] Ready")
    yield

    print("\n[sim] Shutting down...")
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    _kill_sim_processes()
    log.close()
