import signal
import subprocess
import time

import pytest

STARTUP_TIMEOUT = 90


def _wait_for_ready(timeout=STARTUP_TIMEOUT):
    """Poll list-objects until the action server responds."""
    deadline = time.monotonic() + timeout
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        elapsed = int(time.monotonic() - (deadline - timeout))
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


def _kill_existing_sim():
    """Kill any leftover sim processes so we don't get duplicate action servers."""
    subprocess.run(["pkill", "-f", "motion_server"], capture_output=True)
    subprocess.run(["pkill", "-f", "arm_system.launch.py"], capture_output=True)
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    time.sleep(3)
    subprocess.run(["ros2", "daemon", "start"], capture_output=True)


@pytest.fixture(scope="session", autouse=True)
def sim():
    """Launch a fresh sim, wait for ready, teardown."""
    _kill_existing_sim()
    print("\n[sim] Launching sim...")
    proc = subprocess.Popen(
        ["ros2", "launch", "robotic_arm_bringup", "arm_system.launch.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    if not _wait_for_ready():
        proc.kill()
        pytest.fail(f"Sim did not come up within {STARTUP_TIMEOUT}s")

    print("[sim] Ready")
    yield

    print("\n[sim] Shutting down...")
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
