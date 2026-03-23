"""System integration tests for the robotic arm.

Each test calls the CLI exactly as a user would:
    ros2 run robotic_arm_bringup arm <command> [args]

Tests run in file order — the sequence is intentional (pick before place, etc.).
"""

import subprocess

CMD_TIMEOUT = 60


def arm(*args, timeout=CMD_TIMEOUT):
    """Run an arm CLI command, return (exit_code, combined_output)."""
    cmd_str = " ".join(args)
    print(f"\n  >>> arm {cmd_str}")
    result = subprocess.run(
        ["ros2", "run", "robotic_arm_bringup", "arm", *args],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    output = result.stdout + result.stderr
    for line in output.strip().splitlines():
        print(f"  {line}")
    if result.returncode != 0:
        print(f"  EXIT CODE: {result.returncode}")
    return result.returncode, output


# ── Basic motion ──────────────────────────────────────────────────────────────

def test_home():
    rc, out = arm("home")
    assert rc == 0, f"home failed:\n{out}"


def test_open_gripper():
    rc, out = arm("open-gripper")
    assert rc == 0, f"open-gripper failed:\n{out}"


def test_close_gripper():
    rc, out = arm("close-gripper")
    assert rc == 0, f"close-gripper failed:\n{out}"


# ── Scene ─────────────────────────────────────────────────────────────────────

def test_list_objects():
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "blue_cylinder" in out, f"blue_cylinder not in output:\n{out}"
    assert "red_cylinder" in out, f"red_cylinder not in output:\n{out}"
    assert "green_cylinder" in out, f"green_cylinder not in output:\n{out}"
    assert "basket" in out, f"basket not in output:\n{out}"


# ── Error handling ────────────────────────────────────────────────────────────

def test_pick_unknown_object():
    """Picking a nonexistent object should fail gracefully."""
    rc, out = arm("pick", "nonexistent_thing")
    assert rc != 0, f"expected failure for unknown object:\n{out}"


def test_move_to_unreachable():
    """Moving to a position far beyond workspace should fail."""
    rc, out = arm("move-to", "2.0", "0.0", "0.0")
    assert rc != 0, f"expected failure for unreachable position:\n{out}"


# ── Move to ───────────────────────────────────────────────────────────────────

def test_move_to():
    rc, out = arm("move-to", "0.3", "0.2", "0.4")
    assert rc == 0, f"move-to failed:\n{out}"


def test_move_to_negative_y():
    """Arm can reach negative Y side."""
    rc, out = arm("move-to", "0.3", "-0.2", "0.4")
    assert rc == 0, f"move-to negative Y failed:\n{out}"


def test_move_to_high():
    """Arm can reach higher Z positions."""
    rc, out = arm("move-to", "0.2", "0.0", "0.6")
    assert rc == 0, f"move-to high Z failed:\n{out}"


# ── Pick and place ────────────────────────────────────────────────────────────

def test_pick_blue_cylinder():
    rc, out = arm("pick", "blue_cylinder")
    assert rc == 0, f"pick blue_cylinder failed:\n{out}"


def test_list_objects_held():
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "held" in out, f"expected 'held' in output:\n{out}"


def test_place_blue_cylinder_on_basket():
    """Place blue cylinder above the basket (tray top ~0.27, plus half cylinder height)."""
    rc, out = arm("place", "blue_cylinder", "0.45", "0.30", "0.35")
    assert rc == 0, f"place blue_cylinder on basket failed:\n{out}"


def test_pick_red_cylinder():
    rc, out = arm("pick", "red_cylinder")
    assert rc == 0, f"pick red_cylinder failed:\n{out}"


def test_place_red_cylinder_on_basket():
    """Place red cylinder on basket next to blue cylinder."""
    rc, out = arm("place", "red_cylinder", "0.35", "0.20", "0.35")
    assert rc == 0, f"place red_cylinder on basket failed:\n{out}"


# ── Verify positions after place ──────────────────────────────────────────────

def test_verify_blue_on_basket():
    """After placing, blue_cylinder should show basket-area coordinates."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "blue_cylinder" in out, f"blue_cylinder missing:\n{out}"
    # Should be at placed position (0.45, 0.30), not original (0.45, 0.0)
    assert "0.300" in out, f"expected y=0.300 in output:\n{out}"
    assert "held" not in out.split("blue_cylinder")[1].split("\n")[0], \
        f"blue_cylinder should not be held:\n{out}"


def test_verify_red_on_basket():
    """After placing, red_cylinder should show basket-area coordinates."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "red_cylinder" in out, f"red_cylinder missing:\n{out}"
    assert "0.200" in out, f"expected y=0.200 in output:\n{out}"


# ── Pick and place green cylinder (no coords — returns to original position) ─

def test_pick_green_cylinder():
    rc, out = arm("pick", "green_cylinder")
    assert rc == 0, f"pick green_cylinder failed:\n{out}"


def test_place_green_cylinder_no_coords():
    """Place green cylinder back at its original position (no explicit coords)."""
    rc, out = arm("place", "green_cylinder")
    assert rc == 0, f"place green_cylinder failed:\n{out}"


# ── Round-trip: re-pick green cylinder and place at a different position ──────

def test_repick_green_cylinder():
    """Pick green_cylinder from its original position (placed back earlier)."""
    rc, out = arm("pick", "green_cylinder")
    assert rc == 0, f"repick green_cylinder failed:\n{out}"


def test_place_green_cylinder_new_position():
    """Place green cylinder at a new open-air position (not on basket)."""
    rc, out = arm("place", "green_cylinder", "0.40", "-0.15", "0.3")
    assert rc == 0, f"place green_cylinder at new position failed:\n{out}"


# ── Post-sequence ─────────────────────────────────────────────────────────────

def test_home_after_sequence():
    rc, out = arm("home")
    assert rc == 0, f"home after sequence failed:\n{out}"


def test_reset():
    rc, out = arm("reset")
    assert rc == 0, f"reset failed:\n{out}"


def test_list_objects_after_reset():
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "held" not in out, f"unexpected 'held' after reset:\n{out}"


def test_verify_positions_after_reset():
    """After reset, all cylinders should be back at their YAML positions."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    # Original positions from objects.yaml
    assert "0.450" in out, f"expected blue_cylinder x=0.450 after reset:\n{out}"
    assert "-0.450" in out, f"expected red_cylinder y=-0.450 after reset:\n{out}"
    assert "-0.300" in out, f"expected green_cylinder y=-0.300 after reset:\n{out}"


# ── Vision pipeline tests ────────────────────────────────────────────────────

TOPIC_TIMEOUT = 15


def ros2_topic(*args, timeout=TOPIC_TIMEOUT):
    """Run a ros2 topic command, return (exit_code, output)."""
    cmd = ["ros2", "topic", *args]
    cmd_str = " ".join(cmd)
    print(f"\n  >>> {cmd_str}")
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    output = result.stdout + result.stderr
    for line in output.strip().splitlines():
        print(f"  {line}")
    return result.returncode, output


def test_camera_topic_active():
    """Verify /camera/image_raw topic has a publisher."""
    rc, out = ros2_topic("info", "/camera/image_raw")
    assert rc == 0, f"topic info failed:\n{out}"
    assert "Publisher count: 0" not in out, f"no publishers on /camera/image_raw:\n{out}"


def test_detected_objects_topic_active():
    """Verify /detected_objects topic has a publisher."""
    rc, out = ros2_topic("info", "/detected_objects")
    assert rc == 0, f"topic info failed:\n{out}"
    assert "Publisher count: 0" not in out, f"no publishers on /detected_objects:\n{out}"


def test_all_objects_detected():
    """Verify vision detects all 4 objects from the scene."""
    rc, out = ros2_topic("echo", "/detected_objects", "--once", timeout=30)
    assert rc == 0, f"topic echo failed:\n{out}"
    assert "blue_cylinder" in out, f"blue_cylinder not detected:\n{out}"
    assert "red_cylinder" in out, f"red_cylinder not detected:\n{out}"
    assert "green_cylinder" in out, f"green_cylinder not detected:\n{out}"
    assert "basket" in out, f"basket not detected:\n{out}"


def test_detected_positions_match_scene():
    """Verify detected positions are close to YAML defaults (within 3cm)."""
    rc, out = ros2_topic("echo", "/detected_objects", "--once", timeout=30)
    assert rc == 0, f"topic echo failed:\n{out}"

    # Parse detected positions from echo output
    # Expected from objects.yaml: blue(0.45,0.0), red(0.0,-0.45), green(0.30,-0.30), basket(0.4,0.25)
    expected = {
        "blue_cylinder": (0.45, 0.0),
        "red_cylinder": (0.0, -0.45),
        "green_cylinder": (0.30, -0.30),
        "basket": (0.4, 0.25),
    }

    for name, (exp_x, exp_y) in expected.items():
        assert name in out, f"{name} not in detection output:\n{out}"
        # Extract x and y values after the object name
        # The echo format has fields like: name: blue_cylinder\n  x: 0.45\n  y: 0.0
        section = out[out.index(name):]
        lines = section.split("\n")
        x_val = None
        y_val = None
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("x:"):
                x_val = float(stripped.split(":")[1])
            elif stripped.startswith("y:"):
                y_val = float(stripped.split(":")[1])
                break  # y comes after x, so we have both
        assert x_val is not None and y_val is not None, \
            f"could not parse x,y for {name}:\n{section[:200]}"
        tolerance = 0.03  # 3cm — accounts for pixel quantization
        assert abs(x_val - exp_x) < tolerance, \
            f"{name} x: expected {exp_x}, got {x_val}"
        assert abs(y_val - exp_y) < tolerance, \
            f"{name} y: expected {exp_y}, got {y_val}"
