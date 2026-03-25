"""System integration tests for the robotic arm.

Each test calls the CLI exactly as a user would:
    ros2 run robotic_arm_bringup arm <command> [args]

Tests run in file order — the sequence is intentional (pick before place, etc.).
"""

import re
import subprocess

CMD_TIMEOUT = 60


def parse_object_position(output, name):
    """Parse 'name: (x, y, z)' from list-objects output. Returns (x,y,z) tuple or None."""
    for line in output.splitlines():
        if name in line and "(" in line and "held" not in line:
            m = re.search(r'\(([+-]?\d+\.\d+),\s*([+-]?\d+\.\d+),\s*([+-]?\d+\.\d+)\)', line)
            if m:
                return float(m.group(1)), float(m.group(2)), float(m.group(3))
    return None


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


def test_scene_has_four_objects():
    """Scene should contain exactly 4 objects at startup."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    positions = [l for l in out.splitlines() if "(" in l and "," in l and "held" not in l]
    assert len(positions) == 4, f"expected 4 objects, got {len(positions)}:\n{out}"


def test_scene_no_held_at_start():
    """No object should be held at start (clean sim state)."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "held" not in out, f"unexpected 'held' at start:\n{out}"


# ── Error handling ────────────────────────────────────────────────────────────

def test_pick_unknown_object():
    """Picking a nonexistent object should fail gracefully."""
    rc, out = arm("pick", "nonexistent_thing")
    assert rc != 0, f"expected failure for unknown object:\n{out}"


def test_move_to_unreachable():
    """Moving to a position far beyond workspace should fail."""
    rc, out = arm("move-to", "2.0", "0.0", "0.0")
    assert rc != 0, f"expected failure for unreachable position:\n{out}"


def test_move_to_negative_z():
    """Position below table should fail."""
    rc, out = arm("move-to", "0.3", "0.0", "-0.1")
    assert rc != 0, f"expected failure for negative Z:\n{out}"


def test_place_nothing_held():
    """Placing when nothing is held should fail."""
    rc, out = arm("place", "blue_cylinder")
    assert rc != 0, f"expected failure when nothing held:\n{out}"


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


def test_move_to_on_axis():
    """On-axis position — tests symmetry (y=0)."""
    rc, out = arm("move-to", "0.35", "0.0", "0.4")
    assert rc == 0, f"move-to on-axis failed:\n{out}"


def test_move_to_basket_area():
    """Basket-side position — tests +Y workspace coverage."""
    rc, out = arm("move-to", "0.4", "0.25", "0.4")
    assert rc == 0, f"move-to basket area failed:\n{out}"


def test_home_twice():
    """Calling home twice in a row should both succeed."""
    rc1, out1 = arm("home")
    assert rc1 == 0, f"first home failed:\n{out1}"
    rc2, out2 = arm("home")
    assert rc2 == 0, f"second home failed:\n{out2}"


def test_gripper_cycle():
    """open → close → open sequence should all succeed."""
    rc, out = arm("open-gripper")
    assert rc == 0, f"open-gripper failed:\n{out}"
    rc, out = arm("close-gripper")
    assert rc == 0, f"close-gripper failed:\n{out}"
    rc, out = arm("open-gripper")
    assert rc == 0, f"second open-gripper failed:\n{out}"


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


def test_pick_emits_feedback():
    """pick should emit progress feedback containing '%'."""
    rc, out = arm("pick", "green_cylinder")
    assert rc == 0, f"pick green_cylinder failed:\n{out}"
    assert "%" in out, f"no progress feedback (%) in pick output:\n{out}"
    rc2, out2 = arm("place", "green_cylinder", "0.30", "-0.30", "0.3")
    assert rc2 == 0, f"restore place failed:\n{out2}"


def test_place_emits_feedback():
    """place should emit progress feedback containing '%'."""
    rc, out = arm("pick", "green_cylinder")
    assert rc == 0, f"pick for feedback test failed:\n{out}"
    rc2, out2 = arm("place", "green_cylinder", "0.30", "-0.30", "0.3")
    assert rc2 == 0, f"place failed:\n{out2}"
    assert "%" in out2, f"no progress feedback (%) in place output:\n{out2}"


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


def test_verify_blue_position_exact():
    """blue_cylinder placed at (0.45, 0.30) — verify with per-object parsing."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    pos = parse_object_position(out, "blue_cylinder")
    assert pos is not None, f"could not parse blue_cylinder position:\n{out}"
    x, y, z = pos
    assert abs(x - 0.45) < 0.01, f"blue_cylinder x: expected ~0.45, got {x}"
    assert abs(y - 0.30) < 0.01, f"blue_cylinder y: expected ~0.30, got {y}"


def test_verify_red_on_basket():
    """After placing, red_cylinder should show basket-area coordinates."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    assert "red_cylinder" in out, f"red_cylinder missing:\n{out}"
    assert "0.200" in out, f"expected y=0.200 in output:\n{out}"


def test_verify_red_position_exact():
    """red_cylinder placed at (0.35, 0.20) — verify with per-object parsing."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    pos = parse_object_position(out, "red_cylinder")
    assert pos is not None, f"could not parse red_cylinder position:\n{out}"
    x, y, z = pos
    assert abs(x - 0.35) < 0.01, f"red_cylinder x: expected ~0.35, got {x}"
    assert abs(y - 0.20) < 0.01, f"red_cylinder y: expected ~0.20, got {y}"


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


def test_scene_unchanged_after_move_to():
    """move-to should not alter object positions in the planning scene."""
    rc1, out1 = arm("list-objects")
    assert rc1 == 0, f"list-objects before failed:\n{out1}"
    arm("move-to", "0.3", "0.0", "0.4")
    rc2, out2 = arm("list-objects")
    assert rc2 == 0, f"list-objects after failed:\n{out2}"
    for name in ["blue_cylinder", "red_cylinder", "green_cylinder", "basket"]:
        before = parse_object_position(out1, name)
        after = parse_object_position(out2, name)
        assert before is not None, f"{name} missing before move-to:\n{out1}"
        assert after is not None, f"{name} missing after move-to:\n{out2}"
        assert before == after, f"{name} moved: {before} → {after}"


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


def test_verify_positions_after_reset_exact():
    """After reset, all objects at YAML positions — verified per-object with tolerance."""
    rc, out = arm("list-objects")
    assert rc == 0, f"list-objects failed:\n{out}"
    expected = {
        "blue_cylinder":  (0.45,  0.0,   0.3),
        "red_cylinder":   (0.0,  -0.45,  0.3),
        "green_cylinder": (0.30, -0.30,  0.3),
        "basket":         (0.4,   0.25,  0.26),
    }
    for name, (ex, ey, ez) in expected.items():
        pos = parse_object_position(out, name)
        assert pos is not None, f"could not parse {name} position after reset:\n{out}"
        x, y, z = pos
        assert abs(x - ex) < 0.01, f"{name} x after reset: expected {ex}, got {x}"
        assert abs(y - ey) < 0.01, f"{name} y after reset: expected {ey}, got {y}"


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


def test_camera_publishes_at_rate():
    """camera_node should publish /camera/image_raw at > 5 Hz."""
    cmd = ["ros2", "topic", "hz", "/camera/image_raw", "--window", "5"]
    print(f"\n  >>> {' '.join(cmd)}")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        out = result.stdout + result.stderr
    except subprocess.TimeoutExpired as e:
        # ros2 topic hz never exits — timeout is expected
        out = (e.stdout or b"").decode() + (e.stderr or b"").decode()
    for line in out.strip().splitlines():
        print(f"  {line}")
    assert "average rate" in out, f"no rate data from topic hz:\n{out}"
    m = re.search(r'average rate:\s*([\d.]+)', out)
    assert m is not None, f"could not parse rate from hz output:\n{out}"
    rate = float(m.group(1))
    assert rate > 5.0, f"camera rate too low: {rate:.1f} Hz (expected > 5):\n{out}"


def test_vision_confidence_positive():
    """All detected objects should have confidence > 0."""
    rc, out = ros2_topic("echo", "/detected_objects", "--once", timeout=30)
    assert rc == 0, f"topic echo failed:\n{out}"
    confidences = re.findall(r'confidence:\s*([\d.]+)', out)
    assert len(confidences) > 0, f"no confidence values in detection output:\n{out}"
    for c in confidences:
        assert float(c) > 0.0, f"confidence {c} not positive:\n{out}"
