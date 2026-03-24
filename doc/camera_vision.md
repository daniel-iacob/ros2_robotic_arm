# Phase 4 — Camera + Vision Pipeline

> **Status**: 26/28 tests passing. Camera renders correctly. Two vision position tests failing — `motion_server` position updates from vision temporarily disabled. See CHANGELOG.md for bug history.

## Overview

Replace hardcoded object positions (`objects.yaml`) with a real vision pipeline that discovers positions from camera images. No Gazebo — uses a synthetic camera that renders the MoveIt scene. The detection algorithm is real and works from pixels only.

---

## Architecture — What's real vs synthetic

```mermaid
flowchart LR
    classDef synthetic fill:#fff3cd,stroke:#ffc107,color:#000
    classDef real fill:#d4edda,stroke:#28a745,color:#000
    classDef existing fill:#e2e3e5,stroke:#6c757d,color:#000

    subgraph SYNTHETIC["Synthetic (replaced by Gazebo later)"]
        direction TB
        MG_read["MoveIt\nplanning scene"]
        CN["camera_node\n— reads 3D positions\n— draws 2D image"]
        MG_read -->|"object positions\nshapes, dims"| CN
    end

    subgraph REAL_VISION["Real Vision (keeps working on HW)"]
        direction TB
        VN["vision_node\n— HSV threshold\n— contour detection\n— pixel → world XY"]
    end

    subgraph EXISTING["Existing (unchanged)"]
        direction TB
        MS["motion_server\n— updated positions\n— pick / place"]
        CLI["arm_cli"]
        CLI -->|actions| MS
    end

    CN -->|"/camera/image_raw\n(pixels only)"| VN
    VN -->|"/detected_objects\n(name, x, y, z)"| MS

    class MG_read,CN synthetic
    class VN real
    class MS,CLI existing
```

---

## The swap boundary — why this design works

The `/camera/image_raw` topic is the clean boundary. Everything to its right is real, permanent code — written once, works on synthetic images, Gazebo, and real hardware.

```mermaid
flowchart TD
    classDef swap fill:#fff3cd,stroke:#ffc107,color:#000
    classDef keep fill:#d4edda,stroke:#28a745,color:#000

    subgraph now["Phase 4 — Now"]
        A1["camera_node\nsynthetic renderer"]:::swap
        A1 -->|/camera/image_raw| B1
        B1["vision_node"]:::keep
        B1 -->|/detected_objects| C1["motion_server"]:::keep
    end

    subgraph later["Phase 4.5 — Gazebo"]
        A2["Gazebo camera\nreal rendered image"]:::swap
        A2 -->|/camera/image_raw| B2
        B2["vision_node\nSAME CODE"]:::keep
        B2 -->|/detected_objects| C2["motion_server\nSAME CODE"]:::keep
    end

    subgraph hw["Future — Real HW"]
        A3["USB camera\nreal sensor"]:::swap
        A3 -->|/camera/image_raw| B3
        B3["vision_node\nSAME CODE"]:::keep
        B3 -->|/detected_objects| C3["motion_server\nSAME CODE"]:::keep
    end
```

**Yellow** = swappable image source (changes per stage) | **Green** = real pipeline code (written once)

---

## Data flow

```mermaid
flowchart TD
    classDef startup fill:#e2e3e5,stroke:#6c757d,color:#000
    classDef runtime fill:#d4edda,stroke:#28a745,color:#000

    subgraph startup["Startup (one-shot, unchanged)"]
        YAML["objects.yaml"] --> SM["scene_manager"]
        SM -->|"ADD objects"| MG["MoveIt planning scene"]
    end

    subgraph runtime["Runtime (new in Phase 4)"]
        MG2["MoveIt planning scene"] -->|"/get_planning_scene"| CN["camera_node"]
        CN -->|"/camera/image_raw"| VN["vision_node"]
        VN -->|"/detected_objects"| MS["motion_server"]
        MS -->|"pick/place with\ndetected positions"| MG2
        CLI["arm_cli"] -->|"actions"| MS
    end

    MG --> MG2

    class YAML,SM,MG startup
    class MG2,CN,VN,MS,CLI runtime
```

---

## How the synthetic camera works

`camera_node` renders a top-down orthographic image of the scene:

1. Queries MoveIt `/get_planning_scene` at ~10Hz for current object positions
2. Reads colors from `objects.yaml` (MoveIt doesn't reliably return colors)
3. Projects world XY → pixel UV: `u = cx + x * scale`, `v = cy - y * scale`
4. Draws colored shapes with OpenCV (circles for cylinders, rectangles for boxes)
5. Publishes `sensor_msgs/Image` on `/camera/image_raw`

## How the vision detection works

`vision_node` does real HSV color detection — it only sees pixels:

1. Subscribes to `/camera/image_raw`
2. Reads `objects.yaml` for color→name mapping and default Z height
3. For each object color: HSV threshold → binary mask → contours → centroid
4. Back-projects pixel centroid → world XY: `x = (u - cx) / scale`, `y = -(v - cy) / scale`
5. Z = fixed from `objects.yaml` (top-down camera can't see height)
6. Publishes `DetectedObjects` on `/detected_objects`

**Does NOT** query MoveIt, read planning scene, or access ground truth.

---

## Decisions

| Decision | Reasoning |
|----------|-----------|
| Synthetic camera (no Gazebo) | Fastest path to working pipeline. Gazebo swap is clean — only `camera_node` replaced |
| New `robotic_arm_perception` package | Clean separation. Entire package swappable for Gazebo |
| Color config from `objects.yaml` | Single source of truth — colors already defined there |
| Fixed Z from YAML | Top-down camera can't see height. All objects on same table surface |
| `motion_server` subscribes to detections | Minimal change — just a new subscriber + position update |

## Gazebo migration path (future)

Delete `camera_node.py`. Add Gazebo camera sensor to URDF + `ros_gz_bridge`. `vision_node`, messages, and `motion_server` subscriber stay identical.
