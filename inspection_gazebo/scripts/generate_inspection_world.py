#!/usr/bin/env python3
"""Generate the readable Gazebo inspection world.

The scene is intentionally built from primitive SDF geometry so it remains
self-contained and deterministic in Gazebo 11.  The tower is a three-leg
triangular lattice tower; the conductors are separate, thin cylinders.  This
is a visual acceptance scene, not a physics or PX4 model.
"""

from __future__ import annotations

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "worlds" / "inspection_line.world"


def f(value: float) -> str:
    return f"{value:.6f}"


def pose(p: tuple[float, float, float], rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> str:
    return " ".join(f(v) for v in (*p, *rpy))


def material(name: str, rgba: tuple[float, float, float, float], transparency: float | None = None) -> str:
    r, g, b, a = rgba
    extra = "" if transparency is None else f"<transparency>{f(transparency)}</transparency>"
    return (
        f"<material><ambient>{f(r)} {f(g)} {f(b)} {f(a)}</ambient>"
        f"<diffuse>{f(r)} {f(g)} {f(b)} {f(a)}</diffuse>"
        f"<specular>0.18 0.18 0.18 1</specular>{extra}</material>"
    )


def cylinder(name: str, p0: tuple[float, float, float], p1: tuple[float, float, float], radius: float, mat: str) -> str:
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = tuple((p0[i] + p1[i]) / 2.0 for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (
        f'    <visual name="{name}"><pose>{pose(center, (0.0, pitch, yaw))}</pose>'
        f"<geometry><cylinder><radius>{f(radius)}</radius><length>{f(length)}</length></cylinder></geometry>"
        f"{mat}</visual>"
    )


def vertical_cylinder(name: str, x: float, y: float, z: float, radius: float, length: float, mat: str) -> str:
    return cylinder(name, (x, y, z - length / 2.0), (x, y, z + length / 2.0), radius, mat)


def sphere(name: str, p: tuple[float, float, float], radius: float, mat: str) -> str:
    return (
        f'    <visual name="{name}"><pose>{pose(p)}</pose>'
        f"<geometry><sphere><radius>{f(radius)}</radius></sphere></geometry>{mat}</visual>"
    )


def box(name: str, p: tuple[float, float, float], size: tuple[float, float, float], mat: str) -> str:
    sx, sy, sz = size
    return (
        f'    <visual name="{name}"><pose>{pose(p)}</pose>'
        f"<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size></box></geometry>{mat}</visual>"
    )


def tower_visuals(x0: float, label: str) -> list[str]:
    steel = material("steel", (0.22, 0.25, 0.29, 1.0))
    brace = material("brace", (0.38, 0.41, 0.46, 1.0))
    porcelain = material("porcelain", (0.86, 0.88, 0.92, 1.0))
    # Three vertices form a triangular footprint and converge near the top.
    base = [(-0.95, -0.90), (0.95, -0.90), (0.0, 1.05)]
    top = [(-0.13, -0.12), (0.13, -0.12), (0.0, 0.14)]
    levels = [0.30, 2.05, 3.80, 5.55, 7.30, 9.05]
    top_z = 9.55

    def point(level: float, index: int) -> tuple[float, float, float]:
        u = level / top_z
        bx, by = base[index]
        tx, ty = top[index]
        return (x0 + (1.0 - u) * bx + u * tx, (1.0 - u) * by + u * ty, level)

    out: list[str] = []
    for index in range(3):
        out.append(cylinder(f"{label}_leg_{index}", point(0.30, index), point(top_z, index), 0.065, steel))
    for level in levels:
        for index in range(3):
            out.append(cylinder(
                f"{label}_ring_{int(level * 100):03d}_{index}",
                point(level, index), point(level, (index + 1) % 3), 0.035, brace,
            ))
    for lower, upper in zip(levels[:-1], levels[1:]):
        band = int(lower * 100)
        for index in range(3):
            out.append(cylinder(
                f"{label}_diag_{band:03d}_{index}",
                point(lower, index), point(upper, (index + 1) % 3), 0.028, brace,
            ))
            out.append(cylinder(
                f"{label}_diag_back_{band:03d}_{index}",
                point(lower, (index + 1) % 3), point(upper, index), 0.022, brace,
            ))
    # The top crossarm carries the three phase conductors across the line.
    out.append(cylinder(f"{label}_crossarm", (x0, -2.35, 9.62), (x0, 2.35, 9.62), 0.075, steel))
    out.append(vertical_cylinder(f"{label}_top_mast", x0, 0.0, 9.95, 0.055, 0.70, steel))
    for index, y in enumerate((-0.45, 0.0, 0.45)):
        out.append(vertical_cylinder(f"{label}_insulator_{index}", x0, y, 9.88, 0.095, 0.42, porcelain))
        out.append(sphere(f"{label}_cap_{index}", (x0, y, 10.08), 0.075, porcelain))
    return out


def scene_model() -> str:
    conductor = material("conductor", (0.78, 0.64, 0.20, 1.0))
    # Keep this indicator thinner and almost transparent so it cannot be
    # mistaken for a second, oversized conductor in the GUI.
    safety = material("safety", (0.92, 0.08, 0.08, 1.0), transparency=0.88)
    lines = [
        '<model name="inspection_scene">',
        "  <static>true</static>",
        '  <link name="scene_link">',
        '    <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>',
    ]
    for index, y in enumerate((-0.45, 0.0, 0.45)):
        lines.append(
            f'    <visual name="conductor_{index}"><pose>{pose((25.0, y, 10.08), (0.0, math.pi / 2.0, 0.0))}</pose>'
            f'<geometry><cylinder><radius>0.003</radius><length>50.0</length></cylinder></geometry>{conductor}</visual>'
        )
    lines.append(
        f'    <visual name="line_safety_indicator"><pose>{pose((25.0, 0.0, 10.08), (0.0, math.pi / 2.0, 0.0))}</pose>'
        f'<geometry><cylinder><radius>0.009</radius><length>50.0</length></cylinder></geometry>{safety}</visual>'
    )
    for tower_x, label in ((0.0, "tower_0"), (50.0, "tower_50")):
        lines.extend(tower_visuals(tower_x, label))
    # Green spheres are the observable coverage cells. They are separate
    # moving models so the demo can reveal them only after each cell is
    # recorded; a static green lane was removed because it visually merged
    # with the conductors.
    lines.extend(["  </link>", "</model>"])
    return "\n".join(lines)


def drone_model() -> str:
    # Keep this as one ordinary model. Gazebo Classic 11 can crash while
    # resolving an included model that itself contains nested sensor models;
    # the ROS sensor instances remain separate world models and are moved with
    # this aircraft by inspection_gazebo_demo.py.
    dark = material("aircraft", (0.72, 0.76, 0.84, 1.0))
    graphite = material("graphite", (0.03, 0.03, 0.04, 1.0))
    camera = material("camera", (0.04, 0.05, 0.07, 1.0))
    cyan = material("sensor", (0.03, 0.16, 0.20, 1.0))
    return "\n".join([
        '<model name="inspection_drone">',
        '  <pose>0 3.8 1.5 0 0 0</pose>',
        '  <static>false</static>',
        '  <link name="body">',
        '    <gravity>false</gravity>',
        '    <kinematic>true</kinematic>',
        '    <inertial><mass>1.5</mass><inertia><ixx>0.029125</ixx><iyy>0.029125</iyy><izz>0.055225</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>',
        '    <visual name="iris_body">',
        '      <geometry><mesh><uri>model://inspection_aircraft/meshes/iris.stl</uri><scale>1 1 1</scale></mesh></geometry>',
        f'      {dark}',
        '    </visual>',
        '    <visual name="prop_front_right"><pose>0.13 -0.22 0.025 0 0 0</pose><geometry><mesh><uri>model://inspection_aircraft/meshes/iris_prop_ccw.dae</uri></mesh></geometry>' + graphite + '</visual>',
        '    <visual name="prop_back_left"><pose>-0.13 0.20 0.025 0 0 0</pose><geometry><mesh><uri>model://inspection_aircraft/meshes/iris_prop_cw.dae</uri></mesh></geometry>' + graphite + '</visual>',
        '    <visual name="prop_front_left"><pose>0.13 0.22 0.025 0 0 0</pose><geometry><mesh><uri>model://inspection_aircraft/meshes/iris_prop_cw.dae</uri></mesh></geometry>' + graphite + '</visual>',
        '    <visual name="prop_back_right"><pose>-0.13 -0.20 0.025 0 0 0</pose><geometry><mesh><uri>model://inspection_aircraft/meshes/iris_prop_ccw.dae</uri></mesh></geometry>' + graphite + '</visual>',
        f'    {box("d435i_mount", (0.28, 0.0, -0.11), (0.16, 0.10, 0.06), camera)}',
        f'    {box("mid360_mount", (0.0, 0.0, 0.12), (0.12, 0.12, 0.06), cyan)}',
        f'    {cylinder("mid360_ring", (0.0, 0.0, 0.15), (0.0, 0.0, 0.162), 0.06, cyan)}',
        '  </link>',
        '</model>',
    ])


def coverage_model(index: int) -> str:
    green = material("coverage", (0.10, 1.0, 0.10, 1.0))
    return "\n".join([
        f'<model name="coverage_{index:02d}">',
        '  <!-- Hidden until the demo records this coverage cell. -->',
        '  <pose>0 0 -2 0 0 0</pose>',
        '  <static>false</static>',
        '  <link name="marker">',
        '    <gravity>false</gravity>',
        '    <kinematic>true</kinematic>',
        '    <inertial><mass>0.01</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>',
        f'    {sphere("covered_cell", (0.0, 0.0, 0.0), 0.12, green)}',
        '  </link>',
        '</model>',
    ])


def main() -> None:
    header = """<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"inspection_line_world\">
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.72 0.78 0.86 1</background><shadows>true</shadows></scene>
    <physics type=\"ode\"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <include>
      <uri>model://inspection_depth_camera</uri>
      <name>inspection_d435i</name>
      <pose>0.35 3.8 1.32 0 0 0</pose>
    </include>
    <include>
      <uri>model://inspection_lidar</uri>
      <name>inspection_mid360</name>
      <pose>0.0 3.8 1.68 0 0 0</pose>
    </include>
    <plugin name=\"gazebo_ros_state\" filename=\"libgazebo_ros_state.so\">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>30.0</update_rate>
    </plugin>
"""
    chunks = [header, scene_model(), drone_model()]
    chunks.extend(coverage_model(index) for index in range(25))
    chunks.append("  </world>\n</sdf>\n")
    WORLD.write_text("\n\n".join(chunks), encoding="utf-8")
    print(f"wrote {WORLD}")


if __name__ == "__main__":
    main()
