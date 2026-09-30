#!/usr/bin/env python3
"""Build a PX4 Classic world from the validated inspection scene."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
source = ROOT / "worlds" / "inspection_line.world"
target = ROOT / "worlds" / "px4_inspection.world"

tree = ET.parse(source)
world = tree.getroot().find("world")
assert world is not None

# The PX4 SITL world is intentionally standalone.  ``libgazebo_ros_state``
# belongs to the optional ROS Gazebo Classic package and is not needed by
# PX4's MAVLink interface; leaving it in a machine without that package makes
# gzserver print a plugin-load error before the vehicle is spawned.
for plugin in list(world.findall("plugin")):
    if "gazebo_ros_state" in (plugin.get("filename") or ""):
        world.remove(plugin)

for include in list(world.findall("include")):
    if (include.findtext("uri") or "") == "model://ground_plane":
        world.remove(include)

for model in list(world.findall("model")):
    name = model.get("name", "")
    if name == "inspection_drone" or name.startswith("coverage_"):
        world.remove(model)

# PX4's default spawn point is close to the first tower. Compress the 50 m
# source span to a 20 m near-field view so both towers and the aircraft fit in
# the initial Gazebo camera view.
for model in world.findall("model"):
    for pose in model.findall(".//pose"):
        fields = pose.text.split() if pose.text else []
        if len(fields) >= 6:
            fields[0] = f"{float(fields[0]) * 0.4:.6f}"
            pose.text = " ".join(fields)
    for visual in model.findall(".//visual"):
        if visual.get("name") == "line_safety_indicator":
            model.find("link").remove(visual)
    for cylinder in model.findall(".//cylinder"):
        length = cylinder.find("length")
        if length is not None and length.text == "50.0":
            length.text = "20.0"

ground = ET.Element("model", {"name": "light_ground"})
ET.SubElement(ground, "static").text = "true"
link = ET.SubElement(ground, "link", {"name": "link"})
visual = ET.SubElement(link, "visual", {"name": "visual"})
geometry = ET.SubElement(visual, "geometry")
plane = ET.SubElement(geometry, "plane")
ET.SubElement(plane, "normal").text = "0 0 1"
ET.SubElement(plane, "size").text = "160 80"
material = ET.SubElement(visual, "material")
ET.SubElement(material, "ambient").text = "0.55 0.60 0.64 1"
ET.SubElement(material, "diffuse").text = "0.62 0.68 0.72 1"
collision = ET.SubElement(link, "collision", {"name": "collision"})
cgeometry = ET.SubElement(collision, "geometry")
cplane = ET.SubElement(cgeometry, "plane")
ET.SubElement(cplane, "normal").text = "0 0 1"
ET.SubElement(cplane, "size").text = "160 80"
world.insert(0, ground)

ET.indent(tree, space="  ")
tree.write(target, encoding="utf-8", xml_declaration=True)
print(f"wrote {target}")
