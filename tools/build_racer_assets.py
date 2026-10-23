"""Reproduce the bundled racer assets from a pinned MIT-licensed upstream.

Run with Python from any directory. No downloading happens during simulation.
"""

from copy import deepcopy
from pathlib import Path
import urllib.request
from xml.etree import ElementTree as ET

COMMIT = "01563120e060c9233a61f9cf00e6d4e5a8b331ab"
SOURCE = f"https://raw.githubusercontent.com/learnsyslab/gym-pybullet-drones/{COMMIT}"
DESTINATION = Path(__file__).resolve().parents[1] / "src/bullet_drone/assets/racer"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
# Match the mesh's visual quadrants after the upstream 3x scale / 55 rad yaw.
PROPELLERS = {0: "Propeller_mockup-4", 1: "Propeller_mockup-3", 2: "Propeller_mockup-2", 3: "Propeller_mockup-1"}


def download(path: str) -> bytes:
    with urllib.request.urlopen(f"{SOURCE}/{path}", timeout=30) as response:
        return response.read()


def split_mesh(source: ET.Element, selected_propeller: str | None) -> ET.Element:
    mesh = deepcopy(source)
    assembly = mesh.find(".//c:visual_scene/c:node", NS)
    for child in list(assembly):
        if child.tag != f"{{{NS['c']}}}node":
            continue
        name = child.get("name", "")
        is_propeller = name.startswith("Propeller_mockup-")
        if (selected_propeller is None and is_propeller) or (
            selected_propeller is not None and name != selected_propeller
        ):
            assembly.remove(child)
    # Retain only the geometry referenced by the selected scene nodes.
    used = {node.get("url")[1:] for node in assembly.findall(".//c:instance_geometry", NS)}
    geometries = mesh.find("c:library_geometries", NS)
    for geometry in list(geometries):
        if geometry.get("id") not in used:
            geometries.remove(geometry)
    return mesh


def main() -> None:
    DESTINATION.mkdir(parents=True, exist_ok=True)
    urdf = ET.fromstring(download("gym_pybullet_drones/assets/racer.urdf"))
    collada = ET.fromstring(download("gym_pybullet_drones/assets/cf2.dae"))
    ET.register_namespace("", NS["c"])
    ET.ElementTree(split_mesh(collada, None)).write(
        DESTINATION / "body.dae", encoding="utf-8", xml_declaration=True,
    )
    base_mesh = urdf.find("link[@name='base_link']/visual/geometry/mesh")
    base_mesh.set("filename", "body.dae")
    for index, node_name in PROPELLERS.items():
        filename = f"prop{index}.dae"
        ET.ElementTree(split_mesh(collada, node_name)).write(
            DESTINATION / filename, encoding="utf-8", xml_declaration=True,
        )
        link = urdf.find(f"link[@name='prop{index}_link']")
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", rpy="0 0 55", xyz="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename=filename, scale="3 3 3")
        front = index in (0, 3)
        material = ET.SubElement(visual, "material", name="front_orange" if front else "rear_blue")
        ET.SubElement(material, "color", rgba="1 0.18 0.02 1" if front else "0.05 0.3 0.9 1")
    ET.indent(urdf)
    ET.ElementTree(urdf).write(DESTINATION / "racer.urdf", encoding="utf-8", xml_declaration=True)
    (DESTINATION / "LICENSE.upstream").write_bytes(download("LICENSE"))


if __name__ == "__main__":
    main()
