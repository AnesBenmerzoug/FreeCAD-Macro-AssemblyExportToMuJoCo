import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Literal

import FreeCAD as App
import Mesh
import MeshPart
import UtilsAssembly

from freecad.assembly2mujoco.constants import (
    DEFAULT_MESH_ANGULAR_DEFLECTION,
    DEFAULT_MESH_LINEAR_DEFLECTION,
    DEFAULT_MESH_EXPORT_FORMAT,
    DEFAULT_MJCF_INTEGRATOR,
    DEFAULT_MJCF_SOLVER,
    DEFAULT_MJCF_TIMESTEP,
    DEFAULT_MJCF_ARMATURE,
    DEFAULT_MJCF_DAMPING,
    WORKBENCH_NAME,
)
from freecad.assembly2mujoco.core.assembly_parser import (
    Graph,
    GraphNode,
    GraphEdge,
    find_minimum_spanning_tree,
    convert_to_directed_tree,
)
from freecad.assembly2mujoco.utils.helpers import log_message

__all__ = ["MuJoCoExporter"]


class MuJoCoExporter:
    """Class for exporting a kinematic tree as a MuJoCo MJCF (XML) file and STL files.

    Args:
        integrator: The numerical integrator to be used in MuJoCo.
            The available integrators are the semi-implicit Euler method,
            the fixed-step 4-th order Runge Kutta method,
            the Implicit-in-velocity Euler method, and implicitfast.
        timestep: Simulation time step in seconds.
    """

    def __init__(
        self,
        *,
        mesh_linear_deflection: float = DEFAULT_MESH_LINEAR_DEFLECTION,
        mesh_angular_deflection: float = DEFAULT_MESH_ANGULAR_DEFLECTION,
        mesh_export_format: Literal["STL", "OBJ"] = DEFAULT_MESH_EXPORT_FORMAT,
        integrator: Literal[
            "Euler", "implicit", "implicitfast", "RK4"
        ] = DEFAULT_MJCF_INTEGRATOR,
        solver: Literal["PGS", "CG", "Newton"] = DEFAULT_MJCF_SOLVER,
        timestep: float = DEFAULT_MJCF_TIMESTEP,
        damping: float = DEFAULT_MJCF_DAMPING,
        armature: float = DEFAULT_MJCF_ARMATURE,
    ) -> None:
        self.mesh_linear_deflection = mesh_linear_deflection
        self.mesh_angular_deflection = mesh_angular_deflection
        self.mesh_export_format = mesh_export_format
        self.integrator = integrator
        self.solver = solver
        self.timestep = timestep
        self.damping = damping
        self.armature = armature

        self.mujoco = ET.Element("mujoco")
        self.option = ET.SubElement(
            self.mujoco,
            "option",
            integrator=self.integrator,
            timestep=str(self.timestep),
            solver=self.solver,
        )
        self.compiler = ET.SubElement(
            self.mujoco,
            "compiler",
            meshdir="meshes",
            autolimits="true",
            inertiafromgeom="true",
            angle="radian",
        )
        # Add defaults
        self.default = ET.SubElement(self.mujoco, "default")
        ET.SubElement(
            self.default,
            "joint",
            damping=f"{self.damping}",
            armature=f"{self.armature}",
        )
        ET.SubElement(
            self.default,
            "geom",
            conaffinity="0",
            condim="3",
            friction="1 0.5 0.5",
            margin="0",
        )
        # Add assets
        self.asset = ET.SubElement(self.mujoco, "asset")
        ET.SubElement(
            self.asset,
            "texture",
            type="skybox",
            builtin="gradient",
            rgb1="0.3 0.5 0.7",
            rgb2="0 0 0",
            width="512",
            height="3072",
        )
        ET.SubElement(
            self.asset,
            "texture",
            type="2d",
            name="groundplane",
            builtin="checker",
            mark="edge",
            rgb1="0.2 0.3 0.4",
            rgb2="0.1 0.2 0.3",
            markrgb="0.8 0.8 0.8",
            width="300",
            height="300",
        )
        ET.SubElement(
            self.asset,
            "material",
            name="groundplane",
            texture="groundplane",
            texuniform="true",
            texrepeat="5 5",
            reflectance="0.1",
        )

        self.worldbody = ET.SubElement(self.mujoco, "worldbody")
        # Add multiple light sources for better visualization
        self.worldbody.append(ET.Comment("Lighting and floor"))
        ET.SubElement(
            self.worldbody, "light", diffuse=".8 .8 .8", pos="0 0 4", dir="0 0 -1"
        )
        ET.SubElement(
            self.worldbody, "light", diffuse=".6 .6 .6", pos="4 4 4", dir="-1 -1 -1"
        )
        # Placeholder for elements that will be added when exporting
        self.contact = ET.SubElement(self.mujoco, "contact")
        self.equality = ET.SubElement(self.mujoco, "equality")
        self.tendon = ET.SubElement(self.mujoco, "tendon")
        self.actuator = ET.SubElement(self.mujoco, "actuator")
        self.sensor = ET.SubElement(self.mujoco, "sensor")

    def export_assembly(
        self, assembly: App.DocumentObject, output_dir: str | os.PathLike
    ) -> None:
        """Main export method"""

        # Create graph connecting parts with joints
        assembly_graph = Graph.from_assembly(assembly)

        # Export assembly parts as binary stl meshes
        meshes_dir = Path(output_dir).joinpath("meshes")
        meshes_dir.mkdir(exist_ok=True, parents=True)
        self.worldbody.append(ET.Comment("Part Meshes"))
        self.export_parts_as_meshes_and_add_to_assets(assembly_graph, meshes_dir)

        # Add floorplane (cosmetic)
        self.add_floorplane(assembly_graph)
        self.worldbody.append(ET.Comment("Assembly"))

        # Find minimum spanning tree representing kinematic tree
        # As well as unused edges (joints) that will be converted to equality constraints
        tree, unused_edges = find_minimum_spanning_tree(assembly_graph)

        # Use grounded part as root node
        root_node: GraphNode | None = None
        grounded_joints = [
            joint
            for joint in UtilsAssembly.getJointGroup(assembly).Group
            if hasattr(joint, "ObjectToGround")
        ]
        if grounded_joints:
            grounded_joint = grounded_joints[0]
            grounded_part = grounded_joint.ObjectToGround
            # Find corresponding node
            root_node = [
                node for node in tree.get_nodes() if node.part == grounded_part
            ][0]

        # Build tree
        tree = convert_to_directed_tree(tree, root_node)
        if root_node is None:
            root_node = tree.get_nodes()[0]
        self.process_tree(root_node, tree)

        # Handle kinematic loops
        if unused_edges:
            self.process_kinematic_loops(unused_edges)

        # Save MJCF file
        xml_file = (
            Path(output_dir).joinpath(App.activeDocument().Name).with_suffix(".xml")
        )
        self.write_xml(xml_file)

    def export_parts_as_meshes_and_add_to_assets(
        self,
        assembly_graph: Graph,
        meshes_dir: str | os.PathLike,
    ) -> None:
        for node in assembly_graph.get_nodes():
            part = node.part
            shape = part.Shape.copy(False)
            mesh_file = Path(meshes_dir).joinpath(part.Name)

            if self.mesh_export_format == "STL":
                mesh = MeshPart.meshFromShape(
                    Shape=shape,
                    LinearDeflection=self.mesh_linear_deflection,
                    AngularDeflection=self.mesh_angular_deflection,
                    Relative=False,
                )
                mesh_file = mesh_file.with_suffix(".stl")
                mesh.write(os.fspath(mesh_file))
            elif self.mesh_export_format == "OBJ":
                mesh_file = mesh_file.with_suffix(".obj")
                Mesh.export([part], os.fspath(mesh_file))
            else:
                raise ValueError(
                    f"{WORKBENCH_NAME}: Unexpected mesh export format '{self.mesh_export_format}'"
                )

            # Add new mesh to assets
            ET.SubElement(
                self.asset,
                "mesh",
                name=part.Name,
                file=mesh_file.name,
                # Convert mm to m
                scale="0.001 0.001 0.001",
            )
            # Add material for appearance to assets
            appearance_dict = node.get_body_appearance()
            found_existing_materials = self.asset.findall(
                f"./material[@name='{appearance_dict['name']}']"
            )
            # Add material only if it wasn't added already
            # We do the check by name
            # TODO: Consider using a dictionary to keep track of added materials
            if len(found_existing_materials) == 0:
                ET.SubElement(self.asset, "material", **appearance_dict)

    def add_floorplane(self, assembly_graph: Graph) -> None:
        minimum_z_placement: float | None = None
        for node in assembly_graph.get_nodes():
            part = node.part
            if minimum_z_placement is None:
                minimum_z_placement = part.Placement.Base[2]
            else:
                minimum_z_placement = min(minimum_z_placement, part.Placement.Base[2])
        if minimum_z_placement is None:
            raise RuntimeError(f"{WORKBENCH_NAME}: This should not happen")
        # convert from mm to m
        minimum_z_placement *= 0.001
        # shift placement to account for floorplane size
        floor_z_pos = minimum_z_placement - 0.001
        # Add floorplane under lowest part
        ET.SubElement(
            self.worldbody,
            "geom",
            name="floor",
            pos=f"0 0 {floor_z_pos}",
            size="0 0 1",
            type="plane",
            material="groundplane",
        )

    def process_tree(
        self,
        current_node: GraphNode,
        tree: Graph,
        *,
        parent_node: ET.Element | None = None,
        body_elements: dict[GraphNode, ET.Element] | None = None,
    ) -> ET.Element:
        if body_elements is None:
            body_elements = {}
        parent_body = body_elements.get(parent_node, self.worldbody)

        if (current_body := body_elements.get(current_node)) is None:
            current_body = self.add_body(current_node, parent_body)
            body_elements[current_node] = current_body

        for child_node in tree.get_neighbors(current_node):
            child_body = self.process_tree(
                child_node,
                tree,
                parent_node=current_node,
                body_elements=body_elements,
            )
            edge = tree.get_edge(current_node, child_node)
            self.add_joint_to_body(child_body, edge)
        return current_body

    def add_body(self, node: GraphNode, parent_body: ET.Element) -> ET.Element:
        # Create body element for this part
        pos, quat = node.get_body_position_and_orientation()
        body = ET.SubElement(
            parent_body,
            "body",
            name=node.part.Name,
            pos=pos,
            quat=quat,
        )
        # Add mesh for visualization
        appearance_dict = node.get_body_appearance()
        ET.SubElement(
            body,
            "geom",
            type="mesh",
            name=f"{node.part.Name} geom",
            mesh=node.part.Name,
            material=appearance_dict["name"],
            contype="0",
            conaffinity="0",
        )
        return body

    def add_joint_to_body(
        self,
        body: ET.Element,
        edge: GraphEdge,
    ) -> ET.Element:
        """Add a joint to a body element"""
        joint_type = edge.get_mujoco_joint_type()

        if joint_type is None or joint_type == "fixed":
            return None

        joint_pos_vector, joint_axis_vector = edge.get_joint_position_and_axis()
        joint_pos = " ".join(str(x) for x in joint_pos_vector)
        joint_axis = " ".join(str(x) for x in joint_axis_vector)
        joint_range = edge.get_joint_range()

        # Create the joint element
        joint_element = ET.SubElement(
            body,
            "joint",
            type=joint_type,
            name=edge.joint.Label,
            pos=joint_pos,
            axis=joint_axis,
        )
        if joint_range is not None:
            joint_element.set("range", joint_range)

        # Create actuator element
        actuator_element = ET.SubElement(
            self.actuator,
            "position",
            name=joint_element.get("name"),
            joint=joint_element.get("name"),
            kp="100",
        )
        if joint_range is not None:
            actuator_element.set("ctrlrange", joint_range)

        # Create sensor element
        ET.SubElement(
            self.sensor,
            "jointpos",
            name=joint_element.get("name") + "_pos",
            joint=joint_element.get("name"),
        )
        return joint_element

    def process_kinematic_loops(
        self, unused_edges: list[tuple[GraphNode, GraphNode, GraphEdge]]
    ) -> None:
        log_message(f"Found {len(unused_edges)} kinematic loops in the assembly")
        for u, v, edge in unused_edges:
            joint_type = edge.get_mujoco_joint_type()
            if joint_type is None:
                # For fixed joints, use weld constraint
                ET.SubElement(
                    self.equality,
                    "weld",
                    name=f"loop_weld_{edge.joint.Label}",
                    body1=u.part.Name,
                    body2=v.part.Name,
                    solref="0.01 1",
                    solimp="0.9 0.95 0.001",
                )
            else:
                raise NotImplementedError(
                    f"Processing kinematic loop not implemented for joint type '{joint_type}'"
                )

    def write_xml(self, xml_file: str | os.PathLike) -> None:
        """Writes final XML structure to a file.

        Args:
            xml_file: Output path to XML file.
        """
        ET.indent(self.mujoco)
        tree = ET.ElementTree(self.mujoco)
        tree.write(xml_file, encoding="utf-8", xml_declaration=True)
        log_message(f"Successfully exported to {xml_file}")
