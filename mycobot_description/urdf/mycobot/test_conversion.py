import tempfile
from pathlib import Path

import open3d as o3d
import trimesh

if __name__ == "__main__":
    mesh_path = Path("joint1.dae")

    mesh_format = mesh_path.suffix[1:]
    assert (
        mesh_format in trimesh.exchange.load.mesh_formats()  # type: ignore
    ), f"mesh format {mesh_path} not supported"

    mesh = trimesh.load(mesh_path, process=False, force="mesh")
    assert isinstance(mesh, trimesh.Trimesh), f"mesh type {type(mesh)} not supported"
    print(mesh)

    with tempfile.NamedTemporaryFile(suffix=".glb", delete=False) as f:
        file_path = f.name

        mesh.export(file_path)

        o3d_mesh = o3d.io.read_triangle_model(file_path)
        print(o3d_mesh)

    print(file_path)
