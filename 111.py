import open3d as o3d
import pymesh
#a=o3d.io.read_triangle_mesh("tempMesh.ply")
#o3d.io.write_triangle_mesh("222.stl",a,write_ascii = True)
a=pymesh.load_mesh("tempMesh.ply")
pymesh.save_mesh("222.stl",a)

