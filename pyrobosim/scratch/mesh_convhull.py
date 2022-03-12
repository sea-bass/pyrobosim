import scipy
import collada
import trimesh
import warnings
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from descartes.patch import PolygonPatch

# Load a mesh
filename = "/home/sebastian/.gazebo/models/first_2015_trash_can/meshes/trash_can.dae"
filename = "/home/sebastian/.gazebo/models/monkey_wrench/meshes/monkey_wrench.dae"
# filename = "/home/sebastian/.gazebo/models/dumpster/meshes/dumpster.dae"
# filename = "/home/sebastian/.gazebo/models/mars_rover/meshes/rover.dae"
mesh = trimesh.load_mesh(filename, "dae")

# Get the unit scale
c = collada.Collada(filename)
print(f"Units: {c.assetInfo.unitname} = {c.assetInfo.unitmeter} m")
scale = c.assetInfo.unitmeter

# Get the convex hull of the 2D points
footprint_pts = [[p[0]*scale, p[1]*scale] for p in mesh.convex_hull.vertices]
hull = scipy.spatial.ConvexHull(footprint_pts)
hull_pts = hull.points[hull.vertices, :]

# Plot
polygon = Polygon(hull_pts)
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    viz_patch = PolygonPatch(polygon)
plt.figure()
plt.gca().add_patch(viz_patch)
plt.gca().autoscale()
plt.gca().axis("equal")
plt.show(block=False)
plt.pause(0.01)

mesh.show()