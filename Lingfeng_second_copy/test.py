import numpy as np
from scipy.spatial import Voronoi
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, box

def voronoi_finite_polygons_2d(vor, radius=1000):
    """Construct finite Voronoi polygons from a set of points."""
    center = vor.points.mean(axis=0)
    polygons = []
    for pointidx, regionidx in enumerate(vor.point_region):
        vertices = vor.regions[regionidx]
        if all(v >= 0 for v in vertices):
            polygons.append(Polygon(vor.vertices[vertices]))
        else:
            region = []
            for vidx in vertices:
                if vidx >= 0:
                    region.append(vor.vertices[vidx])
                else:
                    # Find the Voronoi ridge that ends at this vertex
                    ridge = [ridge for ridge in vor.ridge_vertices if vidx in ridge]
                    if len(ridge) == 0:
                        continue
                    ridge = ridge[0]
                    # Find the point that is not at infinity
                    p = ridge[0] if ridge[0] >= 0 else ridge[1]
                    # Create a far point in the same direction as the finite endpoint to the center
                    t = vor.vertices[p] - center
                    far_point = vor.vertices[p] + t * radius / np.linalg.norm(t)
                    region.append(far_point)
            if region:
                polygons.append(Polygon(region))
    return polygons

def generate_voronoi_diagram(width, height, points):
    vor = Voronoi(points)
    regions = voronoi_finite_polygons_2d(vor)
    fig, ax = plt.subplots()
    bounding_box = box(0, 0, width, height)
    for region in regions:
        region = region.intersection(bounding_box)
        x, y = region.exterior.xy
        ax.fill(x, y, alpha=0.5, fc='red', ec='black')
    plt.xlim(0, width)
    plt.ylim(0, height)
    plt.show()

# Example usage
points = np.random.rand(10, 2) * [100, 100]
generate_voronoi_diagram(100, 100, points)


