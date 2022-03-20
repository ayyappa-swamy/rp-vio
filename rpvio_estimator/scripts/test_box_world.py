"""
This file creates a sample box world to test the mapper and planner
It contains all util methods to create boxes/cuboids, get/use their vertices and plane params
"""
# Create 2d vertices, which will be expanded based on height of the cuboid
vertices_bird_view = [
    [],
    [],
    [],
    []
]

vertices = []
ground_vertices = []
roof_vertices = []

for vertex in vertices_bird_view:
    ground_vertices.append(vertices.append([vertex[0], vertex[1], 0]))
    roof_vertices.append(vertices.append([vertex[0], vertex[1], 10]))

vertices += ground_vertices
vertices += roof_vertices

