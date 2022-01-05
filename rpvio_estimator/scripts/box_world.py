"""Utilies to create a box world"""

def cross(np_vec1, np_vec2):
    return np.cross(np_vec1.flatten(), np_vec2.flatten()).reshape((3, 1))

class Box:
    def __init__(self, vertices):
        self.vertices = vertices
        self.front_plane = np.zeros((4, 1))
        self.right_plane = np.zeros((4, 1))
        self.back_plane = np.zeros((4, 1))
        self.left_plane = np.zeros((4, 1))
        self.top_plane = np.zeros((4, 1))
        self.bottom_plane = np.zeros((4, 1))

    def init_from_vertices(self):
        edge20 = vertices[0] - vertices[2]
        edge26 = vertices[6] - vertices[2]
        edge23 = vertices[3] - vertices[2]

        front_normal = cross(edge20, edge26)
        top_normal = cross(edge23, edge20)
        right_normal = cross(edge26, edge23)
        back_normal = -front_normal
        bottom_normal = -top_normal
        left_normal = -right_normal

        front_plane = np.vstack((front_normal, -front_normal.dot(vertices[2])))
        top_plane = np.vstack((top_normal, -top_normal.dot(vertices[2])))
        right_plane = np.vstack((right_normal, -right_normal.dot(vertices[2])))
        back_plane = np.vstack((back_normal, -back_normal.dot(vertices[3])))
        bottom_plane = np.vstack((bottom_normal, -bottom_normal.dot(vertices[6])))
        left_plane = np.vstack((left_normal, -left_normal.dot(vertices[0])))

class BoxWorld:
    boxes = []
    gt_sdfs = None
    geometries = None

    def show(self):
        if self.geometries is None:
            self.geometries = []

            # Add a coordinate frame
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
            self.geometries.append(coord_frame)

            # Add boxes
            self.geometries += [box.mesh for box in self.boxes]

        o3d.visualization.draw_geometries(self.geometries)

    def create_boxes(self, dims, translations):
        # Construct all boxes
        for dim, translation in zip(dims, translations):
            box = Box(dim.flatten())
            box.mesh.translate(translation)

            self.boxes.append(box)

    def is_point_inside(self, point):
        min_sdf = 10000
        for box in self.boxes:
            min_sdf = min(box.get_sdf(point), min_sdf)

        if min_sdf <= 2:
            return True
        else:
            return False


    def is_colliding_trajectory(self, x_s, y_s, z_s):
        pts = np.vstack((x_s.flatten(), y_s.flatten(), z_s.flatten())).T
        
        for pt in pts:
            if self.is_point_inside(pt):
                return True

        return False
    
    def get_plane_params(self):
        params = np.zeros((0, 4))
        
        for box in self.boxes:
            params = np.vstack((params, box.get_face_planes()))
            
        return params
