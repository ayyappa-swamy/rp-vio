"""Utilies to create a box world"""

def cross(np_vec1, np_vec2):
    return np.cross(np_vec1.flatten(), np_vec2.flatten()).reshape((3, 1))

def homogenous(np_arr):
    a = np_arr.flatten()

    return np.array([a[0], a[1], a[2], 1.0]).reshape((3, 1))

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

        self.front_plane = np.vstack((front_normal, -front_normal.dot(vertices[2])))
        self.top_plane = np.vstack((top_normal, -top_normal.dot(vertices[2])))
        self.right_plane = np.vstack((right_normal, -right_normal.dot(vertices[2])))
        self.back_plane = np.vstack((back_normal, -back_normal.dot(vertices[3])))
        self.bottom_plane = np.vstack((bottom_normal, -bottom_normal.dot(vertices[6])))
        self.left_plane = np.vstack((left_normal, -left_normal.dot(vertices[0])))

    def get_sdf(self, point):
        point_ = homogenous(point).flatten()

        distance = max([
            self.front_plane.flatten().dot(point_),
            self.top_plane.flatten().dot(point_),
            self.right_plane.flatten().dot(point_),
            self.back_plane.flatten().dot(point_),
            self.bottom_plane.flatten().dot(point_),
            self.left_plane.flatten().dot(point_)
        ])
        return distance

class BoxWorld:
    boxes = []
    gt_sdfs = None
    geometries = None

    def __init__(self, vertices_msg, odometry):
        self.odometry = odometry
        self.boxes = []
        vertices = points_to_numpy(vertices_msg.points)
        self.init_boxes_from_vertices(vertices)

    def points_to_numpy(self, points):
        point_list = []
        for point in points:
            point_list.append([point.x, point.y, point.z])

        return np.array(point_list)

    def init_boxes_from_vertices(self, vertices):
        for vertex_idx in range(0, vertices.shape[0], 8):
            self.boxes.append(Box(vertices[vertex_idx:vertex_idx+8, :]))

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

