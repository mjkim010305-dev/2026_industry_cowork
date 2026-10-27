# Obstacle on the planned path, seen by the local costmap (v1: costmap cells).
# All geometry is in header.frame_id (= "map"), meters.
# Only the face visible to the lidar/costmap is observed. The BACK EXTENT
# (depth behind the face) is UNOBSERVABLE and therefore unknown.
std_msgs/Header header          # stamp = computation time, frame_id = map
bool present                    # false = no obstacle on the path (other fields undefined)
string source                   # geometry source, "costmap_cells" in v1
geometry_msgs/Point centroid    # mean of the visible cells [m] (NOT the object center)
geometry_msgs/Point face_start  # front-face line endpoint [m] (PCA main-axis projection min)
geometry_msgs/Point face_end    # front-face line endpoint [m] (PCA main-axis projection max)
geometry_msgs/Point[] face      # front-face cell centers [m], ordered along the main axis (<= 32 points)
float32 width_m                 # face width [m] (endpoint distance + resolution)
geometry_msgs/Vector3 normal    # unit face normal (map frame), pointing toward the robot
bool normal_degenerate          # true: line fit not possible, normal = unit(centroid -> robot)
float32 along_m                 # path distance from robot to the nearest obstacle cell [m]
float32 lateral_m               # minimum lateral distance of the cluster to the path polyline [m]
uint32 n_cells                  # number of costmap cells in the cluster
uint8 sweep_count               # times this object was already swept in this goal (0 = not a tracked object)
