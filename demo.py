import glob
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# Path to folder which contains subfolders for depth, camera, 
# rgb and pose in world frame (here its end-effector pose)
OUTPUT_DIR = "/home/dennisushi/output/1"
folder = OUTPUT_DIR

# Get data at interaction start
cam_info = np.load(f"{folder}/camera/{1}.npy", allow_pickle=True).item()
ee_pose = np.load(f"{folder}/end_effector/{1}.npy", allow_pickle=True)
depth = np.load(f"{folder}/depth/{1}.npy")
rgb = np.asarray(Image.open(f"{folder}/rgb/{1}.png"))

# Select some UV pixels - change to desired pixels via mask.
V,U = np.where(np.ones(depth.shape))
# Number of samples from the selected pixels, in order to reduce
# computation
n_samples = 400 

# In isaac sim, the saved "depth" is by default inverse depth.
# thus we reverse it
depth = 1/(1e-20+depth[:,:,0])


def plot_util(im, *, title=None):
    if type(im)==np.ndarray:
        plt.imshow(im)
        if title is not None:
            plt.title(title)
        plt.axis("off")
        plt.show()
        
def get_dist_to_point(points, query_point):
    diff = points - query_point 
    dist = (diff**2).sum(-1)**0.5
    return dist

def uvd2xyz(U,V,D,width,height,z_near,z_far,VP):
    """ Transforms points from 2D image UV to 3D world XYZ
    UV - image coordinates, uint
    D  - depth at UV
    width,height - image shape
    N,F - near,far clipping planes of pinhole camera
    VP  - ViewProjection matrix
    """
    # UVD in image frame. 
    # UV are normalized, D is depth at UV
    U_scaled = U / width
    V_scaled = V / height
    # UVD in camera frame (range of -1 to 1)
    U_scaled = U_scaled*2 - 1
    V_scaled = V_scaled*2 - 1
    # UVD -> XYZ 
    W = -D
    Z = ( (W*z_far/(z_far-z_near)) + z_near*z_far/(z_far-z_near) )/(W)
    xyz = np.array([U_scaled, V_scaled, Z, np.ones(U_scaled.shape)]) * W
    # Transpose and multiply by VPinv, to get XYZ positions (N,3)
    xyz = np.dot(xyz.T, np.linalg.inv(VP))
    return xyz[...,:3]

def xyz2uvd(XYZ, W,H, VP):
    """ Transforms points from 3D world XYZ to 2D image UV
    XYZ - world coordinates, float
    W,H - image size
    VP  - ViewProjection matrix
    """
    homo = np.pad(XYZ, ((0, 0), (0, 1)), constant_values=1.0)
    tf_points = np.dot(homo, VP)
    tf_points = tf_points / (tf_points[..., -1:])
    tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
    U = (tf_points[...,0] * W).astype(np.int)
    V = (tf_points[...,1] * H).astype(np.int)
    return U,V

### Get object points in 3D, at unoccluded frame
sample_ids = np.random.choice(len(V), n_samples, replace=False)

V_sample = V[sample_ids]
U_sample = U[sample_ids]
D_sample = depth[V_sample,U_sample]
N = cam_info['clipping_range'][0]
F = cam_info['clipping_range'][1]
VP = cam_info['view_projection_matrix']
height, width = depth.shape[:2]
XYZ = uvd2xyz(U_sample,V_sample,D_sample,
              width,height,N,F,VP)

# Select point/s closest to EE
ee_pos = ee_pose[:3,3]

print ("In isaac sim 2021, this distance is in cm, in 2022 - meters. Change when upgrading!")
U_ee,V_ee = xyz2uvd(ee_pos[None], width,height, VP)

plt.imshow(rgb)
plt.scatter(U_ee,V_ee)
plt.show()
