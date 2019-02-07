import numpy as np
import common.transformations.orientation as orient
import cv2

FULL_FRAME_SIZE = (1164, 874)
W, H = FULL_FRAME_SIZE[0], FULL_FRAME_SIZE[1]
eon_focal_length = FOCAL = 910.0

# aka 'K' aka camera_frame_from_view_frame
eon_intrinsics = np.array([
  [FOCAL,   0.,   W/2.],
  [  0.,  FOCAL,  H/2.],
  [  0.,    0.,     1.]])

# aka 'K_inv' aka view_frame_from_camera_frame
eon_intrinsics_inv = np.linalg.inv(eon_intrinsics)


# device/mesh : x->forward, y-> right, z->down
# view : x->right, y->down, z->forward
device_frame_from_view_frame = np.array([
  [ 0.,  0.,  1.],
  [ 1.,  0.,  0.],
  [ 0.,  1.,  0.]
])
view_frame_from_device_frame = device_frame_from_view_frame.T


def get_calib_from_vp(vp):
  vp_norm = normalize(vp)
  yaw_calib = np.arctan(vp_norm[0])
  pitch_calib = np.arctan(vp_norm[1]*np.cos(yaw_calib))
  # TODO should be, this but written
  # to be compatible with meshcalib and
  # get_view_frame_from_road_fram
  #pitch_calib = -np.arctan(vp_norm[1]*np.cos(yaw_calib))
  roll_calib = 0
  return roll_calib, pitch_calib, yaw_calib

# aka 'extrinsic_matrix'
# road : x->forward, y -> left, z->up
def get_view_frame_from_road_frame(roll, pitch, yaw, height):
  # TODO
  # calibration pitch is currently defined
  # opposite to pitch in device frame
  pitch = -pitch
  device_from_road = orient.rot_from_euler([roll, pitch, yaw]).dot(np.diag([1, -1, -1]))
  view_from_road = view_frame_from_device_frame.dot(device_from_road)
  return np.hstack((view_from_road, [[0], [height], [0]]))


def vp_from_ke(m):
  """
  Computes the vanishing point from the product of the intrinsic and extrinsic
  matrices C = KE.

  The vanishing point is defined as lim x->infinity C (x, 0, 0, 1).T
  """
  return (m[0, 0]/m[2,0], m[1,0]/m[2,0])

def roll_from_ke(m):
  # note: different from calibration.h/RollAnglefromKE: i think that one's just wrong
  return np.arctan2(-(m[1, 0] - m[1, 1] * m[2, 0] / m[2, 1]),
                    -(m[0, 0] - m[0, 1] * m[2, 0] / m[2, 1]))


def normalize(img_pts, intrinsics=eon_intrinsics):
  # normalizes image coordinates
  # accepts single pt or array of pts
  intrinsics_inv = np.linalg.inv(intrinsics)
  img_pts = np.array(img_pts)
  input_shape = img_pts.shape
  img_pts = np.atleast_2d(img_pts)
  img_pts = np.hstack((img_pts, np.ones((img_pts.shape[0],1))))
  img_pts_normalized = intrinsics_inv.dot(img_pts.T).T
  img_pts_normalized[(img_pts < 0).any(axis=1)] = np.nan
  return img_pts_normalized[:,:2].reshape(input_shape)


def denormalize(img_pts, intrinsics=eon_intrinsics):
  # denormalizes image coordinates
  # accepts single pt or array of pts
  img_pts = np.array(img_pts)
  input_shape = img_pts.shape
  img_pts = np.atleast_2d(img_pts)
  img_pts = np.hstack((img_pts, np.ones((img_pts.shape[0],1))))
  img_pts_denormalized = intrinsics.dot(img_pts.T).T
  img_pts_denormalized[img_pts_denormalized[:,0] > W] = np.nan
  img_pts_denormalized[img_pts_denormalized[:,0] < 0] = np.nan
  img_pts_denormalized[img_pts_denormalized[:,1] > H] = np.nan
  img_pts_denormalized[img_pts_denormalized[:,1] < 0] = np.nan
  return img_pts_denormalized[:,:2].reshape(input_shape)


def device_from_ecef(pos_ecef, orientation_ecef, pt_ecef):
  # device from ecef frame
  # device frame is x -> forward, y-> right, z -> down
  # accepts single pt or array of pts
  input_shape = pt_ecef.shape
  pt_ecef = np.atleast_2d(pt_ecef)
  ecef_from_device_rot = orient.rotations_from_quats(orientation_ecef)
  device_from_ecef_rot = ecef_from_device_rot.T
  pt_ecef_rel = pt_ecef - pos_ecef
  pt_device = np.einsum('jk,ik->ij', device_from_ecef_rot, pt_ecef_rel)
  return pt_device.reshape(input_shape)


def img_from_device(pt_device):
  # img coordinates from pts in device frame
  # first transforms to view frame, then to img coords
  # accepts single pt or array of pts
  input_shape = pt_device.shape
  pt_device = np.atleast_2d(pt_device)
  pt_view = np.einsum('jk,ik->ij', view_frame_from_device_frame, pt_device)

  # This function should never return negative depths
  pt_view[pt_view[:,2] < 0] = np.nan

  pt_img = pt_view/pt_view[:,2:3]
  return pt_img.reshape(input_shape)[:,:2]


def rotate_img(img, eulers, crop=None, intrinsics=eon_intrinsics):
  size = img.shape[:2]
  rot = orient.rot_from_euler(eulers)
  quadrangle = np.array([[0, 0],
                         [size[1]-1, 0],
                         [0, size[0]-1],
                         [size[1]-1, size[0]-1]], dtype=np.float32)
  quadrangle_norm = np.hstack((normalize(quadrangle, intrinsics=intrinsics), np.ones((4,1))))
  warped_quadrangle_full = np.einsum('ij, kj->ki', intrinsics.dot(rot), quadrangle_norm)
  warped_quadrangle = np.column_stack((warped_quadrangle_full[:,0]/warped_quadrangle_full[:,2],
                                       warped_quadrangle_full[:,1]/warped_quadrangle_full[:,2])).astype(np.float32)
  if crop:
    W_border = (size[1] - crop[0])/2
    H_border = (size[0] - crop[1])/2
    outside_crop = (((warped_quadrangle[:,0] < W_border) |
                     (warped_quadrangle[:,0] >= size[1] - W_border)) &
                    ((warped_quadrangle[:,1] < H_border) |
                     (warped_quadrangle[:,1] >= size[0] - H_border)))
    if not outside_crop.all():
      raise ValueError("warped image not contained inside crop")
  else:
    H_border, W_border = 0, 0
  M = cv2.getPerspectiveTransform(quadrangle, warped_quadrangle)
  img_warped = cv2.warpPerspective(img, M, size[::-1])
  return img_warped[H_border: size[0] - H_border,
                    W_border: size[1] - W_border]
