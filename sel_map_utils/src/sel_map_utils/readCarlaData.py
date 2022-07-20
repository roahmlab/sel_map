import io
import os
import re
import math
import yaml
import numpy as np
import open3d as o3d
from PIL import Image
from numpy.matlib import repmat
from scipy.spatial.transform import Rotation as R


LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses


def retreiveCarlaPointCloud(filename):
	pcd = o3d.io.read_point_cloud(filename)
	points = np.asarray(pcd.points)
	mask = points[:,2] < 1 

	processed_points = points[mask]
	processed_points[:,2] = processed_points[:,2] + 1.8

	return 	processed_points

def carlaPointCloudFromDepth(depth, max_depth=0.9, logscale=True):
	if logscale:
		far = 100.0  # max depth in meters.
	else:
		# far = 100000.0  # max depth in meters
		far = 10.0  # max depth in meters

	# (Intrinsic) K Matrix
	k = np.zeros((3,3))
	k[:3,:3] = np.identity(3)
	k[0, 2] = 400
	k[1, 2] = 300
	k[0, 0] = k[1, 1] = 800 / (2 * math.tan(90 * 3.14159 / 360))

	# 2d pixel coordinates
	pixel_length = depth.shape[1] * depth.shape[0]
	u_coord = repmat(np.r_[depth.shape[1]-1:-1:-1],
					 depth.shape[0], 1).reshape(pixel_length)
	v_coord = repmat(np.c_[depth.shape[0]-1:-1:-1],
					 1, depth.shape[1]).reshape(pixel_length)

	depth = np.reshape(depth, pixel_length)

	# Search for pixels where the depth is greater than max_depth to
	# delete them
	max_depth_indexes = np.where(depth > max_depth)
	depth = np.delete(depth, max_depth_indexes)
	u_coord = np.expand_dims(np.delete(u_coord, max_depth_indexes), axis=0)
	v_coord = np.expand_dims(np.delete(v_coord, max_depth_indexes), axis=0)

	# pd2 = [u,v,1,c]
	p2d = np.vstack((u_coord, v_coord, np.ones_like(u_coord)))

	# P = [X,Y,Z,C]
	p3d = np.dot(np.linalg.inv(k), p2d)
	p3d[:3,:] *= depth * far

	# Swap frames: x' = z, y' = -x, z' = y
	R = np.array([[0,0,1],[-1,0,0],[0,1,0]])
	pc = np.dot(R, p3d[:3,:])

	# Depth camera from Carla gives poor results for points close by,
	# Use the following to accoutn for this poor resolution and re-
	# compute the point HEIGHTS only
	pc[2,:] += abs(np.min(pc[2,:]))
	weight = 0.1 * pc[2,:] / (0.1 * pc[2,:] + 1)
	pc[2,:] = weight * pc[2,:]
	p3d[:3,:] = pc

	return p3d

def carlaPointCloudClassFromDepth(depthscores, max_depth=0.9, logscale=True):
	if logscale:
		far = 100.0  # max depth in meters.
	else:
		# far = 100000.0  # max depth in meters
		far = 10.0  # max depth in meters
	depth = depthscores[:,:,0]
	scores = depthscores[:,:,1:]

	# (Intrinsic) K Matrix
	k = np.zeros((153,153))
	k[:3,:3] = np.identity(3)
	k[0, 2] = 400
	k[1, 2] = 300
	k[0, 0] = k[1, 1] = 800 / (2 * math.tan(90 * 3.14159 / 360))
	k[3:,3:] = np.identity(150)

	# 2d pixel coordinates
	pixel_length = depth.shape[1] * depth.shape[0]
	u_coord = repmat(np.r_[depth.shape[1]-1:-1:-1],
					 depth.shape[0], 1).reshape(pixel_length)
	v_coord = repmat(np.c_[depth.shape[0]-1:-1:-1],
					 1, depth.shape[1]).reshape(pixel_length)

	depth = np.reshape(depth, pixel_length)
	scores = np.transpose(np.reshape(scores, (pixel_length, 150)))

	# Search for pixels where the depth is greater than max_depth to
	# delete them
	max_depth_indexes = np.where(depth > max_depth)
	depth = np.delete(depth, max_depth_indexes)
	u_coord = np.expand_dims(np.delete(u_coord, max_depth_indexes), axis=0)
	v_coord = np.expand_dims(np.delete(v_coord, max_depth_indexes), axis=0)
	scores = np.delete(scores, max_depth_indexes, axis=1)

	# pd2 = [u,v,1,c]
	p2d = np.vstack((u_coord, v_coord, np.ones_like(u_coord), scores))

	# P = [X,Y,Z,C]
	p3d = np.dot(np.linalg.inv(k), p2d)
	p3d[:3,:] *= depth * far

	# Swap frames: x' = z, y' = -x, z' = y
	R = np.array([[0,0,1],[-1,0,0],[0,1,0]])
	pc = np.dot(R, p3d[:3,:])

	# Depth camera from Carla gives poor results for points close by,
	# Use the following to accoutn for this poor resolution and re-
	# compute the point HEIGHTS only
	pc[2,:] += abs(np.min(pc[2,:]))
	weight = 0.1 * pc[2,:] / (0.1 * pc[2,:] + 1)
	pc[2,:] = weight * pc[2,:]
	p3d[:3,:] = pc

	return p3d


class Pose():
	def __init__(self, location, rotation):
		self.location = location
		self.rotation = rotation


class CarlaDataLoader():
	def __init__(self, directory, pointheightlimit=3.0, lidarheight=1.7,
				 semantic=False, lidar=True, pose=True, groundtruth=False, logdepth=False):

		self.pose = pose
		self.lidar = lidar
		self.semantic = semantic
		self.groundtruth = groundtruth
		self.logdepth = logdepth

		if self.pose:
			self.posedir = os.path.join(directory, 'carla-pose')
			self.posefiles = self.getFilesInDir(self.posedir)

		if not self.semantic:
			self.rgbimagesdir = os.path.join(directory, 'carla-image')
			self.depthimagesdir = os.path.join(directory, 'carla-depth')
			self.rgbimagefiles = self.getFilesInDir(self.rgbimagesdir)
			self.depthimagefiles = self.getFilesInDir(self.depthimagesdir)

			if self.lidar:
				self.pointclouddir = os.path.join(directory, 'carla-point-cloud')
				self.pointcloudfiles = self.getFilesInDir(self.pointclouddir)

		else:
			self.semanticlidardir = os.path.join(directory, 'carla-semantic-point-cloud')
			self.semanticlidarfiles = self.getFilesInDir(self.semanticlidardir)

		if self.groundtruth:
			self.groundtruthdir = os.path.join(directory, 'carla-semseg-gt')
			self.groundtruthfiles = self.getFilesInDir(self.groundtruthdir)

		self.framedict = self.computeFrameDictionary()
		self.initframe = min(self.framedict.keys())
		self.currentframe = self.initframe
		self.finalframe = max(self.framedict.keys())

		self.pointheightlimit = pointheightlimit
		self.lidarheight = lidarheight

	def getFilesInDir(self, directory):
		f = []
		if os.path.isdir(directory):
			for (_, _, filenames) in os.walk(directory):
				for filename in filenames:
					f.append(os.path.join(directory, filename))
				return f
		else:
			print('Directory ', directory, ' does not exist!')
			return f

	def computeFrameDictionary(self):
		keys = set()

		if self.pose:
			self.poseframelist = [int(re.sub('\.txt$', '', file.split("/")[-1])) for file in self.posefiles]
			self.poseframelist.sort()
			keys.update(self.poseframelist)

		if not self.semantic:
			self.rgbframelist = [int(re.sub('\.png$', '', file.split("/")[-1])) for file in self.rgbimagefiles]
			self.depthframelist = [int(re.sub('\.png$', '', file.split("/")[-1])) for file in self.depthimagefiles]
			self.rgbframelist.sort()
			self.depthframelist.sort()
			keys.update(self.rgbframelist)
			keys.update(self.depthframelist)
			
			if self.lidar:
				self.pointcloudframelist = [int(re.sub('\.ply$', '', file.split("/")[-1])) for file in self.pointcloudfiles]
				self.pointcloudframelist.sort()			
				keys.update(self.pointcloudframelist)

		else:
			self.semanticlidarframelist = [int(re.sub('\.ply$', '', file.split("/")[-1])) for file in self.semanticlidarfiles]
			self.semanticlidarframelist.sort()

			keys.update(self.semanticlidarframelist)

		if self.groundtruth:
			self.groundtruthlist = [int(re.sub('\.png$', '', file.split("/")[-1])) for file in self.groundtruthfiles]
			self.groundtruthlist.sort()
			keys.update(self.groundtruthlist)

		framedict = {}
		for key in keys:
			listOfDataWithFrame = []
			if not self.groundtruth:
				if not self.semantic:
					if self.pose:
						if self.lidar:
							for datalist in [self.posefiles, self.rgbimagefiles, self.depthimagefiles, self.pointcloudfiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
						else:
							for datalist in [self.posefiles, self.rgbimagefiles, self.depthimagefiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
					else:
						if self.lidar:
							for datalist in [self.rgbimagefiles, self.depthimagefiles, self.pointcloudfiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
						else:
							for datalist in [self.rgbimagefiles, self.depthimagefiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
			
				else:
					for datalist in [self.posefiles, self.semanticlidarfiles]:
						listOfDataWithFrame.append([file for file in datalist if str(key) in file])

			if self.groundtruth:
				if not self.semantic:
					if self.pose:
						if self.lidar:
							for datalist in [self.posefiles, self.rgbimagefiles, self.depthimagefiles, self.pointcloudfiles, self.groundtruthfiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
						else:
							for datalist in [self.posefiles, self.rgbimagefiles, self.depthimagefiles, self.groundtruthfiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
					else:
						if self.lidar:
							for datalist in [self.rgbimagefiles, self.depthimagefiles, self.pointcloudfiles, self.groundtruthfiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
						else:
							for datalist in [self.rgbimagefiles, self.depthimagefiles, self.groundtruthfiles]:
								listOfDataWithFrame.append([file for file in datalist if '%06d' % key in file])
			
				else:
					for datalist in [self.posefiles, self.semanticlidarfiles, self.groundtruthfiles]:
						listOfDataWithFrame.append([file for file in datalist if str(key) in file])


			flat_list = [item for sublist in listOfDataWithFrame for item in sublist]
			framedict[key] = flat_list

		return framedict

	def getPose(self, filename):
		if os.path.isfile(filename):
			if filename[-4:] == 'yaml':
				with open(filename, 'r') as f:
				    input(yaml.safe_load(f))
			with open(filename) as f:
				lines = f.readlines()

			location = [float(coord) for coord in lines[0].replace('[', '').replace(']','').replace('\n','').split(' ') if coord != '']
			rotation = [float(rot) for rot in lines[1].replace('[', '').replace(']','').replace('\n','').split(' ') if rot != '']

			rot_mat = R.from_euler('z', -rotation[2], degrees=True)
			# return Pose(np.array(location), np.array(rotation))
			return Pose(np.array(location), rot_mat.as_matrix())

		else:
			return None

	def getRGBImage(self, filename):
		try:
			img = Image.open(filename).convert('RGB')
			return np.asarray(img)

		except:
			return None

	def getDepthImage(self, filename):
		try:
			# return np.load(filename)
			img = Image.open(filename).convert('RGB')
			img = np.asarray(img).astype(np.float32)
			img = img[:,:,::-1]
			normalized_depth = np.dot(img, [65536.0, 256.0, 1.0])
			normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)

			return normalized_depth
			# img = Image.open(filename).convert('I')
			# img = np.asarray(img)
			# return img

		except:
			return None

	def getLogDepthImage(self, filename):
		try:
			# Load the image and isolate just a single layer for a carla logDepth Image
			img = Image.open(filename)
			img = np.asarray(img).astype(np.float32)

			# convert to float and undo log conversion
			normalized_depth = np.exp((img / 255.0 - 1) * 1.45)*10*1.45
			# The 1.45 and 10 are magic numbers that I can't figure out the source of, and it's possibly not carla...
			# This is otherwise derived from the colorConverter class of the Carla sim
			# of which a more readable python equivalent version can be found here:
			# https://github.com/carla-simulator/driving-benchmarks/blob/master/version084/carla/image_converter.py
			# normalized_depth = np.exp((img / 255.0 - 1) * np.log(300))


			# remove clipped bounds
			normalized_depth[img >= 255] = np.Inf
			normalized_depth[img <= 4] = -np.Inf
			
			# normalized_depth = np.dot(img, [65536.0, 256.0, 1.0])
			# normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)

			return normalized_depth
			# img = Image.open(filename).convert('I')
			# img = np.asarray(img)
			# return img

		except:
			return None

	def getGroundTruthImage(self, filename):
		try:
			img = Image.open(filename).convert('RGB')
			colors = np.asarray(img)

			for idx, label in enumerate(LABEL_COLORS):
				indices = np.where(np.all(colors == np.array(label * 255.0), axis=-1))
				labels = np.transpose(np.vstack((indices[0], indices[1])))
				colors[labels[:,0], labels[:,1],:] = np.array([idx, idx, idx])

			return colors[:,:,0]

		except:
			return None

	def getPointCloud(self, filename):
		if os.path.isfile(filename):
			pcd = o3d.io.read_point_cloud(filename)

			if not self.semantic:
				points = np.asarray(pcd.points)
			else:
				points = np.hstack((np.asarray(pcd.points), np.asarray(pcd.colors)))

			mask = points[:,2] < self.pointheightlimit

			processed_points = points[mask]
			processed_points[:,2] = processed_points[:,2] + self.lidarheight

			# Remove the vehicle from point cloud
			car_ll = np.array([-2.85, -1])
			car_ur = np.array([2.75, 1])
			car_idx = np.all(np.logical_and(car_ll <= processed_points[:,:2], processed_points[:,:2] <= car_ur), axis=1)
			processed_points = processed_points[np.logical_not(car_idx)]

			return 	processed_points

		else:
			return None

	def nextSetOfData(self):
		if self.currentframe in self.framedict:
			files = self.framedict[self.currentframe]
		else:
			self.currentframe += 1
			return None, None, None, None

		if self.pose:
			posefile = [file for file in files if self.posedir in file]

			if posefile:
					pose = self.getPose(posefile[0])
			else:
				pose = None
		else:
			pose = None

		if not self.groundtruth:
			if not self.semantic:
				rgbfile = [file for file in files if self.rgbimagesdir in file]
				depthfile = [file for file in files if self.depthimagesdir in file]

				if rgbfile:
					rgbimage = self.getRGBImage(rgbfile[0])
				else:
					rgbimage = None

				if depthfile:
					if self.logdepth:
						depthimage = self.getLogDepthImage(depthfile[0])
					else:
						depthimage = self.getDepthImage(depthfile[0])
				else:
					depthimage = None

				if self.lidar:
					pointcloudfile = [file for file in files if self.pointclouddir in file]
					if pointcloudfile:
						pointcloud = self.getPointCloud(pointcloudfile[0])
					else:
						pointcloud = None

					self.currentframe = self.currentframe + 1
					return pose, rgbimage, depthimage, pointcloud

				else:
					self.currentframe = self.currentframe + 1
					return pose, rgbimage, depthimage, None
			
			else:
				semanticfile = [file for file in files if self.semanticlidardir in file]

				if semanticfile:
					points = self.getPointCloud(semanticfile[0])

					coords = points[:,:3]
					colors = points[:,3:]
					terrain  = np.zeros((coords.shape[0], len(LABEL_COLORS)))
					terrain = np.zeros((coords.shape[0], 1))
					for idx, color in enumerate(points[:,3:]):
						label = np.where((LABEL_COLORS == color).all(axis=1))[0][0]
						terrain[idx] = label
					points = np.hstack((coords, terrain))

				else:
					points = None

				self.currentframe += 1
				return pose, None, None, points

		else:
			if not self.semantic:
				rgbfile = [file for file in files if self.rgbimagesdir in file]
				depthfile = [file for file in files if self.depthimagesdir in file]
				groundtruthfile = [file for file in files if self.groundtruthdir in file]

				if rgbfile:
					rgbimage = self.getRGBImage(rgbfile[0])
				else:
					rgbimage = None

				if depthfile:
					depthimage = self.getDepthImage(depthfile[0])
				else:
					depthimage = None

				if groundtruthfile:
					groundtruthimage = self.getGroundTruthImage(groundtruthfile[0])
				else:
					groundtruthimage = None

				if self.lidar:
					pointcloudfile = [file for file in files if self.pointclouddir in file]
					if pointcloudfile:
						pointcloud = self.getPointCloud(pointcloudfile[0])
					else:
						pointcloud = None

					self.currentframe = self.currentframe + 1
					return pose, rgbimage, depthimage, pointcloud, groundtruthimage

				else:
					self.currentframe = self.currentframe + 1
					return pose, rgbimage, depthimage, None, groundtruthimage
			
			else:
				semanticfile = [file for file in files if self.semanticlidardir in file]

				if semanticfile:
					points = self.getPointCloud(semanticfile[0])

					coords = points[:,:3]
					colors = points[:,3:]
					terrain  = np.zeros((coords.shape[0], len(LABEL_COLORS)))
					terrain = np.zeros((coords.shape[0], 1))
					for idx, color in enumerate(points[:,3:]):
						label = np.where((LABEL_COLORS == color).all(axis=1))[0][0]
						terrain[idx] = label
					points = np.hstack((coords, terrain))

				else:
					points = None

				self.currentframe += 1
				return pose, None, None, points

if __name__ == '__main__':
	item = CarlaDataLoader('dataset')

	for i in range(len(item.framedict)):
		print(item.currentframe)
		item.nextSetOfData()