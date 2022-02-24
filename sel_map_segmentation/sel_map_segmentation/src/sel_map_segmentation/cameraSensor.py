import math
import warnings
import numpy as np
# import open3d as o3d


from PIL import Image
from numpy.matlib import repmat
# from .semsegNetwork import SemanticSegmentationNetwork
# from .FastSCNNWrapper import FastSCNNWrapper
# from .PytorchEncodingWrapper import PytorchEncodingWrapper
from .BypassWrapper import BypassWrapper
import importlib
import rospy
import torch
import time


class Pose():
	def __init__(self, location=np.array([0,0,0]), rotation=np.zeros((3,3))):
		self.location = location
		self.rotation = rotation


class CameraSensor():
	def __init__(self):
		self.poseCameraToMap = Pose()

		self.rgb = None
		self.depth = None
		self.scores = None
		
		wrapper_package 		= rospy.get_param("semseg/package", "")
		model_name 				= rospy.get_param("semseg/model", "")
		args 					= rospy.get_param("semseg/extra_args", None)
		self.labels 			= rospy.get_param("semseg/num_labels", 0)
		self.onGPU			 	= rospy.get_param("semseg/ongpu_projection", True)
		self.onehot_projection 	= rospy.get_param("semseg/onehot_projection", False)
		
		# make sure args is either something or nothing
		if args is not None and len(args) == 0:
			args = None

		if self.labels > 0:
			SemsegNetworkWrapper = importlib.import_module(wrapper_package).SemsegNetworkWrapper
			self.network = SemsegNetworkWrapper(model=model_name, args=args)
		else:
			self.network = BypassWrapper()
			self.onGPU = False

	def setPoseCameraToMap(self, poseCameraToMap):
		self.poseCameraToMap = poseCameraToMap

	def updateSensorMeasurements(self, rgb, depth):
		self.rgb = rgb
		if self.onGPU:
			self.depth = torch.from_numpy(depth).float().to(self.network.device, non_blocking=True)
		else:
			self.depth = depth

	def runSemanticSegmentation(self, vis=False, vispath=None):
		warnings.simplefilter("ignore")
		self.scores = self.network.runSegmentation(Image.fromarray(self.rgb), return_numpy=not self.onGPU, one_hot=self.onehot_projection)
		warnings.simplefilter("default")

		if vis and vispath:
			self.network.visualize_result(self.rgb, self.scores, vispath)

	def projectMeasurementsIntoSensorFrame(self, intrinsic=None, R=None, min_depth=0, max_depth=5.0):
		# (Intrinsic) K Matrix
		k = intrinsic
		if intrinsic is None:
			# Zed camera
			k = np.identity(3)
			k[0, 2] = 349.3575
			k[1, 2] = 197.7895
			k[0, 0] = k[1, 1] = 349.925
		
		# 2d pixel coordinates
		pixel_length = self.depth.shape[1] * self.depth.shape[0]
		u_coord = repmat(np.r_[self.depth.shape[1]-1:-1:-1],
						 self.depth.shape[0], 1).reshape(pixel_length)
		v_coord = repmat(np.c_[self.depth.shape[0]-1:-1:-1],
						 1, self.depth.shape[1]).reshape(pixel_length)

		self.scores = np.moveaxis(self.scores, 0, -1)

		depth = np.reshape(self.depth, pixel_length)
		scores = np.transpose(np.reshape(self.scores, (pixel_length, self.scores.shape[2])))

		# Isolate in bound pixels
		# masking this way will also remove all nan's and inf's
		mask = np.argwhere(depth <= max_depth).flatten() # boolean masking is slooooooow
		mask = mask[depth[mask] >= min_depth]
		p3d = np.ones((3,mask.shape[0]))
		p3d[0,:] = u_coord[mask]
		p3d[1,:] = v_coord[mask]

		# P = [X,Y,Z,C]
		p3d = np.dot(np.linalg.inv(k), p3d)
		p3d *= depth[mask]

		# Swap frames: x' = z, y' = -x, z' = y
		if R is None:
			R = np.array([[0,0,1],[-1,0,0],[0,1,0]])
		p3d = np.dot(R, p3d)

		# return np.transpose(p3d)
		return np.transpose(p3d), scores[:,mask].T

	def projectMeasurementsIntoSensorFrame_torch(self, intrinsic=None, R=None, min_depth=0, max_depth=5.0):
		with torch.no_grad():
			# (Intrinsic) K Matrix
			k = intrinsic
			if intrinsic is None:
				# Zed camera
				k = np.identity(3)
				k[0, 2] = 349.3575
				k[1, 2] = 197.7895
				k[0, 0] = k[1, 1] = 349.925
			k = torch.from_numpy(k).float().to(self.network.device, non_blocking=True)
			if R is None:
				R = np.array([[0,0,1],[-1,0,0],[0,1,0]])
			R = torch.from_numpy(R).float().to(self.network.device, non_blocking=True)

			# 2d pixel coordinates
			pixel_length = self.depth.shape[1] * self.depth.shape[0]
			# u_coord = repmat(np.r_[self.depth.shape[1]-1:-1:-1],
			# 				 self.depth.shape[0], 1).reshape(pixel_length)
			# v_coord = repmat(np.c_[self.depth.shape[0]-1:-1:-1],
			# 				 1, self.depth.shape[1]).reshape(pixel_length)
			# Make a row tensor
			u_coord = torch.linspace(self.depth.shape[1]-1,0,self.depth.shape[1], dtype=torch.float, device=self.network.device).unsqueeze(0)
			u_coord = u_coord.expand(self.depth.shape[0],-1).reshape(pixel_length)
			# Make a column tensor
			v_coord = torch.linspace(self.depth.shape[0]-1,0,self.depth.shape[0], dtype=torch.float, device=self.network.device).unsqueeze(1)
			v_coord = v_coord.expand(-1,self.depth.shape[1]).reshape(pixel_length)

			self.scores = torch.moveaxis(self.scores, 0, -1)

			depth = self.depth.reshape(pixel_length)
			scores = torch.transpose(torch.reshape(self.scores, (pixel_length, self.scores.shape[2])), 0, 1)

			# Isolate in bound pixels
			# masking this way will also remove all nan's and inf's
			mask = torch.nonzero(depth <= max_depth).flatten() # boolean masking is slooooooow
			mask = mask[depth[mask] >= min_depth]

			p3d = torch.ones((3,mask.shape[0]), pin_memory=True).to(self.network.device)
			p3d[0,:] = u_coord[mask]
			p3d[1,:] = v_coord[mask]

			scores = scores[:,mask]
			if self.onehot_projection:
				scores = torch.argmax(scores, 0, keepdim=True)

			# P = [X,Y,Z,C]
			p3d = torch.mm(torch.linalg.inv(k), p3d)
			p3d *= depth[mask]

			# Swap frames: x' = z, y' = -x, z' = y
			p3d = torch.mm(R, p3d)

			# return np.transpose(p3d)
			return torch.transpose(p3d, 0, 1), torch.transpose(scores, 0, 1)

	def computePointCovariance(self, point):
		# Covariance matrix of sensor model
		sigma_s = 0.001063 + (0.0007278 * point[2]) + (0.003949 * point[2]**2)

		# Covariance matrix for sensor rotation
		S_r = 0.1 * np.eye(3)

		J_s = self.sensorProjectionJacobian()
		J_r = self.sensorRotationJacobian(point)

		var_s = np.dot(np.dot(J_s, sigma_s), np.transpose(J_s))
		var_r = np.dot(np.dot(J_r, S_r), np.transpose(J_r))
		var = var_s + var_r
		return var
	
	def computePointCovariance_test(self, point):
		# Covariance matrix of sensor model
		sigma_s = 0.001063 + (0.0007278 * point[:,2]) + (0.003949 * point[:,2]**2)

		# Covariance matrix for sensor rotation
		S_r = 0.1 * np.eye(3)

		J_s = self.sensorProjectionJacobian()
		J_r = self.sensorRotationJacobian_test(point)

		var_s = np.dot(J_s, np.transpose(J_s))*sigma_s
		var_r = np.sum(np.dot(J_r, S_r)*J_r, axis=1)
		var = var_s + var_r
		return var

	def sensorProjectionJacobian(self):
		P = np.array([0,0,1])
		return np.dot(P, np.transpose(self.poseCameraToMap.rotation))

	def sensorRotationJacobian(self, positionCameraToPoint):
		skewSym = np.array([[0, -positionCameraToPoint[2], positionCameraToPoint[1]], 
		                    [positionCameraToPoint[2], 0, -positionCameraToPoint[0]], 
		                    [-positionCameraToPoint[1], positionCameraToPoint[0], 0]])
		return np.dot(self.sensorProjectionJacobian(), skewSym)
		
	def sensorRotationJacobian_test(self, positionCameraToPoint):
		ret = np.zeros((positionCameraToPoint.shape[0],3))
		J_r = self.sensorProjectionJacobian()
		ret[:,0] = J_r[1]*positionCameraToPoint[:,2] - J_r[2]*positionCameraToPoint[:,1]
		ret[:,1] = J_r[0]*-positionCameraToPoint[:,2] + J_r[2]*positionCameraToPoint[:,0]
		ret[:,2] = J_r[0]*positionCameraToPoint[:,1] - J_r[1]*positionCameraToPoint[:,0]
		return ret

	def computePointCovariance_torch(self, point):
		J_s = self.sensorProjectionJacobian()
		J_s = torch.from_numpy(J_s).float().to(self.network.device, non_blocking=True)

		# Covariance matrix of sensor model
		sigma_s = 0.001063 + (0.0007278 * point[:,2]) + (0.003949 * point[:,2]**2)

		# Covariance matrix for sensor rotation
		S_r = 0.1 * torch.eye(3, device=self.network.device)

		J_r = self.sensorRotationJacobian_torch(J_s, point)

		var_s = torch.mm(J_s.unsqueeze(0), J_s.unsqueeze(1))*sigma_s
		var_r = torch.sum(torch.mm(J_r, S_r)*J_r, dim=1)
		var = var_s + var_r
		return var.squeeze(0)
	
	def sensorRotationJacobian_torch(self, J_s, positionCameraToPoint):
		# Flip the defs
		skewSymGen = (torch.tensor([0.0,0,0,  0,0,1,  0,-1,0]).view(3,3).to(self.network.device).t(),
						torch.tensor([0.0,0,-1,  0,0,0,  1,0,0]).view(3,3).to(self.network.device).t(),
						torch.tensor([0.0,1,0,  -1,0,0,  0,0,0]).view(3,3).to(self.network.device).t())
		skewSym = torch.stack([torch.matmul(J_s, skewSymGen[0]),
								torch.matmul(J_s, skewSymGen[1]),
								torch.matmul(J_s, skewSymGen[2])])
		# skewSym = torch.tensor([[       0,  J_s[2], -J_s[1]], 
		#                   	    [ -J_s[2],       0,  J_s[0]], 
		#                   	    [  J_s[1], -J_s[0],       0]], device=self.network.device)
		return torch.mm(positionCameraToPoint, skewSym)

	def getProjectedPointCloudWithLabels(self, intrinsic=None, R=None, min_depth=0, max_depth=5.0):
		self.runSemanticSegmentation()
		if self.onGPU:
			with torch.no_grad():
				pc, scores = self.projectMeasurementsIntoSensorFrame_torch(intrinsic=intrinsic, R=R, min_depth=min_depth, max_depth=max_depth)
				# Reduce unneccesary memory copies
				var = self.computePointCovariance_torch(pc)
				point_cloud_with_labels = np.column_stack((pc.data.cpu().numpy(), var.data.cpu().numpy(), scores.data.cpu().numpy()))
				# pc = pc.data.cpu().numpy()
				# scores = scores.data.cpu().numpy()
				# point_cloud_with_labels = np.column_stack((pc, self.computePointCovariance_test(pc), scores))
		else:
			pc, scores = self.projectMeasurementsIntoSensorFrame(intrinsic=intrinsic, R=R, min_depth=min_depth, max_depth=max_depth)
			# Reduce unneccesary memory copies
			point_cloud_with_labels = np.column_stack((pc, self.computePointCovariance_test(pc), scores))

		return point_cloud_with_labels

	def getProjectedPointCloudWithoutLabels(self):
		pc = self.projectMeasurementsIntoSensorFrame(semseg=False)

		points_with_var = np.zeros((pc.shape[0], 4))
		points_with_var[:,:3] = pc[:,:3]
		for idx, point in enumerate(pc):
			points_with_var[idx, 4] = self.computePointCovariance(point[:3])

		point_cloud_without_labels = np.array((points_with_var))

		return point_cloud_without_labels
