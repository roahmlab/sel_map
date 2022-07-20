import os
import rospkg
import torch
import types
import numpy as np
import torch.nn as nn
from PIL import Image
from scipy.io import loadmat
from torchvision import transforms

from mit_semseg.config import cfg
from mit_semseg.utils import colorEncode
from mit_semseg.lib.utils import as_numpy
from mit_semseg.lib.nn import async_copy_to
from mit_semseg.models import ModelBuilder, SegmentationModule


class SemanticSegmentationNetwork():
	def __init__(self, model="ade20k-resnet50dilated-ppm_deepsup.yaml", args=None, verbose=False):
		'''
		Initializes and runs the CSAIL Semantic Segmentation network for use in the
		terrain estimation mapping algoritm. Loads network weights, prepares input
		images for network, runs the segmentation network, and outputs visualizations
		to a file.

		Parameters
		------------
		args : obj, provides necessary arguements for network initialization

		Returns
		-----------
		'''

		self.verbose = verbose
		rospack = rospkg.RosPack()
		path = rospack.get_path('mit_semseg_wrapper')
		if args is None:
			args = types.SimpleNamespace()
			args.gpu = 0
			args.cfg = model
			args.opts = []
		args.cfg = os.path.join(path, os.path.join('src/mit_semseg/config/', args.cfg))
		self.args = args

		cfg.merge_from_file(args.cfg)
		cfg.merge_from_list(args.opts)
		cfg.MODEL.arch_encoder = cfg.MODEL.arch_encoder.lower()
		cfg.MODEL.arch_decoder = cfg.MODEL.arch_decoder.lower()

		# absolute paths of model weights
		cfg.MODEL.weights_encoder = os.path.join(
			path, cfg.DIR, 'encoder_' + cfg.TEST.checkpoint)
		cfg.MODEL.weights_decoder = os.path.join(
			path, cfg.DIR, 'decoder_' + cfg.TEST.checkpoint)

		if not os.path.exists(cfg.MODEL.weights_encoder) and \
			os.path.exists(cfg.MODEL.weights_decoder):
			print("Could not find saved model weights!")
			return

		self.imgSizes = cfg.DATASET.imgSizes
		self.imgMaxSize = cfg.DATASET.imgMaxSize
		self.padding_constant = cfg.DATASET.padding_constant
		self.normalize = transforms.Normalize(
			mean=[0.485, 0.456, 0.406],
			std=[0.229, 0.224, 0.225])

		torch.cuda.set_device(args.gpu)

		# Network Builders
		if self.verbose:
			print("Building encoder and decoder networks")
		net_encoder = ModelBuilder.build_encoder(
			arch=cfg.MODEL.arch_encoder,
			fc_dim=cfg.MODEL.fc_dim,
			weights=cfg.MODEL.weights_encoder)
		net_decoder = ModelBuilder.build_decoder(
			arch=cfg.MODEL.arch_decoder,
			fc_dim=cfg.MODEL.fc_dim,
			num_class=cfg.DATASET.num_class,
			weights=cfg.MODEL.weights_decoder,
			use_softmax=True)

		crit = nn.NLLLoss(ignore_index=-1)

		if self.verbose:
			print("Building final semantic segmentation network")
		self.segmentation_module = SegmentationModule(net_encoder, net_decoder, crit)
		self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
		self.segmentation_module.to(self.device)

	def prepareImage(self, image):
		'''
		Resizes an image as necessary in order to be able to act as input into
		the semantic segmentation network.

		Parameters
		------------
		image : PIL.Image, image to use as input

		Returns
		-----------
		image : PIL.Image, resized and padded image
		'''
		if isinstance(image, Image.Image):
			ori_width, ori_height = image.size
		elif isinstance(image, np.ndarray):
			image = Image.fromarray(image.astype(np.uint8))
			ori_width, ori_height = image.size

		img_resized_list = []
		for this_short_size in self.imgSizes:
			# calculate target height and width
			scale = min(this_short_size / float(min(ori_height, ori_width)),
						self.imgMaxSize / float(max(ori_height, ori_width)))
			target_height, target_width = int(ori_height * scale), int(ori_width * scale)

			# to avoid rounding in network
			target_width = self.round2nearest_multiple(target_width, self.padding_constant)
			target_height = self.round2nearest_multiple(target_height, self.padding_constant)

			# resize images
			img_resized = self.imresize(image, (target_width, target_height), interp='bilinear')

			# image transform, to torch float tensor 3xHxW
			img_resized = self.img_transform(img_resized)
			img_resized = torch.unsqueeze(img_resized, 0)
			img_resized_list.append(img_resized)

		output = dict()
		output['img_ori'] = np.array(image)
		output['img_data'] = [x.contiguous() for x in img_resized_list]
		return output

	def imresize(self, im, size, interp='bilinear'):
		if interp == 'nearest':
			resample = Image.NEAREST
		elif interp == 'bilinear':
			resample = Image.BILINEAR
		elif interp == 'bicubic':
			resample = Image.BICUBIC
		else:
			raise Exception('resample method undefined!')

		return im.resize(size, resample)

	def img_transform(self, img):
		# 0-255 to 0-1
		img = np.float32(np.array(img)) / 255.
		img = img.transpose((2, 0, 1))
		img = self.normalize(torch.from_numpy(img.copy()))
		return img

	def round2nearest_multiple(self, x, p):
		return ((x - 1) // p + 1) * p

	def runSegmentation(self, image:Image, return_numpy=True, one_hot=False):
		'''
		Passes an image through the network and returns the pixelwise terrain class
		categorical probabilities.

		Parameters
		------------
		image : PIL.Image, image to input to network

		Returns
		-----------
		array : (w,h,k) shape array, pixelwise terrain class probability scores
		'''

		feed_dict = {}
		self.segmentation_module.eval()
		image = self.prepareImage(image)

		segSize = (image['img_ori'].shape[0],
				   image['img_ori'].shape[1])
		img_resized_list = image['img_data']

		with torch.no_grad():
			scores = torch.zeros(1, cfg.DATASET.num_class, segSize[0], segSize[1])
			scores = async_copy_to(scores, self.args.gpu)

			for img in img_resized_list:
				feed_dict = image.copy()
				feed_dict['img_data'] = img
				del feed_dict['img_ori']
				feed_dict = async_copy_to(feed_dict, self.args.gpu)

				pred_tmp = self.segmentation_module(feed_dict, segSize=segSize)
				scores = scores + pred_tmp / len(cfg.DATASET.imgSizes)
			
			scores = scores.squeeze(0)
			if one_hot and return_numpy:
				scores = torch.argmax(scores, 0, keepdim=True)
			if return_numpy:
				return as_numpy(scores.cpu())
			else:
				return scores

	def visualize_result(self, img, scores, savepath):
		'''
		Saves the segmented image where each pixel is colored based on the
		most likely terrain class. All red colored classes changed to cyan.

		Parameters
		------------
		img : PIL.Image, reference image
		scores : (w,h,k) shape array, pixelwise terrain class probability scores
		savepath : str, location to save segmented image
		'''

		colors = loadmat('network/color150.mat')['colors']

		# Change all red-colored classes to cyan
		colors[15, :] = [10, 186, 181]
		colors[18, :] = [10, 186, 181]
		colors[22, :] = [10, 186, 181]
		colors[24, :] = [10, 186, 181]
		colors[28, :] = [10, 186, 181]
		colors[34, :] = [10, 186, 181]
		colors[42, :] = [10, 186, 181]
		colors[49, :] = [10, 186, 181]
		colors[52, :] = [10, 186, 181]
		colors[66, :] = [10, 186, 181]
		colors[83, :] = [10, 186, 181]
		colors[108, :] = [10, 186, 181]

		# _, pred = torch.max(scores, dim=1)
		pred = np.argmax(scores, axis=0)

		# print predictions in descending order
		pred = np.int32(pred)
		pixs = pred.size
		uniques, counts = np.unique(pred, return_counts=True)

		# colorize prediction
		pred_color = colorEncode(pred, colors).astype(np.uint8)

		# aggregate images and save
		im_vis = np.concatenate((img, pred_color), axis=1)

		# Image.fromarray(im_vis).save(savepath)
		Image.fromarray(im_vis).show()
		Image.fromarray(pred_color).save(savepath)


if __name__ == '__main__':
	image = Image.open('00001.jpg').convert('RGB')

	network = SemanticSegmentationNetwork()
	scores = network.runSegmentation(image)
