import os
import torch
import rospkg

from torchvision import transforms
from PIL import Image
import importlib

# so it can just be a git clone
# from .models.fast_scnn import get_fast_scnn
# get_fast_scnn = importlib.import_module("Fast-SCNN-pytorch.models.fast_scnn").get_fast_scnn
FastSCNN = importlib.import_module("fast_scnn.models.fast_scnn").FastSCNN


class FastSCNNWrapper():
    def __init__(self, model='citys', args=None, map_cpu=False):
        # select the device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # image transformget_color_pallete
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

        # Get the fast-SCNN model
        # self.model = get_fast_scnn('citys', pretrained=True, root='Fast-SCNN-pytorch/weights', map_cpu=map_cpu).to(device)
        acronyms = {
            'pascal_voc': 'voc',
            'pascal_aug': 'voc',
            'ade20k': 'ade',
            'coco': 'coco',
            'citys': 'citys',
        }
        dataset = 'citys'
        datasets = importlib.import_module("fast_scnn.data_loader").datasets
        rospack = rospkg.RosPack()
        path = rospack.get_path('sel_map_segmentation')
        self.model = FastSCNN(datasets[dataset].NUM_CLASS)
        if(map_cpu):
            self.model.load_state_dict(torch.load(os.path.join(path, 'ckpt/fast_scnn_git', 'fast_scnn_%s.pth' % acronyms[dataset]), map_location='cpu'))
        else:
            self.model.load_state_dict(torch.load(os.path.join(path, 'ckpt/fast_scnn_git', 'fast_scnn_%s.pth' % acronyms[dataset])))
        self.model.to(self.device)

        self.model.eval()
    
    def runSegmentation(self, pil_image, return_numpy=True, one_hot=False):
        # Pad the image if needed
        width, height = pil_image.size
        width_pad = 32 - (width % 32)
        height_pad = 32 - (height % 32)
        if width_pad + height_pad > 0:
            input_image = Image.new(pil_image.mode, (width+width_pad, height+height_pad), 0)
            input_image.paste(pil_image, (0,0))
            input_image = self.transform(input_image).unsqueeze(0).to(self.device)
        else:
            input_image = self.transform(pil_image).unsqueeze(0).to(self.device)
        
        # Perform the segmentation
        with torch.no_grad():
            outputs = self.model(input_image)
        
            # Crop and return the scores
            scores = outputs[0].squeeze(0)
            if one_hot and return_numpy:
                scores = torch.argmax(scores, 0, keepdim=True)
        if return_numpy:
            return scores.cpu().numpy()[:,:height,:width]
        else:
            return scores[:,:height,:width]

