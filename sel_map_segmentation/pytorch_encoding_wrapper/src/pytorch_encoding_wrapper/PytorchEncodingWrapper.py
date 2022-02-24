import os
import torch
import rospkg

from torchvision import transforms
from PIL import Image
import encoding


class PytorchEncodingWrapper():
    def __init__(self, model='Encnet_ResNet50s_PContext', args=None):
        # select the device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([.485, .456, .406], [.229, .224, .225])])

        # Get the model
        self.model = encoding.models.get_model(model, pretrained=True).to(self.device)
        self.model.eval()
    
    def runSegmentation(self, pil_image, return_numpy=True, one_hot=False):
#        # Pad the image if needed
        width, height = pil_image.size
#        width_pad = 32 - (width % 32)
#        height_pad = 32 - (height % 32)
#        if width_pad + height_pad > 0:
#            input_image = Image.new(pil_image.mode, (width+width_pad, height+height_pad), 0)
#            input_image.paste(pil_image, (0,0))
#            input_image = self.transform(input_image).unsqueeze(0).to(self.device)
#        else:
#            input_image = self.transform(pil_image).unsqueeze(0).to(self.device)
#        
        # Perform the segmentation
        img = self.transform(pil_image).cuda().unsqueeze(0)
        with torch.no_grad():
            output = self.model.evaluate(img)
#        predict = torch.max(output, 1)[1].cpu().numpy() + 1
        
        # Crop and return the scores
            scores = output.squeeze(0)
            if one_hot and return_numpy:
                scores = torch.argmax(scores, 0, keepdim=True)
        if return_numpy:
            return scores.data.cpu().numpy()[:,:height,:width]
        else:
            return scores[:,:height,:width]

