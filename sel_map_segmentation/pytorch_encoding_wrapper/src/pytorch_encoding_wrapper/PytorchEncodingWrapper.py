import os
import torch
import rospkg

from torchvision import transforms
from PIL import Image
import encoding


class PytorchEncodingWrapper():
    def __init__(self, model='Encnet_ResNet50s_PContext', args=None):
        # select the device
        # PyTorch encoding doesn't work on the CPU!
        self.device = torch.device("cuda")
        
        # Convert the PIL image to a tensor and normalize it for the network
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([.485, .456, .406], [.229, .224, .225])])

        # Get the model
        self.model = encoding.models.get_model(model, pretrained=True).to(self.device)
        self.model.eval()
    
    def runSegmentation(self, pil_image, return_numpy=True, one_hot=False):
        # Pad the image if needed
        width, height = pil_image.size

        # Perform the segmentation
        img = self.transform(pil_image).cuda().unsqueeze(0)
        with torch.no_grad():
            output = self.model.evaluate(img)
        
            # Perform one-hot encoding if needed
            scores = output.squeeze(0)
            if one_hot and return_numpy:
                scores = torch.argmax(scores, 0, keepdim=True)

        # Crop and return the scores
        if return_numpy:
            return scores.data.cpu().numpy()[:,:height,:width]
        else:
            return scores[:,:height,:width]

