"""
A template semantic segmentation network wrapper so that sel_map can be quickly
extended with any semantic segmentation network written with Python.

Although this is written with PyTorch in mind, it can be modified to work with
Tensorflow networks as well. Those networks just would not be able to utilize direct
PyTorch tensor returns back to the remaining code. See the README.MD in this folder
for more information.
"""
import torch
from torchvision import transforms
from PIL import Image


## Useful helper functions
def padImageToBlocks(pil_image:Image, block_size:int) -> Image:
    """
    Helper function to pad the width and height to multiples of block_size

    Args:
        pil_image (Image): PIL Image object to pad
        block_size (int): The block size to ensure the Image is padded to multiples of.

    Returns:
        Image: A PIL Image object containing the image in the upper left corner,
        with the height and widths multiples of the desired block_size.
    """
    width, height = pil_image.size

    # Identify padding residuals
    width_pad = block_size - (width % block_size)
    height_pad = block_size - (height % block_size)

    # If we don't need to pad, return pil_image
    if width_pad + height_pad == 0:
        return pil_image
    
    # Otherwise pad and return
    new_image = Image.new(pil_image.mode, (width+width_pad, height+height_pad), 0)
    new_image.paste(pil_image, (0,0))
    return new_image


## Class Code
class SemsegNetworkWrapper():
    """
    Generic template for wrapping pytorch semantic segmentation networks
    """
    def __init__(self, model:str = '', args:dict = {}):
        """
        Initialize the semantic segmentation wrapper based on the model and args.

        Args:
            model (str): A string identifier to select the specific network or
                            architecture to use. Defaults to an empty string.
            args (dict): Extra arguments to pass onto the network in the form of
                            a dictionary. Defaults to an empty dictionary.
        """
        # Set the device.
        # !! Please make sure to set self.device to something !!
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # setup required torch transforms for PIL images into self.transforms
        self.transform = transforms.Compose([
            transforms.ToTensor()
        ])

        # setup the network model
        self.model = None
        # self.model = self.model.to(self.device)
        # self.model.eval()
    
    def runSegmentation(self, pil_image:Image, return_numpy:bool=True, one_hot:bool=False):
        """
        Pass the a PIL image through the segmentation network specified and return the pixelwise
        terrain class categorical probabilities. If one_hot is specified along with return_numpy,
        it will return the one-hot encoding in numpy.

        Args:
            pil_image (Image): A PIL object of the image to be segmented.
            return_numpy (bool, optional): Whether or not to return a numpy ndarray. Defaults to True.
            one_hot (bool, optional): Whether or not to perform one_hot encoding on the GPU. If
                                      return_numpy is False, then this has no effect. Defaults to False.

        Returns:
            3-dimensional ndarr or tensor: If return_numpy is specified as true, a 3-d numpy ndarr
            is returned with shape (num_classes, height, width) if one_hot is not specified, and
            (1, height, width) if one_hot is specified, where the first dimension is used for the
            one-hot classification. If return_numpy is false, then a tensor of shape (num_classes,
            height, width) is returned regardless of what one_hot is specified as.
        """
        # Store the image dimensions to truncate if needed
        width, height = pil_image.size
        
        ## Pad the image if needed
        ## The below will pad the width and height to multiples of 32.
        # padImageToBlocks(pil_image, 32)

        # Perform the segmentation
        img = self.transform(pil_image).unsqueeze(0).to(self.device)
        with torch.no_grad():
            # Update this section if needed for the network
            output = self.model.evaluate(img)
        
            # Process the output for the single image
            scores = output.squeeze(0)
            if one_hot and return_numpy:
                # We save the one-hot process for the camera sensor model
                # instead if we're returning the torch tensor instead.
                # This prevents unnecessary early blocking.
                scores = torch.argmax(scores, 0, keepdim=True)
        
        # Crop and return the scores, returning it as a torch tensor
        # if return_numpy is false, or as a numpy ndarray otherwise
        if return_numpy:
            return scores.data.cpu().numpy()[:,:height,:width]
        else:
            return scores[:,:height,:width]

