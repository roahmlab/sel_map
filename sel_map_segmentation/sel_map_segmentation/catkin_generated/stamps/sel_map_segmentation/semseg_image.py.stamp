import os
import numpy as np
from PIL import Image
import torch
from pathlib import Path

from sel_map_segmentation import BypassWrapper, ColorScale
import importlib
import yaml
# from randimage import get_random_image, show_array #pip package randimage https://github.com/nareto/randimage
# after testing, the above difference was generally negligible
import time
import sys
import rospy

if __name__ == '__main__':
    network_config_filename = None
    property_filename = None
    input_filename = None
    output_filename = None
    colorscale_filename = None
    # Get arguments
    argv = rospy.myargv(argv=sys.argv)
    try:
        network_config_filename = argv[1]
        property_filename = argv[2]
        input_filename = argv[3]
    except:
        print("Please provide arguments:")
        print("network_config property_file input_filename output_filename[optional] colorscale[optional]")
        print("If no colorscale is provided, the output image will use the colors from of the properties file. If a colorscale is provided, the frictional colors corresponding to the terrain will be output.")
        exit()
    try:
        output_filename = argv[4]
        colorscale_filename = argv[5]
    except:
        pass

    # Identify if the input filename is a directory
    if os.path.isdir(input_filename):
        input_list = [f for f in os.listdir(input_filename) if os.path.isfile(os.path.join(input_filename, f))]
        as_folder = True
    else:
        input_list = [input_filename]
        as_folder = False        

    # Create the output folder if we're doing as folder
    if as_folder and output_filename is not None:
        Path(output_filename).mkdir(parents=True, exist_ok=True)

    network_config = {'package': '', 'model': '', 'extra_args': [], 'num_labels': 0}
    # Open the passed in network config
    with open(network_config_filename) as file:
        cfg = yaml.load(file, Loader=yaml.FullLoader)
        network_config = cfg['semseg']
        # make sure args is either something or nothing
        if network_config['extra_args'] is not None and len(network_config['extra_args']) == 0:
            network_config['extra_args'] = None

    # Open the passed in property config
    with open(property_filename) as file:
        cfg = yaml.load(file, Loader=yaml.FullLoader)
        properties = list(cfg.values())
        # Set the colors now if we don't have a colorscale
        if colorscale_filename is None:
            colors = np.zeros([256,3], dtype=np.uint8)
            for prop in properties:
                colors[prop['index'],:] = np.array(list(prop['color'].values()))

    # Load in a colorscale
    if colorscale_filename is not None:
        with open(colorscale_filename) as file:
            colorscale = ColorScale(args=yaml.load(file, Loader=yaml.FullLoader)['colorscale'])
        colors = np.zeros([256,3], dtype=np.uint8)
        if colorscale.bypass:
            for prop in properties:
                colors[prop['index'],:] = np.array(list(prop['color'].values()))
        else:
            for prop in properties:
                colors[prop['index'],:] = np.array(colorscale.mapToColor(prop['friction']['mean']))*255
    
    # Open the network
    if network_config['num_labels'] > 0:
        SemsegNetworkWrapper = importlib.import_module(network_config['package']).SemsegNetworkWrapper
        network = SemsegNetworkWrapper(model=network_config['model'], args=network_config['extra_args'])
    else:
        network = BypassWrapper()

    # Make the pallete
    palette = colors.flatten()

    # Run the network
    for infile_name in input_list:
        with torch.no_grad():
            if as_folder:
                img = Image.open(os.path.join(input_filename, infile_name)).convert('RGB')
            else:
                img = Image.open(infile_name).convert('RGB')
            output = network.runSegmentation(img, return_numpy=False)
            labels = torch.max(output, 0)[1].cpu().numpy()
        
        mask = Image.fromarray(labels.squeeze().astype('uint8'))
        mask.putpalette(palette)
        if output_filename is not None:
            if as_folder:
                mask.save(os.path.join(output_filename, infile_name))
            else:
                mask.save(output_filename)
        else:
            mask.show()

