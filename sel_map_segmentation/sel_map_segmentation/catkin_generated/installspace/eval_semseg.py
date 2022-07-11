import os
import numpy as np
from PIL import Image

from sel_map_segmentation import BypassWrapper
import importlib
import yaml
# from randimage import get_random_image, show_array #pip package randimage https://github.com/nareto/randimage
# after testing, the above difference was generally negligible
import time
import sys
import rospy

if __name__ == '__main__':
    img_size = (480, 640) #height by width
    num_imgs = 1000
    network_config = None
    result_to_cpu = True
    # Get arguments
    argv = rospy.myargv(argv=sys.argv)
    try:
        network_config = argv[1]
    except:
        print("Please provide arguments:")
        print("network_config image_size_x[default=640] image_size_y[default=480] num_images[default=1000] network_res_to_cpu[default=true]")
        exit()
    try:
        width = int(argv[2])
        height = int(argv[3])
        img_size = (height, width)
        num_imgs = int(argv[4])
        if argv[5].lower() in ("yes", "true", "t", "1"):
            result_to_cpu = True
        else:
            result_to_cpu = False
    except:
        pass

    semseg = {'package': '', 'model': '', 'extra_args': [], 'num_labels': 0}
    # Open the passed in config
    if network_config is not None:
        with open(network_config) as file:
            cfg = yaml.load(file, Loader=yaml.FullLoader)
            semseg = cfg['semseg']
            # make sure args is either something or nothing
            if semseg['extra_args'] is not None and len(semseg['extra_args']) == 0:
                semseg['extra_args'] = None

    # Open the network
    if semseg['num_labels'] > 0:
        SemsegNetworkWrapper = importlib.import_module(semseg['package']).SemsegNetworkWrapper
        network = SemsegNetworkWrapper(model=semseg['model'], args=semseg['extra_args'])
    else:
        network = BypassWrapper()

    # Set savefile
    save_filename = "times_"+semseg["package"]+"_"+semseg["model"]+"_"+str(num_imgs)
    save_filename = save_filename+"_"+str(img_size[1])+"x"+str(img_size[0])
    if result_to_cpu:
        save_filename += "_res_to_cpu"
    print("Saving to", save_filename+".csv")

    # Get times
    times = []
    for i in range(num_imgs):
        # img = get_random_image(img_size)
        img = np.random.rand(img_size[0],img_size[1],3)
        img = (img * 255).astype('uint8')
        image = Image.fromarray(img)
        
        tic = time.perf_counter()
        scores = network.runSegmentation(image, return_numpy=result_to_cpu)
        toc = time.perf_counter()

        delta = toc-tic
        times.append(delta)

        if (i%100) == 0:
            calc = np.array(times)
            print("images:", i+1, ", mean time:", np.mean(calc), ", stddev:", np.std(calc))
            np.savetxt(save_filename+".csv", calc, delimiter=",")

    calc = np.array(times)
    print("images:", i+1, ", mean time:", np.mean(calc), ", stddev:", np.std(calc))
    print("initial time:", calc[0])
    np.savetxt(save_filename+".csv", calc, delimiter=",")

