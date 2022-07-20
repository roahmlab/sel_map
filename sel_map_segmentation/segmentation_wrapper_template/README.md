# Semantic Segmentation Wrapper Template Package for Integration with sel_map.

To help integrate your own semantic segmentation network of choice with our package, we have included this wrapper template package.
The most important file in this is the `SemsegNetworkWrapper.py` file under `src/segmentation_wrapper_template`.
The semantic segmentation configuration files under `config/semseg` in the `sel_map` package, used by `sel_map_segmentation` expect this format in order to find these wrappers.

## Using the Template
You can copy the template as another package if you'd like, or you can modify this package template directly.
In order to use this as a template, you must specify your desired package name at the tops of `package.xml`, `CMakeLists.txt`, as well as under the src folder.
That is, you should rename the `src/segmentation_wrapper_template` folder to `src/{desired_package_name}`.
Additionally, you should rename this parent folder holding the entire template package to `{desired_package_name}` and delete the `CATKIN_IGNORE` file so the package gets included.

This should result in the following directory structure:
```
{desired_package_name}/
├── src/{desired_package_name}/
│   ├── __init__.py
│   └── SemsegNetworkWrapper.py
├── CMakeLists.txt
├── package.xml
├── README.md
└── setup.py
```

To make sure that Python can find the wrapper, edit line 4 of `setup.py` to replace `segmentation_wrapper_template` with your `{desired_package_name}`, and if needed to find your code, update `my_network_root` with the root folder name of your network code or remove it.
After that, you can update `SemsegNetworkWrapper.py`, where in the `__init__` function, you should have it initialize the network based on parameters passed into `model` and `args`.
In the `runSegmentation` function, you have it semantically segment the image, returning it based on the format specified by `return_numpy` and `one_hot`.
Further specifics can be found in the `SemsegNetworkWrapper.py` files.

This are specifically made for the semseg yaml config files to associate as follows:
```yaml
semseg:
  package: # Your {desired_package_name}
  model: # String that gets passed to __init__ as "model"
  extra_args: # YAML dict that gets passed to __init__ as "args"
  num_labels: # Integer number of classes used for the sel_map_segmentation package
  ongpu_projection: # Boolean flag of which the inverse is passed to runSegmentation as "return_numpy"
  onehot_projection: # Boolean flag which is passed to runSegmentation as "one_hot"
```

## General Wrapper Class Requirements
In case you want to make your own wrapper class from scratch, the requirements in general are as follows:

* The class should be importable with `import {package}.SemsegNetworkWrapper`, where `{package}` is the respective entry from the semseg configuration file. This should generally match the ROS package name.
* The class constructor, or `__init__` function should include `model` and `args` arguments, where their values come from the `model` and `extra_args` entries in the semseg configuration file. These values can be used to specify the specific weights used for a set of architectures or similar.
* There should be a class function `runSegmentation` which takes a PIL Image object along with two additional boolean arguments `return_numpy` and `one_hot` that specify whether or not to return it as a numpy ndarr and specify whether to use one_hot encoding before sending to the mesh.
* Inside this class, a member called `device` should be set to something to communicate to the sensor model what device the segmentation is being performed on.

As long as you meet these requirements, you can use expect this mapping framework to work with your own custom semantic segmentation network, based on the config yaml files under `config/semseg` in the `sel_map` package.