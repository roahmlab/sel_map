from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['my_network_root', 'segmentation_wrapper_template'],
    package_dir={'': 'src'}
)
setup(**d)

