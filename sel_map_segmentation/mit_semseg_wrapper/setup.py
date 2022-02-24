from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['mit_semseg_wrapper', 'mit_semseg'],
    package_dir={'': 'src'}
)
setup(**d)

