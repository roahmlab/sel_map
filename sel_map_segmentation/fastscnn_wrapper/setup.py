from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['fastscnn_wrapper', 'fast_scnn'],
    package_dir={'': 'src'}
)
setup(**d)

