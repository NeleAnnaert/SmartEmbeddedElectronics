
m distutils.core import setup


def readme():
    with open('README.rst') as f:
        return f.read()

setup(
    name='mapper',
    version='0.0.2',
    packages=['mapper'],
    url='',
    license='MIT License',
    author='Gill Bouwen and Stijn Goethals',
    author_email='',
    description='Mapper for ROS project',
    long_description=readme(),
    install_requires=[
        'path_finder == 0.3',
      ],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.4',
        'Topic :: Scientific/Engineering',
    ],
    keywords='ROS node mapper map',
    include_package_data=True,
    zip_safe=True,
)
