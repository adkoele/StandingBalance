import setuptools

setuptools.setup(
    name='musculo_skeletal_generator',
    version='0.1',
    description='MusculoSkeletalSystem generator',
    url='https://gitlab.com/BioRobAnimals/muscle_generator.git',
    author='biorob-farms',
    author_email='biorob-farms@groupes.epfl.ch',
    license='MIT',
    packages=setuptools.find_packages(),
    install_requires=['pyyaml',
                      'casadi',
                      'numpy'],
    zip_safe=False
)
