from setuptools import setup

# read the contents of your README file
from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='DynamicAxisControl',
    long_description=long_description,
    long_description_content_type='text/markdown',
    version='1.0.0',    
    description='Linear trajectory generator for robotic axes, with the ability to synchronize up to two speed profiles',
    author='Davide Zuanon',
    author_email='d.zuanon87@gmail.com',
    license='unlicense',
    packages=['DynamicAxisControl'],
    install_requires=['matplotlib',
                      'numpy',
                      'scipy',
                      'time',                     
                      ],


    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: The Unlicense (Unlicense)',  
        'Operating System :: POSIX :: Linux',        
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.10',
    ],
    entry_points={
        'console_scripts': [
            'Dynamic-Axis-Control = DynamicAxisControl:DynamicAxis_Ready.............',
        ]
    }
)