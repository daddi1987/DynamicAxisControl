from setuptools import setup


setup(
    name='DynamicAxisControl',
    version='0.1.0',    
    description='Linear trajectory generator for robotic axes, with the ability to synchronize up to two speed profiles, to maximize performance. Also perfect for calculating the trajectories of the CoreXY Axes.',
    url='https://github.com/daddi1987/Dynamic-Axis-Control/tree/main',
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
        'License :: OSI Approved :: Historical Permission Notice and Disclaimer (HPND)',  
        'Operating System :: POSIX :: Linux',        
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.10',
    ],
    entry_points={
        'console_scripts': [
            'Dynamic-Axis-Control = timmins:DynamicAxis_Hello!!!!',
        ]
    }
)