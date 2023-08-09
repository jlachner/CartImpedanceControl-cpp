# Cartesian Impedance Controller
This repository contains a class for a Cartesian Impedance Controller, based on Elastic Potentials. More Details can be found here:

    @phdthesis{lachner2022geometric,
    title={A geometric approach to robotic manipulation in physical human-robot interaction},
    author={Lachner, Johannes},
    school={University of Twente},
    year={2022}
    }

# Setting up the software 
To clone the repository, run 
```
git clone https://github.com/jlachner/CartImpedanceControl-cpp.git [SET_UP_YOUR_DIRECTORY]
```

For vector and matrix calculations, we use [Eigen version 3.4.0](https://gitlab.com/libeigen/eigen/-/releases/3.4.0), which is added as a git submodule. To update the submodules, type:
```
    git submodule update --init --recursive
```
You always need to check for Explicit-cpp updates. For that, type:
```
    git submodule update --remote
```

To compile the code, run
```
make -f Makefile-lib
```

# Authors
This software is under development by [Johannes Lachner](https://jlachner.github.io/).
