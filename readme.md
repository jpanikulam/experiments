Jake's C++ Library (jcc)
=======================

If you find yourself using this package, please send me an email, and I'll make this more convenient.

# Dependencies

```shell
bash ./setup.sh  # This script is small, but dispatches to sudo; read it before you run it.
```

# Compile
```shell
mkdir bin/
cd bin/
cmake ..
make
```

Note, you'll have to re-run cmake any time you add/remove headers or otherwise change dependencies, because this repo uses an implicit dependency management system, which infers dependencies from header chains. Any folder without a CMakeLists.txt is generating libraries in this way.

# Compiling Jetsim

#### Just jetsim
```shell
cd experiments/bin
cmake .. && make jetsim &&  ./run/jetsim
```

# PyMake

####
```shell
cd experiments/pymake
sudo python setup.py install

pymake
```


# What's Jake Doing Next?
[ ] Add bombela to pymake
[ ] Make Gandi refer to github
* Figure out how to make my github.io display images (Wowee!)
* pdfs on control
* Code Generation & compute-graph specification
    * Generate differential equation solvers
    * Generate malloc-free sparse matrix operations
* stm32f adventures
* Lanczos
    * Lagrangian physics tools
* Little 4-legged walker
* "Plausible" 3D fluids -- how did they do `From Dust`?
* Signed distance field rendering fmwk
* Gl4 Window

# Tutorials
* How to think about rotation


# Read
http://hhoppe.com/ravg.pdf
https://www.researchgate.net/profile/Paulo_Oliveira11/publication/224230400_Geometric_Approach_to_Strapdown_Magnetometer_Calibration_in_Sensor_Frame/links/00b7d527b9b7297d54000000/Geometric-Approach-to-Strapdown-Magnetometer-Calibration-in-Sensor-Frame.pdf
https://radionavlab.ae.utexas.edu/images/stories/files/papers/sierra_for_distribution.pdf
# TODO
* Unify VecN...
