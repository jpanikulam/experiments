
# Dependencies

```shell
sudo pip install colorama
```

The C++ dependencies can be found [here](https://github.com/jpanikulam/jpanikulam.github.io/blob/master/bash/setup.sh). If you find yourself using this package, please send me an email, and I'll make this more convenient.


# Compile
```shell
mkdir bin/
cd bin/
cmake ..
make
```

Note, you'll have to re-run cmake any time you add/remove headers or otherwise change dependencies, because this repo uses an implicit dependency management system, which infers dependencies from header chains. Any folder without a CMakeLists.txt is generating libraries in this way.


# What's Jake Doing Next?
[ ] Add bombela to cppmv
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

# TODO
* Unify VecN...
