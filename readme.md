
# Dependencies

```shell
sudo pip install colorama
```


# Compile
```shell
mkdir bin/
cd bin/
cmake ..
make
```

Note, you'll have to re-run cmake any time you add/remove headers or otherwise change dependencies, because this repo uses an implicit dependency management system, which infers dependencies from header chains.


# What's Jake Doing Next?
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