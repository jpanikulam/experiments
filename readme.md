
# Dependencies

```shell
sudo pip install colorama
```


# Compile
```shell
# Setup implicit cmakes
cd experiments/
python cppmv/cppmv.py

# Then do conventional build
mkdir bin/
cd bin/
cmake ..
make
```


# What's Jake Doing Next?
* Figure out how to make my github.io display images (Wowee!)
* pdfs on control
* Code Generation & compute-graph specification
    * Generate differential equation solvers
    * Generate malloc-free sparse matrix operations
* stm32f adventures
* Lanczos
* Little 4-legged walker