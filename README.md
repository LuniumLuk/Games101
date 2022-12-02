# Games101

My Personal Games101 Assignments

# Progress

- [x] Assignment0 
- [ ] Assignment1
- [ ] Assignment2 
- [ ] Assignment3 

# Compile in MacOS with VSCode

## Install OpenCV and Eigen via brew

1. Open Terminal (zsh)
2. Install Packages
    ```shell
    brew install eigen
    brew install opencv
    ```

## Compile via cmake and make

1. `cd AssignmentX` or `cd AssignmentX/Code`
2. `mkdir build`
3. `cd build`
4. `cmake ..`
5. `make`

# Modifications

## Assignment0

Add the following two lines in `CMakeLists.txt`:

```cmake
set(CMAKE_CXX_STANDARD 17)
include_directories(/usr/local/include ./include)
```
