# Games101

My Personal Games101 Assignments

# Progress

- [x] Assignment0 
- [x] Assignment1
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

# Details

## Assignment0

- Fulfill the basic requirement.
- Add the following two lines in `CMakeLists.txt`:
    ```cmake
    set(CMAKE_CXX_STANDARD 17)
    include_directories(/usr/local/include ./include)
    ```

## Assignment1

- Fulfill the basic requirement.
- Implement BONUS task: `get_rotation()`
- Additional function: press space to toggle auto rotation

# About brew

## Update your brew in case of some warnings

```shell
brew update && brew upgrade
```

## Clean up old version packages

You can use `brew cleanup -n` to preview how many space will be freed

```shell
brew cleanup
```