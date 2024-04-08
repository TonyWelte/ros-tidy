# ros-tidy

`ros-tidy` is a custom clang-tidy module designed specifically for ROS (Robot Operating System) projects.

## Features

- **ros-interface-names**: This check enforces naming conventions for ROS interfaces (Publisher, Subscriber, Service and Client) ensuring consistency and readability across your ROS projects.

## Prerequisites

Before using `ros-tidy`, ensure you have the following prerequisites installed:

- CMake
- Clang
- LLVM

## Installation

1. Clone the repository:

```bash

git clone https://github.com/Tonywelte/ros-tidy.git
```

2. Navigate to the project directory:

```bash
cd ros-tidy
```

3. Build the project using CMake:

```bash
mkdir build && cd build
cmake ..
make
```

## Usage

To use `ros-tidy` with `clang-tidy`, you have to load the module with the `--load` option and configure the `ros-interface-names` check. Here's an example command:

```bash
clang-tidy --load=/path/to/ros-tidy.so --checks='-*,ros-interface-names' <source_file>.cpp
```

Replace `/path/to/ros-tidy.so` with the actual path to the `ros-tidy` library generated during the build process. Replace `<source_file>.cpp` with the path to the source file you want to analyze.