<div align="center">

# ManipStudio

**Model a robot. Plan its motion. See the loads.**

![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square)
![Qt](https://img.shields.io/badge/Qt-6.4%2B-41CD52?style=flat-square)
![Platforms](https://img.shields.io/badge/platforms-Windows%20%7C%20Linux-667085?style=flat-square)

[Русский](README.ru.md)

</div>

![ManipStudio: robot simulation, motion program, and joint-moment plots](docs/images/manipstudio.png)

A desktop workbench for modeling and simulating six-axis robot arms.
Configure the robot, plan a trajectory, and inspect its motion and joint loads in one window.

## Features

- Interactive 3D view, TCP path, and timeline playback.
- MoveJ, MoveL, MoveC, TCP jog, and JSON motion programs.
- Editable DH geometry, joint limits, link dynamics, and payload.
- Collision checks and joint speed, acceleration, and moment limits.
- Motion and joint-moment plots with CSV export.
- English and Russian interface.

## Build and run

Requires a C++17 compiler, CMake, Ninja, Git, and Qt 6.4+ with Quick, Quick Controls, Quick3D, Charts, and Test.
On Windows, use the MSYS2 UCRT64 environment.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 8
./build/Studio/ManipStudio
```

On Windows, launch `./build/Studio/ManipStudio.exe`.
Robot settings are available in the app and saved to `ManipulatorSettigns.ini`.
A sample motion program is included in [Studio/examples/demo_program.json](Studio/examples/demo_program.json).

## Tests

```bash
ctest --test-dir build --output-on-failure
```
