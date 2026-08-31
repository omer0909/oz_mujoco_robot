## glfw install
``` bash
cd ../deps/
curl -LO https://codeload.github.com/glfw/glfw/zip/refs/tags/3.5.1
unzip 3.5.1 && rm 3.5.1
cd glfw-3.5.1/
mkdir build
cd build/
cmake -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=OFF ..
make install -j4
```

## mujoco install
download 3.10 from:
[https://github.com/google-deepmind/mujoco/releases/tag/3.10.0](https://github.com/google-deepmind/mujoco/releases/tag/3.10.0)
and copy mujoco-3.10.0 folder to ../deps

## project folders
``` bash
❯ tree .. -L 2
..
├── deps
│   ├── glfw-3.5.1
│   └── mujoco-3.10.0
└── mujoco_robot
    ├── Cargo.lock
    ├── Cargo.toml
    ├── MUJOCO_LOG.TXT
    ├── README.md
    ├── robot.xml
    ├── src
    └── target
```
