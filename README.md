# Barracuda 2 - RoboSub 2026 Complete Workspace

Latest project snapshot from the development server, captured on **2026-11-19**.

## Included

- `2026Robosub Full Simulation/ROS2_ws_Main_Body`: all ROS 2 package sources, configuration, launchers, tests, documentation and stored small models.
- PID, MPPI and GRU controllers; Isaac bridge; ZED sensing; YOLO segmentation and geometric perception; mission control and autonomy.
- `2026Robosub Full Simulation/Model_Datas`: all three current PT weights.
- `2026Robosub Full Simulation/Develop_Reaserch_ws`: research/example source workspace.
- `scenes/Full Process AUV Simulation 2026`: pool, Barracuda robot assets, gripper, task assets, textures and asset-generation source scripts.

## Download the complete files

Large binary assets and weights use **Git LFS**. Install Git LFS before cloning:

```bash
git lfs install
git clone https://github.com/LukeLiu2000/Barracuda2-RoboSub-2026-Complete-Workspace.git
cd Barracuda2-RoboSub-2026-Complete-Workspace
git lfs pull
```

GitHub's ZIP download may contain LFS pointers depending on repository archive settings. Clone with Git LFS for a complete checkout.

## Build

Use Ubuntu with ROS 2 Jazzy and the project-specific Isaac Sim / ZED SDK dependencies described inside the package documentation.

```bash
cd "2026Robosub Full Simulation/ROS2_ws_Main_Body"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Some configuration and USD asset references retain original server absolute paths or external vendor URLs. Adjust these when relocating the workspace. Vendor SDKs, Isaac Sim installation, Python environments and externally referenced vendor assets are not redistributed here.

## Snapshot integrity and scope

`SNAPSHOT_MANIFEST.json` records every copied source file, its size and SHA-256 checksum. It also lists excluded generated/cache files and historical backups. Build/install products, runtime logs, old backups and Python environments can be regenerated and are intentionally excluded. The separate 82 GB recorded sensor-data directory and unrelated projects are not part of this workspace snapshot.

This upload is a source/asset backup, not a newly executed validation of all controllers or hardware behavior. Original server files were not modified during snapshot creation. Third-party asset and code licenses remain with their respective owners; repository privacy does not change those licenses.
