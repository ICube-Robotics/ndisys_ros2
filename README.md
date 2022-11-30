# ndisys_ros2
This stack includes drivers for `ros2_control` for communication with NDI measurement systems.

## Compatible devices
The driver is compatible with the following NDI measurement systems:
- Polaris
- Aurora

## Installation
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current developpment is based of `ros2 humble`. Installation steps are decribed [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
4. Pull relevant packages, install dependencies by using rosdep:
    ```shell
    git clone https://github.com/ICube-Robotics/ndisys_ros2.git src/ndisys_ros2
    rosdep install --ignore-src --from-paths . -y -r
    ```
5. Download the newest version of NDI CAPI and copy its headers in the `ndi_hardware/external/ndi_capi` directory and the `libndicapi.so` file to `ndi_hardware/external/`.

6. Compile and source the workspace by using:
    ```shell
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
## Running the driver
After sourcing ros2 and this package in a shell run:
    ```shell
    ros2 launch polaris_bringup vega.launch.py
    ```
    Take a look at the `polaris_bringup/launch/vega.launch.py` and `polaris_description/config/polaris_vega.yaml` files for instructuion on how to configure and define your application-specific markers.

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
