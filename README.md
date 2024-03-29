# ndisys_ros2
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![CI](https://github.com/ICube-Robotics/ndisys_ros2/actions/workflows/ci.yaml/badge.svg)](https://github.com/ICube-Robotics/ndisys_ros2/actions/workflows/ci.yaml)


This stack includes drivers for `ros2_control` for communication with NDI measurement systems.

## Compatible devices
The driver is compatible with the following NDI measurement systems:
- Polaris
- Aurora

## Installation
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```bash
    sudo apt install python3-colcon-common-extensions
     ```
4. Pull relevant packages, install dependencies by using rosdep:
    ```bash
    git clone https://github.com/ICube-Robotics/ndisys_ros2.git src/ndisys_ros2
    rosdep install --ignore-src --from-paths . -y -r
    ```

5. Compile and source the workspace by using:
    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
## Running the driver

-   Add you markers in the `polaris_description/ros2_control/polaris.ros2_control.xacro` file using the following macro. All the mentioned `.rom` files here will be loaded to the NDI device and will be tracked. _It is advised_ to load only the markers you are interested in broadcasting.

    ```XML
    <xacro:tracker name="marker1" srom="path_to_rom_bin_file1" />
    <xacro:tracker name="marker2" srom="path_to_rom_bin_file2" />
    ```



- You can now choose what trackers to publish in the `polaris_bringup/config/polaris_broadcaster.yaml` file by adding their names in `sensor_names` and IDs in `sensor_ids`. Also, edit the `world_frame` and `state_publish_rate` parameters to match your application.


    _**One constraint**: Make sure to match_ `sensor_names[]` _content with the names you chose earlier in the `polaris.ros2_control.xacro` file. Preserving order._

    Example:
    ```YAML
    rigid_pose_broadcaster:
        ros__parameters:
            state_publish_rate: 10
            sensor_names:
            - marker1
            - marker2
            sensor_ids:
            - 1998
            - 2023
            world_frame: polaris_frame
    ```

- After sourcing ros2 and this package in a shell run:
    ```bash
    source install/setup.bash
    ros2 launch polaris_bringup vega.launch.py
    ```

- Start and activate the controller `rigid_pose_broadcaster` in a new terminal after sourcing `ndisys_ros2` and ROS2. See this [link](https://control.ros.org/master/doc/ros2_control/ros2controlcli/doc/userdoc.html#load-controller) for a guide to do


## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)

__Adnan SAOOD:__ [asaood@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [adnan-saood](https://github.com/adnan-saood)
