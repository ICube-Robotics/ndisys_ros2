# ndisys_ros2
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
5. Download the newest version of NDI CAPI and copy its headers in the `ndi_hardware/external/ndi_capi` directory and the `libndicapi.so` file to `ndi_hardware/external/`.

6. Compile and source the workspace by using:
    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
## Running the driver
- After sourcing ros2 and this package in a shell run:
    ```bash
    ros2 launch polaris_bringup vega.launch.py
    source install/setup.bash
    ```

-   Add you markers in the `polaris_description/config/polaris.config.xacro` file using the following macro. All the mentioned `.rom` files here will be loaded to the NDI device and will be tracked. It is advised to load only the markers you are interested in broadcasting.

    ```XML
    <xacro:ndi_sensor name="costum_name" srom="path_to_rom_bin_file"/>
    ```

    All the mentioned `.rom` files here will be loaded to the NDI device and will be tracked. It is advised to load only the markers you are interested in broadcasting.


- You can now choose what trackers to publish in the `polaris_bringup/config/polaris_broadcaster.yaml` file by adding their names in `sensor_names` and IDs in `sensor_ids`. Also, edit the `world_frame` and `state_publish_rate` parameters to match your application.

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

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)

__Adnan SAOOD:__ [asaood@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [adnan-saood](https://github.com/adnan-saood)
