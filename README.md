# ndisys_ros2
This stack includes drivers for `ros2_control` for communication with NDI measurement systems.

## Compatible devices
The driver is compatible with the following NDI measurement systems:
- Polaris
- Aurora

## Usage
### Getting Started
***Required setup : Ubuntu 20.04 LTS***

1.  Install `ros2` packages. The current developpment is based of `ros2 galactic`. Installation steps are decribed [here](https://docs.ros.org/en/galactic/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/galactic/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies by using :
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/ndisys_ros2.git src/ndisys_ros2
    rosdep install --ignore-src --from-paths . -y -r
    ```
5. Download the newest version of NDI CAPI and copy its content in the `ndi_hardware/external/ndi_capi` directory.
6. Compile and source the workspace by using:
    ```shell
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
### Running the driver

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
