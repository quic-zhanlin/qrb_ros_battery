# QRB ROS BATTERY

## Overview

`qrb_ros_battery` is a package to publish the battery state data from system node.
- This package communicates with the battery service via DBus. The battery service obtains the current battery status by accessing the sys nodes.

## Quick Start

> **Note：**
> This document 's build & run is the latest.
> If it conflict with the online document, please follow this.

We provide two ways to use this package.

<details>
<summary>Docker</summary>

#### Setup
1. Please follow this [steps](https://github.com/qualcomm-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart) to setup docker env.
2. Download qrb_ros_battery and dependencies
    ```bash
    cd ${QRB_ROS_WS}/src

    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_battery.git
    ```

#### Build
```bash
colcon build
```

#### Run
```bash
cd ${QRB_ROS_WS}/src

source install/local_setup.sh
ros2 run qrb_ros_battery battery_node
```

</details>
 

<details>
<summary>QIRP-SDK</summary>

#### Setup
1. Please follow this [steps](https://qualcomm-qrb-ros.github.io/getting_started/index.html) to setup qirp-sdk env.
2. Download qrb_ros_battery and dependencies
    ```bash
    mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_battery.git
    ```

#### Build
1. Build the project
    ```bash
    export AMENT_PREFIX_PATH="${OECORE_NATIVE_SYSROOT}/usr:${OECORE_TARGET_SYSROOT}/usr"
    export PYTHONPATH=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/:${OECORE_TARGET_SYSROOT}/usr/lib/python3.12/site-packages/

    colcon build --continue-on-error --cmake-args \
      -DCMAKE_TOOLCHAIN_FILE=${OE_CMAKE_TOOLCHAIN_FILE} \
      -DPYTHON_EXECUTABLE=${OECORE_NATIVE_SYSROOT}/usr/bin/python3 \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -DBUILD_TESTING=OFF
    ```
2. Install the package
    ```bash
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install/qrb_ros_battery
    tar -czvf qrb_ros_battery.tar.gz include lib share
    scp qrb_ros_battery.tar.gz root@[ip-addr]:/home/
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install/qrb_battery_client
    tar -czvf qrb_battery_client.tar.gz include lib share
    scp qrb_battery_client.tar.gz root@[ip-addr]:/home/
    ssh root@[ip-addr]
    (ssh) mount -o remount rw /
    (ssh) tar --no-overwrite-dir --no-same-owner -zxf /home/qrb_ros_battery.tar.gz -C /usr/
    (ssh) tar --no-overwrite-dir --no-same-owner -zxf /home/qrb_battery_client.tar.gz -C /usr/
    ```

#### Run
```bash
(ssh) export HOME=/home
(ssh) setenforce 0
(ssh) source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
(ssh) ros2 run qrb_ros_battery battery_node
```

</details>

<br>

You can get more details from [here](https://qualcomm-qrb-ros.github.io/main/index.html).

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)


## Authors

* **Padmanabha Kavasseri** - *Initial work* - [PadmanabhaKavasseri](https://github.com/PadmanabhaKavasseri)

See also the list of [contributors](https://github.com/qualcomm-qrb-ros/qrb_ros_battery/graphs/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
