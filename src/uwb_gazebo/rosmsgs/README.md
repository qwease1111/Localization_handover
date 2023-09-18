# ROS messages

The original repository is: https://github.com/GTEC-UDC/rosmsgs, some changes have been made in order to be able to compile it for [ROS2 Humble](https://docs.ros.org/en/humble/index.html)

**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

## Compile the project

```bash
colcon build
```

To use the messages:

```sh
source install/setup.bash
```

You can also include the source into your bashrc adding the following line:

```sh
source ~/rosmsgs/install/setup.bash
```
