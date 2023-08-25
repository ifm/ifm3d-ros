
# ROS network: Docker container

There are several objectives and limitations to consider when running a ifm3d-ros Docker container on the VPU:

There are two goals that must be met when running a Docker container directly on the VPU:
1. Use `IFM3D_IP=127.0.0.1` for local communication to avoid network drop propagation of ETH0 into the container.
2. Allow communication between ROS nodes through the localhost / default gateway (0.0.0.0) instead of the Ethernet interface ETH0.

All Docker processes should run in a single Docker container instance. Only by running all ROS processes in the same Docker context can the communication overhead be limited.
This communication overhead consists of serializing, sending ROS data frames over the network, and de-serializing this data at each ROS receiver.

The ifm3d-ros nodes are designed in a ROS nodelet architecture: To take advantage of the nodelet performance gains, all dependent ROS nodes should also be implemented as ROS nodelets.
Only then the full advantage of a nodelet architecture can be used: i.e. passing data reference pointers instead of serializing / deserializing the data to be sent in the network.

Please see the solutions below to setup a network communication to allow running a ifm3d-ros node in a Docker container on the VPU.

## Solution 1: localhost based communication - network host

**Limitations of ROS in a Docker container context**
+ ifm3d camera communication: use localhost - 127.0.0.1 to avoid ETH0 network drops resulting in ifm3d objects re-init
	+ (192.168.0.69 Interface used for ROS communications is not feasible: all network connection drops get propagated into the container / ROS master and node)
+ manual port selection for roslaunch is not possible:
	+ Use `network host` to share the host network to the Docker container during runtime
+ Since the ROS master and node communication can not be separated for the network interfaces:
	+ both have to communicate on the same network - default route: 0.0.0.0 interface

**Alternative:**
+  2 ROS Masters and a ROS bridge to connect them: This however is not feasible as ROS bridges are not designed to handle large datasets such as point cloud data.

**Solution**
**Please do not route the ROS communication via the Default gateway - a communication via ETH0 (192.168.0.0 subnet) is highly discouraged as its routing tables will change when connections on ETH0 NIC are established or closed!**

+ For name resolution manually modify the /etc/hosts
	Docker container
	```
	127.0.0.1	localhost.localdomain		localhost
	0.0.0.0		ifm3d_rosmaster
	0.0.0.0		ifm3d_ros
	```

+ IPC / Laptop: /etc/hosts
	```
	192.168.0.69    ifm3d_rosmaster
	192.168.0.69    ifm3d_ros
	```

During Runtime these parameters env variables have to be set:
+ Docker Containers:
```
export IFM3D_IP="127.0.0.1"
export ROS_MASTER_URI=http://ifm3d_rosmaster:11311
export ROS_HOSTNAME=ifm3d_ros
```
+ Laptop IPC:
```
export ROS_MASTER_URI=http://ifm3d_rosmaster:11311
export ROS_HOSTNAME=ifm3d_ros_listener
```

The corresponding DNS `/etc/hosts` - see above

Dockerfile
```yml
version: "2.3"
services:
  ifm3d_ros:
    tty: true
    image: ifm3d-ros:noetic-arm64_v8

    restart: unless-stopped
    volumes:
      - ./launch/four_cameras.launch:/home/ifm/catkin_ws/src/ifm3d_ros_driver/launch/four_cameras.launch
      - ./hosts_ros_master:/etc/hosts/

    logging:
      driver: "json-file"
      options:
        mode: non-blocking
        max-buffer-size: "20m"

    command:
      - /bin/bash
      - -c
      - |
        set -a
        . /opt/ros/noetic/setup.sh;
        . /home/ifm/catkin_ws/devel/setup.sh;
        export IFM3D_IP="127.0.0.1";
        export ROS_MASTER_URI=http://ifm3d_rosmaster:11311;
        export ROS_HOSTNAME=ifm3d_ros;
        echo $$ROS_MASTER_URI;
        echo $$ROS_IP;
        roslaunch ifm3d_ros_driver four_cameras.launch --wait -t 10

    network_mode: host
```

## Solution 2: separate Docker network

**Motivation**
Separate the two networks: localhost and dedicated Docker network for ROS container communication on the VPU.
This will solve the problem of all ROS communication being shared on the same node.

**Limitations**.
Please note that the following changes may result in routing tables that are not functional. This may cause the VPU to be inaccessible via its Ethernet interface. Please proceed with caution.

**When creating/modifying networks on the VPU, please consider your existing network topology and possible conflicts!**.

**Docker compose including network setup:**.
This includes setting up a dedicated `ros_net` Docker network for shared communication between Docker containers.

```yml
version: "2.3"
services:
  ifm3d_ros:
    tty: true
    image: ifm3d-ros:noetic-arm64_v8

    extra_hosts:
      - "dockerhost:172.20.0.1"

    networks:
      ros_net:
        ipv4_address: 172.20.0.2

    ports:
      - 11311:11311


    restart: unless-stopped
    volumes:
      - ./launch/four_cameras.launch:/home/ifm/catkin_ws/src/ifm3d_ros_driver/launch/four_cameras.launch
    logging:
      driver: "json-file"
      options:
        mode: non-blocking
        max-buffer-size: "20m"

    command:
      - /bin/bash
      - -c
      - |
        (. /opt/ros/noetic/setup.sh;
        . /home/ifm/catkin_ws/devel/setup.sh;
        export ROS_MASTER_URI=http://172.20.0.2:11311;
        export ROS_HOSTNAME=172.20.0.2;
        echo ROS_MASTER_URI=$$ROS_MASTER_URI;
        echo ROS_HOSTNAME=$$ROS_HOSTNAME;
        roscore &
        roslaunch ifm3d_ros_driver four_cameras.launch --wait -t 10)

networks:
  ros_net:
    ipam:
      driver: default
      config:
        - subnet: 172.20.0.0/16
          gateway: 172.20.0.1

```

Depending on Docker and the docker-compose version a first initial manual setup of such a Docker network might be required:
```shell
docker network create --gateway 172.20.0.1 --subnet 172.20.0.0/24 ros_net
```

The resulting routing table
```
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.201   0.0.0.0         UG    0      0        0 eth0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
172.20.0.0      0.0.0.0         255.255.0.0     U     0      0        0 br-5866d929f186
192.168.0.0     0.0.0.0         255.255.255.0   U     0      0        0 eth0
```


To allow communication between this dedicated Docker network one has to attach this virtual interface (the Docker network) to the ETH0 interface:
**THIS IS CURRENTLY ONLY POSSIBLE AS root user and NOT as oem user**
```bash
brctl show
bridge name		bridge id		STP enabled	interfaces
br-5866d929f186		8000.02428756949b	no		eth0
																vethd1fc3b6

```
Add the virtual interface to ETH0 interface
```bash
brctl addif br-5866d929f186 eth0
```

**IPC / Host machine**
To listen to ifm3d-ros topics and services (additionally to the ROS masters topics) an additional route has to be configured on the host:
```bash
sudo ip route add 172.20.0.1/24 via 192.168.0.69
```

After exporting the correct `ROS_MASTER_URI` and `ROS_HOSTNAME` the communication with the ROS core and ifm3d-ros node should be possible:
```bash
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROS_HOSTNAME=172.20.0.1
```