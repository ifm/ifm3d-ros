# ifm3d_ros_msgs: Dump and Config

The ifm3d_ros_msgs package allows us to configure the O3R camera platform via two separate ways:  
1. Use ROS native service calls
2. Use dump and config service proxies


## 1. ROS native service calls
Per camera head connected to the visual processing unit (VPU) of the O3R platform these services are available:  

| Name | Service Definition | Description |
| ---- | ---- | ---- |
| Dump | ifm3d/Dump | Dumps the state of the camera system as a JSON (formatted as a string) |
| Config | ifm3d/Config | Provides a means to configure the VPU and Heads (imager settings), declaratively from a JSON (string) encoding of the desired settings. |
| SoftOff | ifm3d/SoftOff | Sets the active application of the camera into software triggered mode which will turn off the active illumination reducing both power and heat. |
| SoftOn | ifm3d/SoftOn | Sets the active application of the camera into free-running mode. Its intention is to act as the inverse of `SoftOff`. |
| Trigger | ifm3d/Trigger | Requests the driver to software trigger the imager for data acquisition. |

As you can see the two services `Dump` and `Config` are also part of this list. 
### Dump
Calling the native rosservice `/ifm3d_ros_examples/camera/Dump` for a certain `camera` will return a string containing with the camera configuration as a JSON string. Please notice the use of backslashes (`\` before each `"`) to esacpe each upper quotation mark. This is necessary to allow us to keep the JSON syntax native to the underlying API (ifm3d).  

>Note: We have mapped some camera configurations to native rosservice calls to make their handling easier, e.g. `rosservice call /ifm3d_ros_examples/camera/SoftOn`. If you think you will benefit from similar rosservices, please let us know so we can add more comfort handling.

Call this service via, e.g. for camera:
```
$ rosservice call /ifm3d_ros_examples/camera/Dump 
status: 0
config: "{\"device\":{\"clock\":{\"currentTime\":1581287336449588512},\"diagnostic\":{\"temperatures\"\
  :[],\"upTime\":196692000000000},\"info\":{\"device\":\"0301\",\"deviceTreeBinaryBlob\"\
  :\"tegra186-quill-p3310-1000-c03-00-base.dtb\",\"features\":{},\"name\":\"test\"\
  ,\"partNumber\":\"M03975\",\"productionState\":\"AA\",\"serialNumber\":\"000201234160\"\
  ,\"vendor\":\"0001\"},\"network\":{\"authorized_keys\":\"\",\"ipAddressConfig\"\
  :0,\"macEth0\":\"00:04:4B:EA:9F:0E\",\"macEth1\":\"00:02:01:23:41:60\",\"networkSpeed\"\
  :1000,\"staticIPv4Address\":\"192.168.0.69\",\"staticIPv4Gateway\":\"192.168.0.201\"\
  ,\"staticIPv4SubNetMask\":\"255.255.255.0\",\"useDHCP\":false},\"state\":{\"errorMessage\"\
  :\"\",\"errorNumber\":\"\"},\"swVersion\":{\"kernel\":\"4.9.140-l4t-r32.4+gc35f5eb9d1d9\"\
  ,\"l4t\":\"r32.4.3\",\"os\":\"0.13.13-221\",\"schema\":\"v0.1.0\",\"swu\":\"0.15.12\"\
  }},\"ports\":{\"port0\":{\"acquisition\":{\"framerate\":10.0,\"version\":{\"major\"\
  :0,\"minor\":0,\"patch\":0}},\"data\":{\"algoDebugConfig\":{},\"availablePCICOutput\"\
  :[],\"pcicTCPPort\":50010},\"info\":{\"device\":\"2301\",\"deviceTreeBinaryBlobOverlay\"\
  :\"001-ov9782.dtbo\",\"features\":{\"fov\":{\"horizontal\":127,\"vertical\":80},\"\
  resolution\":{\"height\":800,\"width\":1280},\"type\":\"2D\"},\"name\":\"\",\"partNumber\"\
  :\"M03969\",\"productionState\":\"AA\",\"sensor\":\"OV9782\",\"sensorID\":\"OV9782_127x80_noIllu_Csample\"\
  ,\"serialNumber\":\"000000000382\",\"vendor\":\"0001\"},\"mode\":\"experimental_autoexposure2D\"\
  ,\"processing\":{\"extrinsicHeadToUser\":{\"rotX\":0.0,\"rotY\":0.0,\"rotZ\":0.0,\"\
  transX\":0.0,\"transY\":0.0,\"transZ\":0.0},\"version\":{\"major\":0,\"minor\":0,\"\
  patch\":0}},\"state\":\"RUN\"},\"port2\":{\"acquisition\":{\"exposureLong\":5000,\"\
  exposureShort\":400,\"framerate\":10.0,\"offset\":0.0,\"version\":{\"major\":0,\"\
  minor\":0,\"patch\":0}},\"data\":{\"algoDebugConfig\":{},\"availablePCICOutput\"\
  :[],\"pcicTCPPort\":50012},\"info\":{\"device\":\"3101\",\"deviceTreeBinaryBlobOverlay\"\
  :\"001-irs2381c.dtbo\",\"features\":{\"fov\":{\"horizontal\":105,\"vertical\":78},\"\
  resolution\":{\"height\":172,\"width\":224},\"type\":\"3D\"},\"name\":\"\",\"partNumber\"\
  :\"M03969\",\"productionState\":\"AA\",\"sensor\":\"IRS2381C\",\"sensorID\":\"IRS2381C_105x78_4x2W_110x90_C7\"\
  ,\"serialNumber\":\"000000000382\",\"vendor\":\"0001\"},\"mode\":\"standard_range2m\"\
  ,\"processing\":{\"diParam\":{\"anfFilterSizeDiv2\":2,\"enableDynamicSymmetry\"\
  :true,\"enableStraylight\":true,\"enableTemporalFilter\":true,\"excessiveCorrectionThreshAmp\"\
  :0.3,\"excessiveCorrectionThreshDist\":0.08,\"maxDistNoise\":0.02,\"maxSymmetry\"\
  :0.4,\"medianSizeDiv2\":0,\"minAmplitude\":20.0,\"minReflectivity\":0.0,\"mixedPixelFilterMode\"\
  :1,\"mixedPixelThresholdRad\":0.15},\"extrinsicHeadToUser\":{\"rotX\":0.0,\"rotY\"\
  :0.0,\"rotZ\":0.0,\"transX\":0.0,\"transY\":0.0,\"transZ\":0.0},\"version\":{\"\
  major\":0,\"minor\":0,\"patch\":0}},\"state\":\"RUN\"}}}"
```
## Config
Below you can see an example on how to configure your camera via a rosservice call. The JSON string can be a partial JSON string. It only needs to follow basic JSON syntax.

```
$ rosservice call /ifm3d_ros_examples/camera/Config "json: '{\"ports\":{\"port2\":{\"mode\":\"standard_range2m\"}}}'" 
status: 0
msg: "OK"
```

This is equivalent to:
```
$ rosservice call /ifm3d_ros_examples/camera/Config '"{\"ports\":{\"port2\":{\"mode\":\"standard_range2m\"}}}"'
status: 0
msg: "OK"

```

## 2. dump and config service proxies
`ifm3d-ros` provides access to each camera parameter via the `Dump` and `Config` services exposed by the `camera_nodelet`. 

Additionally, command-line scripts called `dump` and `config` are provided as wrapper interfaces to the native API `ifm3d`. This gives a feel similar to using the underlying C++ API's command-line tool, from the ROS-independent driver except proxying the calls through the ROS network.
They are part of the `ifm3d_ros_msgs` subpackage, so if you can't access them please make sure it is installed on your client.

For example, to dump the state of the camera:  
(exemplary output from an O3R perception platform with one camera head connected is shown)

>Note: The service proxying only works on the `/ifm3d_ros_examples/camera/` namespace at the moment.

```
$ roslaunch ifm3d_ros_examples camera.launch &
$ rosrun ifm3d_ros_msgs dump
{
  "device": {
    "clock": {
      "currentTime": 1581193835185485800
    },
    "diagnostic": {
      "temperatures": [],
      "upTime": 103190000000000
    },
    "info": {
      "device": "0301",
      "deviceTreeBinaryBlob": "tegra186-quill-p3310-1000-c03-00-base.dtb",
      "features": {},
      "name": "",
      "  partNumber": "M03975",
      "productionState": "AA",
      "serialNumber": "000201234160",
      "vendor": "0001"
    },
    "network": {
      "authorized_keys": "",
      "ipAddressConfig": 0,
      "macEth0": "00:04:4B:EA:9F:0E",
      "macEth1": "00:02:01:23:41:60",
      "networkSpeed": 1000,
      "staticIPv4Address": "192.168.0.69",
      "staticIPv4Gateway": "192.168.0.201",
      "staticIPv4SubNetMask": "255.255.255.0",
      "useDHCP": false
    },
    "state": {
      "errorMessage": "",
      "errorNumber": ""
    },
    "swVersion": {
      "kernel": "4.9.140-l4t-r32.4+gc35f5eb9d1d9",
      "l4t": "r32.4.3",
      "os": "0.13.13-221",
      "schema": "v0.1.0",
      "swu": "0.15.12"
    }
  },
  "ports": {
    "port0": {
      "acquisition": {
        "framerate": 10,
        "version": {
          "major": 0,
          "minor": 0,
          "patch": 0
        }
      },
      "data": {
        "algoDebugConfig": {},
        "availablePCICOutput": [],
        "pcicTCPPort": 50010
      },
      "info": {
        "device": "2301",
        "deviceTreeBinaryBlobOverlay": "001-ov9782.dtbo",
        "features": {
          "fov": {
            "horizontal": 127,
            "vertical": 80
          },
          "  resolution": {
            "height": 800,
            "width": 1280
          },
          "type": "2D"
        },
        "name": "",
        "partNumber": "M03969",
        "productionState": "AA",
        "sensor": "OV9782",
        "sensorID": "OV9782_127x80_noIllu_Csample",
        "serialNumber": "000000000382",
        "vendor": "0001"
      },
      "mode": "experimental_autoexposure2D",
      "processing": {
        "extrinsicHeadToUser": {
          "rotX": 0,
          "rotY": 0,
          "rotZ": 0,
          "  transX": 0,
          "transY": 0,
          "transZ": 0
        },
        "version": {
          "major": 0,
          "minor": 0,
          "  patch": 0
        }
      },
      "state": "RUN"
    },
    "port2": {
      "acquisition": {
        "exposureLong": 5000,
        "  exposureShort": 400,
        "framerate": 10,
        "offset": 0,
        "version": {
          "major": 0,
          "  minor": 0,
          "patch": 0
        }
      },
      "data": {
        "algoDebugConfig": {},
        "availablePCICOutput": [],
        "pcicTCPPort": 50012
      },
      "info": {
        "device": "3101",
        "deviceTreeBinaryBlobOverlay": "001-irs2381c.dtbo",
        "features": {
          "fov": {
            "horizontal": 105,
            "vertical": 78
          },
          "  resolution": {
            "height": 172,
            "width": 224
          },
          "type": "3D"
        },
        "name": "",
        "partNumber": "M03969",
        "productionState": "AA",
        "sensor": "IRS2381C",
        "sensorID": "IRS2381C_105x78_4x2W_110x90_C7",
        "serialNumber": "000000000382",
        "vendor": "0001"
      },
      "mode": "standard_range4m",
      "processing": {
        "diParam": {
          "anfFilterSizeDiv2": 2,
          "enableDynamicSymmetry": true,
          "enableStraylight": true,
          "enableTemporalFilter": true,
          "excessiveCorrectionThreshAmp": 0.3,
          "excessiveCorrectionThreshDist": 0.08,
          "maxDistNoise": 0.02,
          "maxSymmetry": 0.4,
          "medianSizeDiv2": 0,
          "minAmplitude": 20,
          "minReflectivity": 0,
          "mixedPixelFilterMode": 1,
          "mixedPixelThresholdRad": 0.15
        },
        "extrinsicHeadToUser": {
          "rotX": 1,
          "rotY": 0,
          "rotZ": 0,
          "transX": 100,
          "transY": 0,
          "transZ": 0
        },
        "version": {
          "  major": 0,
          "minor": 0,
          "patch": 0
        }
      },
      "state": "RUN"
    }
  }
}
```

Chaining together Linux pipelines works just as it does in `ifm3d`. For example, using a combination of `dump` and `config` to set a new name on the camera would look like:

```
$ rosrun ifm3d dump | jq '.ports.port0.mode="standard_range2m"' | rosrun ifm3d config 
$ rosrun ifm3d dump | jq .ports.port0.mode
"experimental_high_2m"
```

>**NOTE:** If you do not have `jq` on your system, it can be installed with: `$ sudo apt-get install jq`  

For the `config` command, one difference between our ROS implementation and the `ifm3d` implementation is that we only accept input on `stdin`. So, if you had a file with JSON you wish to configure your camera with, you would simply use the file I/O redirection facilities of your shell (or something like `cat`) to feed the data to `stdin`. For example, in `bash`:

```
$ rosrun ifm3d dump > dump.json
$ cat dump.json | jq '.ports.port0.mode="experimental_high_2m"' > config.json
$ rosrun ifm3d config < config.json
```

Beyond the requirement of prefacing your command-line with `rosrun` to invoke the ROS version of these tools, they operate the same. To learn more about the functionality and concepts, you can read the docs [here](https://github.com/ifm/ifm3d/blob/master/doc/configuring.md).
