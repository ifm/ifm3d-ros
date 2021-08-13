# ifm3d-ros: Dump and Config

`ifm3d-ros` provides access to the camera/imager parameters via the `Dump` and `Config` services exposed by the `camera_nodelet`. Additionally, command-line scripts called `dump` and `config` are provided as wrapper interfaces to those services. This gives a feel similar to using the underlying `ifm3d` command-line tool from the ROS-independent driver except proxying the calls through the ros network.

For example, to dump the state of the camera:

(exemplary output from an O3R perception platform with one 3D imager is shown)

```
$ rosrun ifm3d dump
{
  "device": {
    "clock": {
      "currentTime": 1581091399246785500
    },
    "diagnostic": {
      "temperatures": [],
      "upTime": 754000000000
    },
    "info": {
      "device": "0301",
      "deviceTreeBinaryBlob": "tegra186-quill-p3310-1000-c03-00-base.dtb",
      "features": {},
      "name": "",
      "  partNumber": "M03903",
      "productionState": "AA",
      "serialNumber": "000201233338",
      "vendor": "0001"
    },
    "network": {
      "ipAddressConfig": 0,
      "macEth0": "00:04:4B:E4:DA:9E",
      "macEth1": "00:02:01:23:33:38",
      "networkSpeed": 1000,
      "sshPublicKeys": [
        ""
      ],
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
      "kernel": "4.9.140-l4t-r32.4+gb329485969c4",
      "l4t": "r32.4.3",
      "os": "0.12.9-174",
      "swu": "0.12.9-174"
    }
  },
  "ports": {
    "  port0": {
      "acquisition": {
        "exposureLong": 5000,
        "exposureShort": 400,
        "framerate": 10,
        "offset": 0,
        "version": {
          "major": 0,
          "minor": 0,
          "patch": 0
        }
      },
      "data": {
        "algoDebugConfig": {
          "channels": []
        },
        "availablePCICOutput": [
          "AMPLITUDE_COMPRESSED",
          "CONFIDENCE",
          "RADIAL_DISTANCE_COMPRESSED",
          "RADIAL_DISTANCE_NOISE",
          "REFLECTIVITY",
          "TOF_INFO"
        ],
        "pcicTCPPort": 50010
      },
      "info": {
        "device": "3101",
        "deviceTreeBinaryBlobOverlay": "001-irs2381c.dtbo",
        "features": {
          "fov": {
            "horizontal": 78,
            "vertical": 105
          },
          "  resolution": {
            "height": 172,
            "width": 224
          },
          "type": "3D"
        },
        "name": "",
        "partNumber": "M03957",
        "productionState": "AA",
        "sensor": "IRS2381C",
        "sensorID": "IRS2381C_105x78_4x2W_110x90_Csample",
        "serialNumber": "000000000186",
        "vendor": "0001"
      },
      "mode": "experimental_high_4m",
      "processing": {
        "diParam": {
          "anfFilterSizeDiv2": 2,
          "enableDynamicSymmetry": true,
          "enableStraylight": true,
          "enableTemporalFilter": true,
          "maxDistNoise": 0.02,
          "  maxSymmetry": 0.4,
          "medianSizeDiv2": 0,
          "minAmplitude": 20,
          "minReflectivity": 0,
          "mixedPixelFilterMode": 1,
          "mixedPixelThresholdRad": 0.15
        },
        "extrinsicHeadToUser": {
          "rotX": 0,
          "rotY": 0,
          "rotZ": 0,
          "transX": 0,
          "transY": 0,
          "transZ": 0
        },
        "version": {
          "major": 0,
          "minor": 0,
          "patch": 0
        }
      },
      "state": "CONF"
    }
  }
}
```

Chaining together Linux pipelines works just as it does in `ifm3d`. For example, using a combination of `dump` and `config` to set a new name on the camera would look like:

```
$ rosrun ifm3d dump | jq '.ports.port0.mode="experimental_high_2m"' | rosrun ifm3d config 
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
