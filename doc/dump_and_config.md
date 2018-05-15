
ifm3d-ros: Dump and Config
==========================

`ifm3d-ros` provides access to the camera/imager parameters via the `Dump` and
`Config` services exposed by the `camera_nodelet`. Additionally, command-line
scripts called `dump` and `config` are provided as wrapper interfaces to those
services. This gives a feel similar to using the underlying `ifm3d`
command-line tool from the ROS-independent driver except proxying the calls
through the ros network.

For example, to dump the state of the camera:

(exemplary output from an O3D303 is shown)

```
$ rosrun ifm3d dump
{
    "ifm3d": {
        "Apps": [
            {
                "Description": "",
                "Id": "322089668",
                "Imager": {
                    "AutoExposureMaxExposureTime": "10000",
                    "AutoExposureReferencePointX": "88",
                    "AutoExposureReferencePointY": "66",
                    "AutoExposureReferenceROI": "{\"ROIs\":[{\"id\":0,\"group\":0,\"type\":\"Rect\",\"width\":130,\"height\":100,\"angle\":0,\"center_x\":88,\"center_y\":66}]}",
                    "AutoExposureReferenceType": "0",
                    "Channel": "0",
                    "ClippingBottom": "131",
                    "ClippingCuboid": "{\"XMin\": -3.402823e+38, \"XMax\": 3.402823e+38, \"YMin\": -3.402823e+38, \"YMax\": 3.402823e+38, \"ZMin\": -3.402823e+38, \"ZMax\": 3.402823e+38}",
                    "ClippingLeft": "0",
                    "ClippingRight": "175",
                    "ClippingTop": "0",
                    "ContinuousAutoExposure": "false",
                    "ContinuousUserFrameCalibration": "false",
                    "EnableAmplitudeCorrection": "true",
                    "EnableFastFrequency": "false",
                    "EnableFilterAmplitudeImage": "true",
                    "EnableFilterDistanceImage": "true",
                    "EnableRectificationAmplitudeImage": "false",
                    "EnableRectificationDistanceImage": "false",
                    "ExposureTime": "5000",
                    "ExposureTimeList": "125;5000",
                    "ExposureTimeRatio": "40",
                    "FrameRate": "10",
                    "MaxAllowedLEDFrameRate": "23.2",
                    "MinimumAmplitude": "42",
                    "Resolution": "0",
                    "SpatialFilter": {},
                    "SpatialFilterType": "0",
                    "SymmetryThreshold": "0.4",
                    "TemporalFilter": {},
                    "TemporalFilterType": "0",
                    "ThreeFreqMax2FLineDistPercentage": "80",
                    "ThreeFreqMax3FLineDistPercentage": "80",
                    "TwoFreqMaxLineDistPercentage": "80",
                    "Type": "under5m_moderate",
                    "UseSimpleBinning": "false"
                },
                "Index": "1",
                "LogicGraph": "{\"IOMap\": {\"OUT1\": \"RFT\",\"OUT2\": \"AQUFIN\"},\"blocks\": {\"B00001\": {\"pos\": {\"x\": 200,\"y\": 200},\"properties\": {},\"type\": \"PIN_EVENT_IMAGE_ACQUISITION_FINISHED\"},\"B00002\": {\"pos\": {\"x\": 200,\"y\": 75},\"properties\": {},\"type\": \"PIN_EVENT_READY_FOR_TRIGGER\"},\"B00003\": {\"pos\": {\"x\": 600,\"y\": 75},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT1\"},\"B00005\": {\"pos\": {\"x\": 600,\"y\": 200},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT2\"}},\"connectors\": {\"C00000\": {\"dst\": \"B00003\",\"dstEP\": 0,\"src\": \"B00002\",\"srcEP\": 0},\"C00001\": {\"dst\": \"B00005\",\"dstEP\": 0,\"src\": \"B00001\",\"srcEP\": 0}}}",
                "Name": "Sample Application",
                "PcicEipResultSchema": "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"binary\", \"order\": \"big\" }, \"elements\" : [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"records\", \"id\": \"models\", \"elements\": [ { \"type\": \"int16\", \"id\": \"boxFound\" }, { \"type\": \"int16\", \"id\": \"width\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"height\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"length\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"xMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"zMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yawAngle\" }, { \"type\": \"int16\", \"id\": \"qualityLength\" }, { \"type\": \"int16\", \"id\": \"qualityWidth\" }, { \"type\": \"int16\", \"id\": \"qualityHeight\" } ] }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
                "PcicPnioResultSchema": "{\"layouter\" : \"flexible\", \"format\": { \"dataencoding\": \"binary\", \"order\": \"big\" }, \"elements\" : [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"records\", \"id\": \"models\", \"elements\": [ { \"type\": \"int16\", \"id\": \"boxFound\" }, { \"type\": \"int16\", \"id\": \"width\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"height\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"length\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"xMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"zMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yawAngle\" }, { \"type\": \"int16\", \"id\": \"qualityLength\" }, { \"type\": \"int16\", \"id\": \"qualityWidth\" }, { \"type\": \"int16\", \"id\": \"qualityHeight\" } ] }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
                "PcicTcpResultSchema": "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"ascii\" }, \"elements\": [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"blob\", \"id\": \"normalized_amplitude_image\" }, { \"type\": \"blob\", \"id\": \"distance_image\" }, { \"type\": \"blob\", \"id\": \"x_image\" }, { \"type\": \"blob\", \"id\": \"y_image\" }, { \"type\": \"blob\", \"id\": \"z_image\" }, { \"type\": \"blob\", \"id\": \"confidence_image\" }, { \"type\": \"blob\", \"id\": \"diagnostic_data\" }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
                "RtspOverlayStyle": "{\"ROI\": {\"default\": {\"visible\": true, \"filled\": false, \"use_symbol\": false, \"label_alignment\": \"top\", \"label_content\": \"\", \"label_background\": \"black\", \"label_fontsize\": 8, \"label_failonly\": false}, \"model_defaults\": {}, \"specific\": {} } }",
                "TemplateInfo": "",
                "TriggerMode": "1",
                "Type": "Camera"
            }
        ],
        "Device": {
            "ActiveApplication": "1",
            "ArticleNumber": "O3D303",
            "ArticleStatus": "AD",
            "Description": "",
            "DeviceType": "1:2",
            "EIPConsumingSize": "8",
            "EIPProducingSize": "450",
            "EnableAcquisitionFinishedPCIC": "false",
            "EthernetFieldBus": "1",
            "EthernetFieldBusEndianness": "0",
            "EvaluationFinishedMinHoldTime": "10",
            "ExtrinsicCalibRotX": "0",
            "ExtrinsicCalibRotY": "0",
            "ExtrinsicCalibRotZ": "0",
            "ExtrinsicCalibTransX": "0",
            "ExtrinsicCalibTransY": "0",
            "ExtrinsicCalibTransZ": "0",
            "IODebouncing": "true",
            "IOExternApplicationSwitch": "0",
            "IOLogicType": "1",
            "IPAddressConfig": "0",
            "ImageTimestampReference": "1524246144",
            "Name": "FooBarBaz",
            "OperatingMode": "0",
            "PNIODeviceName": "",
            "PasswordActivated": "false",
            "PcicProtocolVersion": "3",
            "PcicTcpPort": "50010",
            "PcicTcpSchemaAutoUpdate": "false",
            "SaveRestoreStatsOnApplSwitch": "true",
            "ServiceReportFailedBuffer": "15",
            "ServiceReportPassedBuffer": "15",
            "SessionTimeout": "30",
            "TemperatureFront1": "3276.7",
            "TemperatureFront2": "3276.7",
            "TemperatureIMX6": "53.1839981079102",
            "TemperatureIllu": "60.6",
            "UpTime": "3.13861111111111"
        },
        "Net": {
            "MACAddress": "00:02:01:40:7D:96",
            "NetworkSpeed": "0",
            "StaticIPv4Address": "192.168.0.69",
            "StaticIPv4Gateway": "192.168.0.201",
            "StaticIPv4SubNetMask": "255.255.255.0",
            "UseDHCP": "false"
        },
        "Time": {
            "CurrentTime": "1524246142",
            "NTPServers": "",
            "StartingSynchronization": "false",
            "Stats": "",
            "SynchronizationActivated": "false",
            "Syncing": "false",
            "WaitSyncTries": "2"
        },
        "_": {
            "Date": "Tue May 15 13:38:32 2018",
            "HWInfo": {
                "Connector": "#!02_A300_C40_02452814_008023176",
                "Diagnose": "#!02_D322_C32_03038544_008023267",
                "Frontend": "#!02_F342_C34_17_00049_008023607",
                "Illumination": "#!02_I300_001_03106810_008001175",
                "MACAddress": "00:02:01:40:7D:96",
                "Mainboard": "#!02_M381_003_03181504_008022954",
                "MiraSerial": "0e30-59af-0ef7-0244"
            },
            "SWVersion": {
                "Algorithm_Version": "2.1.5",
                "Calibration_Device": "00:02:01:40:7d:96",
                "Calibration_Version": "0.9.0",
                "Diagnostic_Controller": "v1.0.33-92f88d349f-dirty",
                "IFM_Recovery": "unversioned",
                "IFM_Software": "1.23.1522",
                "Linux": "Linux version 3.14.34-rt31-yocto-standard-00009-ge4ab4d94f288-dirty (jenkins@dettlx190) (gcc version 4.9.2 (GCC) ) #1 SMP PREEMPT RT Tue Mar 13 16:06:07 CET 2018",
                "Main_Application": "1.0.33"
            },
            "ifm3d_version": 900
        }
    }
}
```

Chaining together Linux pipelines works just as it does in `ifm3d`. For
example, using a combination of `dump` and `config` to set a new name on the
camera would look like:

```
$ rosrun ifm3d dump | jq '.ifm3d.Device.Name="My 3D Camera"' | rosrun ifm3d config
$ rosrun ifm3d dump | jq .ifm3d.Device.Name
"My 3D Camera"
```

**NOTE:** If you do not have `jq` on your system, it can be installed with: `$ sudo apt-get install jq`

For the `config` command, one difference between our ROS implementation and the
`ifm3d` implementation is that we only accept input on `stdin`. So, if you had
a file with JSON you wish to configure your camera with, you would simply use the
file I/O redirection facilities of your shell (or something like `cat`) to feed
the data to `stdin`. For example, in `bash`:

```
$ rosrun ifm3d config < camera.json
```

Beyond the requirement of prefacing your command-line with `rosrun` to invoke
the ROS version of these tools, they operate the same. To learn more about the
functionality and concepts, you can read the docs
[here](https://github.com/lovepark/ifm3d/blob/master/doc/configuring.md).
