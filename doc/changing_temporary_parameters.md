Changing the temporary application parameters on a running camera
=======================================================

This article explains the facilities provided by `ifm3d-ros` to manually
change temporary application parameters on the fly for a running camera.  

### Exposure times:

The set exposure times are obtained from the camera for every frame received, the units of the exposure times are in microseconds.  
Below command would be used to get the exposure times as an array:

```
$ rostopic echo /ifm3d/camera/exposure_times
```
#### 1-exposure mode  
You can set the exposure time in <b>1-exposure mode</b> by publishing on the
`SetExposureTime` topic:  
```
$ rostopic pub -1 /ifm3d/camera/SetExposureTime ifm3d/SetExposureTime "exposure_usecs: 1000"  
```
#### 2-exposure mode  
You can set the exposure times in <b>2-exposure mode</b> by publishing on the
`SetExposureTimes` topic:  
```
$ rostopic pub -1 /ifm3d/camera/SetExposureTimes ifm3d/SetExposureTimes "exposure_usecs: 2000
exposure_time_ratio:10"  
```
### Channel:
The Frequency channel to be used by your device can be set by publishing on the `SetChannel` topic:  
The below example sets the camera to use channel1.
```
$ rostopic pub -1 /ifm3d/camera/SetChannel ifm3d/SetChannel "channel: 1"
```

<b>Notes:</b>

* Changes done by the above commands are not persistent and are lost when entering edit mode (for e.g calling the Dump or Config services) or turning the device off.
* Appropriate set of parameters should be provided depending on the exposure mode used,
    * "exposure_usecs" only for 1-exposure mode
    * "exposure_usecs" and "exposure_time_ratio" for 2 exposure mode.
    * All other cases are invalid and lead to undefined behaviour.
* O3D Firmware Version 1.23.2xx or above needs to be used for this feature to work correctly.


