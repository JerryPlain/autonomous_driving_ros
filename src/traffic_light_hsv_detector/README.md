# Package: Traffic Light HSV Detector

## Explanation:
### This package detects traffic lights and it's current state (red, yellow, green). It uses it's own message type (TrafficLight.msg)
### The pipeline is:
1.  The semantic camera detects the localization of the traffic lights. Since the scale of the semantic camera is different to depth- and 
    the RGB cameras its scale needs to be normalized. 
2.  The semantic and the depth camera are placed at the same position, the RGB cameras have an offset. To get the pixel area of each traffic
    light object, the pixel coordinates of the depth  camer need to be projected into the RGB image. This is done by measuring the distance 
    to each object (via depth camera) and use the pinhole-camera-model. The camera intrinsics can be gathered by reading out the camera_info.
3.  The projected RGB-pixel-coordinates are used in the last node. Here a HSV-Filter measures the current traffic light state. 

### How to get started:
1.  Clone this branch
2.  Build repository
    ```
    catkin build traffic_light_hsv_detector 
    ```
3.  open 5 terminals and source 5 times!
    ```
    source devel/setup.bash
    ```
4.  In the first terminal: 
    ```
    roscore
    ```
5.  In the second terminal:
    ```
    roslaunch simulation simulation.launch
    ```
6.   In the third terminal: 
    ```
    rosrun traffic_light_hsv_detector semantic_depth_median_node 
    ```
7.  In the forth terminal: 
    ```
    rosrun traffic_light_hsv_detector traffic_light_detection
    ```
8.  In the fith terminal: 
    ```
    rosrun traffic_light_hsv_detector color_detector_node
    ```


## Comment:
### This is not the final package. It should be fine to get started. The next steps for me are:
- create launch file for all nodes
- create a decision logic which traffic light is the relevant one
- (adjust published message type: /traffic_light_color)
