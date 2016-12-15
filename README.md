ME495 Embedded Systems in Robotics: Final Project
==============
#### *Baxter the Barista*
#### *Group 6: Nate Kaiser, Brianna Odom, Chainatee Tanakulrungson, Jiarui Yang*

## Introduction / Overview of Project
The objective of our project was to have [Baxter][99] work as a personal barista. Given a desired drink, he would search through his inventory and pour a combination of ingredients into a shaker held in his other hand. This would be accomplished by sending him a given input of ingredients he has available on the table, and he would then execute the task by searching through them, grabbing the desired combination, and pouring them into the shaker.  

**Procedure:**   
1. Baxter sweeps the table looking for ingredients with his hand camera   
2. The user inputs a desired drink recipe to have mixed   
3. He goes to the given positions of the desired drinks   
4. And grabs each ingredient one-at-a-time   
5. He pours each into the mixing cup in his left hand   
6. And continues this procedure until the mixture is completed   

![Screenshot](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/media/baxter_pouring.JPG)

The project can be found on GitHub at [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project][0] and video recordings of the demonstration at [https://www.youtube.com/watch?v=ngZCMMJjaLA][1] or [https://vimeo.com/195121510][2]


## Overview of Functionalities
### Master Node & User Input
The [`master`][3] node provides a framework for the overall package functionality, acting as a high level manager to each of the underlying nodes. After initializing and waiting for all required [custom services][4], `master.py` calls the [`ingredient_search`][5] node to extract all ingredient positions as a `list` type object. The user enters the indices of the ingredients they wish to have Baxter mix, with the index values starting at 1 and incrementing from left to right. Master then checks the input and asks for it again if the input is invalid - the input must be integers separated by spaces.

Once the initial steps are completed and user input received, Baxter calls [`grab_object_miss_detecting.py`][6] service to grab the ingredient of interest. If the grab is executed successfully, the [`left_limb_moving`][7] node is then called to bring the left hand to the correct position for pouring on the `/left_limb_command` service topic. Finally, the pour is executed with a `pour` service call, triggering a callback in [`pour.py`][8], and the ingredient is returned to its original position.

This process is repeated for each of the user-input indices, until all ingredients have been mixed. After all ingredients have been picked, poured, and returned, the left hand brings the mixing cup to the counter for service.


### Vision Processing
The vision processing node [`vision.py`][9] is called during the initial ingredient search sweep. It subscribes to `/cameras/right_hand_camera/image` and `/robot/range/right_hand_range/state` topics, and publishes a `Point()` object to `/object_image_command` containing the x- and y-pixel differences between object center and image center. The z-coordinate contains the infrared range sensor data, in case it's beneficial to incorporate it into the vision algorithms in future improvements or expansions.

The node works by using Baxter's right hand camera image and [*OpenCV*][10] to extract only the pixels whose HSV values correspond to red (in the ranges [0, 10] and [170, 179]). The centroid of any large contiguous red areas are extracted, processed, and published for use by the IK and movement services discussed below. The message is published using a [rospy timer][11] to lower the corresponding image callback's computational footprint.

The original intent of vision processing was to implement visual servoing as the hand approached the object, but time considerations precluded it working robustly enough for use given Baxter's dynamic challenges in such a short time frame. The node runs as expected and can be found in the [`unused/`][12] directory.


### AR Tag Tracking
Ar_tag tracking using [`ar_track_alvar`][13] is a tracking method in ROS that gives pose estimates to an object and provides a unique way to identify a particular object in a set of objects that have uniform or similar attributes.

![Examples of ar_tags](http://mirror-eu.wiki.ros.org/attachments/ar_track_alvar/markers9to17.png)

The [`ar_detect.py`][15] node that was written for this project provided an alternative to object color detection to identify each object. It subscribes to the topics `cameras/right_hand_camera/image`, `cameras/right_hand_camera/camera_info`, and `/ar_pose/marker` on Baxter and publishes the x,y,z coordinates and ID of the object.

Particular issues stemming from resolution and lighting deterred us from integrating this node into our project. The [`object_detect.py`][16] node was used in its place.

Two good sources for tutorials and information on using `ar_track_alvar` and AR tags:   
1. [Github for ar_track_alvar by Sniekum](https://github.com/sniekum/ar_track_alvar)  
2. [Former Northwestern University MSR students' mini project](https://github.com/ablarry91/ros-tag-tracking)  

### Inverse Kinematics & Motion Planning
One of the main challenges for this project was to have Baxter maintain a vertical orientation for each of the bottles he poured. This section is broken down into how to move Baxter to a preferred point in space, how to maintain the bottle's vertical orientation along the path, and the code behind it.

##### Moving Baxter Using `IKSolver`
IKSolver is a service provided within Baxter. The protocol uses the desired pose as a [`PoseStamped`][19] message. The `IKSolver` then finds the joint angles required to move from the present joint state to the called state.

This service is very easy to use. However, some issues may yield invalid solutions. One such example is when the `Pose` portion of `PoseStamped` is out of Baxter's reachable workspace or the default seed angles yield no solution.

To fix such problems, the user has to ensure the pose is in the reachable workspace. If it is, `IKSolver` can be called repeatedly, or different seed angles can be given using strategies such as randomization.

##### Maintaining Vertical Orientation
In order to ensure Baxter doesn't spill any liquid, we set the `Quaternion` field in the requested `PoseStamped` message to a constant value. Moreover, since we have to ensure that the container will not tilt while moving from one point to the next, we move incrementally by segmenting the path into small, discrete steps. With small enough steps, the `IKSolver` will almost always return the desired joint solution (the one that produces small joint movements), ensuring the bottle does not twist too far off-center or flip over during an unwanted inverting of his arm.

##### Code Implemenation
To facilitate an easy use of this functionality, we made a custom service message called [`move.srv`][20] used to request movement from the node [`move.py`][21], which takes arguments for limb side, desired x-y-z position, desired quaternion, speed multiplier, and optionally a number of discrete steps to take along the trajectory. If this optional argument is passed a zero, the node calculates its own optimized step count instead.


### Gripper Control & Grasping
The grabbing of objects is accomplished using the node [grab_object_miss_detecting.py][22]. The node service begins when the starting position is received from the master node. The arm travels to this position and the IR sensor data is then checked by subscribing to `/robot/range/right_hand_range/state` of message type `Range`. This message gives us the depth distance of the object in the gripper frame, which is passed to the movement service to approach the bottle. Once the distance measurement is less than 0.06 m, it stops advancing and closes the gripper.

Some difficulties were encountered when incorporating the IR sensor measurement. Since the sensor is a few centimeters off-center from the gripper, and since our objects are fairly narrow, sometimes the sensor is not able to see the bottle at close range. Thus, we set a threshold to ignore measurement values greater than 0.2 m and average the measurement over 20 the previous values. This greatly increases our chance of successfully estimating the correct distance.

Once positioned and a grasp has been initiated, a service call to the custom service [`grasp_status`][23] is invoked. It takes an integer 1 or 0 to tell it close() function was being called. Then it will detect if the object is actually being successfully grabbed. If it is, then return int 1, otherwise 0. The service provider [`check_holding_object.py`][24] has two criteria, position and force, to determine if an object is grasped properly. If no force is detected and when the gripper closes, it returns a failed status.

In addition, in order for the `check_holding_object.py` to work properly, certain grasping parameters were set in [`grab_object_miss_detecting.py`][22]. Important parameters are `moving_force` - the force that, when a grasp is executed, will be applied until the gripper reaches the commanded position. In our code this position is 0. `holding_force` is the force that the gripper will apply after the position command is complete. The `dead_band` parameter is the position error threshold where a move is considered successful. All parameters were set by performing several tests and hard-coding the values into the software. As is it works ~90% of the time. The feature is currently available, but Baxter will not react according to this feedback. In the future, an addition can be implemented to enhance Baxter's behavior by incorporating this feedback into a feature that retries grasping if he misses an object.


### Left Limb
The left arm simply sends position requests to `move.py`. [`left_limb_moving.py`][7] uses the ROS standard service message `SetBool`, which sends a boolean to the server and returns both a boolean and a string message. The node contains two primary functions, `come_to_center` and `go_back`, corresponding to the operation of moving the left arm to the pour position (center of Baxter's body) or going back to original state. Note that the container bottle is simply placed in Baxter's hand prior to the demo instead of having him grab the bottle from the table automatically to save time, although this is something that could easily be adapted to the left arm from the right.


<!-- ## File Locations -->
[99]: [http://www.rethinkrobotics.com/baxter/]
[0]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project]
[1]: [https://www.youtube.com/watch?v=ngZCMMJjaLA]
[2]: [https://vimeo.com/195121510]
[3]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/master.py]
[4]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/tree/master/srv]
[5]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/ingredient_search.py]
[6]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/grab_object_miss_detecting.py]
[7]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/left_limb_moving.py]
[8]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/pour.py]
[9]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/vision.py]
[10]: [http://opencv.org/]
[11]: [http://wiki.ros.org/rospy/Overview/Time]
[12]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/tree/master/unused]
[13]: [http://wiki.ros.org/ar_track_alvar]
[14]: [http://mirror-eu.wiki.ros.org/attachments/ar_track_alvar/markers9to17.png=20x20]
[15]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/unused/ar_detect.py]
[16]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/object_detect.py]
<!-- THESE LINKS DON'T WORK AS IS WITH GITHUB'S MARKDOWN RENDERING -->
<!-- [17]: [https://github.com/sniekum/ar_track_alvar]
[18]: [https://github.com/ablarry91/ros-tag-tracking] -->
[19]: [http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html]
[20]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/srv/move.srv]
[21]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/move.py]
[22]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/master/src/grab_object_miss_detecting.py]
[23]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/srv/grasp_status.srv]
[24]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/check_holding_object.py]
