ME495 Embedded Systems in Robotics: Final Project
==============
#### Baxter the Barista
#### Group 6: Nate Kaiser, Brianna Odom, Chainatee Tanakulrungson, Jiarui Yang

## Introduction / Overview of Project
TODO: ADD INTRO

The project can be found at [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project][1] and a video recording of the running software at [https://www.youtube.com/watch?v=EI69LpWt1M4][2]


## Overview of Functionalities
### Master Node & User Input
The [`master`][3] node provides a framework for the overall package functionality, acting as a high level manager to each of the underlying nodes. After initializing and waiting for all required [custom services][4], `master.py` calls the [`ingredient_search`][5] node to extract all ingredient positions as a `list` type object. The user enters the indices of the ingredients they wish to have Baxter mix, with the index values starting at 1 and incrementing from left to right. Master then checks the input and asks for it again if the input is invalid - the input must be integers separated by spaces.

Once the initial steps are completed and user input received, Baxter calls the custom [`/grasp`][6] service to grab the ingredient of interest. If the grab is executed successfully, the [`/left_limb_command`][7] service from the [`left_arm_moving`][8] node is then called to bring the left hand to the correct position for pouring. Finally, the pour is executed using the [`pour`][9] service located in [`pour.py`][10], and the ingredient is returned to its original position.

This process is repeated for each of the user-input indices, until all ingredients have been mixed. After all ingredients have been picked, poured, and returned, the left hand brings the mixing cup to the counter for service.


### Vision Processing
The vision processing node [`vision.py`][11] is called during the initial ingredient search sweep. It subscribes to `/cameras/right_hand_camera/image` and `/robot/range/right_hand_range/state` topics, and publishes a `Point()` object to `/object_image_command` containing the x- and y-pixel differences between object center and image center. The z-coordinate contains the infrared range sensor data, in case it's beneficial to incorporate it into the vision algorithms in future improvements or expansions.

The node works by using Baxter's right hand camera image and [*OpenCV*][12] to extract only the pixels whose HSV values correspond to red (in the ranges [0, 10] and [170, 179]). The centroid of any large contiguous red areas are extracted, processed, and published for use by the IK and movement services. The message is published using a [rospy timer][13] to lower the corresponding image callback's computational footprint.

The original intent of vision processing was to implement visual servoing as the hand approached the object, but time considerations precluded it working robustly enough for use given Baxter's dynamic challenges in such a short time frame. The node runs as expected and can be found in the [`unused/`][14] directory.


### AR Tag Tracking


### Inverse Kinematics & Motion Planning


### Gripper Control
The grabbing of objects is accomplished by the [grab_object_miss_detecting.py](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/grab_object_miss_detecting.py) node. After the vision processing node  coordinates of the bottles, gripper will go to the corresponding grabbing position. Then IR sensor data was taken by subscribing to `/robot/range/right_hand_range/state` topic with `Range` message type. This message will give us the x coordinate of object in gripper frame. Then using [move.py](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/move.py) to move gripper towards the bottles. Once the distance between bottle and gripper sensor is less than 0.06, it will stop and then close the gripper.

Some difficulties were encountered when incorporate with IR sensor. Since it is a few centermeters off the center frame of the camera, it will be at a slightly different y axis value as detected by the camera. And since our object bottles are fairly thin columns, sometimes it will not see the bottle. Thus, we set a threshold of the max IR sensor data to be 0.2 and average values over 20 samples. This somehow increase our chance of successfully detecting objects.

After call the [close](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/grab_object_miss_detecting.py#L93) function of the gripper, a service call to a custom service [grasp_status](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/srv/grasp_status.srv) is invoked. This service contain one input and one output. It takes an integer 1 or 0 to tell it close() function was being called. Then it will detect if the object is actully being successfully grabbed. If it is, then return int 1, otherwise 0. The service server [check_holding_object.py](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/check_holding_object.py) has two criteria, position and force, to determine if an object is being grabbed. Gripper has force and position function will check the status of the gripper end. If no force/very small force is deteced or very close position between two grippers, it will know that grasping failed. Note that here all the parameters of object size/force being used are all determined by repeating tests and then hard coded into.

In addition, in order for the `check_holding_object.py` to work properly, certain grabbing parameters need to be set by [grab_object_miss_detecting.py](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/grab_object_miss_detecting.py#L54-L57). moving force is the force when exceeded, it will stop achieving the commanded position, in this case is the position 0. Holding force is the force that gripper will continue applying after a position command is completed. And dead band is the position error threshold where a move will be considered successful. Again, all these parameters are set by performing several tests and hard coded into.

[gripper_test.py](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/gripper_test.py) was used to test this feature of detecting if successfully grabbed objects. It works 90% times. But right now after knowing if he has object in hand or not, baxter will not react correspondingly. It will just act as if he has the objects in gripper. Some improvement can be done to enhance baxter's behavior, such as if he missed objects, then retake data from camera/IR sensor, and then regrab the object.

### Left Limb
Left limb mainly works depend on IK service. [left_limb_moving.py](https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project/blob/9554cc2bbe60da78325f366dd5018dc12ccd75ec/src/left_limb_moving.py) will use std_srv `SetBool` service which can sent a boolean and return a bool and a string message. It has two major function, `come_to_center` and `go_back`, corresponding to the operation of moving left limb to the center for getting the pour and go back to original state. Note that for the left limb gripper holding the container bottle, it is "put" into baxter's hand instead of himself grabbing the bottle from the table.


## File Locations


another [link][2].

[1]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project]
[2]: [https://www.youtube.com/watch?v=EI69LpWt1M4]
[3]: [master_node]
[4]: [link_to_services_directory]
[5]: [link_to_initial_sweep_node]
[6]: [link_to_grasp_service_node]
[7]: [/left_limb_command]
[8]: [link]
[9]: [link]
[10]: [link]
[11]: [link]
[12]: [http://opencv.org/]
[13]: [http://wiki.ros.org/rospy/Overview/Time]
[14]: [unused_directory]
[14]: [link]
[16]: [link]
