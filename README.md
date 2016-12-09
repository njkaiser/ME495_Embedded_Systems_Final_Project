ME495 Embedded Systems in Robotics: Final Project
==============
#### Baxter the Barista
#### Group 6: Nate Kaiser, Brianna Odom, Chainatee Tanakulrungson, Jiarui Yang

## Introduction / Overview of Project
TODO: ADD INTRO

The project can be found at [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project][1]


## Overview of Functionalities
### Master Node


### Vision Processing


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





another [link][2].

[1]: [https://github.com/njkaiser/ME495_Embedded_Systems_Final_Project]
[2]: http://example.org/
