# Connect to NUC using SSH Keys

If you cannot login to the NUC without a password, run the following commands:

```
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/apc_id_rsa
```

Launch the joint trajectory action server on another computer!!!

Run our modified head wobbler code to turn the screen away:

```
rosrun baxter_examples head_wobbler.py
```

Manually move the right arm out of the kinect's view of the shelf. 

```
roslaunch state_machine baxter_perception.launch
roslaunch state_machine baxter_motion_planning.launch
roslaunch googlenet_classification_node googlenet_classifer.launch
roslaunch ros_kinfu ros_kinfu.launch
```

Manually clear the scene to remove the shelf and to accommodate a bug that stops
grasp points working before the scene is cleared.

```
roslaunch state_machine baxter_state_machine.launch
```
