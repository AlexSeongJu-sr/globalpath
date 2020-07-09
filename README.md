# 3D scene graph generation

A robot(Turtlebot3) moves around in an indoor environment and captures 3D point cloud of wanted_objects. <br> 
Point clouds are processed by camera calibration and icp_registration to make complete 3D data.<br>
ROS2(navigation/registration), Detectron2(object detection), Pyrealsense(capturing), Open3D(registration) are combined to implement each functional module. 

input : 2D slam map

![slam_map](./_images/slam_map.png)

output : 3D point cloud(global coordinate tagged)


 ## Process
  1. Extract global path & navigation goals(red : navigation_goals, yellow : path)
  ![global_path](./_images/global_path.png)
  
  2. Navigate to global path
  
  3. Extract local navigation goals<br>
    3.1. Navigate to local goals<br>
    3.2. Capture 3D point cloud<br>
    3.3. Repeat
  
  4. Registration<br>
    - The robot odometry is obtained by ROS2 topic message '/amcl_pose'.
    With camera calibration combined, we can calculate the transformations needed for rough registration.<br>
    - After rough registrations, ICP registration is applied to refine the data.
  
  5. Return to step 2
 
 ## Example
 
 - before registration
![res_separate](./_images/res_separate.png) 
 
 - after registration
![res1](./_images/res_front.png)
![res2](./_images/res_up.png)
 
 filename : tv_[-3.29162005  1.05766124  0.84587418  1.        ] <br>
 left : before icp_registration <br>
 right : after icp_registration(Point-to-point)
 
 Seen from above, left image is a little inaccurate. Since Detectron2 percepts a monitor as tv, we typed 'tv' as a wanted category.
 
 
updating....
 
 
 
 
 
 
 
