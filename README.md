# skyviper_ros
ROS files related to work with the skyviper drone

skyviper_capture_images : captures images from the drone's camera and publishes it over /skyviper/camera.

pose_publisher: subscribes to /skyviper/camera, does aruco detection and pose estimation and publishes the pose of the camera onboard skyviper wrt a self determined frame in a "world" of multiple aruco markers (not recommended for simple testing). The pose is published over /mavros/vision_position/pose

pose_publisher_single: subscribes to /skyviper/camera, does aruco detection and pose estimation and  publishes the pose of the camera onboard skyviper wrt to a single aruco marker in the camera's scene.(simple and easy for testing). The pose is published over /mavros/vision_position/pose

local_pose_publisher: subscribes to /mavros/local_position/pose and outputs a tf which is used to visualise the position estimated by the FCU by fusion of IMU and Aruco pose estimation data.

Note:
So, one has to run at least the following in different terminals:

roslaunch mavros apm.launch 
(Dont forget to put the right UDP address in the launch file and whitelist vision and local in the apm_plugins.yaml of mavros package)

roslaunch skyviper_capture_images skyviper_cam,launch

roslaunch pose_publisher_single pose_publisher_single.launch

rostopic echo relevant topics to see the outputs and also make sure that your camera is viewing an aruco marker, otherwise you may not get any output in /mavros/local_position/pose 

# important
Enable EKF3 
Set VISO Type to MAV




