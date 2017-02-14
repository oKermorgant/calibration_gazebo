The goal of this package is to allow virtual camera calibration from Gazebo.

A calibration chessboard is spawned inside Gazebo in front of the existing camera, and can be moved in all directions.

The camera topic can then be used to perform classical OpenCV calibration through the camera_calibration package, or custom calibration if other camera projection models are investigated.

In order to spawn and move the landmark, launch `roslaunch calibration_gazebo calibration.launch`

This will try to identify the camera link (actually the first link containing 'camera') and spawn the chessboard in front of the camera. Otherwise the chessboard will be spawn at a fixed position. A basic  GUI can then be used to give velocities to the chessboard.

An example can be found by launching `roslaunch calibration_gazebo camera.launch`, that outputs the images on the '/camera/image' topic.

The default chessboard dimensions are 8x6 with squares of 0.03 m. They can be changed by editing `landmark.xacro`.
