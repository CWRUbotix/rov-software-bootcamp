# Challenge 1 - Part 2: Displaying Camera Streams

Just controlling thrusters isn't enough to pilot a robot. We also need to see what we're doing. We have cameras on the robot that are connected to ROS nodes on the surface laptop. These nodes publish messages containing each frame of video that arrives from the cameras.

To receive these frames, we'll subscribe to the topic `front_cam/image_raw`, convert the images to a format that PyQt supports, then use `setPixmap` to display the frames to a PyQt `QLabel` widget.

 - For testing, we'll use a video from a previous robot's forward camera as the input for the camera stream node.
 # TODO: DOWNLOAD VIDEO
 - Subscriptions are very computation heavy, as they are constantly checking whether a new message has come in.