# Challenge 3: Autonomous Flight

Now that we've identified objects in 2D image space, we can write an autonomous mode for the robot to navigate to those objects. This is an unsolved problem for our team: we've tried autonomous flight before, but it's never worked as well as we wanted it to. It will be your job to come up with a good algorithm for controlling the thrusters based on what the robot sees and navigate to the red object. There won't be much explicit instruction in this challenge.

## Running the Simulation Video
There should be a video called `coral_approach.mp4` in your file tree. If not, you should pull/fetch/sync. Edit `front_cam.py` to stream this new video file instead of `front_cam.mp4`. This video is from our simulation, and has a red square representing coral the coral restoration area on a 3D model of one of the 2024 competition props. Tweak your code from Challenge 2 to consistently identify only the restoration area.

## Autonomous Mode
I'd recommend creating a boolean flag as a field of your GUI to represent whether you're in autonomous mode, and publishing something to the thrusters from within the `handle_frame` function if the flag indicates you are autonomous.

## Output
For testing, I'd recommend creating an output for yourself that's easy to see. Something like rendering arrows to the video output on your GUI (see `cv2.putText`) or showing a human-readable output to the console ("turning left", "pitching down", "←", etc.) could be helpful.

## Final Test
Once you have a prototype that you think works pretty well, we can test it in the actual simulation. When we have a pool test day, we'll test any auto flight modes that are ready in the pool with actual props.