(
    trap 'kill 0' SIGINT;
    python bootcamp_harness/rclpy/broker.py &
    python front_cam.py &
    python mavros_launch.py &
    wait
)