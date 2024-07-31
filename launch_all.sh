(
    trap 'kill 0' SIGINT;
    python bootcamp_harness/rclpy/broker.py &
    python camera_launch.py &
    python mavros_launch.py &
    wait
)