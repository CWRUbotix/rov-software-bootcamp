# Challenge 1 - Part 2: Displaying Camera Streams

Just controlling thrusters isn't enough to pilot a robot. We also need to see what we're doing. We have cameras on the robot that are connected to ROS nodes on the surface laptop. These nodes publish messages containing each frame of video that arrives from the cameras.

To receive these frames, we'll subscribe to the topic `front_cam/image_raw`, convert the images to a format that PyQt supports, then use `setPixmap` to display the frames to a PyQt `QLabel` widget.

For testing, we'll use a video from a previous robot's forward camera as the input for the camera stream node. If you don't see `front_cam.mp4` in your `rov-software-bootcamp` folder, make sure to sync/fetch in GitHub Desktop (or `git pull` in the Git CLI). If you're on a branch other than main, switch to main or merge main into your branch.

## Signals & Slots

Subscriptions are very computationly heavy, as they are constantly checking whether a new message has come in, so we run them in separate threads to avoid hanging the GUI. Unfortunately, we can't give instructions to the GUI from a different thread, so our callback function won't be able to display the frames it receives to the GUI. To send instructions from another thread to the GUI thread, we have to create a custom `pyqtSignal` and `pyqtSlot`.

`pyqtSignals` are signals that threads can "emit". They work kind of like function calls, with parameters that can be included by the emitting thread. Signals are then connected to `pyqtSlot`s, which are functions that will run in the GUI thread when their signal is emitted.

 - Before adding a signal and slot to our GUI, we'll need to import a whole bunch more stuff:
    ```python
    from threading import Thread
    from numpy.typing import NDArray

    from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
    from PyQt6.QtWidgets import QLabel
    from PyQt6.QtGui import QImage, QPixmap

    from bootcamp_harness.sensor_msgs.msg import Image
    from bootcamp_harness.cv_bridge import CvBridge
    from bootcamp_harness.rclpy.executors import SingleThreadedExecutor
    ```
 - Now we can create a signal as a static field in `ButtonPanel`:
    ```python
    class ButtonPanel(QWidget):
        handle_frame_signal = pyqtSignal(Image)

        def __init__(self) -> None:
            ...
    ```
 - Then we'll need to create a method in `ButtonPanel` to handle image frames that arrive over the signal. We'll use a *decorator* to label it as a PyQt slot that accepts an `Image` (see the imports above where we imported the `Image` type).
    ```python
    class ButtonPanel(QWidget):
        ...
        @pyqtSlot(Image)
        def handle_frame(self, ros_image: Image) -> None:
            print('Got an image!')
    ```
 - Then we can connect our signal to the slot in `ButtonPanel`'s constructor:
    ```python
    def __init__(self) -> None:
        ...
        self.handle_frame_signal.connect(self.handle_frame)
    ```
 - Now any time we emit on `handle_frame_signal` (from any thread), the `handle_frame` function will run in the GUI thread. Let's make the ROS subscription which will be emitting the signal when it receives a frame. First create the node and subscription objects in the `ButtonPanel` constructor. We'll use a message type of `Image`, a topic name of `front_cam/image_raw`, the default QoS, and a lambda function which emits the signal for the callback function.
    ```python
    def __init__(self) -> None:
        ...
        subscriber_node = Node('camera_gui_subscriber')
        subscriber_node.create_subscription(
            Image,
            'front_cam/image_raw',
            lambda message: self.handle_frame_signal.emit(message),
            QoSPresetProfiles.DEFAULT.value
        )
    ```
 - To make the subscription poll for new messages without freezing the GUI, we must "spin" the node in a separate thread. We'll use Python's `Thread` and ROS's `Executor` classes to run the nodes `spin` function on a separate thread:
    ```python
    def __init__(self) -> None:
        ...
        executor = SingleThreadedExecutor()
        executor.add_node(subscriber_node)
        Thread(target=executor.spin, daemon=True).start()
    ```

 - To test our signal, we can start up the ROS network and see if the GUI starts printing "Got an image!" to the console. These are the scripts you need to run (remember to run them in separate terminals with your virtual environment active!):
    ```python
    python bootcamp_harness/rclpy/broker.py
    python front_cam.py
    python gui.py
    ```

## Displaying Frames

Now we'll populate the `handle_frame` function. To convert our frame from a ROS `Image` message to an image we can display with PyQt, we first need to convert it to an OpenCV image (which uses the Numpy type `NDArray`), then convert that OpenCV image to a PyQt `QImage`. Once we have the `QImage` we can display it inside a `QLabel` widget in the GUI.

The next sections will go over the details of implementing these conversions, but they won't explicitly lay out all the code you should write. Your goal is to fill in the `handle_frame` function, and to create any other variables/constants necessary elsewhere in the file. Once you've got something you think will work, test by running the scripts above again.

### ROS Image -> OpenCV Image (NDArray)

To convert from a ROS `Image` to an OpenCV `NDArray`, we'll use `CvBridge`, which is a ROS package specifically for this conversion. We'll need to create a CvBridge like this:
```python
cv_bridge = CvBridge()
```
and use it like this:
```python
cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
```

### OpenCV Image (NDArray) -> PyQt QImage

To convert an OpenCV `NDArray` image to a PyQt QImage, we'll use this function from the actual ROV codebase:
```python
def convert_cv_qt(self, cv_img: NDArray, width: int = 0, height: int = 0) -> QImage:
    """Convert from an opencv image to QPixmap."""
    # Color image
    if len(cv_img.shape) == 3:
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w

        img_format = QImage.Format.Format_BGR888

    # Grayscale image
    elif len(cv_img.shape) == 2:
        h, w = cv_img.shape
        bytes_per_line = w

        img_format = QImage.Format.Format_Grayscale8

    else:
        raise ValueError('Somehow not color or grayscale image.')

    qt_image = QImage(cv_img.data, w, h, bytes_per_line, img_format)
    qt_image = qt_image.scaled(width, height, Qt.AspectRatioMode.KeepAspectRatio)

    return qt_image
```

You'll need to copy and paste this function into your code.

For example, for our video frames (which have a width of 890 pixels and a height of 682 pixels) you might define `WIDTH` and `HEIGHT` constants of 890 and 682, then use the function like this:

```python
qt_image: QImage = self.convert_cv_qt(cv_image, WIDTH, HEIGHT)
```

### Rendering the QImage to the GUI
To render a `QImage` to the GUI, we need a `QLabel` widget somewhere on the GUI. If the widget was called `video_frame_label`, then we could display a `QImage` called `qt_image` inside it with:

```python
video_frame_label.setPixmap(QPixmap.fromImage(qt_image))
```

## Improvements
 1. Right now we create two nodes: one has a publisher to publish `PixhawkInstruction`s, and the other has a subscription to receive `Image`s. That's inefficient. Create a single node that does both instead of two different nodes.
 2. Our actual robot has two cameras, one facing forward and one facing down. Create a new widget that displays the down cam footage. The topic will be `down_cam/image_raw`, and the frame dimensions will be the same as the front cam. You'll need to run `down_cam.py` in the network to receive the frames. Note that the two videos we have for the different cam streams are slightly different lengths, so don't worry if the down cam freezes about 50 seconds in when the front cam is still going.
 3. (Harder) Most of the time, we try to give different widgets different functionality, so one widget might be doing video streaming while a separate widget acts as a button panel. Define two different widget classes (one for the ButtonPanel & one for the videos) and add them to one GUI, rather than putting all the functionality on a single widget class.

## Challenge 2
Head over to [challenge 2](challenge_2.md) to continue!