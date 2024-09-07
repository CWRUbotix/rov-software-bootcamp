# MATE ROV Software Bootcamp

This bootcamp introduces the most important libraries we use on CWRUbotix MATE ROV:
 - Python
 - ROS 2
 - PyQt
 - OpenCV

## Overview
Bootcamp includes three challenges. The first two challenges don't require ROS 2 to be installed, and instead use a Python package (called a "test harness") that roughly simulates how ROS works. You'll be able to install the test harness on any operating system. You'll need to install the real ROS 2 at least be the third challenge. While you're working on the first two bootcamp challenges, start figuring out how you'll install ROS using [the instructions in our README](https://github.com/CWRUbotix/rov-25?tab=readme-ov-file#setup). If you have *any* questions about installation or the challenges, ask them in our Discord or during the in-person meetings.

### Challenge 1: Making a PyQt GUI
For this challenge, you'll use Python and the PyQt package to make a Graphical User Interface.
It will include buttons which to control the robot's thrusters and a video widget to display the stream from the robot's cameras.
Once this challenge is done, we'll test all the solutions using our robot simulation on laptops that have ROS.

### Challenge 2: Identifying Objects with OpenCV
For this challenge, you'll use the OpenCV package to identify a red button in the frames of video streamed from the robot's forward camera.
You'll display an annotated copy of the video on your PyQt GUI as you receive the frames.
We'll test solutions to this challenge using the simulation as well.

### Challenge 3: Autonomous Flight
You should have ROS set up in time for this challenge! Work on [these installation instructions](https://github.com/CWRUbotix/rov-25?tab=readme-ov-file#setup) while you're going through Challenges 1 & 2.

For this challenge, you need to act on the button position information you gathered in Challenge 2 by driving the robot toward the button. This is an unsolved problem. We tried one possible solution a few years ago, and it sort of worked, but it wasn't consistant enough. If your solution works well, it could end up in the official codebase if the competition runs this challenge again.
We'll test solutions to this challenge in the pool using the actual robot.


## First Step: Setup & Installation
 - [Make a GitHub account](https://github.com/signup).
 - Set up a Git client. That could be [GitHub Desktop](https://desktop.github.com/download/) (recommended) or the [Git CLI](https://git-scm.com/downloads) (for those with more experience).
 - Give your GitHub username to the software lead so they can add you to the CWRUbotix organization and MATE ROV team.
 - Download and install [Visual Studio Code](https://code.visualstudio.com/) (note that *Visual Studio* is a different thing).
 - Check if you already have Python installed by opening your terminal/command prompt/whatever and running:
    ```bash
    python3 --version
    ```
 - Download and install [Python 3.12.5](https://www.python.org/downloads/) if you don't already have a version of Python 3.12 installed.
 - Clone the [rov-software-bootcamp](https://github.com/CWRUbotix/rov-software-bootcamp) repo using your Git client.
 - Open the `rov-software-bootcamp` repo in VSCode. Go to `View > Terminal` to open a terminal. Create a [virtual environment](https://docs.python.org/3/library/venv.html) called `venv` by running:
    ```bash
    python3 -m venv venv
    ```
 - Activate your virtual environment.
    - Windows command prompt:
        ```bat
        venv\Scripts\activate.bat
        ```
    - Windows powershell:
        ```bat
        venv\Scripts\Activate.ps1
        ```
    - Linux/MacOS:
        ```bash
        source venv/bin/activate
        ```
 - Now we can install all of our Python packages to the virtual environment, and it will be easy to get rid of them if necessary by just deleting the `venv` folder that was just created. Run the following to install all the dependencies listed in the `requirements.txt` file:
    ```bash
    pip install -r requirements.txt
    ```
 - Test your installation by running each of these commands in its own terminal. You can open new terminals by clicking the `+` button on the top right of the VSCode terminal.
    ```bash
    python3 bootcamp_harness/rclpy/broker.py
    ```

    ```bash
    python3 basic_pub_launch.py
    ```

    ```bash
    python3 basic_sub_launch.py
    ```
 - You should see messages being sent in the `basic_pub_launch` terminal, and received in the `basic_sub_launch` terminal.
 - Now take a look at the [Challenge 1 instructions](docs/challenge_1.md) to start the first challenge!