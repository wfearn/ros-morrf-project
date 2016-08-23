## H2 How to get the ros-morrf-project up and running on your machine:

Before anything, make sure that you're running Ubuntu 14.04 or a variant that runs it. We will discuss briefly small changes that you need to make if you're running Elementary OS, if your variant is not Elementary then you will be on your own if you run into issues.

#### H4 1. Install ROS
    You can follow the tutorial here: wiki.ros.org/indigo/Installation/Ubuntu

    You may also install Jade, however if what you're doing requires use of the turtlebot, its better to install Indigo. If you install Jade you will not be able to run any of the turtlebot sims or anything like that.

    Elementary OS Only:
        You will be required to do the following command: echo "export ROS_OS_OVERRIDE=elementary" >> ~/.bashrc

        And on the part where you set up your sources.list in the ROS tutorial,
        you need to replace $(lsb_release-sc) with the word "trusty" in order to get it to work correctly.

#### H4 2. Follow the tutorial link at the bottom of the ROS installation page.

    Look where it says "Please proceed to the ROS Tutorials," to set up your ROS Environment.

    Section 3 of "Installing and Configuring your ROS Environment" is the only one you need to follow, it will allow you to set up your catkin workspace. This is where you will put the ros-morrf-project in order to run it.


#### H4 3. Clone the ros-morrf-project and mm_apriltags_tracker into your ~/catkin_ws/src directory

    the links are found here respectively:

        ros-morrf-project: https://github.com/wfearn/ros-morrf-project
        mm_apriltags_tracker: https://github.com/darin-costello/mm_apriltags_tracker

    Don't run catkin_make yet, we need to install dependencies still.

#### H4 4. Install dependencies.

    Go to somewhere that isn't your catkin_ws folder. I personally like to use the Documents folder in the home directory for this, but it doesn't really matter where you go.

    Clone the following two repositories into the folder of your choice:

        1. https://github.com/dqyi11/MORRF
        2. https://github.com/dqyi11/TopologyPathPlanning

    The third repository you will need to use svn. Run the following command:
        svn co https://svn.csail.mit.edu/apriltags

    At this point you will need to install QT and other packages if you don't have them installed already.
    Run the following commands:

        sudo apt-get install qt-sdk

        sudo apt-get install python-xlib

        sudo apt-get install gengetopt

        sudo apt-get install libcgal-dev

        sudo apt-get install ros-indigo-usb-cam

   *Note*: You might have some of these installed already, if you don't when you try and install the libraries we have cloned the console output will let you know.

    a. Go to the MORRF folder that you have cloned

    b. type: mkdir build

    c. type: cd build

    d. type: cmake ..

    e. Once it finishes, type: sudo make install

    f. At this point a bunch of lines should come up showing you that the program is compiling, if it is successful and has no error messages then you've installed the library correctly.

    Repeat steps a through f for the TopologyPathPlanning and apriltags folders.

    The only errors I've run into with these are dependency errors, but I have covered most of them, if not all, with the four package installations above.

    At this point you should be ready to run catin_make

5. cd to ~/catkin_ws and run the catkin_make command

    This may fail at first and you might have to run it a couple times. Check the progress and the error messages.
    Frequently catkin will mark error messages as saying that you just need to run the command again to get it to work.

    If you keep running into the same error at the same place during the compilation, then check and make sure that you have all of the dependencies that we've discussed.

    Try running "source ~/catkin_ws/devel/setup.bash" (It's even a good idea to put this into your ~/.bashrc) and running catkin_make again.

    If this doesn't work then you're on your own.

6. Run the ros-morrf-project.

    The base command is "roslaunch commander ..." where the "..." can be one of several options.

    a. morrf_launch.launch
        This will run the basic GUI. You'll need to load an image to run morrf on by clicking "file" and "load image"
        if you don't have images to run there are 3 that you can choose from in ~/catkin_ws/src/ros-morrf-project/commander/data/

    b. simple_launch.launch
        This will run an even simpler version of morrf_launch. Here you just need to select an image and decide what objectives you want.

    The other launch files will require you to run the following command in order for them to work:
        roslaunch mm_apriltags_tracker launchAll.launch

    They use an apriltag tracking system, so you will just get a white window unless you have a usb camera hooked up to your computer and pointed at the apriltags.

    Apriltags number 50 and 51 correspond to the start and goal icons.
    Apriltags in the 40s correspond to enemy positions
    Apriltags in the 30s correspond to obstacles.

    Read below to find out how to set up your environment correctly.

    c. turtlebot_config.launch
        This will run a window specifically made for the turtlebot. You can find more advanced options to mess with things such as the number of iterations MORRF will go through under file->advanced options

        You will not see obstacles here unless the apriltags are specific numbers. If you open commander/scripts/commander_gui/turtlebot_config.py you will see the variable OBS_DICT which has the numbers of the apriltags that correspond to obstacles, and the width and height of the obstacles they represent.

    d. topology.launch
        This runs a variant of morrf where you select the path that you want morrf to build other paths around. You may only select one objective.

        Once you have your obstacles, and start and end positions, you may click on the white window and create a path for MORRF to base its planning around. All subsequent clicks after the first click will draw paths from where you previously clicked to where you last clicked.

        If the path is undesirable, you can right click and select "reset" to clear the path you have created.

    e. camera_config.launch
        This runs something similar to turtlebot_config.launch except it uses a different costmap generator meant for the sphero robot. The paths generated with this will be farther from the obstacles than what you'll see with the other config files.
