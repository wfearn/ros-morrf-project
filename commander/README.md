##QUICK GUIDE TO THE MORRF COMMANDER GUI

Below are samples of what the GUI will look like:

![Alt text](/commander/readme_images/Morrf_config.png?raw=true "MORRF Config")

![Alt text](/commander/readme_images/MORRF_GUI.jpg?raw=true "MORRF Window")

###MORRF Config
If you click "file" you will see the "Load Image" option. You need to load an image before you can do anything.

The image should be black and white, where black is an obstacle, and white is where an obstacle isn't.

If you don't have your own images, you can find three at ros-morrf-project/commander/data/

The GUI only supports 3 objectives currently (Stealthy, Safely, Quickly). The options that aren't these objectives have to do with how MORRF runs, and many of these options are found under file->advanced options in other GUI versions.

*Number of Iterations* has to do with how many cycles MORRF will do. It runs a variation of RRTstar, so the more iterations the smoother that paths will be, typically.

*Number of trees* gives you more paths that you can select from.

*Segment length* has to do with how long each part of the path is. If its too low MORRF might not find a path, and if its too high your paths will be very rough.

I have no idea what *Method Type* does, but it affects the way MORRF computes the results of the algorithm.

*Launch MORRF* will send off the map you have loaded and the variables you have selected to the MORRF algorithm node, in the project its called "morrf_ros."

Once MORRF returns, the other buttons should activate.

*Pick path* can be ignored if you're not working in our HCMI lab directly because it sends the results from MORRF to Dropbox where another student within the lab picks them up to use in their path selection GUI.

*Continue MORRF* is pretty self explanatory, it will run MORRF again, taking what it has already given you and running further iterations on it. It will run for whatever amount is in the *number of iterations* box.

*Send to Robot* runs the path follower, on this specific GUI it does nothing, but on the ones that use the cameras and apriltags it will have the robot follow the path assuming you have everything set up correctly.


###Image Window

The image window is more or less self explanatory, it displays the image that you select, or, in the case of the apriltag image, the image render of the obstacle setup.

Right clicking will bring up a window, there can be up to 4 options:

####a. Set Goal

This will place the goal marker, it marks where the robot will travel to.

####b. Set Start

This will place the start marker, it marks where the robot will start the path from.

####c. Set Enemy

This will place an enemy marker, which marks an enemy.

####d. Reset

This will reset all current icons, or paths, depending on the GUI you're utilizing.

*Note:* The right click menu will be different depending on the GUI you pick, some will have none, some will only have reset, and others will have all of the options listed above.

##DEBUGGING

This will be by no means comprehensive, but I just wanted to cover the general structure of the programs so you know where to look if you run into problems.

The ros-morrf-project/commander/scripts folder has all of the executables that the launch files run.

If you check the .launch files you can see which they are as they will be assigned to the "type" variable.

Generally the executable will be tied to a config file inside the ros-morrf-project/commander/scripts/commander_gui/ folder.

Each config class has a corresponding window class inside of it that is unique to each config class.

Sorry for the lack of proper OOP, it only occured to me after the fact that there is a lot of redundancy between many of the config and image classes.


That should be everything, if you have further questions feel free to email me at wilson.fearn+hcmilab@gmail.com
