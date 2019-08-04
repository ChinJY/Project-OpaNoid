-   Refer to this page to install Pigpio on Raspberry
    Pi: <http://abyz.me.uk/rpi/pigpio/download.html>

-   Install VNC solution (e.g. VNC viewer)

-   *Generate motion capture logs using Kinect on PC*

    -   Install kinect
        sdk: <https://www.microsoft.com/en-us/download/details.aspx?id=44561>

    -   Open ```BodyBasics-WPF.sln``` in Microsoft Visual Studio and run it
    
    -   Copy and paste the console output into an empty plain text file
    
    -   Delete all non-numerical items from the output in the beginning and the end, and save it as a text file to generate the motion capture log

        -   Please note that toe motions are not supported by
            kinect, so an additional capture device would be required
    
    -   Copy the motion capture log into ```Software/project_opanoid/walk_data```

-   *Controller program on raspberry pi*

    -   Save ```Software/Raspberry Pi/Opanoid_controller.c``` into your robot's Raspberry Pi 

    -   On your Raspberry Pi, place the motion capture log that you generated using Kinect into the same folder as the controller program

    -   Replace the path in line 29 of the controller program to the that of the motion capture log

    -   Run the controller program to make OpaNoid move in accordance with the motion capture log
    
-   *ROS package*

    -   Copy the ```project_opanoid``` folder into the source folder of your ROS workspace
        
    -   In Terminal, cd to your ROS workspace
    
    -   Type ```source devel/setup.bash``` (source the workspace)
    
    -   Enter different commands to open various functions:
    
        -   ```roslaunch project_opanoid display.launch``` to view the model of OpaNoid in Rviz
        
        -   ```roslaunch project_opanoid world.launch``` to view the model of OpaNoid in a Gazebo simulation
        
             -   Open another tab, go to the root folder of your ROS workspace,  source it and type ```rosrun project_opanoid_project_opanoid_node``` to move the robot using motion capture logs located in the Software/project_opanoid/walk_data folder
             
                 -   Change the file path in ```Software/project_opanoid/src/walk.cpp```, line 15 to that of your movement log in order to play it
                 
        -   ```roslaunch project_opanoid main_DQN.launch``` to make OpaNoid walk in Gazebo using a Deep Q Network Controller (not working at the moment)
    
    -   You can alter the simulated robot robot at ```Software/project_opanoid/urdf/opanoid_full.urdf```
