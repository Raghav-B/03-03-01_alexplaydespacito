# alexplaydespacito

## Setting up everything
  - **Creating catkin workspace**                      
      - [Install ROS using these instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu)
      - [Create a catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
      - Run the following command: `vim ~/.bashrc`. Append the following line and save and exit: `source <your catkin workspace path>`.
      - Close all terminals.
  - **Setting up Alex's ROS package**
      - Navigate to your caktin workspace, in the `src` folder, go and clone this repository.
      - Open a new terminal here and run `chmod a+x -R 03-03-01_alexplaydespacito/*`.
      - ***Skip this step if you do not intend to interface betwen two different machines.***                            
      Enter this folder and edit `alex_startup.bash`. In lines 9 and 10, you want to change the IP in ROS_IP to your machine's current IP address. You have to ensure that you're connected to the same network as the Pi though.
      - `cd` back to the catkin workspace, one directory above the current `src` folder.
      - Open a terminal here and run `catkin_make`. Hopefully everything should compile properly.
  - **Running alex**
    - `./alex_startup.bash`. Enter the password of the Pi whenever it asks. It should ask you 3 or 4 times.
    - After about a minute, RVIZ should open up and there should be one terminal that says `CLI node has started`. Use this node to interface with the Pi.
    - Command format: `<w, a, s, d, W, A, S, D> <distance> <speed>` OR `<p, P>` for taking photo. Example: `w 90 90`
    
## Running individual modules
  - Do this if you need to debug a certain module on a single machine without communication across different machines.
    - Ensure you've followed the steps in "Creating catkin workspace".
    - Open a new terminal and run `roscore`
    - Open a new terminal and run `rosrun alex_main_pkg main_node`
    - Open a new terminal and run `rosrun alex_main_pkg cli_node`
    - Open a new terminal and run `rosrun alex_main_pkg camera_node`
    - Now you can test the different nodes easily. Just remember to push your changes to the raspberry pi as well if they're final.
