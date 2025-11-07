# ssh to mur620b and run roscore (assumes key-based authentication is set up)
ssh rosmatch@mur620b 'bash -s' << 'ENDSSH'
#!/bin/bash
source .bashrc 
roscore
roslaunch mur_launch_hardware mur620b.launch 
ENDSSH



# Run the GUI application locally
source ~/.bashrc
rosrun match_claw_machine main.py


