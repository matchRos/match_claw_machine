import subprocess
import rospy

def launch_drivers(gui):
    """Startet die Treiber via SSH wie in deinen Dateien."""
    selected_robots = gui.get_selected_robots()
    workspace = gui.workspace_name
    for robot in selected_robots:
        cmd = (
            f"ssh -t -t {robot} "
            f"'source ~/.bashrc; export ROS_MASTER_URI=http://roscore:11311/; "
            f"source /opt/ros/noetic/setup.bash; source ~/{workspace}/devel/setup.bash; "
            f"roslaunch mur_launch_hardware {robot}.launch; exec bash'"
        )
        print(f"[drivers] {robot}: {cmd}")
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"])

def move_to_initial_pose_for(gui, robot, ur_prefix):
    """Entspricht deiner move_to_initial_pose-Logik (Mapping von home_position & move_group_name)."""
    move_group_name = "UR_arm_l" if ur_prefix == "UR10_l" else "UR_arm_r"

    # home_position gemäß deinem Schema
    if robot == "mur620c" and ur_prefix == "UR10_r":
        home_position = "Home_custom"
    elif robot in ["mur620a", "mur620b"]:
        home_position = "Home_custom"
    else:
        home_position = "Home_custom"

    cmd = (
        f"ROS_NAMESPACE={robot} roslaunch ur_utilities move_UR_to_home_pose.launch "
        f"tf_prefix:={robot} UR_prefix:={ur_prefix} "
        f"home_position:={home_position} move_group_name:={move_group_name}"
    )
    print(f"[start-pose] {robot}/{ur_prefix}: {cmd}")
    subprocess.Popen(cmd, shell=True)

def move_to_start_pose(gui):
    """Bewegt alle selektierten URs der selektierten Roboter in die Startpose (wie in deinen Dateien)."""
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return
    if not rospy.core.is_initialized():
        rospy.init_node("simple_gui_move_start_pose", anonymous=True)
    for robot in selected_robots:
        for ur in selected_urs:
            move_to_initial_pose_for(gui, robot, ur)
