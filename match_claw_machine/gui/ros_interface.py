import subprocess
import rospy

PKG_UTIL = "match_claw_machine"
LAUNCH_MOVE_HOME = "move_UR_to_home_pose.launch"
LAUNCH_ENABLE_UR = "enable_all_URs.launch" 

# NEU: exakte Launch-Dateien für Controller-Switch
LAUNCH_TO_TWIST = "turn_on_all_twist_controllers.launch"
LAUNCH_TO_ARM   = "turn_on_all_arm_controllers.launch"
LAUNCH_TRUE_START = "move_to_true_start.launch"  
LAUNCH_KEYBOARD_TO_JOY = "keyboard_to_joy.launch"
LAUNCH_JOY_TO_TWIST = "joy_to_twist_bounded.launch"

def _open_in_terminal(cmd: str):
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"])

def launch_drivers(gui):
    selected_robots = gui.get_selected_robots()
    workspace = gui.workspace_name
    for robot in selected_robots:
        cmd = (
            f"ssh -t -t {robot} "
            f"'source ~/.bashrc; export ROS_MASTER_URI=http://{robot}:11311/; "
            f"source /opt/ros/noetic/setup.bash; source ~/{workspace}/devel/setup.bash; "
            f"roslaunch mur_launch_hardware {robot}.launch; exec bash'"
        )
        print(f"[drivers] {robot}: {cmd}")
        _open_in_terminal(cmd)

def _move_to_initial_pose_for(robot: str, ur_prefix: str):
    move_group_name = "UR_arm_l" if ur_prefix == "UR10_l" else "UR_arm_r"
    if robot == "mur620c" and ur_prefix == "UR10_r":
        home_position = "Home_custom"
    elif robot in ["mur620a", "mur620b"]:
        home_position = "Home_custom"
    else:
        home_position = "Home_custom"

    cmd = (
        f"ROS_NAMESPACE={robot} roslaunch {PKG_UTIL} {LAUNCH_MOVE_HOME} "
        f"tf_prefix:={robot} UR_prefix:={ur_prefix} "
        f"home_position:={home_position} move_group_name:={move_group_name}"
    )
    print(f"[initial-pose] {robot}/{ur_prefix}: {cmd}")
    subprocess.Popen(cmd, shell=True)

def move_to_initial_pose(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return
    if not rospy.core.is_initialized():
        rospy.init_node("simple_gui_move_initial_pose", anonymous=True)
    for robot in selected_robots:
        for ur in selected_urs:
            _move_to_initial_pose_for(robot, ur)

def _switch_controller(robot: str, ur_prefix: str, target: str):
    """target: 'twist' oder 'arm' — nutzt exakt die gewünschten Launchfiles."""
    launch = LAUNCH_TO_TWIST if target == "twist" else LAUNCH_TO_ARM
    cmd = (
        f"ROS_NAMESPACE='{robot}' roslaunch {PKG_UTIL} {launch} "
        f"robot_names:=[\"{robot}\"] UR_prefixes:=[\"{ur_prefix}\"]"
    )
    print(f"[switch-{target}] {robot}/{ur_prefix}: {cmd}")
    _open_in_terminal(cmd)

def switch_to_twist_controller(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return
    for robot in selected_robots:
        for ur in selected_urs:
            _switch_controller(robot, ur, target="twist")

def switch_to_arm_controller(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return
    for robot in selected_robots:
        for ur in selected_urs:
            _switch_controller(robot, ur, target="arm")


def _enable_ur(gui,robot: str, ur_prefix: str):

    # wenn nur ein roboter ausgewählt ist, dann füge [] um den robot namen hinzu
    if len(gui.get_selected_robots()) == 1:
        robot_ = f"[{robot}]"
    # wenn nur eine ur ausgewählt ist, dann füge [] um den ur namen hinzu
    if len(gui.get_selected_urs()) == 1:
        ur_prefix = f"[{ur_prefix}]"

    # cmd = (
    #     f"ROS_NAMESPACE={robot} roslaunch {PKG_UTIL} {LAUNCH_ENABLE_UR} "
    #     f"tf_prefix:={robot_} UR_prefix:={ur_prefix}"
    # )

    cmd = (
        f"roslaunch {PKG_UTIL} {LAUNCH_ENABLE_UR} "
        f"robot_names:={robot_} UR_prefix:={ur_prefix}"
    )
    print(f"[enable-ur] {robot}/{ur_prefix}: {cmd}")
    _open_in_terminal(cmd)

def enable_selected_urs(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return
    for robot in selected_robots:
        for ur in selected_urs:
            _enable_ur(gui, robot, ur)

def start_move_to_true_start(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return

    for robot in selected_robots:
        for ur in selected_urs:
            pose_topic = f"/{robot}/{ur}/ur_calibrated_pose"
            cmd_topic  = f"/{robot}/{ur}/twist"  # laut Vorgabe: Twist (nicht -stamped)
            cmd = (
                f"ROS_NAMESPACE={robot} roslaunch {PKG_UTIL} {LAUNCH_TRUE_START} "
                f"pose_topic:={pose_topic} cmd_topic:={cmd_topic}"
            )
            print(f"[true-start] {robot}/{ur}: {cmd}")
            _open_in_terminal(cmd)

def keyboard_to_joy(gui):
    
    cmd = (
        f"roslaunch {PKG_UTIL} {LAUNCH_KEYBOARD_TO_JOY} "
    )
    print(f"[keyboard-to-joy]: {cmd}")
    _open_in_terminal(cmd)

def joy_to_twist(gui):
    selected_robots = gui.get_selected_robots()
    selected_urs = gui.get_selected_urs()
    if not selected_robots or not selected_urs:
        print("Keine Roboter/URs selektiert.")
        return

    for robot in selected_robots:
        for ur in selected_urs:
            pose_topic = f"/{robot}/{ur}/ur_calibrated_pose"
            cmd_topic  = f"/{robot}/{ur}/twist"  # laut Vorgabe: Twist (nicht -stamped)
            cmd = (
                f"ROS_NAMESPACE={robot} roslaunch {PKG_UTIL} {LAUNCH_JOY_TO_TWIST} "
                f"pose_topic:={pose_topic} cmd_topic:={cmd_topic}"
            )
            print(f"[joy-to-twist] {robot}/{ur}: {cmd}")
            _open_in_terminal(cmd)


def launch_joystick_driver(gui):
    selected_robots = gui.get_selected_robots()
    workspace = gui.workspace_name
    for robot in selected_robots:
        cmd = (
            f"ssh -t -t {robot} "
            f"'source ~/.bashrc; export ROS_MASTER_URI=http://{robot}:11311/; "
            f"source /opt/ros/noetic/setup.bash; source ~/{workspace}/devel/setup.bash; "
            f"rosrun joy joy_node; exec bash'"
        )
        print(f"[joystick_driver] {robot}: {cmd}")
        _open_in_terminal(cmd)