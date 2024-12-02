#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi


    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# r, p and y in rads
def plan_joint_space_orientation(group, r, p, y, rotate=0, lift=0, bend=0, scale=1, bool_wait = True):
    joint_target = group.get_current_joint_values()
    joint_target[0] += scale * rotate  # Affects Base (Rotating)
    joint_target[1] += scale * lift  # Affects Shoulder (Lifting)
    joint_target[2] += scale * bend  # Affects Elbow (Bending)
    joint_target[3] += scale * r  # Affects Wrist 1 (Roll)
    joint_target[4] += scale * p  # Affects Wrist 2 (Pitch)
    joint_target[5] += scale * y  # Affects Wrist 3 (Yaw)
    succeeded = group.go(joint_target, wait=bool_wait)
    return succeeded

def plan_cartesian_path_orientation(group, axis, angle_degrees, scale=1):
    waypoints = []

    # Start with the current pose
    start_pose = group.get_current_pose().pose

    # Convert the desired rotation into a quaternion
    angle_radians = math.radians(angle_degrees)
    half_angle = angle_radians / 2.0
    sin_half_angle = math.sin(half_angle)
    cos_half_angle = math.cos(half_angle)

    # Create the quaternion for the desired rotation
    if axis == 'x':
        rotation_quaternion = [sin_half_angle, 0, 0, cos_half_angle]  # Rotate around X-axis
    elif axis == 'y':
        rotation_quaternion = [0, sin_half_angle, 0, cos_half_angle]  # Rotate around Y-axis
    elif axis == 'z':
        rotation_quaternion = [0, 0, sin_half_angle, cos_half_angle]  # Rotate around Z-axis
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")

    # Extract the current orientation
    current_orientation = [
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w,
    ]

    # Multiply the current orientation with the desired rotation quaternion
    new_orientation = quaternion_multiply_manual(rotation_quaternion, current_orientation)

    # Set the new orientation
    rotated_pose = copy.deepcopy(start_pose)
    rotated_pose.orientation.x = new_orientation[0]
    rotated_pose.orientation.y = new_orientation[1]
    rotated_pose.orientation.z = new_orientation[2]
    rotated_pose.orientation.w = new_orientation[3]

    waypoints.append(rotated_pose)

    # Plan the Cartesian path
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # step size (meters)
        0.0          # jump threshold
    )

    return plan, fraction

def quaternion_multiply_manual(q1, q2):
    """
    Manual quaternion multiplication.
    Args:
        q1: First quaternion as [x, y, z, w]
        q2: Second quaternion as [x, y, z, w]
    Returns:
        The resulting quaternion [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return [x, y, z, w]


def plan_cartesian_path_orientation(group, axis, angle_degrees, scale=1):
    waypoints = []

    # Start with the current pose
    start_pose = group.get_current_pose().pose

    # Convert the desired rotation into a quaternion
    angle_radians = math.radians(angle_degrees)
    half_angle = angle_radians / 2.0
    sin_half_angle = math.sin(half_angle)
    cos_half_angle = math.cos(half_angle)

    # Create the quaternion for the desired rotation
    if axis == 'x':
        rotation_quaternion = [sin_half_angle, 0, 0, cos_half_angle]  # Rotate around X-axis
    elif axis == 'y':
        rotation_quaternion = [0, sin_half_angle, 0, cos_half_angle]  # Rotate around Y-axis
    elif axis == 'z':
        rotation_quaternion = [0, 0, sin_half_angle, cos_half_angle]  # Rotate around Z-axis
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")

    # Extract the current orientation
    current_orientation = [
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w,
    ]

    # Multiply the current orientation with the desired rotation quaternion
    new_orientation = quaternion_multiply_manual(rotation_quaternion, current_orientation)

    # Set the new orientation
    rotated_pose = copy.deepcopy(start_pose)
    rotated_pose.orientation.x = new_orientation[0]
    rotated_pose.orientation.y = new_orientation[1]
    rotated_pose.orientation.z = new_orientation[2]
    rotated_pose.orientation.w = new_orientation[3]

    waypoints.append(rotated_pose)

    # Plan the Cartesian path
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # step size (meters)
        0.0          # jump threshold
    )

    return plan, fraction

def plan_cartesian_path_pose(group, x, y, z, w, scale=1):
    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += scale * x  # Move forward/backwards in (x)
    wpose.position.z += scale * z  # Move up/down (z)
    wpose.position.y += scale * y  # Move sideways (y)
    wpose.orientation.w += scale * w  # Rotation of the arm
    waypoints.append(copy.deepcopy(wpose))

    # wpose = group.get_current_rpy().pose
    # wpose.position.x += scale * x  # Move forward/backwards in (x)
    # wpose.position.z -= scale * z  # Move up/down (z)
    # wpose.position.y += scale * y  # Move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


def execute_plan(group, plan, bool_wait  = True):
    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    succeeded = group.execute(plan, wait=bool_wait)
    return succeeded

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

def error_back_initial_state(arm, gripper, y, ymax):
    
    succeeded = False
    print("\033[91m ur10_1 Error Point 1 - Open Gripper \033[0m")
    while (not succeeded) and (y < ymax):
        gripper.set_named_target("open")
        succeeded = gripper.go(wait=True)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    succeeded = False
    print("\033[91m ur10_1 Error Point 2 - Moving arm up \033[0m")
    while (not succeeded) and (y < ymax):
        
        cartesian_plan, fraction = plan_cartesian_path_pose(arm, 0.0, 0, 0.7, 0, 1)
        succeeded = execute_plan(arm, cartesian_plan)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    succeeded = False
    print("\033[91m ur10_1 Error Point 3 - back home \033[0m")
    while (not succeeded) and (y < ymax):
        arm.set_named_target("home")
        succeeded = arm.go(wait=True)
        if succeeded and (y < ymax):
          print("\033[92m Completed trajectory succesfully \033[0m")
        y+=1

    if (not succeeded) and (y > ymax):
        print("\033[91m ur10_1 Error Point 4 - Too many errors exiting \033[0m")
        moveit_commander.roscpp_shutdown()
        #error_back_initial_state(arm, gripper, 0, ymax)

def hanging_pose(arm, gripper):
    current_joint = arm.get_current_joint_values()

    # Rotate Base Link
    plan_joint_space_orientation(
        group=arm,
        rotate=(math.radians(90)-current_joint[0]),
        lift=0,
        bend=0,
        r=0, 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Lift Shoulder Link
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=(-math.radians(90)-current_joint[1]),
        bend=0,
        r=0, 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Lift Elbow Link
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=0,
        bend=(-math.radians(90)-current_joint[2]),
        r=0, 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Roll Wrist
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=0,
        bend=0,
        r=(-math.radians(90)-current_joint[3]), 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Pitch Wrist
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=0,
        bend=0,
        r=0, 
        p=(math.radians(90)-current_joint[4]), 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Align Gripper Position
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=0,
        bend=0,
        r=0, 
        p=0, 
        y=(math.radians(0)-current_joint[5]),
        scale=1,
        bool_wait = True
    )
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True) 
    # print("Waypoint Hanging Pose Coordinate:\n",arm.get_current_pose())

def finding_pose(arm, pose_x, pose_y, pose_z, offset_x=0.002, offset_y=0.002):
    current_pose = copy.deepcopy(arm.get_current_pose().pose)

    cartesian_plan, fraction = plan_cartesian_path_pose(
        group=arm, 
        # x=(pose_x-current_pose.position.x), 
        x=(pose_x-current_pose.position.x-offset_x),
        y=0, 
        z=0, 
        w=0, 
        scale=1)
    execute_plan(arm, cartesian_plan)
    cartesian_plan, fraction = plan_cartesian_path_pose(
        group=arm, 
        x=0, 
        # y=(pose_y-current_pose.position.y-0.003),
        y=(pose_y-current_pose.position.y-offset_y), 
        z=0, 
        w=0, 
        scale=1)
    execute_plan(arm, cartesian_plan)  
    cartesian_plan, fraction = plan_cartesian_path_pose(
        group=arm, 
        x=0, 
        y=0, 
        z=(pose_z-current_pose.position.z+0.20), 
        w=0, 
        scale=1)
    execute_plan(arm, cartesian_plan)
    # print("Find Cube 1 Coordinate:\n",arm.get_current_pose())

def picking_pose(arm, gripper):
    current_pose = copy.deepcopy(arm.get_current_pose().pose)

    gripper.set_named_target("gripper_open")
    gripper.go(wait=True)
    cartesian_plan, fraction = plan_cartesian_path_pose(
        group=arm, 
        x=0, 
        y=0, 
        z=-0.13, 
        w=0, 
        scale=1)
    execute_plan(arm, cartesian_plan)
    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)
    cartesian_plan, fraction = plan_cartesian_path_pose(
        group=arm, 
        x=0, 
        y=0, 
        z=(1.20-current_pose.position.z), 
        w=0, 
        scale=1)
    execute_plan(arm, cartesian_plan)
    # print("Pick Cube 1 Coordinate:\n",arm.get_current_pose())

def placing_pose(arm, gripper):
    current_joint = arm.get_current_joint_values()

    # Rotate Base Link
    plan_joint_space_orientation(
        group=arm,
        rotate=(math.radians(160)-current_joint[0]),
        lift=0,
        bend=0,
        r=0, 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Lift Shoulder Link
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=(-math.radians(135)-current_joint[1]),
        bend=0,
        r=0, 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    # Roll Wrist
    plan_joint_space_orientation(
        group=arm,
        rotate=0,
        lift=0,
        bend=0,
        r=(-math.radians(80)-current_joint[3]), 
        p=0, 
        y=0,
        scale=1,
        bool_wait = True
    )
    gripper.set_named_target("gripper_open")
    gripper.go(wait=True) 
    # print("Place Cube 1 Coordinate:\n",arm.get_current_pose())

def main():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print("============ Starting dual arms moveit")
    #moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_moveit',
                  anonymous=True)

    PLANNING_GROUP_GRIPPER = "gripper"
    PLANNING_GROUP_ARM = "manipulator"
    PLANNING_NS = ""

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander("%srobot_description"%PLANNING_NS, ns="")

    ## Instantiate the MoveGroupCommander objects.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the ur10
    ## arm and gripper. This interface can be used to plan and execute motions on the ur10
    ## arm and gripper.
    
    #gripper = robot.get_group(PLANNING_GROUP_GRIPPER)
    #arm = robot.get_group(PLANNING_GROUP_ARM)
    arm = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_ARM,"%srobot_description"%PLANNING_NS, ns="")
    gripper = moveit_commander.move_group.MoveGroupCommander(PLANNING_GROUP_GRIPPER, "%srobot_description"%PLANNING_NS, ns="")

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    rospy.sleep(2)
    #scene.remove_world_object("floor")
    
    # publish a demo scene
    #p = geometry_msgs.msg.PoseStamped()
    #p.header.frame_id = robot.get_planning_frame()
    #p.pose.position.x = 0.0
    #p.pose.position.y = 0.0
    #p.pose.position.z = -0.01
    #p.pose.orientation.w = 1.0
    #scene.add_box("floor", p, (2.0, 2.0, 0.02))

    cube_red_x = -0.528712
    cube_red_y = -0.497973
    cube_red_z = 0.748964  

    cube_yellow_x = -0.624805
    cube_yellow_y = -0.597791
    cube_yellow_z = 0.748964  

    dropbox_x = 0.701084459759
    dropbox_y = -0.371338030476
    dropbox_z = 0

    # p = geometry_msgs.msg.PoseStamped()
    # p.header.frame_id = arm.get_planning_frame()

    # p.pose.position.x = cube_red_x
    # p.pose.position.y = cube_red_y
    # p.pose.position.z = cube_red_z
    # p.pose.orientation.w = 1.0
    # scene.add_box("cube_custom_red_2", p, (0.05, 0.05, 0.05))

    # p.pose.position.x = -0.6
    # p.pose.position.y = -0.6
    # p.pose.position.z = 0.75
    # p.pose.orientation.w = 1.0
    # scene.add_box("cube_blue", p, (0.05, 0.05, 0.05))

    # p.pose.position.x = -0.7
    # p.pose.position.y = -0.7
    # p.pose.position.z = 0.75
    # p.pose.orientation.w = 1.0
    # scene.add_box("cube_yellow", p, (0.05, 0.05, 0.05))

    # Dynamically retrieve and assign the gripper link
    gripper_link = gripper.get_end_effector_link()

    # # Allow the gripper to touch the cube during grasping
    # gripper.set_support_surface_name("cube_red")  # Specify the object

    # # Get the gripper's end-effector link
    # gripper_link = gripper.get_end_effector_link()

    # print("Gripper's end-effector link: " + gripper_link)
    
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print( "============ Reference frame: %s" % arm.get_planning_frame() )

    ## We can also print the name of the end-effector link for this group
    print( "============ Reference frame: %s" % arm.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print( "============ Robot Groups:" )
    print( robot.get_group_names())

    ## Sometimes for debugging it is useful to print the entire state of the robot.
    print( "============ Printing robot state" )
    print( robot.get_current_state())
    print( "============" )
    
    # arm.set_planner_id("RRT")
    # arm.set_num_planning_attempts(15)
    # arm.allow_looking(True)
    # arm.allow_replanning(True)

    # gripper.set_planner_id("RRTConnect")
    # gripper.set_num_planning_attempts(15)
    # gripper.allow_replanning(True)
    # gripper.allow_looking(True)


    # Planning to a Pose goal
    # ^^^^^^^^^^^^^^^^^^^^^^^
    # We can plan a motion for this group to a desired pose for the 
    # end-effector

    current_pose = copy.deepcopy(arm.get_current_pose().pose)
    print("Initial Pose Coordinate:\n",arm.get_current_pose())

    arm.set_named_target("home")
    arm.go(wait=True)
    print("Home Pose Coordinate:\n",arm.get_current_pose())

    gripper.set_named_target("gripper_close")
    gripper.go(wait=True)

    # Hanging Pose
    hanging_pose(arm, gripper)
    print("Waypoint Hanging Pose Coordinate:\n",arm.get_current_pose())

    # Find Cube 1
    finding_pose(arm, cube_red_x, cube_red_y, cube_red_z, offset_x=0.001, offset_y=0.001)
    print("Find Cube 1 Coordinate:\n",arm.get_current_pose())

    # Pick Cube 1
    picking_pose(arm, gripper)
    print("Pick Cube 1 Coordinate:\n",arm.get_current_pose())

    # Placing Cube 1
    placing_pose(arm, gripper)
    print("Place Cube 1 Coordinate:\n",arm.get_current_pose())

    # Hanging Pose
    hanging_pose(arm, gripper)
    print("Waypoint Hanging Pose Coordinate:\n",arm.get_current_pose())

    # Find Cube 2
    finding_pose(arm, cube_yellow_x, cube_yellow_y, cube_yellow_z, offset_x=0.001, offset_y=0.001)
    print("Find Cube 2 Coordinate:\n",arm.get_current_pose())

    # Pick Cube 2
    picking_pose(arm, gripper)
    print("Pick Cube w Coordinate:\n",arm.get_current_pose())

    # Placing Cube 2
    placing_pose(arm, gripper)
    print("Place Cube 2 Coordinate:\n",arm.get_current_pose())

    arm.set_named_target("home")
    arm.go(wait=True)

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

