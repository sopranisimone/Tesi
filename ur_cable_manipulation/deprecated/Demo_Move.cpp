#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>



//Tutorial on how to use moveit to move the robot sequentially
//To move the two arms together, we need a planning group containing both the arms

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_ur5_arm");
    ros::NodeHandle nh;

    ros::start();

    ros::AsyncSpinner spinner(1);
    spinner.start();


    static const std::string UR5_PLANNING_GROUP = "ur5_arm";
    static const std::string UR5E_PLANNING_GROUP = "ur5e_arm";
    static const std::string UR5_HAND_PLANNING_GROUP = "ur5_hand";
    static const std::string UR5E_HAND_PLANNING_GROUP = "ur5e_hand";

    static const std::string PLANNING_GROUP = "dual_arms";


    tf::TransformListener listenTCP_TF; 
    tf::TransformListener listen_ur5_base_TF;
    tf::TransformListener listen_ur5e_base_TF;
    //TransformListener is a subclass of :class:`tf.Transformer` that 
    //subscribes to the ``"/tf"`` message topic, and calls :meth:`tf.Transformer.setTransform`with each incoming transformation message.
    //In this way a TransformListener object automatically stays up to to date with all current transforms. 

    //rosrun tf tf_echo world ur5_base_link
    //rosrun tf tf_echo ur5_base_link ur5_finger_tip
    std::string world = "/world";

    std::string ur5_base_link = "/ur5_base_link";
    std::string ur5e_base_link = "/ur5e_base_link";

    std::string ur5_tcp = "/ur5_finger_tip";
    std::string ur5e_tcp = "/ur5e_finger_tip";
    //std::string target_frame = "/tcp_gripper";  
    tf::StampedTransform transform; 

    // tf::StampedTransform transform_ur5_base;
    // tf::StampedTransform transform_ur5e_base;


    moveit::planning_interface::MoveGroupInterface ur5_move_group_interface(UR5_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5_hand_move_group_interface(UR5_HAND_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5e_move_group_interface(UR5E_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface ur5e_hand_move_group_interface(UR5E_HAND_PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface dual_arms_move_group_interface(PLANNING_GROUP);


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    ur5_move_group_interface.setPlanningTime(10.0);
    ur5e_move_group_interface.setPlanningTime(10.0);
    ur5_hand_move_group_interface.setPlanningTime(10.0);
    ur5e_hand_move_group_interface.setPlanningTime(10.0);
    dual_arms_move_group_interface.setPlanningTime(10.0);
    

    const robot_state::JointModelGroup* ur5_joint_model_group = ur5_move_group_interface.getCurrentState()->getJointModelGroup(UR5_PLANNING_GROUP);
    const robot_state::JointModelGroup* ur5e_joint_model_group = ur5e_move_group_interface.getCurrentState()->getJointModelGroup(UR5E_PLANNING_GROUP);

    const robot_state::JointModelGroup* ur5_hand_joint_model_group = ur5_hand_move_group_interface.getCurrentState()->getJointModelGroup(UR5_HAND_PLANNING_GROUP);
    const robot_state::JointModelGroup* ur5e_hand_joint_model_group = ur5e_hand_move_group_interface.getCurrentState()->getJointModelGroup(UR5E_HAND_PLANNING_GROUP);

    const robot_state::JointModelGroup* dual_arms_joint_model_group = dual_arms_move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ur5_move_group_interface.allowReplanning(true); 
    ur5_move_group_interface.setNumPlanningAttempts(10); 

    ur5e_move_group_interface.allowReplanning(true); 
    ur5e_move_group_interface.setNumPlanningAttempts(10); 

    ur5_hand_move_group_interface.allowReplanning(true); 
    ur5_hand_move_group_interface.setNumPlanningAttempts(10); 

    ur5e_hand_move_group_interface.allowReplanning(true); 
    ur5e_hand_move_group_interface.setNumPlanningAttempts(10); 

    dual_arms_move_group_interface.allowReplanning(true);
    dual_arms_move_group_interface.setNumPlanningAttempts(10);

    ROS_INFO_NAMED("PLANNING","START PLANNING\n\n\n");

    //ros::Duration(5.0).sleep();

    moveit::core::RobotStatePtr ur5_current_state = ur5_move_group_interface.getCurrentState();
    std::vector<double> ur5_joint_group_positions;
    ur5_current_state->copyJointGroupPositions(ur5_joint_model_group, ur5_joint_group_positions);

    moveit::core::RobotStatePtr ur5e_current_state = ur5e_move_group_interface.getCurrentState();
    std::vector<double> ur5e_joint_group_positions;
    ur5e_current_state->copyJointGroupPositions(ur5e_joint_model_group, ur5e_joint_group_positions);

    moveit::core::RobotStatePtr ur5_hand_current_state = ur5_hand_move_group_interface.getCurrentState();
    std::vector<double> ur5_hand_joint_group_positions;
    ur5_hand_current_state->copyJointGroupPositions(ur5_hand_joint_model_group, ur5_hand_joint_group_positions);

    moveit::core::RobotStatePtr ur5e_hand_current_state = ur5e_hand_move_group_interface.getCurrentState();
    std::vector<double> ur5e_hand_joint_group_positions;
    ur5e_hand_current_state->copyJointGroupPositions(ur5e_hand_joint_model_group, ur5e_hand_joint_group_positions);



    robot_state::RobotState start_state(*ur5_move_group_interface.getCurrentState());


    

    //Planning frame: /world, End effector link: /ur5_finger_tip, Reference frame: /world

    //To command wrt base_link need to convert target_pose from world to base_link or viceversa

    ROS_INFO_NAMED("PLANNING", "Planning frame: %s\n", ur5_move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("PLANNING", "End effector link: %s\n", ur5_move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("PLANNING", "Reference frame: %s\n", ur5_move_group_interface.getPoseReferenceFrame().c_str());
    std::cout<<"\n\n\n\n"<<std::endl;

    //PLANNING

    geometry_msgs::PoseStamped current_ur5_arm_pose;
    geometry_msgs::PoseStamped current_ur5e_arm_pose;
    geometry_msgs::PoseStamped target_ur5_arm_pose;
    geometry_msgs::PoseStamped target_ur5e_arm_pose;

    // geometry_msgs::Pose ur5_arm_pose;
    // geometry_msgs::Pose ur5e_arm_pose;


    std::vector<geometry_msgs::Pose> ur5_waypoints;
    std::vector<geometry_msgs::Pose> ur5e_waypoints;
    moveit_msgs::RobotTrajectory ur5_trajectory;
    moveit_msgs::RobotTrajectory ur5e_trajectory;


    // moveit_msgs::CollisionObject ground;
    // ground.header.frame_id = ur5_move_group_interface.getPlanningFrame();
    // ground.id = "ground";
    // shape_msgs::SolidPrimitive primitive_ground;
    // primitive_ground.type = primitive_ground.BOX;
    // primitive_ground.dimensions.resize(3);
    // primitive_ground.dimensions[primitive_ground.BOX_X] = 10.5;
    // primitive_ground.dimensions[primitive_ground.BOX_Y] = 10.5;
    // primitive_ground.dimensions[primitive_ground.BOX_Z] = 0.01;

    // geometry_msgs::Pose ground_pose;
    // ground_pose.orientation.w = 1.0;
    // ground_pose.position.x = 0.0;
    // ground_pose.position.y = 0.0;
    // ground_pose.position.z = -0.01;

    // ground.primitives.push_back(primitive_ground);
    // ground.primitive_poses.push_back(ground_pose);
    // ground.operation = ground.ADD;

    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(ground);

    // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);





    //IMPORTANTE: ora come ora i goal dati sono rispetto al frame world non ai due ur5_base_link e ur5e_base_link
    //es: target_ur5_arm_pose.pose.position.x += 0.10; andrà a modificare la posizione del tcp rispetto al frame world quindi allontanandolo di 10 cm
    // rispetto all'origine del frame world e non rispetto al frame ur5_base_link
    // x del frame world non è uguale a x del frame ur5_base_link (dovrebbe essere la y del frame ur5_base_link)

    int dual_flag = 0;
    switch (dual_flag)
    {
    case 1:
        ROS_INFO_NAMED("DUAL ARM PLANNING","Planning to ready pose\n\n\n");
        dual_arms_move_group_interface.setNamedTarget("ready");
        break;
    
    default:
        
        break;
    }
    

    int arm_flag = 4;
    switch (arm_flag)
    {
        case 1:
            // Set the goals to be the pre-defined/named "ready" pose
            ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning both arms to ready pose\n\n\n");
            ur5_move_group_interface.setNamedTarget("ready");
            ur5e_move_group_interface.setNamedTarget("ready");
            break;

        case 2:
            ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to joint poses\n\n\n");
            ur5_joint_group_positions[0] = -1 / 6;  // -1/6 turn in radians
            ur5_joint_group_positions[3] = -1 / 6;  // -1/6 turn in radians
            ur5_move_group_interface.setJointValueTarget(ur5_joint_group_positions);
            ur5_move_group_interface.setMaxVelocityScalingFactor(0.05);
            ur5_move_group_interface.setMaxAccelerationScalingFactor(0.05);

            ur5e_joint_group_positions[0] = -1 / 6; 
            ur5e_joint_group_positions[3] = -1 / 6; 
            ur5e_move_group_interface.setJointValueTarget(ur5e_joint_group_positions);
            ur5e_move_group_interface.setMaxVelocityScalingFactor(0.05);
            ur5e_move_group_interface.setMaxAccelerationScalingFactor(0.05);
            break;
        
        case 3:
            ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to pose goal\n\n\n");
            current_ur5_arm_pose = ur5_move_group_interface.getCurrentPose();
            target_ur5_arm_pose = current_ur5_arm_pose;

            // listen_ur5_base_TF.lookupTransform(world, ur5_base_link, ros::Time(), transform_ur5_base);
            // listen_ur5_base_TF.transformPose(ur5_base_link, ur5_base_link, current_ur5_arm_pose);

            target_ur5_arm_pose.pose.position.x += 0.10;

            ur5_move_group_interface.setPoseTarget(target_ur5_arm_pose);


            current_ur5e_arm_pose = ur5e_move_group_interface.getCurrentPose();
            target_ur5e_arm_pose = current_ur5e_arm_pose;

            target_ur5e_arm_pose.pose.position.x += 0.10;

            ur5e_move_group_interface.setPoseTarget(target_ur5e_arm_pose);
            break;

        case 4:

            ROS_INFO_NAMED("SINGLE ARM PLANNING","Planning to pose goal with cartesian path\n\n\n");
            listenTCP_TF.waitForTransform(ur5_base_link, ur5_tcp, ros::Time(), ros::Duration(4.0)); //rosrun tf tf_echo ur5_base_link ur5_finger_tip
            listenTCP_TF.lookupTransform(ur5_base_link, ur5_tcp, ros::Time(), transform);

            current_ur5_arm_pose.pose.position.x    = transform.getOrigin().x();
            current_ur5_arm_pose.pose.position.y    = transform.getOrigin().y();
            current_ur5_arm_pose.pose.position.z    = transform.getOrigin().z();
            current_ur5_arm_pose.pose.orientation.x = transform.getRotation().getX();
            current_ur5_arm_pose.pose.orientation.y = transform.getRotation().getY();
            current_ur5_arm_pose.pose.orientation.z = transform.getRotation().getZ();
            current_ur5_arm_pose.pose.orientation.w = transform.getRotation().getW(); 

            start_state.setFromIK(ur5_joint_model_group, current_ur5_arm_pose.pose);
            ur5_move_group_interface.setStartState(start_state);  
            target_ur5_arm_pose = current_ur5_arm_pose; 

            target_ur5_arm_pose.pose.position.x -= 0.1;
            ur5_waypoints.push_back(target_ur5_arm_pose.pose);

            target_ur5_arm_pose.pose.orientation.y -= 1.57;
            ur5_waypoints.push_back(target_ur5_arm_pose.pose);


            target_ur5_arm_pose.pose.position.z -=  0.5;
            std::cout<<current_ur5_arm_pose.pose.position.z<<std::endl;
            ur5_waypoints.push_back(target_ur5_arm_pose.pose);    


            // target_ur5_arm_pose.pose.position.y += 0.72554;
            // std::cout<<current_ur5_arm_pose.pose.position.y<<std::endl;
            // ur5_waypoints.push_back(target_ur5_arm_pose.pose);    
            ur5_move_group_interface.computeCartesianPath(ur5_waypoints, 0.01, 0.0, ur5_trajectory); // 0.01 eef_step, 0.0 jump_threshold
            ur5_waypoints.clear();

            ur5_move_group_interface.execute(ur5_trajectory);
        default:
            break;

    }


    int hand_flag = 4;
    switch (hand_flag)
    {
    case 1:
        ROS_INFO_NAMED("SINGLE HAND PLANNING","Open both hands\n\n\n");
        ur5_hand_joint_group_positions[0] = 0.0225;
        ur5_hand_joint_group_positions[1] = 0.045;
        ur5_hand_move_group_interface.setJointValueTarget(ur5_hand_joint_group_positions);

        ur5e_hand_joint_group_positions[0] = 0.0225;
        ur5e_hand_joint_group_positions[1] = 0.045;
        ur5e_hand_move_group_interface.setJointValueTarget(ur5e_hand_joint_group_positions);
        break;
    
    case 2:
        ROS_INFO_NAMED("SINGLE HAND PLANNING","Plan to gripper pose open\n\n\n");
        ur5_hand_move_group_interface.setNamedTarget("open");
        ur5e_hand_move_group_interface.setNamedTarget("open");
        break;

    default:
        
        break;
    }


    //COLLISION OBJECTS
    //Adding ground to the planning scene

    // moveit_msgs::CollisionObject ground;
    // ground.header.frame_id = ur5_move_group_interface.getPlanningFrame();
    // ground.id = "ground";
    // shape_msgs::SolidPrimitive primitive_ground;
    // primitive_ground.type = primitive_ground.BOX;
    // primitive_ground.dimensions.resize(3);
    // primitive_ground.dimensions[primitive_ground.BOX_X] = 10.5;
    // primitive_ground.dimensions[primitive_ground.BOX_Y] = 10.5;
    // primitive_ground.dimensions[primitive_ground.BOX_Z] = 0.01;

    // geometry_msgs::Pose ground_pose;
    // ground_pose.orientation.w = 1.0;
    // ground_pose.position.x = 0.0;
    // ground_pose.position.y = 0.0;
    // ground_pose.position.z = -0.01;

    // ground.primitives.push_back(primitive_ground);
    // ground.primitive_poses.push_back(ground_pose);
    // ground.operation = ground.ADD;

    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(ground);

    // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);

    //Adding box to the planning scene

    // // moveit_msgs::CollisionObject collision_object;
    // // collision_object.header.frame_id = ur5_move_group_interface.getPlanningFrame();
    // // collision_object.id = "box1";
    //  shape_msgs::SolidPrimitive primitive;
    // // primitive.type = primitive.BOX;
    // // primitive.dimensions.resize(3);
    // // primitive.dimensions[primitive.BOX_X] = 1.0;
    // // primitive.dimensions[primitive.BOX_Y] = 1.5;
    // // primitive.dimensions[primitive.BOX_Z] = 0.5;

    // // geometry_msgs::Pose box_pose;
    // // box_pose.orientation.w = 1.0;
    // // box_pose.position.x = 0.0;
    // // box_pose.position.y = 0.0;
    // // box_pose.position.z = 0.25;

    // // collision_object.primitives.push_back(primitive);
    // // collision_object.primitive_poses.push_back(box_pose);
    // // collision_object.operation = collision_object.ADD;

    // // collision_objects.push_back(collision_object);

    // // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // // planning_scene_interface.addCollisionObjects(collision_objects);


    // // //Adding cylinder to the planning scene
    // moveit_msgs::CollisionObject object_to_attach;
    // object_to_attach.id = "cylinder1";

    // shape_msgs::SolidPrimitive cylinder_primitive;
    // cylinder_primitive.type = primitive.CYLINDER;
    // cylinder_primitive.dimensions.resize(2);
    // cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
    // cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.045;

    // object_to_attach.header.frame_id = ur5_move_group_interface.getEndEffectorLink();
    // geometry_msgs::Pose grab_pose;
    // grab_pose.orientation.w = 1.0;
    // grab_pose.position.z = 0.25;

    // object_to_attach.primitives.push_back(cylinder_primitive);
    // object_to_attach.primitive_poses.push_back(grab_pose);
    // object_to_attach.operation = object_to_attach.ADD;
    // planning_scene_interface.applyCollisionObject(object_to_attach);

    // ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    // ur5_move_group_interface.attachObject(object_to_attach.id, "ur5_finger_tip");


    // Planning to a joint-space goal

    moveit::planning_interface::MoveGroupInterface::Plan ur5_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5e_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_hand_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5e_hand_plan;

    moveit::planning_interface::MoveGroupInterface::Plan dual_arms_plan;

    bool ur5_success = false;
    bool ur5e_success = false;
    bool ur5_hand_success = false;
    bool ur5e_hand_success = false;
    bool dual_arms_success = false;

    //std::cout << arm_flag << " ," << hand_flag << std::endl;

    // if (arm_flag < 4){
    //     std::cout << "Come sei finito qui maledetto \n\n\n\n\n\n\n\n\n\n" << std::endl;
    // }

    if (arm_flag < 4 && arm_flag > 0){
        ur5_success = (ur5_move_group_interface.plan(ur5_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("PLANNING", "Visualizing plan 1 (pose goal) %s\n\n", ur5_success ? "" : "FAILED");

        ur5e_success = (ur5e_move_group_interface.plan(ur5e_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("PLANNING", "Visualizing plan 2 (pose goal) %s\n\n", ur5e_success ? "" : "FAILED");

    }

    if (hand_flag < 3 && hand_flag > 0){
        ur5_hand_success = (ur5_hand_move_group_interface.plan(ur5_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("PLANNING", "Visualizing plan 3 (pose goal) %s\n\n", ur5_hand_success ? "" : "FAILED");

        ur5e_hand_success = (ur5e_hand_move_group_interface.plan(ur5e_hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("PLANNING", "Visualizing plan 4 (pose goal) %s\n\n", ur5e_hand_success ? "" : "FAILED");
    }

    if (dual_flag == 1){
        dual_arms_success = (dual_arms_move_group_interface.plan(dual_arms_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("PLANNING", "Visualizing plan 5 (pose goal) %s\n\n", dual_arms_success ? "" : "FAILED");
    }

    //ros::Duration(5.0).sleep();
    //geometry_msgs::PoseStamped target_pose1 = ur5_move_group_interface.getCurrentPose();

    if (dual_arms_success){
        dual_arms_move_group_interface.move();
    }

    //If the planning is successful, execute the arms motions (move the robot)
    if (ur5_hand_success){
        ur5_hand_move_group_interface.move();
    }

    if (ur5e_hand_success){
        ur5e_hand_move_group_interface.move();
    }

    if (ur5_success){
        ur5_move_group_interface.move();
    }

    if (ur5e_success){
        ur5e_move_group_interface.move();
    }

    ROS_INFO_NAMED("MOVEMENT END","Movement ended\n\n\n\n\n\n");

    ros::shutdown();
    return 0;
}