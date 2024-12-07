#include "RB5_Client.h"


std::queue<target> targetQueue;

void quaternionToRPY(const tf2::Quaternion& q, float rpy[3]) {
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    rpy[0] = roll * 180.0 / M_PI;  // Roll
    rpy[1] = pitch * 180.0 / M_PI; // Pitch
    rpy[2] = yaw * 180.0 / M_PI;   // Yaw
}

void targetCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {

    float translation[3];
    translation[0] = msg->transform.translation.x;
    translation[1] = msg->transform.translation.y;
    translation[2] = msg->transform.translation.z;

    tf2::Quaternion q(
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z,
        msg->transform.rotation.w
    );
    float rpy[3];
    quaternionToRPY(q, rpy);

    target tempTarget;
    tempTarget.x = translation[0];
    tempTarget.y = translation[1];
    tempTarget.z = translation[2];
    tempTarget.ax = rpy[0];
    tempTarget.ay = rpy[1];
    tempTarget.az = rpy[2];

    targetQueue.push(tempTarget);

    std::cout << "Translation: ["
              << translation[0] << ", "
              << translation[1] << ", "
              << translation[2] << "]" << std::endl;

    std::cout << "RPY (degrees): ["
              << rpy[0] << ", "
              << rpy[1] << ", "
              << rpy[2] << "]" << std::endl;
}






int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rb5_ros_client");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/position_of_object", 1, setCoords);

    actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> ac_("motion", true);
    ros::Subscriber targetSub = n.subscribe("/rb5_target", 10, targetCallback);
    
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

    initialize();
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    setRealMode(true);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    setSpeed(0.4);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ROS_INFO("Starting tests.");

    ros::Rate loop_rate(10); // Sleep to regulate loop frequency (10 Hz)

    while(ros::ok()){
        ros::spinOnce();
        if(!targetQueue.empty()){
            target doTarget;
            doTarget = targetQueue.front();
            targetQueue.pop();
            moveTCP(doTarget.x,doTarget.y,doTarget.z,doTarget.ax,doTarget.ay,doTarget.az);
            ac_.sendGoalAndWait(rb5_goal,ros::Duration(2.0));

            std::cout << "Moved to Target: X=" << doTarget.x
                      << ", Y=" << doTarget.y
                      << ", Z=" << doTarget.z
                      << ", AX=" << doTarget.ax
                      << ", AY=" << doTarget.ay
                      << ", AZ=" << doTarget.az
                      << std::endl;
        }  

        loop_rate.sleep();
    }

    return 0;
}
