int main(int argc, char **argv)
{
    //Subscriber

    //End Subscriber

    //Publisher

    //End Publisher

    ros::Rate loop_rate(50);

    ROS_INFO("Started Parking Node");

    while(ros::ok())
    {
        ROS_INFO("Parking Test, spin once");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
