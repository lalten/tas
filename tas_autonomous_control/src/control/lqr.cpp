#include "lqr.h"

lqr::lqr()
{    
      vel_ref=1.0;      //veloctiy at which control gains match Kvec_1

      //gains at speed vel_ref:
      Kvec_1[0]=0.8232;   //phi-dot gain
      Kvec_1[1]=-2.0107;  //dphi error gain
      Kvec_1[2]=-1.4772;  //laterl d error gain

      Kvec[0]=0.5232;       //gains at speed 0
      Kvec[1]=-3.0107;
      Kvec[2]=-5.4772;

      decc_distance = 1.5;
      acc_distance = 1;
      corner_speed = 0.2;

      int_err = 0;  //speed controller integtraged error

      max_vel = 1;      //set maximum speed

      inited=0;

      glpath_sub_ = node.subscribe<nav_msgs::Path>("/move_base_node/TrajectoryPlannerROS/global_plan", 100, &lqr::glpathCallback,this);
      imu_sub_ = node.subscribe<sensor_msgs::Imu>("/imu", 100, &lqr::imuCallback,this);
      odom_sub_ = node.subscribe<nav_msgs::Odometry>("/odometry/filtered", 100, &lqr::odomCallback,this);

      pub_ball = node.advertise<visualization_msgs::Marker>( "closest_pt", 0, true);
      pub_arrow = node.advertise<visualization_msgs::Marker>( "closest_pt_dir", 0);
      pub_arrow_array = node.advertise<visualization_msgs::MarkerArray>( "des_speed_dir", 0);
      pub_vel = node.advertise<std_msgs::Float32MultiArray>( "lqr_vel", 0);
      pub_ackermann_sim = node.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 10);
      pub_servo = node.advertise<geometry_msgs::Vector3>("servo", 1);
}



double lqr::control()
{
    double perpen_vec[2]; //vector perpendicular to current heading
    perpen_vec[0] = cos(mapcoord[2]/180*PI + PI/4);
    perpen_vec[1] = sin(mapcoord[2]/180*PI + PI/4);

    double con_vec[2] = {closestpt[0] - mapcoord[0], closestpt[1] - mapcoord[1]};   //connecting vector from current point to goal point
    double lateral_d= perpen_vec[0]* con_vec[0] + perpen_vec[1] * con_vec[1];       //lateral distance

    err[0] = dphi/180*PI;    //angular velocity
    err[1] = (closestpt[2] - mapcoord[2])/180*PI;  //angle deviation
    err[2] = lateral_d;

    double Kvec_res[3];
    for(int i=0; i< 3; i++)
    {
        double vel_t = fabs(vel);
        Kvec_res[i]= vel_t/vel_ref*Kvec_1[i] + Kvec[i]*(1-vel_t/vel_ref);
    }

    steering_deg = -(Kvec_res[0]*err[0] + Kvec_res[1]*err[1]  + Kvec_res[2]*err[2])*180/PI * des_dir;
    //steering_deg = -(Kvec[0]*err[0] + Kvec[1]*err[1]  + Kvec[2]*err[2])*180/PI;

    ROS_INFO_STREAM( "err[0] (dphi)"  <<  err[0]  << "err[1] (delt phi)"  <<  err[1] << "lateral_d err err[2]" << lateral_d);
    ROS_INFO_STREAM("steering angle  "  << steering_deg << "desired speed: " << des_vel);

    double speed_with_full_gas = 8;
    double pc = des_dir*des_vel/speed_with_full_gas;

    cmd_thrust =  pc; // 1 full thrust, 0 no thrust , -1 reverse full thrust
    if(cmd_thrust > 1)
        cmd_thrust=1;
    if(cmd_thrust < -1)
        cmd_thrust=-1;

    publish_car();
    publish_sim();
}

void lqr::getclosestpoint()
{
    double shortestdistance = 100000;
    int indclose = 0;    
    //ROS_INFO_STREAM("calc closestpt  " );
    for(int i = 0; i<glpath.size(); i++)
    {
        double px = glpath.at(i).at(0);
        double py = glpath.at(i).at(1);
        //ROS_INFO_STREAM("distance calc: px " << px << " py " << py << " mapx " << mapcoord[0] << " mapy " << mapcoord[1]);

        double distance =  sqrt(pow(px-mapcoord[0],2) + pow(py-mapcoord[1],2));
        //ROS_INFO_STREAM("dist:" << distance );

        if(distance < shortestdistance)
        {
            shortestdistance = distance;
            indclose = i;
        }
    }
    closestpt.clear();
    closestpt.push_back( glpath.at(indclose).at(0));
    closestpt.push_back( glpath.at(indclose).at(1));
    closestpt.push_back( glpath.at(indclose).at(2));

    //ROS_INFO_STREAM("des_speed_vec  ");
    //for(int i=0; i<des_speed_vec.size(); i++)
    //    ROS_INFO_STREAM("des_speed_vec  " << des_speed_vec.at(i));
    des_vel = des_speed_vec.at(indclose);


    //ROS_INFO_STREAM("des_dir_vec  ");
    //for(int i=0; i<dir_vec.size(); i++)
     //   ROS_INFO_STREAM("des_dir_vec  " << dir_vec.at(i));


    //ROS_INFO_STREAM("indclose  " << indclose);
    if(indclose > dir_vec.size()-1)
        indclose = dir_vec.size()-2;
    des_dir = dir_vec.at(indclose);

    //ROS_INFO_STREAM("closes pt index  "  << indclose);
    //ROS_INFO_STREAM("shortest distance  "  << shortestdistance);
}


double lqr::get_z_euler_from_quad(vector <double> q) //uses quaternion definiton : w, x, y, z !!
{
    //returns the z angle in rad!
    return atan2( 2*(q.at(0)*q.at(3) + q.at(1)*q.at(2)), 1- 2*(pow(q.at(2),2) + pow(q.at(3),2)));
}

vector <double> lqr::get_quad_from_euler(double alpha)
{
    vector <double> q(4);  //quaternion
    q.at(0) = cos(alpha/2*PI/180);      //w
    q.at(1) = 0;                //x
    q.at(2) = 0;                //y
    q.at(3) = sin(alpha/2*PI/180);     //z

    return q;
}

void lqr::visualize()
{
    visualization_msgs::Marker ball;
    ball.header.frame_id = "map";
    ball.header.stamp = ros::Time();
    //marker.ns = "lqr";
    ball.id = 0;
    ball.type = visualization_msgs::Marker::SPHERE;
    ball.action = visualization_msgs::Marker::ADD;
    ball.pose.position.x = closestpt[0];
    ball.pose.position.y = closestpt[1];
    ball.pose.position.z = 0.06;
    ball.pose.orientation.x = 0;
    ball.pose.orientation.y = 0;
    ball.pose.orientation.z = 1;
    ball.pose.orientation.w = 1;
    ball.scale.x = 0.1;
    ball.scale.y = 0.1;
    ball.scale.z = 0.1;
    ball.color.a = 1.0; // Don't forget to set the alpha!
    ball.color.r = 0.0;
    ball.color.g = 0.0;
    ball.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    ball.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    //ROS_INFO_STREAM("pub marker");
    pub_ball.publish( ball );

    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time();
    //marker.ns = "lqr";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.position.x = closestpt[0];
    arrow.pose.position.y = closestpt[1];
    arrow.pose.position.z = 0.06;

    vector <double> q(get_quad_from_euler(closestpt[2]));
    arrow.pose.orientation.w = q.at(0);
    arrow.pose.orientation.x = q.at(1);
    arrow.pose.orientation.y = q.at(2);
    arrow.pose.orientation.z = q.at(3);

    arrow.scale.x = 0.3;
    arrow.scale.y = 0.03;
    arrow.scale.z = 0.03;
    arrow.color.a = 1.0; // Don't forget to set the alpha!
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    arrow.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    //ROS_INFO_STREAM("pub marker");
    pub_arrow.publish( arrow );


    visualization_msgs::MarkerArray arrow_ar;

    int interval = 10;
    int num_makers = des_speed_vec.size()/interval;
    arrow_ar.markers.clear();
    arrow_ar.markers.resize(num_makers);
    //ROS_INFO_STREAM("num markers "  << num_makers);

    for(int i=0; i <  num_makers; i++)
    {

        arrow_ar.markers.at(i).header.frame_id = "map";
        arrow_ar.markers.at(i).header.stamp = ros::Time();
        //marker.ns = "lqr";
        arrow_ar.markers.at(i).id = i;
        arrow_ar.markers.at(i).type = visualization_msgs::Marker::ARROW;
        arrow_ar.markers.at(i).action = visualization_msgs::Marker::ADD;
        arrow_ar.markers.at(i).pose.position.x = glpath.at(i*interval).at(0);
        arrow_ar.markers.at(i).pose.position.y = glpath.at(i*interval).at(1);
        arrow_ar.markers.at(i).pose.position.z = 0.08;

        vector <double> q(get_quad_from_euler( glpath.at(i*interval).at(2)) );
        arrow_ar.markers.at(i).pose.orientation.w = q.at(0);
        arrow_ar.markers.at(i).pose.orientation.x = q.at(1);
        arrow_ar.markers.at(i).pose.orientation.y = q.at(2);
        arrow_ar.markers.at(i).pose.orientation.z = q.at(3);

        arrow_ar.markers.at(i).scale.x = 0.2*des_speed_vec.at(i*interval)/max_vel;
        arrow_ar.markers.at(i).scale.y = 0.02;
        arrow_ar.markers.at(i).scale.z = 0.02;
        arrow_ar.markers.at(i).color.a = 1.0; // Don't forget to set the alpha!

        if(dir_vec.at(i*interval) < 0) {
            arrow_ar.markers.at(i).color.r = 0.7;
            arrow_ar.markers.at(i).color.g = 0.0;
            arrow_ar.markers.at(i).color.b = 0.7;
        }
        else
        {
            arrow_ar.markers.at(i).color.r = 0.0;
            arrow_ar.markers.at(i).color.g = 0.7;
            arrow_ar.markers.at(i).color.b = 0.7;
        }

        //only if using a MESH_RESOURCE marker type:
        arrow_ar.markers.at(i).mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    }

    pub_arrow_array.publish( arrow_ar );

}

void lqr::estimate_state()
{
    timenow = ros::Time::now();
    ros::Duration rosdt = timenow -timelast;
    double dt = rosdt.toSec();

    double dphi_uf = (mapcoord[2] - last_mapcoord[2])/dt;  //unfiltered
    dphi = filter_phi.filter(dphi_uf, 0.1);

    double hvec[2];
    hvec[0]= cos(mapcoord[2]/180*PI);         //heading unit-vector
    hvec[1]= sin(mapcoord[2]/180*PI);         //heading unit-vector

    double shiftvec[2];
    shiftvec[0]= mapcoord[0] - last_mapcoord[0];
    shiftvec[1]= mapcoord[1] - last_mapcoord[1];
    double vel_uf = (hvec[0]*shiftvec[0] +  hvec[1]*shiftvec[1]) / dt; //unfiltered
    vel = filter_vel.filter(vel_uf,0.05);

    memcpy(last_mapcoord, mapcoord, 3*sizeof(double));
    timelast = timenow;

    //ROS_INFO_STREAM("mapcoord  x" <<  mapcoord[0]  << "y"<<  mapcoord[1] << "phi" <<  mapcoord[2]);
    //ROS_INFO_STREAM("lastmapcoord  x" <<  last_mapcoord[0]  << "y"<<  last_mapcoord[1] << "phi" <<  last_mapcoord[2]);
    //ROS_INFO_STREAM("hvec 0  " << hvec[0] << "hvec 1  " << hvec[1] << "shiftvec 0  " << shiftvec[0] << "shiftvec 1 " << shiftvec[1]);

    //ROS_INFO_STREAM("dt  " << dt << "dphi grad/s  " << dphi << "vel_uf m/s  " << vel_uf << "vel m/s  " << vel);
    std_msgs::Float32MultiArray lqr_vel;
    lqr_vel.data.resize(6);
    lqr_vel.data.at(0)=dphi;
    lqr_vel.data.at(1)=dphi_uf;
    lqr_vel.data.at(2)=vel;
    lqr_vel.data.at(3)=vel_uf;
    lqr_vel.data.at(4)=imu_angular_z_vel_uf;
    //ROS_INFO_STREAM("imu_angular_z_vel unfiltered: " <<  imu_angular_z_vel_uf);
    double imu_angular_z_vel =filter_imu.filter(imu_angular_z_vel_uf,0.1);
    lqr_vel.data.at(5)=imu_angular_z_vel;
    //ROS_INFO_STREAM("imu_angular_z_vel filtered: " <<  imu_angular_z_vel);

    pub_vel.publish(lqr_vel);
}

void lqr::test_motor()
{
    geometry_msgs::Vector3 control_servo;
    control_servo.x = 1500;
    control_servo.y = 1500;

    for(int i=0; i< 30; i++)
    {
        control_servo.x += 3;
        pub_servo.publish(control_servo);
        ROS_INFO_STREAM(" servo.x " <<  control_servo.x);
        ros::Duration(0.5).sleep();
    }

    control_servo.x = 1500;

    for(int i=0; i< 30; i++)
    {
        control_servo.x -= 3;
        pub_servo.publish(control_servo);
        ROS_INFO_STREAM(" servo.x " <<  control_servo.x);
        ros::Duration(0.5).sleep();
    }


    control_servo.x = 1500;
    control_servo.y = 1500;

    for(int i=0; i< 30; i++)
    {
        control_servo.y += 30;
        pub_servo.publish(control_servo);
        ROS_INFO_STREAM(" servo.y " <<  control_servo.y);
        ros::Duration(0.5).sleep();
    }

    control_servo.y = 1500;

    for(int i=0; i< 30; i++)
    {
        control_servo.y -= 30;
        pub_servo.publish(control_servo);
        ROS_INFO_STREAM(" servo.y " <<  control_servo.y);
        ros::Duration(0.5).sleep();
    }



}

void lqr::calc_des_speed()
{
    ROS_INFO_STREAM("calcspeed");
    des_speed_vec.clear();
    des_speed_vec.resize(glpath.size());
    //ROS_INFO_STREAM("size des speed" << des_speed_vec.size());



    for(int i= 0; i < des_speed_vec.size()-1; i++)      //setting corner speed velocities:
    {
        if(fabs(angle_diff_per_m.at(i)) > 1)
        {
            des_speed_vec.at(i) =  corner_speed;
        }
        else
        {
            des_speed_vec.at(i) = max_vel;
        }
    }

    ROS_INFO_STREAM("after corner ");
   /* for(int i= 0; i < des_speed_vec.size(); i++)
        ROS_INFO_STREAM(" "   << des_speed_vec.at(i));
*/

    double intdistance = 0;
    for(int i= des_speed_vec.size()-1; i>= 0; i--)      //asigning velocities backwards from goal to start
    {
        if(i>0)
            intdistance += distance_to_last.at(i-1);
        if(intdistance < decc_distance)
        {
            double newspeed =  max_vel * intdistance/decc_distance;
            if( newspeed < des_speed_vec.at(i))
                des_speed_vec.at(i) = newspeed;

        }
    }

/*
    ROS_INFO_STREAM("asigning velocities backwards from goal to star ");
    for(int i= 0; i < des_speed_vec.size(); i++)
        ROS_INFO_STREAM(" "   << des_speed_vec.at(i));
*/

    vector <int> start_curve;   /////////////// getting the start end points of curves:
    vector <int> end_curve;
    start_curve.clear();
    end_curve.clear();

//    for(int i = 0; i < angle_diff_per_m.size(); i++)
  //      ROS_INFO_STREAM("angle_diff_per_m  " << i << " "  << angle_diff_per_m.at(i));

    int last_angle_diff = angle_diff_per_m.at(0);
    for(int i = 1; i< angle_diff_per_m.size(); i++)
    {
        if(fabs(angle_diff_per_m.at(i)) > 1 &&  fabs(last_angle_diff) < 1)
        {
            start_curve.push_back(i);
            //ROS_INFO_STREAM("start curve at  " << i);
        }
        if(fabs(angle_diff_per_m.at(i)) < 1 && fabs(last_angle_diff) > 1)
        {
            end_curve.push_back(i);
            //ROS_INFO_STREAM("end curve at  " << i);
        }
        last_angle_diff = angle_diff_per_m.at(i);
    }

    //ROS_INFO_STREAM("num start curve  " << start_curve.size());
    //ROS_INFO_STREAM("num start curve  " << end_curve.size());


    double min_vel = 0.1;

    // going through curv starting points, each iterating backwards
    for(int i = 0; i < start_curve.size(); i++)
    {
        intdistance = 0;
        int j = start_curve.at(i);
        double firstspeed = des_speed_vec.at(j);
        while(j > 0)
        {
            intdistance += distance_to_last.at(j);

            double newspeed =   (max_vel-min_vel) * intdistance/acc_distance+ firstspeed;
            if(newspeed < des_speed_vec.at(j))
                des_speed_vec.at(j) = newspeed;
            j--;
        }
    }

     ROS_INFO_STREAM("cout1  ");
/*
     ROS_INFO_STREAM("going through curv starting points, each iterating backwards ");
     for(int i= 0; i < des_speed_vec.size(); i++)
         ROS_INFO_STREAM(" "   << des_speed_vec.at(i));
    */

    intdistance = 0 ;

    for(int i= 0; i < des_speed_vec.size()-1; i++)      //asigning velocities from start to acc_distance
    {
        intdistance += distance_to_last.at(i);        
        if(intdistance < acc_distance)
        {
            double newspeed =  (max_vel-min_vel) * intdistance/acc_distance + min_vel;
            if(newspeed < des_speed_vec.at(i))
                des_speed_vec.at(i) =  newspeed;
        }

        //ROS_INFO_STREAM("i  " << i << " des speed  " << des_speed_vec.at(i) <<  " dir  " << dir_vec.at(i));
        //ROS_INFO_STREAM("i  " << i << " dist to last:  " << distance_to_last.at(i)  << "  angle_diff_per_m:  " << angle_diff_per_m.at(i));


    }

    /*ROS_INFO_STREAM("asigning velocities from start to acc_distance ");
    for(int i= 0; i < des_speed_vec.size(); i++)
        ROS_INFO_STREAM(" "   << des_speed_vec.at(i));
*/

    ROS_INFO_STREAM("cout2  ");

    // going through curv ending points forward
    for(int i = 0; i < end_curve.size(); i++)
    {
        intdistance = 0;
        int j = end_curve.at(i);
        double firstspeed =  des_speed_vec.at(j-1);
        while(j < des_speed_vec.size() -1 )
        {
            intdistance += distance_to_last.at(i);

            double newspeed =  (max_vel-min_vel) * intdistance/decc_distance + firstspeed;
            if(newspeed < des_speed_vec.at(j))
                des_speed_vec.at(j) = newspeed;

            j++;
        }
    }
    ROS_INFO_STREAM("cout3  ");

    /*for(int i= 0; i < des_speed_vec.size()-1; i++)
    {
        ROS_INFO_STREAM("i  " << i << " des speed  " << des_speed_vec.at(i) <<  " dir  " << dir_vec.at(i));
        ROS_INFO_STREAM("i  " << i  << "  angle_diff_per_m:  " << angle_diff_per_m.at(i));
    }*/

    //for(int i= 0; i < des_speed_vec.size(); i++)
    //    ROS_INFO_STREAM(" "   << des_speed_vec.at(i));


}


void lqr::glpathCallback(const nav_msgs::Path::ConstPtr& path)
{
    ROS_INFO_STREAM("new path received  ");
    int num_points = path->poses.size();

    inited = 1;
    glpath.clear();
    distance_to_last.clear();
    dir_vec.clear();
    angle_diff_per_m.clear();

    vector <double> last_pt(3,0);

    for(int i = 0; i<num_points; i++)
    {
        //getting the orientation:
        vector <double> q(4);  //quaternion
        q.at(0) = path->poses.at(i).pose.orientation.w;
        q.at(1) = path->poses.at(i).pose.orientation.x;
        q.at(2) = path->poses.at(i).pose.orientation.y;
        q.at(3) = path->poses.at(i).pose.orientation.z;

        double zangle = get_z_euler_from_quad(q)/PI*180.0;

        //getting the path points in x and y:
        double x = path->poses.at(i).pose.position.x;
        double y = path->poses.at(i).pose.position.y;

        /*if(i==0)
        {
            ROS_INFO_STREAM("path coords at 0: x"  << x << "  y:  " << y );
            ROS_INFO_STREAM("orientation at 0,  w: " << q.at(0) << " x: "<< q.at(1)<< " y: "<< q.at(2)<<  " z: " << q.at(3));
            ROS_INFO_STREAM("euler, z angle:  " << get_z_euler_from_quad(q)/PI*180.0);
        }*/


        if(i>0)
        {
            distance_to_last.push_back( sqrt(pow(last_pt.at(0)-x,2) + pow(last_pt.at(1)-y,2)) );            
            double angle_diff = zangle - last_pt.at(2);
            angle_diff_per_m.push_back(angle_diff/distance_to_last.back());
            //ROS_INFO_STREAM("distance_to_last at iter: "  << i << "  is:  " << distance_to_last[i-1]  << "  angle_diff_per_m:  " << angle_diff_per_m.back());

            double heading[2] = {cos(zangle*PI/180),sin(zangle*PI/180)};
            double shiftvec[2] = {x - last_pt.at(0), y - last_pt.at(1)};

            if((shiftvec[0]*heading[0]+ shiftvec[1]*heading[1] ) > 0 )
                dir_vec.push_back(1);
            else
                dir_vec.push_back(-1);
        }

        last_pt.at(0) = x;
        last_pt.at(1) = y;
        last_pt.at(2) = zangle;

        vector <double > pathpoint;
        pathpoint.push_back(x);
        pathpoint.push_back(y);
        pathpoint.push_back(zangle);
        glpath.push_back(pathpoint);
    }
    ROS_INFO_STREAM("new path saved  ");

    calc_des_speed();
}

void lqr::imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
    imu_angular_z_vel_uf = data->angular_velocity.z;
    //ROS_INFO_STREAM("imu ang z "  << imu_angular_z_vel_uf );

}

void lqr::odomCallback(const nav_msgs::Odometry::ConstPtr &data)
{
    odom_vel = data->twist.twist.linear.x;
    //ROS_INFO_STREAM("odomvel "  <<  odom_vel);
}

void lqr::publish_sim()
{
    ackermann_msgs::AckermannDriveStamped ackermannMsg;

    ackermannMsg.drive.speed = des_vel*des_dir;
    ackermannMsg.drive.steering_angle = steering_deg/180*PI;

    pub_ackermann_sim.publish(ackermannMsg);

}

void lqr::publish_car()
{
    double cmd_steeringAngle = steering_deg;

    cmd_steeringAngle = 1500 + 500/30*cmd_steeringAngle;

    if(cmd_steeringAngle > 2000)
    {
        cmd_steeringAngle = 2000;
    }
    else if(cmd_steeringAngle < 1000)
    {
        cmd_steeringAngle = 1000;
    }


    double addition = 500;
    double forward_treshold = 1539;
    double backward_treshold = 1488;

    geometry_msgs::Vector3 control_servo;
    if (cmd_thrust > 0 )
        control_servo.x = forward_treshold + addition*cmd_thrust;
    if (cmd_thrust < 0 )
        control_servo.x = backward_treshold + addition*cmd_thrust;

    control_servo.y = cmd_steeringAngle;

    ROS_INFO_STREAM("servo x " << control_servo.x << "servo y " << control_servo.y);

    pub_servo.publish(control_servo);
}

void lqr::test_speed_control()
{
    double speed_with_full_gas = 8;
    double pc = des_dir*des_vel/speed_with_full_gas;

    cmd_thrust =  pc; // 1 full thrust, 0 no thrust , -1 reverse full thrust
    if(cmd_thrust > 1)
        cmd_thrust=1;
    if(cmd_thrust < -1)
        cmd_thrust=-1;

    ROS_INFO_STREAM("thrust "  << cmd_thrust << "des_vel "  << des_dir*des_vel << "odom_vel "  << odom_vel  );

    steering_deg =0;

    publish_car();

    std_msgs::Float32MultiArray lqr_vel;
    lqr_vel.data.resize(2);
    lqr_vel.data.at(0)=odom_vel;
    lqr_vel.data.at(1)=des_vel;

    pub_vel.publish(lqr_vel);

}



