#include "gocator_interface_node.h"

Gocator_interfaceNode::Gocator_interfaceNode():
    nh_(ros::this_node::getName()),
    is_request_(false)
{      
    save_path = "";
    save_request = false;
    //set the subscriber
    snapshot_request_ = nh_.subscribe("snapshot_request", 1, &Gocator_interfaceNode::snapshotRequestCallback, this);
    
    //set the subscriber
    saveshot_request_ = nh_.subscribe("saveshot_request", 1, &Gocator_interfaceNode::saveshotRequestCallback, this);
    
    //set the point cloud publisher
    snapshot_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("pcl_output", 1);
    
    //Set the fov marker publisher
    fov_publisher_ = nh_.advertise<visualization_msgs::Marker>("fov_markers",1);
    
    //set the server
    //pcl_server_ = nh_.advertiseService("pcl_snapshot", &Gocator_interfaceNode::pointCloudSnapshotService, this);

    //Read params from the yaml configuration file
    std::string ip_addr;
    int int_param;
    capture_counter_ = 1;
    z_max_ = 500;
    z_min_ = -500;
    nh_.getParam("ip_address", ip_addr);
    nh_.getParam("run_mode", int_param); this->run_mode_ = (RunMode)int_param;
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->frame_name_);
    nh_.getParam("exposure", this->capture_params_.exposure_time_);
    nh_.getParam("spacing", this->capture_params_.spacing_interval_);
    nh_.getParam("capture_counter", this->capture_counter_);
    nh_.getParam("z_max", this->z_max_);
    nh_.getParam("z_min", this->z_min_);
    
    fov_viz_ = true; //TODO: get it from param server
    
    //create a device object
    g_interface_camera_ = new Gocator_interface::Device(ip_addr); 
    
    //configure according yaml params
    g_interface_camera_->configure(capture_params_);

    //print
    std::cout << "ROS node Setings: " << std::endl; 
    std::cout << "\trun mode: \t" << run_mode_ << std::endl;
    std::cout << "\trate [hz]: \t" << rate_  << std::endl;
    std::cout << "\tframe name: \t" << frame_name_ << std::endl;
}

Gocator_interfaceNode::~Gocator_interfaceNode()
{
    delete g_interface_camera_; 
}

void Gocator_interfaceNode::stop()
{
    // std::cout<<"camera stopped\n";
    g_interface_camera_->stop();
    delete g_interface_camera_; 
}

RunMode Gocator_interfaceNode::runMode() const
{
    return run_mode_;
}

bool Gocator_interfaceNode::isRequest() const
{
    return is_request_;
}

void Gocator_interfaceNode::resetRequest()
{
    std::cout<<"request off!\n";
    is_request_ = false; 
}

void Gocator_interfaceNode::publish()
{    
    ros::Time ts;
    
    PointCloudT::Ptr cloud (new PointCloudT);
    //Get snapshot from camera and publish the point cloud
    if ( g_interface_camera_->getSingleSnapshot(*cloud) == 1 )
    {
      ts = ros::Time::now();
      cloud->header.stamp = (std::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the Gocator_interface::Device class
      cloud->header.frame_id = frame_name_;
      snapshot_publisher_.publish(*cloud);
      if (save_request)
        {
          if( pcl::io::savePLYFileBinary (save_path, *cloud) != 0)
          {
              std::cout<<"failed to  save "<<save_path<<"\n";
          }else{
              std::cout<<save_path<<" saved successflly!\n";
          }
          save_request = false;
        }
    }
    else
    {
      std::cout << "Gocator_interfaceNode::publish(): Error with point cloud snapshot acquisition" << std::endl;
    }
}

double Gocator_interfaceNode::rate() const
{
    return rate_;
}

bool Gocator_interfaceNode::isFovViz() const
{
    return fov_viz_;
}

void Gocator_interfaceNode::snapshotRequestCallback(const std_msgs::Empty::ConstPtr& _msg)
{
    std::cout<<"request received!\n";
    is_request_ = true;
}

void Gocator_interfaceNode::saveshotRequestCallback(const std_msgs::String::ConstPtr& _msg)
{
    std::cout<<"request received!\n";
    is_request_ = true;
    save_request = true;
    save_path = _msg->data;
    std::cout<<"save_path is "<<save_path<<std::endl;
}
                
/* bool Gocator_interfaceNode::pointCloudSnapshotService(gocator_interface::PointCloudAsService::Request  & _request, gocator_interface::PointCloudAsService::Response & _reply)
{
    //create a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud; 
    
    std::cout << "Processing service request!" << std::endl;
    
    //call gocator 
    if ( g_interface_camera_.getSingleSnapshot(cloud) == 1 )
    {
        _reply.pcloud.header.frame_id = "gocator_interface";
        _reply.pcloud.header.stamp = ros::Time::now();
        pcl::toROSMsg(cloud,_reply.pcloud);
        return true;
    }
    else
        return false;
}
 */


void Gocator_interfaceNode::publish_fov()
{
    // //fill and publish the field of view wires
    // fov_marker_msg_.header.stamp = ros::Time::now(); 
    // fov_marker_msg_.header.frame_id = frame_name_;
    // fov_marker_msg_.ns = ""; 
    // fov_marker_msg_.id = 1;
    // fov_marker_msg_.type = visualization_msgs::Marker::LINE_LIST; 
    // fov_marker_msg_.action = visualization_msgs::Marker::ADD;
    // fov_marker_msg_.pose.position.x = 0; 
    // fov_marker_msg_.pose.position.y = 0; 
    // fov_marker_msg_.pose.position.z = 0; 
    // fov_marker_msg_.pose.orientation.x = 0; 
    // fov_marker_msg_.pose.orientation.y = 0; 
    // fov_marker_msg_.pose.orientation.z = 0; 
    // fov_marker_msg_.pose.orientation.w = 1; 
    // fov_marker_msg_.scale.x = 0.001;
    // fov_marker_msg_.color.a = 0.9;
    // fov_marker_msg_.color.r = 0.6;
    // fov_marker_msg_.color.g = 0.2;
    // fov_marker_msg_.color.b = 0.2;
    // fov_marker_msg_.lifetime = ros::Duration(0);
    // fov_marker_msg_.points.resize(24);

    // //line 1
    // fov_marker_msg_.points[0].x = 0.0335;
    // fov_marker_msg_.points[0].y = 0.043;
    // fov_marker_msg_.points[0].z = 0.035;
    // fov_marker_msg_.points[1].x = 0.0465;
    // fov_marker_msg_.points[1].y = 0.044;
    // fov_marker_msg_.points[1].z = -0.035;

    // //line 2
    // fov_marker_msg_.points[2].x = 0.0335;
    // fov_marker_msg_.points[2].y = -0.043;
    // fov_marker_msg_.points[2].z = 0.035;
    // fov_marker_msg_.points[3].x = 0.0465;
    // fov_marker_msg_.points[3].y = -0.044;
    // fov_marker_msg_.points[3].z = -0.035;

    // //line 3
    // fov_marker_msg_.points[4].x = -0.0335;
    // fov_marker_msg_.points[4].y = -0.043;
    // fov_marker_msg_.points[4].z = 0.035;
    // fov_marker_msg_.points[5].x = -0.0465;
    // fov_marker_msg_.points[5].y = -0.044;
    // fov_marker_msg_.points[5].z = -0.035;

    // //line 4
    // fov_marker_msg_.points[6].x = -0.0335;
    // fov_marker_msg_.points[6].y = 0.043;
    // fov_marker_msg_.points[6].z = 0.035;
    // fov_marker_msg_.points[7].x = -0.0465;
    // fov_marker_msg_.points[7].y = 0.044;
    // fov_marker_msg_.points[7].z = -0.035;
    
    // //line 5
    // fov_marker_msg_.points[8].x = -0.0465;
    // fov_marker_msg_.points[8].y = 0.044;
    // fov_marker_msg_.points[8].z = -0.035;
    // fov_marker_msg_.points[9].x = 0.0465;
    // fov_marker_msg_.points[9].y = 0.044;
    // fov_marker_msg_.points[9].z = -0.035;
    
    // //line 6
    // fov_marker_msg_.points[10].x = 0.0465;
    // fov_marker_msg_.points[10].y = 0.044;
    // fov_marker_msg_.points[10].z = -0.035;
    // fov_marker_msg_.points[11].x = 0.0465;
    // fov_marker_msg_.points[11].y = -0.044;
    // fov_marker_msg_.points[11].z = -0.035;

    // //line 7
    // fov_marker_msg_.points[12].x = 0.0465;
    // fov_marker_msg_.points[12].y = -0.044;
    // fov_marker_msg_.points[12].z = -0.035;
    // fov_marker_msg_.points[13].x = -0.0465;
    // fov_marker_msg_.points[13].y = -0.044;
    // fov_marker_msg_.points[13].z = -0.035;
    
    // //line 8
    // fov_marker_msg_.points[14].x = -0.0465;
    // fov_marker_msg_.points[14].y = -0.044;
    // fov_marker_msg_.points[14].z = -0.035;
    // fov_marker_msg_.points[15].x = -0.0465;
    // fov_marker_msg_.points[15].y = 0.044;
    // fov_marker_msg_.points[15].z = -0.035;
    
    // //line 9
    // fov_marker_msg_.points[16].x = 0.0335;
    // fov_marker_msg_.points[16].y = 0.043;
    // fov_marker_msg_.points[16].z = 0.035;
    // fov_marker_msg_.points[17].x = 0.0335;
    // fov_marker_msg_.points[17].y = -0.043;
    // fov_marker_msg_.points[17].z = 0.035;
    
    // //line 10
    // fov_marker_msg_.points[18].x = 0.0335;
    // fov_marker_msg_.points[18].y = -0.043;
    // fov_marker_msg_.points[18].z = 0.035;
    // fov_marker_msg_.points[19].x = -0.0335;
    // fov_marker_msg_.points[19].y = -0.043;
    // fov_marker_msg_.points[19].z = 0.035;

    // //line 11
    // fov_marker_msg_.points[20].x = -0.0335;
    // fov_marker_msg_.points[20].y = -0.043;
    // fov_marker_msg_.points[20].z = 0.035;
    // fov_marker_msg_.points[21].x = -0.0335;
    // fov_marker_msg_.points[21].y = 0.043;
    // fov_marker_msg_.points[21].z = 0.035;

    // //line 12
    // fov_marker_msg_.points[22].x = -0.0335;
    // fov_marker_msg_.points[22].y = 0.043;
    // fov_marker_msg_.points[22].z = 0.035;
    // fov_marker_msg_.points[23].x = 0.0335;
    // fov_marker_msg_.points[23].y = 0.043;
    // fov_marker_msg_.points[23].z = 0.035;
    
    // //publish
    // fov_publisher_.publish(fov_marker_msg_);
}
