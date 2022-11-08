#include "gocator_interface.h"

Gocator_interface::Device::Device(const std::string & _ip_address)
{
	kStatus status;
	kIpAddress ipAddress;
  GoDataMsg dataObj;
	kChar model_name[50];

	//init all GO API objects
	go_api_ = kNULL;
	go_system_ = kNULL;
	go_sensor_ = kNULL;
	go_setup_ = kNULL;
	go_dataset_ = kNULL;
	go_stamp_ptr_ = kNULL;

  // construct Gocator API Library
  if ((status = GoSdk_Construct(&go_api_)) != kOK)
  {
      printf("Error: GoSdk_Construct:%s\n", kStatus_Name(status));
      status_ = DEVICE_NOT_FOUND;
      return;
  }

  // construct GoSystem object
  if ((status = GoSystem_Construct(&go_system_, kNULL)) != kOK)
  {
      printf("Error: GoSystem_Construct:%s\n", kStatus_Name(status));
      status_ = DEVICE_NOT_FOUND;
      return;
  }

	// obtain GoSensor object by sensor IP address
	kIpAddress_Parse(&ipAddress, _ip_address.c_str());
	if ((status = GoSystem_FindSensorByIpAddress(go_system_, &ipAddress, &go_sensor_)) != kOK)
	{
		status_ = DEVICE_NOT_FOUND;
		exit(status);
	}

	//Success case. Set status and device fixed params (ip, model name and serial number ).
	status_ = DEVICE_FOUND;
	device_params_.ip_address_ = _ip_address;

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(go_sensor_)) != kOK)
	{
		printf("Error: GoSensor_Connect:%s\n", kStatus_Name(status));
		status_ = DEVICE_NOT_CONNECT;
		return;
	}
	status_ = DEVICE_CONNECT;

	// enable sensor data channel
	if ((status = GoSystem_EnableData(go_system_, kTRUE)) != kOK)
	{
		 printf("Error: GoSensor_EnableData:%s\n", kStatus_Name(status));
		return;
	}

	// retrieve setup handle
	if ((go_setup_ = GoSensor_Setup(go_sensor_)) == kNULL)
	{
		printf("Error: GoSensor_Setup: Invalid Handle\n");
		return;
	}

	//Obtain camera model
	if ((status = GoSensor_Model(go_sensor_, model_name, 50)) != kOK )
	{
		printf("Error: GoSensor_Model:%s\n", kStatus_Name(status));
		return;
	}

	device_params_.model_name_ = model_name;
	//Obtain camera Serial number
	device_params_.sn_ = (unsigned int)GoSensor_Id(go_sensor_);
	//Obtain exposure
	capture_params_.exposure_time_ = GoSetup_Exposure(go_setup_, GO_ROLE_MAIN);
	//Obtain spacing interval
	capture_params_.spacing_interval_ = GoSetup_SpacingInterval(go_setup_, GO_ROLE_MAIN);
	//print info
	std::cout << "Found Sensor: " << std::endl;
	device_params_.print();
}

Gocator_interface::Device::~Device()
{
	kStatus status;

	this->stop();

	// destroy handles
	GoDestroy(go_system_);
	GoDestroy(go_api_);

	//bye bye message
	std::cout << "~Device(). Gocator Sensor Stopped and Device Object Destroyed." << std::endl;
}

int Gocator_interface::Device::configure(const CaptureParams & _configs)
{
	kStatus status;

	//set exposure
	if ((status = GoSetup_SetExposure(go_setup_, GO_ROLE_MAIN, _configs.exposure_time_)) != kOK )
	{
		std::cout << "configure(): Error setting Exposure Time to " << _configs.exposure_time_ << std::endl;
		return -1;
	}

	//set spacing interval
	if ((status = GoSetup_SetSpacingInterval(go_setup_, GO_ROLE_MAIN, _configs.spacing_interval_)) != kOK )
	{
		std::cout << "configure(): Error setting Spacing Interval to " << _configs.spacing_interval_ << std::endl;
		return -1;
	}

	//set this->capture_params_ with true values from camera
	capture_params_.exposure_time_ = GoSetup_Exposure(go_setup_, GO_ROLE_MAIN);
	capture_params_.spacing_interval_ = GoSetup_SpacingInterval(go_setup_, GO_ROLE_MAIN);

	//print
	std::cout << "Configuration Setings: " << std::endl;
	capture_params_.print();

	//return
	return 1;
}

int Gocator_interface::Device::start()
{
	kStatus status;
	// start Gocator sensor
	if ((status = GoSystem_Start(go_system_)) != kOK)
	{
		std::cout << "Device(). Error: GoSystem_Start: " << status << std::endl;
		return -1;
	}
	std::cout<<"device start"<< std::endl;;

	//set this->status_
	this->status_ = DEVICE_RUNNING;

	//return success
	return 1;
}

int Gocator_interface::Device::stop()
{
	kStatus status;
	std::cout<<"device off\n";
	// stop Gocator sensor
	if ((status = GoSystem_Stop(go_system_)) != kOK)
	{
      printf("Error: GoSystem_Stop:%s\n", kStatus_Name(status));
      return -1;
	}

	//set this->status_
	this->status_ = DEVICE_CONNECT;

	//return success
	return 1;
}

int Gocator_interface::Device::startAquisitionThread()
{
  
  return -1;
}

int Gocator_interface::Device::stopAquisitionThread()
{
  
  return -1;
}

int Gocator_interface::Device::getCurrentSnapshot(pcl::PointCloud<pcl::PointXYZ> & _p_cloud) const
{
  
  return -1;
}

int Gocator_interface::Device::getSingleSnapshot(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	GoDataSet dataset = kNULL;
	GoStamp *stamp = kNULL;
	GoDataMsg dataObj;

	//start Gocator acquisition
	this->start();
	bool real_data_arrived = false;
	while (!real_data_arrived)
	{
		if (GoSystem_ReceiveData(go_system_, &dataset, RECEIVE_TIMEOUT) != kOK)
		{
      //stop Gocator acquisition
      this->stop();
			//no message after timeout
			std::cout << "Error: No data received during the waiting period" << std::endl;
			return -1;
		}
		else
		{
			// Loop for each data item in the dataset object
			for (unsigned int ii = 0; ii < GoDataSet_Count(dataset); ii++)
			{
				//get the data item ii
				dataObj = GoDataSet_At(dataset, ii);

				//switch according the type of message
				switch(GoDataMsg_Type(dataObj))
				{
					case GO_DATA_MESSAGE_TYPE_STAMP:
					{
						GoStampMsg stampMsg = dataObj;
						for (unsigned int jj = 0; jj < GoStampMsg_Count(stampMsg); jj++)
						{
							stamp = GoStampMsg_At(stampMsg, jj);
						}
					}
					break;

					case GO_DATA_MESSAGE_TYPE_SURFACE:
					{
            //stop Gocator acquisition
            this->stop();
						//cast to GoSurfaceMsg
						GoSurfaceMsg surfaceMsg = dataObj;

						//Get general data of the surface
						unsigned int row_count = GoSurfaceMsg_Length(surfaceMsg);
						unsigned int width = GoSurfaceMsg_Width(surfaceMsg);
						unsigned int exposure = GoSurfaceMsg_Exposure(surfaceMsg);

						//get offsets and resolutions
						double xResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
						double yResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
						double zResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
						double xOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
						double yOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
						double zOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));
            // static offsets for 3520
						// double zOffset = (0.203+0.075)*1000;


						// Print raw cloud metadata
						std::cout << "Surface Message." << std::endl;
						std::cout << "\tLength: " <<  row_count << std::endl;
						std::cout << "\tWidth: " << width << std::endl;
						// std::cout << "\tExposure: " << exposure << std::endl;
						// std::cout << "\tzOffset: " << zOffset << std::endl;


						//resize the point cloud
						tmp_cloud->height = row_count;
						tmp_cloud->width = width;
						tmp_cloud->resize(row_count*width);
            
            double scale = 0.001;

						std::vector<int> index;
						//run over all rows
						for (unsigned int ii = 0; ii < row_count; ii++)
						{
							//get the pointer to row
							short *data = GoSurfaceMsg_RowAt(surfaceMsg,ii);

							//run over the width of row ii
							for (unsigned int jj = 0; jj < width; jj++)
							{
								tmp_cloud->points.at(ii*width+jj).x = -scale*(xOffset + xResolution*jj);
								tmp_cloud->points.at(ii*width+jj).y =  scale*(yOffset + yResolution*ii);

								//set z
								if (data[jj] != INVALID_RANGE_16BIT )
								{
									tmp_cloud->points.at(ii*width+jj).z = scale*(zOffset + zResolution*data[jj]);
									index.push_back(ii*width+jj);
								}
								else
									tmp_cloud->points.at(ii*width+jj).z = scale*(INVALID_RANGE_DOUBLE);
							}
						}

						real_data_arrived=true;
						
					}
					break;
				}
			}
		}

		//destroys received message
		GoDestroy(dataset);
	}

	PointCloudT::Ptr cloud_filtered (new PointCloudT);
	double bnd = 500;
	pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT> ());
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT>("z",pcl::ComparisonOps::GT,-bnd)));
	range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT>("z",pcl::ComparisonOps::LT,bnd)));
	pcl::ConditionalRemoval<PointT> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(tmp_cloud);
	condrem.setKeepOrganized(false);
	condrem.filter(*cloud_filtered);

	_p_cloud = *cloud_filtered;
	if (_p_cloud.points.size() == 0)
	{
		std::cout<<"_p_cloud empty, may need to adjust measuring distance near 164mm!\n";
		return (-2);
	}
	return 1;
}

void Gocator_interface::Device::getDeviceHealth(std::string & _health_str) const
{
	//local variables
	GoDataSet health_data = kNULL;
	GoHealthMsg health_msg =kNULL;
	GoIndicator *health_indicator = kNULL;
	std::ostringstream sstr;

	//get health from device
	if ( (GoSystem_ReceiveHealth(go_system_, &health_data, RECEIVE_TIMEOUT)) == kOK )
	{
		for (unsigned int ii = 0; ii < GoDataSet_Count(health_data); ii++)
		{
			health_msg = GoDataSet_At(health_data, ii);
			for (unsigned int jj = 0; jj < GoHealthMsg_Count(health_msg); jj++)
			{
				health_indicator = GoHealthMsg_At(health_msg, jj);
				sstr << "Indicator[" << jj << "]:\n"
				<< "\tId: " << health_indicator->id << "\n"
				<< "\tInstance: " << health_indicator->instance << "\n"
				<< "\tValue: " << health_indicator->value << "\n";
			}
		}
		GoDestroy(health_msg);
	}

	_health_str = sstr.str();
}

void Gocator_interface::Device::getTemperature(double & _internal_temp, double & _projector_temp, double & _laser_temp) const
{
	//local variables
	GoDataSet health_data = kNULL;
	GoHealthMsg health_msg =kNULL;
	GoIndicator *health_indicator = kNULL;
	//k32u instance;

	//get health dataset from device
	if ( (GoSystem_ReceiveHealth(go_system_, &health_data, RECEIVE_TIMEOUT)) == kOK )
	{
		for (unsigned int ii = 0; ii < GoDataSet_Count(health_data); ii++)
		{
			//get the health message
			health_msg = GoDataSet_At(health_data, ii);

			//find in the message the internal temperature indicator, and set the value
			health_indicator = GoHealthMsg_Find(health_msg, GO_HEALTH_TEMPERATURE, 0);
			if (health_indicator != kNULL) _internal_temp = health_indicator->value;
			else _internal_temp = -100.;

			//find in the message the projector temperature indicator, and set the value
			health_indicator = GoHealthMsg_Find(health_msg, GO_HEALTH_PROJECTOR_TEMPERATURE, 0);
			if (health_indicator != kNULL) _projector_temp = health_indicator->value;
			else _projector_temp = -100.;

			//find in the message the projector temperature indicator, and set the value
			health_indicator = GoHealthMsg_Find(health_msg, GO_HEALTH_LASER_TEMPERATURE, 0);
			if (health_indicator != kNULL) _laser_temp = health_indicator->value;
			else _laser_temp = -100.;
		}
		GoDestroy(health_msg);
	}

}

int Gocator_interface::Device::close()
{
  return -1;
}

void Gocator_interface::Device::printDeviceData() const
{
  return;
}