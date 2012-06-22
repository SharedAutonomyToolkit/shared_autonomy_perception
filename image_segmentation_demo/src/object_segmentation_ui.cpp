/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <ros/ros.h>
#include<sys/stat.h>
#include<fstream>
#include<iostream>

//#include<sys/type.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>s


#include "image_segmentation_demo/utils.h"
#include "image_segmentation_demo/object_segmentation_ui.h"

#include <wx/wx.h>
#include <wx/app.h>

#include "object_segmentation_frame.h"


using namespace std;
using namespace rgbd_assembler;
namespace enc = sensor_msgs::image_encodings;

namespace image_segmentation_demo
{
static const char WINDOW[] = "Image window";

/**
 * Constructor for ObjectSegmentationUI class
 */
ObjectSegmentationUI::ObjectSegmentationUI(wxWindow* parentwindow ): ObjectSegmentationFrame(parentwindow), object_segmenter_(0), listener_(ros::Duration(120.0))
{
	//can't accept until you segment something
	accept_button_->Enable(false);
	ogre_tools::initializeOgre();
	root_ = Ogre::Root::getSingletonPtr();


	try{
		scene_manager_ = root_->createSceneManager( Ogre::ST_GENERIC, "TestSceneManager" );
		render_window_ = new ogre_tools::wxOgreRenderWindow( root_, ImagePanel );


		render_window_->setAutoRender(false);

		render_window_->SetSize(this->GetSize() );
		//render_window_->SetSize( panel->GetSize() );
		ogre_tools::V_string paths;
		paths.push_back(ros::package::getPath(ROS_PACKAGE_NAME) + "/ogre_media/textures");
		paths.push_back(ros::package::getPath(ROS_PACKAGE_NAME) + "/ui");
		data_location_=ros::package::getPath(ROS_PACKAGE_NAME) + "/data";
		ogre_tools::initializeResources(paths);

		camera_ = scene_manager_->createCamera("Camera");

		render_window_->getViewport()->setCamera( camera_ );

		texture_ = new ROSImageTexture();

		object_segmenter_ = new GrabCut3DObjectSegmenter();

		//setup ROS Pick and Place Interfaces
		SetUpPickandPlaceApp();
		//call sensor data
		GetSensorData();

		display_image_=current_image_;

		boost::shared_ptr<sensor_msgs::Image>temp_image_(new sensor_msgs::Image ());
		*temp_image_=display_image_;
		texture_->setNewImage(temp_image_);

		Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create( "Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
		material->setCullingMode(Ogre::CULL_NONE);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
		material->getTechnique(0)->setLightingEnabled(false);
		Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
		tu->setTextureName(texture_->getTexture()->getName());
		tu->setTextureFiltering( Ogre::TFO_NONE );

		Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
		rect->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
		rect->setMaterial(material->getName());
		rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
		Ogre::AxisAlignedBox aabb;
		aabb.setInfinite();
		rect->setBoundingBox(aabb);

		Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
		node->attachObject(rect);
		node->setVisible(true);

		render_window_->SetSize(texture_->getWidth(), texture_->getHeight());

		start_time_=false;
		num_resets_=0;
		setUpEventHandlers();

	}
	catch(Ogre::Exception& e)
	{
		ROS_ERROR("%s", e.what());
	}

	if (!nh->ok())
	{
		Close();
	}

	render_window_->Refresh();
}

ObjectSegmentationUI::~ObjectSegmentationUI(){
	delete render_window_;
	delete object_segmenter_;
}

/*
 * Sets up event handlers for the mouse
 */

void ObjectSegmentationUI::setUpEventHandlers(){

	// change the render window handler to use our event handlers when the user does something with the mouse

	render_window_->Connect(wxEVT_MOTION,wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_LEFT_DOWN,wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_MIDDLE_DOWN,wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_RIGHT_DOWN,wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_LEFT_UP,wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_MIDDLE_UP, wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_RIGHT_UP, wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_MOUSEWHEEL, wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);
	render_window_->Connect(wxEVT_LEFT_DCLICK, wxMouseEventHandler( ObjectSegmentationUI::onRenderWindowMouseEvents ), NULL, this);

}

/*
 * Callback to handle the accept button in the interface  - segments the selected region and then proceeds call function that will handle manipulation
 */

void ObjectSegmentationUI::acceptButtonClicked( wxCommandEvent& )
{

	// Fetch binary mask from GCApp object
	binary_mask_ = object_segmenter_->binaryMask();

	// reconstruct clusters
	reconstructAndClusterPointCloud();

	// get table parameters in 3d
	if(table_detector_.detectTable(current_point_cloud_, detection_call_.table))
		detection_call_.result = detection_call_.SUCCESS;
	else
		detection_call_.result = detection_call_.NO_TABLE;


	ROS_INFO("ObjectSegmentation was successful.");

	//Now that object segmentation has occurred direct arm to pick up the object and put it down
	RunRestOfPickAndPlace();
}

/*
 *
 * Callback for Cancel button
 *
 */
void ObjectSegmentationUI::cancelButtonClicked( wxCommandEvent& )
{
	cleanupAndQuit();
}
/*
 *Callback for refresh button -> displays new image
 */

void ObjectSegmentationUI::refreshButtonClicked(wxCommandEvent&)
{
	GetSensorData();
	display_image_=current_image_;
	updateView();
	accept_button_->Enable(false);
}

/*
 *Callback for the reset button -> resets the visualization
 */
void ObjectSegmentationUI::resetButtonClicked( wxCommandEvent& )
{
	num_resets_++;
	object_segmenter_->initializedIs(false);
	resetVars();
	
	
}

/*
 *Callback for the segment button -> uses 3d grab cut to segment the area within the box.
 */
void ObjectSegmentationUI::segmentButtonClicked( wxCommandEvent& ){

	if( object_segmenter_->rectState() == GrabCut3DObjectSegmenter::SET)
	{
		object_segmenter_->iterCountInc();
		// update image overlay
		updateImageOverlay();
	}
	else
	{	ROS_INFO("Rectangle must be drawn first.");
		wxMessageBox(wxT("You need to draw a box with the mouse first. \n\n Left Mouse Button: Drag a rectangle around an object to segment it. \n\n\
SHIFT+Left Mouse Button: Mark pixel belonging to the object. \n\n CTRL+Left Mouse Button: Mark pixel belonging to the background."), wxT("Instructions"), wxOK|wxICON_INFORMATION , this);
		return;
	}
	// only if segmentation is ran once labeling can be accepted
	accept_button_->Enable(true);

}


/*
 * set mouse events for the UI window
 */
void ObjectSegmentationUI::onRenderWindowMouseEvents( wxMouseEvent& event )
{
	if(!recording_time_)
	{
		//start timing the user interaction and set up the data collection site
		start_time_=time(0);
		struct tm *now =localtime(&start_time_);

		stringstream data_string_stream;

		data_string_stream<< data_location_<<"/"<< (now->tm_year+1900) << (now->tm_mon+1) << (now->tm_mday) << "_"<<now->tm_hour << "_" << now->tm_min << "_"<<now->tm_sec;
		data_string_=data_string_stream.str();
		ROS_INFO_STREAM("path: "<< data_string_);

		//create folder using date/time info
		mkdir(data_string_.c_str(), 0755);

		recording_time_=true;
	}
	updateView();
	int x = event.GetX();
	int y = event.GetY();
	//convert to coordinates in the original image resolution

	x = floor(x * current_image_.width / texture_->getWidth());
	y = floor(y * current_image_.height / texture_->getHeight());

	//make sure mouse events are handled by this code
	GrabCut3DObjectSegmenter::MouseEvent mouse_event;
	if (event.ButtonDown(wxMOUSE_BTN_LEFT))
	{
		mouse_event = GrabCut3DObjectSegmenter::LEFT_BUTTON_DOWN;
	}
	else if (event.ButtonUp(wxMOUSE_BTN_LEFT))
	{
		mouse_event = GrabCut3DObjectSegmenter::LEFT_BUTTON_UP;
	}
	else if (event.ButtonDown(wxMOUSE_BTN_RIGHT))
	{
		mouse_event = GrabCut3DObjectSegmenter::RIGHT_BUTTON_DOWN;
	}
	else if (event.ButtonUp(wxMOUSE_BTN_RIGHT))
	{
		mouse_event = GrabCut3DObjectSegmenter::RIGHT_BUTTON_UP;
	}
	else
	{
		mouse_event = GrabCut3DObjectSegmenter::MOUSE_MOVE;
	}

	if(event.Dragging())
	{
		mouse_event = GrabCut3DObjectSegmenter::MOUSE_MOVE;
	}

	object_segmenter_->mouseClick(mouse_event, x, y, event.ControlDown(), event.ShiftDown());

	// update image overlay
	updateImageOverlay();
}

void ObjectSegmentationUI::cleanupAndQuit(){
	delete pickup_client;
	delete place_client;
	Close();
}

/*
 * updates view in the display window
 */
void ObjectSegmentationUI::updateView(){
	boost::shared_ptr<sensor_msgs::Image>temp_image_(new sensor_msgs::Image ());
	*temp_image_ = display_image_;

	int width = 0;
	int height = 0;
	ImagePanel->GetSize(&width, &height);

	texture_->setNewImage(temp_image_);	
	texture_->setSize( (uint32_t) width, (uint32_t) height );
	//ROS_INFO( "Texture Size: %d x %d", texture_->getWidth(), texture_->getHeight() );
	render_window_->SetSize(texture_->getWidth(), texture_->getHeight());
	
	root_->renderOneFrame();
}
void ObjectSegmentationUI::update(float wall_dt, float ros_dt){
	updateView();
}
void ObjectSegmentationUI::updateImageOverlay()
{
	sensor_msgs::Image new_display_image;
	convertMatToImageMessage(object_segmenter_->displayImage(), new_display_image);
	display_image_=new_display_image;
	updateView();
}

/*
 * Gets the image and point cloud data
 */
bool ObjectSegmentationUI::GetSensorData(){

	ros::Time start_time = ros::Time::now();
	if (!rgbd_assembler_client_.call(rgbd_assembler_srv_))
	{
		ROS_ERROR_STREAM("Call to rgbd assembler service failed");
		return false;
	}
	ROS_INFO_STREAM("Detection backend: assembled data received after " << ros::Time::now() - start_time << " seconds");
	if (rgbd_assembler_srv_.response.result != rgbd_assembler_srv_.response.SUCCESS)
	{
		std::ostringstream s;
		s << "RGB-D Assembler service returned error " << (int)rgbd_assembler_srv_.response.result;
		ROS_ERROR_STREAM( s.str() );
		return false;
	}

	current_image_ = rgbd_assembler_srv_.response.image;
	current_disparity_image_ = rgbd_assembler_srv_.response.disparity_image;
	current_camera_info_ = rgbd_assembler_srv_.response.camera_info;

	fillRgbImage(current_image_, rgbd_assembler_srv_.response.point_cloud);
        transformPointCloud("/base_link", rgbd_assembler_srv_.response.point_cloud, current_point_cloud_ );

	getDepthImage(current_depth_image_);
	// initialize segmentation
	Mat image, depth_image;
	convertImageMessageToMat(current_image_, image);
	convertImageMessageToMat(current_depth_image_, depth_image);
	object_segmenter_->setImages(image, depth_image);
	return true;
}

/*
 * transforms point cloud to new frame
 * @param target_frame - frame desired for point cloud to be in
 * @param cloud_in - the cloud you want transformed
 * @param cloud_out - the cloud in the target frame
 */
bool ObjectSegmentationUI::transformPointCloud(const std::string &target_frame, const sensor_msgs::PointCloud2& cloud_in, sensor_msgs::PointCloud2& cloud_out)
  {
    //convert cloud to processing frame
    sensor_msgs::PointCloud old_cloud,old_cloud_transformed;
    sensor_msgs::convertPointCloud2ToPointCloud (cloud_in, old_cloud);
    string err_msg;
    if (listener_.canTransform(target_frame.c_str(),cloud_in.header.frame_id.c_str(), ros::Time(0), &err_msg))
    {
      try
      {
        listener_.transformPointCloud(target_frame, old_cloud, old_cloud_transformed);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("Failed to transform cloud from frame %s into frame %s: %s", old_cloud.header.frame_id.c_str(),target_frame.c_str(),ex.what());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Could not transform %s to %s: %s", cloud_in.header.frame_id.c_str(), target_frame.c_str(), err_msg.c_str());
      return false;
    }
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud_transformed, cloud_out);
    ROS_INFO("Input cloud converted to %s frame", target_frame.c_str());
    return true;
  }



void ObjectSegmentationUI::fillRgbImage(sensor_msgs::Image &rgb_img,
		const sensor_msgs::PointCloud2 &point_cloud)
{

	ROS_DEBUG("Width and Height: (%d %d)",point_cloud.height, point_cloud.width );

	rgb_img.header = point_cloud.header;
	rgb_img.height = point_cloud.height;
	rgb_img.width = point_cloud.width;
	rgb_img.encoding = enc::RGB8;
	rgb_img.is_bigendian = false;
	rgb_img.step = 3 * rgb_img.width;
	size_t size = rgb_img.step * rgb_img.height;
	rgb_img.data.resize(size);

	for(unsigned int x=0; x<rgb_img.width; ++x)
	{
		for(unsigned int y=0; y<rgb_img.height; ++y)
		{
			int i = y * rgb_img.width + x;

			float rgb;
			memcpy ( &rgb, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[3].offset], sizeof (float));
			float r, g, b;
			transformRgb(rgb, r, g, b);

			int wide_i = y * rgb_img.step + x*3;
			rgb_img.data[wide_i+0] = round(r*255.0f);
			rgb_img.data[wide_i+1] = round(g*255.0f);
			rgb_img.data[wide_i+2] = round(b*255.0f);

		}
	}
}


/*
 * Gets depth image.  Current topic is "/camera/depth/image" if a different topic is desired you should remap (http://www.ros.org/wiki/roslaunch/XML/remap)  if a different camera topic is desired.
 */
bool ObjectSegmentationUI::getDepthImage(sensor_msgs::Image& image_msg)
{
	// get image
	std::string image_topic = nh->resolveName("/camera/depth/image");
	ROS_INFO("Waiting for image on topic %s", image_topic.c_str());
	sensor_msgs::ImageConstPtr image_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, ros::Duration(30.0));
	if(!image_ptr) {
		ROS_ERROR("Could not receive an image!");
		return false;
	}
	ROS_INFO("Received image on topic %s", image_topic.c_str());
	image_msg = *image_ptr;

	float max_val = -FLT_MAX;
	float min_val = FLT_MAX;
	for(unsigned int x=0; x<image_msg.width; ++x)
	{
		for(unsigned int y=0; y<image_msg.height; ++y)
		{
			int index_src = y * image_ptr->step + x*4;
			float* val = (float*)(&image_ptr->data[index_src]);
			max_val = std::max(*val,max_val);
			min_val = std::min(*val,min_val);
		}
	}

	image_msg.encoding = "bgr8";
	image_msg.width = image_ptr->width;
	image_msg.height = image_ptr->height;
	image_msg.step = 3 * image_ptr->width;
	image_msg.data.resize(image_ptr->step * image_ptr->height);
	for(unsigned int x=0; x<image_msg.width; ++x)
	{
		for(unsigned int y=0; y<image_msg.height; ++y)
		{
			int index_src = y * image_ptr->step + x*4;
			float* val = (float*)(&image_ptr->data[index_src]);

			int index_dest = y * image_msg.step + x*3;
			image_msg.data[index_dest+0] = round(*val);
			image_msg.data[index_dest+1] = round(*val);
			image_msg.data[index_dest+2] = round(*val);
		}
	}
	ROS_INFO("returning depth image %s", image_topic.c_str());

	return true;
}



/*
 * Helper function to convert ROS sensor images to openCV mat images
 * @param image_msg - ROS sensor image
 * @param image - opencv Mat image
 *
 */
bool ObjectSegmentationUI::convertImageMessageToMat(const sensor_msgs::Image& image_msg, Mat& image)
{
	// converts ROS images to OpenCV
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	image = cv_ptr->image;

	return true;
}

/*
 * Helper function to convert openCV mat images to ROS sensor images
 * @param image - opencv Mat image
 * @param image_msg - ROS sensor image
 */
bool ObjectSegmentationUI::convertMatToImageMessage(const Mat& image, sensor_msgs::Image& image_msg)
{
	// converts OpenCV to ROS images
	cv_bridge::CvImage cv_image;
	cv_image.image = image;
	cv_image.encoding = "bgr8";

	try
	{
		cv_image.toImageMsg(image_msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}
	return true;
}


/*
 * Reconstructs clusters
 */
void ObjectSegmentationUI::reconstructAndClusterPointCloud()
{
	clusters_.clear();
	clusters_.resize(1);
	clusters_[0].header.frame_id = current_point_cloud_.header.frame_id;
	clusters_[0].header.stamp = ros::Time(0);
	clusters_[0].points.reserve(1000);

	for(unsigned int x=0; x<current_point_cloud_.width; ++x)
	{
		for(unsigned int y=0; y<current_point_cloud_.height; ++y)
		{
			int i = y * current_point_cloud_.width + x;
			int mask_val = int(binary_mask_.at<unsigned char>(y, x));
			float nan;
			memcpy (&nan,
					&current_point_cloud_.data[i * current_point_cloud_.point_step + current_point_cloud_.fields[0].offset],
					sizeof (float));
			if(!isnan(nan))
			{
				geometry_msgs::Point32 p;
				float x, y, z;
				memcpy (&x,
						&current_point_cloud_.data[i
						                           * current_point_cloud_.point_step
						                           + current_point_cloud_.fields[0].offset],
						                           sizeof (float));
				memcpy (&y,
						&current_point_cloud_.data[i
						                           * current_point_cloud_.point_step
						                           + current_point_cloud_.fields[1].offset],
						                           sizeof (float));
				memcpy (&z,
						&current_point_cloud_.data[i
						                           * current_point_cloud_.point_step
						                           + current_point_cloud_.fields[2].offset],
						                           sizeof (float));
				p.x = x;
				p.y = y;
				p.z = z;

				if (mask_val > 0)
					clusters_[0].points.push_back(p);
			}
		}
	}
	detection_call_.clusters = clusters_;
}

void ObjectSegmentationUI::resetVars()
{
	for (size_t i = 0; i < clusters_.size(); ++i)
		clusters_[i].points.clear();

	clusters_.clear();
	GetSensorData();

	display_image_=current_image_;
	
	accept_button_->Enable(false);
	display_image_=current_image_;
	updateView();
//	updateImageOverlay();

	// only if segmentation is running labeling can be accepted
	accept_button_->Enable(false);

	// only if segmentation is not running, it can be started
	segment_button_->Enable(true);
}

/*
 * Saves the data
 */
void ObjectSegmentationUI::saveData()
{
	//save data to the folder
	std::string filename_orig=data_string_ + "/original_image.jpg";
	std::string filename_mask=data_string_ + "/segmented_image.jpg";
	std::string filename_trial_data=data_string_ + "/trial_info.txt";

	cv_bridge::CvImagePtr cv_orig_ptr, cv_mask_ptr;

	double diff_time=difftime(stop_time_, start_time_);
	ofstream file;
	file.open(filename_trial_data.c_str());
	file << "Time with interface: " << diff_time<<endl;
	file << "Number of resets:" << num_resets_<<endl;
	file << "Grasp plan calculated " << object_grasp_;
	file.close();


	//convert images to OpenCV so we can easily save them
	try{
		cv_orig_ptr=cv_bridge::toCvCopy(current_image_);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Original image cv_bridge exception %s", e.what());
	}

	try{
		cv_mask_ptr=cv_bridge::toCvCopy(display_image_);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("BitMap image cv_bridge exception %s", e.what());
	}


	//cv::namedWindow(WINDOW);
	//cv::imshow(WINDOW, cv_orig_ptr->image);


	//ROS_INFO_STREAM("mat info:" << cv_orig_ptr->image.rows << " " << cv_orig_ptr->image.cols);

	cv::Mat original_bgr;
	cv::cvtColor(cv_orig_ptr->image, original_bgr, CV_RGB2BGR);
	if(!cv::imwrite(filename_orig.c_str(), original_bgr)) ROS_ERROR("Could not save original image");
	if(!cv::imwrite(filename_mask.c_str(), cv_mask_ptr->image)) ROS_ERROR("Could not save mask image");

}

  /**
   ** Initializes the manipulation pipeline
   **
   **/
void ObjectSegmentationUI::SetUpPickandPlaceApp()
{
	COLLISION_PROCESSING_SERVICE_NAME =    "/tabletop_collision_map_processing/tabletop_collision_map_processing";
	PICKUP_ACTION_NAME =  "/object_manipulator/object_manipulator_pickup";
	PLACE_ACTION_NAME ="/object_manipulator/object_manipulator_place";
	RGBD_ASSEMBLY_SERVICE_NAME="/rgbd_assembly";
	ARM_ACTION_NAME="r_arm_controller/joint_trajectory_action";
	//create service and action clients
	nh.reset(new ros::NodeHandle);
	chatter_pub= nh->advertise<tabletop_object_detector::TabletopDetectionResult>("chatter", 1000);
	
	pickup_client= new actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction>(PICKUP_ACTION_NAME, true);
	place_client= new actionlib::SimpleActionClient <object_manipulation_msgs::PlaceAction>(PLACE_ACTION_NAME, true);

	
	//wait for collision map processing client
	while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME,ros::Duration(2.0)) && nh->ok() )
	{
		ROS_INFO("Waiting for collision processing service to come up");
	}
	if (!nh->ok()) exit(0);
	collision_processing_srv =nh->serviceClient	<tabletop_collision_map_processing::TabletopCollisionMapProcessing> (COLLISION_PROCESSING_SERVICE_NAME, true);

	while ( !ros::service::waitForService(RGBD_ASSEMBLY_SERVICE_NAME,ros::Duration(2.0)) && nh->ok() )
	{
		ROS_INFO("Waiting for rgbd assembly service to come up");
	}
	if (!nh->ok()) exit(0);
	rgbd_assembler_client_ =nh->serviceClient	<rgbd_assembler::RgbdAssembly> (RGBD_ASSEMBLY_SERVICE_NAME, true);

	//wait for pickup client
	while(!pickup_client->waitForServer(ros::Duration(2.0)) && nh->ok())
	     		{
	     			ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
	     		}
	     		if (!nh->ok()) exit(0);
	 
	//wait for place client
	while(!place_client->waitForServer(ros::Duration(2.0)) && nh->ok())
	     		{
	     			ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
	     		}
	     		if (!nh->ok()) exit(0);
	
}

/*
 * Performs the manipulation after segmentation is complete
 */

void ObjectSegmentationUI::RunRestOfPickAndPlace()
{

	stop_time_=time(0);
	ROS_INFO("publish publish");
	ROS_INFO_STREAM("frameid " << detection_call_.table.x_max);
	ROS_INFO_STREAM("number of clusters" << detection_call_.clusters.size());
        detection_call_.cluster_model_indices.push_back(0);
	detection_call_.models.push_back(household_objects_database_msgs::DatabaseModelPoseList());

	chatter_pub.publish(detection_call_);

	  if (detection_call_.result !=
	      detection_call_.SUCCESS)
	  {
	    ROS_ERROR("Tabletop detection returned error code %d",
	              detection_call_.result);
	  }
	  if (detection_call_.clusters.empty() &&
	      detection_call_.models.empty() )
	  {
	    ROS_ERROR("The tabletop detector detected the table, but found no objects");

	  }
	  //call collision map processing
	 ROS_INFO("Calling collision map processing");
	  tabletop_collision_map_processing::TabletopCollisionMapProcessing  processing_call;
	  //pass the result of the tabletop detection
	  processing_call.request.detection_result = detection_call_;
	  //ask for the exising map and collision models to be reset
	  //processing_call.request.reset_static_map = true;
	  processing_call.request.reset_collision_models = true;
	  processing_call.request.reset_attached_models = true;
	  //ask for a new static collision map to be taken with the laser
	  //after the new models are added to the environment
	  //processing_call.request.take_static_collision_map = true;
	  //ask for the results to be returned in base link frame
	  processing_call.request.desired_frame = "/base_link";
	  if (!collision_processing_srv.call(processing_call))
	  {
	    ROS_ERROR("Collision map processing service failed");
	    return;
	  }
	  //the collision map processor returns instances of graspable objects
	  if (processing_call.response.graspable_objects.empty())
	  {
	    ROS_ERROR("Collision map processing returned no graspable objects");
	    return;
	  }


	//call object pickup
	  ROS_INFO("Calling the pickup action");
	  object_manipulation_msgs::PickupGoal pickup_goal;
	  //pass one of the graspable objects returned by the collission map processor
	  pickup_goal.target = processing_call.response.graspable_objects.at(0);
	  //pass the name that the object has in the collision environment
	  //this name was also returned by the collision map processor
	  pickup_goal.collision_object_name =
	    processing_call.response.collision_object_names.at(0);
	  //pass the collision name of the table, also returned by the collision
	  //map processor
	  pickup_goal.collision_support_surface_name =
	    processing_call.response.collision_support_surface_name;
	  //pick up the object with the right arm
	  pickup_goal.arm_name = "right_arm";

	  geometry_msgs::Vector3Stamped direction;
	  direction.header.stamp = ros::Time::now();
	  direction.header.frame_id = "base_link";
	  direction.vector.x = 0;
	  direction.vector.y = 0;
	  direction.vector.z = 1;
	  pickup_goal.lift.direction = direction;
	  //request a vertical lift of 10cm after grasping the object
	  pickup_goal.lift.desired_distance = 0.1;
	  pickup_goal.lift.min_distance = 0.05;
	  //do not use tactile-based grasping or tactile-based lift
	  pickup_goal.use_reactive_lift = false;
	  pickup_goal.use_reactive_execution = false;
	  //send the goal
	  pickup_client->sendGoal(pickup_goal);
	  while (!pickup_client->waitForResult(ros::Duration(10.0)))
	  {
	    ROS_INFO("Waiting for the pickup action...");
	  }
	  object_manipulation_msgs::PickupResult pickup_result =
	    *(pickup_client->getResult());
	  if (pickup_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
	    ROS_ERROR("The pickup action has failed with result code %d",
	              pickup_result.manipulation_result.value);
		
	   //object couldn't be grasped.

	   wxMessageBox(wxT("Sorry Alan can't grab that, try again!"), wxT("Grasp failure"), wxOK|wxICON_EXCLAMATION, this); 
	   saveData();
	   GetSensorData();
	   display_image_=current_image_;
	   updateView();	
	   accept_button_->Enable(false);
	   object_grasp_=false;
	   num_resets_=0;
	   recording_time_=false;
	   return;
	  }
	  object_grasp_=true;
	  saveData();


	//move arm up.
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try
	{
		ros::Time now= ros::Time::now();
		listener.waitForTransform("/base_link", "/r_wrist_roll_link", now, ros::Duration(3.0));
		listener.lookupTransform("/base_link", "/r_wrist_roll_link", ros::Time(0), transform);	
		geometry_msgs::PoseStamped newpose;
		newpose.header.frame_id="/base_link";
		
		newpose.pose.position.x=transform.getOrigin()[0];
		newpose.pose.position.y=transform.getOrigin()[1];
		newpose.pose.position.z=transform.getOrigin()[2]+.1;
		newpose.pose.orientation.x=transform.getRotation().getAxis()[0];
		newpose.pose.orientation.y=transform.getRotation().getAxis()[1];
		newpose.pose.orientation.z=transform.getRotation().getAxis()[2];
		newpose.pose.orientation.w=transform.getRotation().getW();

		robot_.right_arm.moveWristRollLinktoPose(newpose );
	}
	catch (tf::TransformException ex)
	{
	 	ROS_ERROR("Transform Error %s",ex.what());
	}
	
	//TODO: Find current /r_gripper_tool_frame (in base link) and then add to z

	robot_.right_arm.moveGripperToPosition(tf::Vector3(0.55,-0.7, 1.03),    
                 "base_link", simple_robot_control::Arm::FROM_ABOVE);
	
	robot_.right_arm.moveGripperToPosition(tf::Vector3(0.55,-0.7, 0.822),    
                 "base_link", simple_robot_control::Arm::FROM_ABOVE);

	ROS_INFO("Publishing arm position");
	
	
	
	//open gripper
	robot_.right_gripper.open(-1);
	


	//move arm back to initial ready position

	robot_.right_arm.moveGripperToPosition(tf::Vector3(0.55,-0.7, 1.03),    
                 "base_link", simple_robot_control::Arm::FROM_ABOVE);
	
	GetSensorData();
	display_image_=current_image_;
	updateView();
	accept_button_->Enable(false);
	num_resets_=0;
	recording_time_=false;
	}



}
