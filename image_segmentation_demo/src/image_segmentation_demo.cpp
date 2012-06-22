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



//#include "image_segmentation_demo/object_segmentation_ui.h"
#include "image_segmentation_demo/image_segmentation_demo.h"
#include "image_segmentation_demo/ui_frame.h"


#include <wx/wx.h>
#include <wx/app.h>
using namespace std;

namespace image_segmentation_demo{

	PickAndPlaceApp::PickAndPlaceApp()
	{

	}

	PickAndPlaceApp::~PickAndPlaceApp()
	{

	}

	bool PickAndPlaceApp::OnInit()
	{
		char ** local_argv_;
		local_argv_ =  new char*[ argc ];
		for ( int i = 0; i < argc; ++i )
		{
			local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
		}
		const std::string name="bosch_image_segmentation_demo";
		ros::init(argc,local_argv_, "bosch_image_segmentation_demo");

	    for ( int i = 0; i < argc; ++i )
	        {
	          free( local_argv_[ i ] );
	        }
	    free(local_argv_);

	   // RunPickandPlaceApp();

	    ObjectSegmentationUI *window=new ObjectSegmentationUI(NULL);//,wxID_ANY, wxT("Grapper"), wxDefaultPosition, wxSize(1550, 1550), wxCAPTION|wxFRAME_FLOAT_ON_PARENT|wxMINIMIZE_BOX|wxTAB_TRAVERSAL, wxT("ObjectSegmentationWindow"));
	    window->Show(true);


	    SetTopWindow(window);
	    return true;
	}



}

IMPLEMENT_APP(image_segmentation_demo::PickAndPlaceApp)

/*int main(int argc, char **argv) {

	ros::init(argc, argv, "bosch_image_segmentation_demo");
	image_segmentation_demo::PickAndPlaceApp pick_and_place;

//	while (ros::ok())
//	 {
		//first pull up window and have them segment object
	pick_and_place.GetObjectSegmentation();

		//then send results of object segmentation to pick up object

		//then move arm to specified location and open the hand

//	 }

  return 0;
}*/
