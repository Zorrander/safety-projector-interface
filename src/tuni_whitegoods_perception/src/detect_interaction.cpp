/*
Class to detect smart interface interactions.
here, we receive the buttons positions in the depthmap from the projected interface, we monitor the depth and trigger an event to openflow
if a button is pressed (i.e if a hand is hovering on top of it)
*/
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tuni_whitegoods_msgs/PointsDepthMap.h>
#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/ElementUI.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <unity_msgs/poiPCL.h>
#include <pcl/console/time.h>
#include <integration/VirtualButtonEventArray.h>
#include <std_msgs/Bool.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace integration;
static const std::string OPENCV_LINE = "Line";

class InteractionDetection
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        cv_bridge::CvImagePtr cv_ptr;
        ros::Publisher pub_event;
        image_transport::Subscriber dm_sub_;
        image_transport::Subscriber safety_sub_;
        image_transport::Subscriber safety_inside_sub_;
        ros::Subscriber sub_cam_info;
        ros::Subscriber sub_poi;
        ros::Subscriber sub_poi_pcl;
        ros::Subscriber sub_poi_rgb;
        ros::Subscriber sub_changes;
        //ros::ServiceClient spawnClient;
        cv::Mat depth_map;
        //cv::Mat res_bitwise;
        cv::Mat baseline_dm;
        cv::Mat safety_line;
        unity_msgs::InterfacePOI poi_interface_dm;
        unity_msgs::InterfacePOI baseline;
        unity_msgs::poiPCL poi_dm_pcl;
        VirtualButtonEventArray events;
        VirtualButtonEventArray previous_events;
        int size_elem;
        float threshold;
        float threshold_depth;
        bool first_baseline;
        pcl::console::TicToc tic;
        bool init_baseline;
        int count_baseline;
        bool test;
        bool changes;
        bool previous_changes;
        
    
  public:
    InteractionDetection():
    it_(nh_)
    {
      sub_poi = nh_.subscribe("/depth_interface/poi_depthmap", 1, &InteractionDetection::pointsOfInterestCb,this);
      sub_changes = nh_.subscribe("/aruco_markers_warped/changes", 1, &InteractionDetection::arucoChangesCallback,this);
      dm_sub_ = it_.subscribe("/detection/depth_map", 1,&InteractionDetection::DepthMapCallback, this);
      pub_event = nh_.advertise<VirtualButtonEventArray> ("/execution/projector_interface/integration/topics/virtual_button_event_array", 1);
      depth_map = cv::Mat(1024, 1024, CV_8U,cv::Scalar(std::numeric_limits<float>::min()));
      //res_bitwise = cv::Mat(1024, 1024, CV_8U,cv::Scalar(std::numeric_limits<float>::min()));
      baseline_dm = cv::Mat(1024, 1024, CV_8U,cv::Scalar(std::numeric_limits<float>::min()));
      safety_line = cv::Mat(1024, 1024, CV_8U,cv::Scalar(std::numeric_limits<float>::min()));
      events.virtual_button_events.clear();
      previous_events.virtual_button_events.clear();
      poi_interface_dm.poi.clear();
      baseline.poi.clear();
      poi_dm_pcl.pts.clear();
      size_elem = 0;
      init_baseline = true;
      first_baseline = false;
      count_baseline = 0;
      threshold = 0.0045;
      threshold_depth = 0.1;
      test = false;
      changes = false;
      cv::namedWindow(OPENCV_LINE,cv::WINDOW_NORMAL);
    }

    ~InteractionDetection()
    {
      //cv::destroyWindow(OPENCV_WINDOW);
      cv::destroyWindow(OPENCV_LINE);
    }

    //subscriber for display - only used for debugging
    void DepthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat img_test;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
          img_test = cv_ptr->image.clone();
          for(int i = 0 ; i < poi_interface_dm.poi.size(); i++)
          {
            cv::Point center(poi_interface_dm.poi[i].elem.x, poi_interface_dm.poi[i].elem.y);
            circle(img_test, center,1, cv::Scalar(0, 0, 0), -1);
          }
          
          cv::imshow(OPENCV_LINE, img_test);
          cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    //Here we detect if there is any changes in the aruco position on the table if the support to display the interface can be moved.
    //This could avoid redefining a baseline all the time
    void arucoChangesCallback(const std_msgs::BoolConstPtr& msg)
    {
      changes = msg->data;
      if(changes)
      {
        init_baseline = true;
        count_baseline = 0;
        events.virtual_button_events.clear();
        previous_events.virtual_button_events.clear();
      }
    }
    //subscriber of the buttons positions. We define a baseline (buttons not pressed) and we check if the table has been moved.
    // Once wwe define a baseline, we monitor the depth positions of the buttons compared to baseline, and if there is a change we throw an event.
    void pointsOfInterestCb(const unity_msgs::InterfacePOIConstPtr& msg)
    {
      events.virtual_button_events.clear();
      poi_interface_dm.poi.clear();
      size_elem = msg->poi.size();
      //check if there is a baseline
      if(previous_changes && !changes)
      {
        ros::Duration(0.5).sleep();
        events.virtual_button_events.clear();
        previous_events.virtual_button_events.clear();
        init_baseline = true;
        count_baseline = 0;
      }
      //if no changes and there isn't a baseline, create one
      if(!changes && !previous_changes)
      {
        if(init_baseline)
        {
          //std::cout<<count_baseline<<"\n";
          //create a single baseline
          if(count_baseline == 0)
          {
            baseline.poi.clear();
            for(int i = 0; i < msg->poi.size(); i++)
            {
              unity_msgs::ElementUI p;
              p.id = msg->poi[i].id;
              p.elem.x = msg->poi[i].elem.x;
              p.elem.y = msg->poi[i].elem.y;
              p.elem.z = msg->poi[i].elem.z;
              baseline.poi.push_back(p);
            }
          }
          //continue the baseline until reaching 10 samples of depth for each buttons. The baseline depth for a button will be the the average of the 10 samples
          else if(count_baseline < 10)
          {
            for(int i = 0; i < baseline.poi.size(); i++)
            {
              for(int j = 0; j < msg->poi.size(); j++)
              {
                int res = baseline.poi[i].id.compare(msg->poi[j].id);
                if(res == 0)
                {
                  baseline.poi[i].elem.z = baseline.poi[i].elem.z + msg->poi[j].elem.z;
                }
              }
            } 
          } 
          //now that we have the samples, getting the average
          else 
          {
            for(int i = 0; i < baseline.poi.size(); i++)
            {
              for(int j = 0; j < msg->poi.size(); j++)
              {
                int res = baseline.poi[i].id.compare(msg->poi[j].id);
                if(res == 0)
                {
                  baseline.poi[i].elem.z = baseline.poi[i].elem.z / count_baseline;
                }
              }
            }
            init_baseline = false;
          }
          count_baseline++;
        }
        //begin comparaison with baseline to see if there is an activation
        else
        {
          for(unity_msgs::ElementUI mg : msg->poi)
          {
            for(unity_msgs::ElementUI bl : baseline.poi)
            {
              int res = mg.id.compare(bl.id);
              if(res == 0)
              {
                //std::cout<<"z : "<<mg.elem.z<<"\n";
                if(mg.elem.z > bl.elem.z + threshold)
                {
                  //std::cout<<"ACTIVATION "<<mg.id<<"\n";
                  VirtualButtonEvent msg_event;
                  msg_event.virtual_button_id = mg.id;
                  msg_event.event_type = msg_event.PRESSED;
                  events.header = msg->header;
                  events.virtual_button_events.push_back(msg_event);
                }
              }
            }
          }
        }
        //if there is a baseline already, compare changes with the current buttons depth
        VirtualButtonEventArray msgs = compareEvents();
        //if changes, send an event
        if(msgs.virtual_button_events.size() > 0)
        {
          pub_event.publish(msgs);
        }
      }
      else
      {
        //std::cout<<"changes\n";
      }
      previous_changes = changes;
      //simple affectation for other use
      for(int i = 0; i < msg->poi.size(); i++)
      {
        unity_msgs::ElementUI p;
        p.id = msg->poi[i].id;
        p.elem.x = msg->poi[i].elem.x;
        p.elem.y = msg->poi[i].elem.y;
        p.elem.z = msg->poi[i].elem.z;
        poi_interface_dm.poi.push_back(p);
      }
      //std::cout<<tic.toc()<<"ms\n";
    }
    //compare 2 arrays of buttons to detect changes. I baseically compare the depth of each buttons in baseline with each current depth buttons
    VirtualButtonEventArray compareEvents()
    {
      VirtualButtonEventArray msgs;
      bool new_in_old = false;
      bool old_in_new = false;
      //check if new events are in the old ones
      for(VirtualButtonEvent vb : events.virtual_button_events)
      {
        new_in_old = isEvent(vb.virtual_button_id,previous_events);
        if(!new_in_old)
        {
          VirtualButtonEvent v;
          v.virtual_button_id = vb.virtual_button_id;
          v.event_type = vb.PRESSED;
          msgs.virtual_button_events.push_back(v);
        }
        new_in_old = false;
      }
      //check if old events are also in new ones
      for(VirtualButtonEvent vb_old : previous_events.virtual_button_events)
      {
        old_in_new = isEvent(vb_old.virtual_button_id,events);
        if(!old_in_new)
        {
          VirtualButtonEvent v;
          v.virtual_button_id = vb_old.virtual_button_id;
          v.event_type = vb_old.RELEASED;
          msgs.virtual_button_events.push_back(v);
        }
        old_in_new = false;
      }
      //copy new in old
      previous_events.virtual_button_events.clear();
      for(VirtualButtonEvent vb : events.virtual_button_events)
      {
        previous_events.virtual_button_events.push_back(vb);
      }

      return msgs;
    }

    //check if an id is inside and EventArray
    bool isEvent(std::string id, VirtualButtonEventArray msgs)
    {
      bool inside = false;
      for(int i = 0; i < msgs.virtual_button_events.size(); i++)
      {
        int res = msgs.virtual_button_events[i].virtual_button_id.compare(id);
        if(res == 0)
        {
          inside = true;
        }
      }

      return inside;
    }
}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_interaction");
  InteractionDetection inter;
  ros::spin();
  return 0;
}