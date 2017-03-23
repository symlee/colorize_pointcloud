#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <image_geometry/pinhole_camera_model.h>
#include <colorize_pointcloud/ColorizerConfig.h>

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
//#include "colorize_pointcloud.h"

// pcl instantiation for custom point type
// PCL_INSTANTIATE(ExtractIndices, multModalCloud);
//template class extract_indices<pcl::PointCloud<multModalCloud> >;


using namespace std;

std::string image_topic_vis;
std::string info_topic_vis;
std::string image_topic_therm;
std::string info_topic_therm;
std::string cloud_topic;
std::string output_topic;
bool keep_outsiders = false;
int color_r;
int color_g;
int color_b;
int32_t color_rgb;

image_geometry::PinholeCameraModel cam_model_vis_;
image_geometry::PinholeCameraModel cam_model_therm_;
tf::TransformListener *listener;
ros::Publisher pub;

// TODO: verify that union is unnecessary for trying to reuse space
struct multModalCloud
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  union 
  { 
    struct 
    { 
      uint8_t b; 
      uint8_t g; 
      uint8_t r; 
    }; 
    float rgb;				
  };
  union 
  { 
    struct 
    { 
      uint8_t b_vis; 
      uint8_t g_vis; 
      uint8_t r_vis; 
    }; 
    float rgb_vis;				
  };
  union 
  { 
    struct 
    { 
      uint8_t b_therm; 
      uint8_t g_therm; 
      uint8_t r_therm; 
    }; 
    float rgb_therm;				
  };
  union 
  { 
    struct 
    { 
      uint8_t b_rad; 
      uint8_t g_rad; 
      uint8_t r_rad; 
    }; 
    float rgb_rad;				
  };


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

// TODO: maybe consider doing reverse conversion from float to rgb
POINT_CLOUD_REGISTER_POINT_STRUCT (multModalCloud,        // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
				   (float, rgb, rgb)
				   (float, rgb_therm, rgb_therm)
				   (float, rgb_rad, rgb_rad)				   
				   )

void configCb(colorize_pointcloud::ColorizerConfig &newconfig, uint32_t level) {
  bool color_changed = false;
  if (newconfig.keep_outsiders != keep_outsiders)
    keep_outsiders = newconfig.keep_outsiders;
  if (newconfig.r != color_r) {
    color_r = newconfig.r;
    color_changed = true;
  }
  if (newconfig.g != color_g) {
    color_g = newconfig.g;
    color_changed = true;
  }
  if (newconfig.b != color_b) {
    color_b = newconfig.b;
    color_changed = true;
  }
  if (color_changed)
    color_rgb = (((uint8_t)color_r) << 16) | (((uint8_t)color_g) << 8) | ((uint8_t)color_b); 
}

int32_t extractImgInten(cv::Point2d uv, cv::Mat img) {
  uint8_t inten = (uint8_t)img.at<uchar>(cv::Point(uv.x,uv.y));
  int32_t rgb = (inten << 16) | (inten << 8) | inten; 

  return rgb;
}


int32_t extractImgColor(cv::Point2d uv, cv::Mat img) {
  uint8_t r = (uint8_t)img.at<cv::Vec3b>(uv)[0];;
  uint8_t g = (uint8_t)img.at<cv::Vec3b>(uv)[1];;
  uint8_t b = (uint8_t)img.at<cv::Vec3b>(uv)[2];;
  int32_t rgb = (r << 16) | (g << 8) | b; 

  return rgb;
}

void colorPointCloud(pcl::PointCloud<multModalCloud>::Ptr pcl_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_therm, image_geometry::PinholeCameraModel cam_model_vis_, cv::Mat img_vis, image_geometry::PinholeCameraModel cam_model_therm_, cv::Mat img_therm, pcl::PointIndices fov_indices) {
  // for VIS
  cv::Point2d uv;
  for(unsigned long i=0; i < pcl_cloud->size(); ++i){

    bool in_vis_FOV = false;
    bool in_therm_FOV = false;    
			  
    // is point in front of camera?
    if(pcl_cloud->points[i].z>0) {

      uv = cam_model_vis_.project3dToPixel(
					   cv::Point3d(pcl_cloud->points.at(i).x, pcl_cloud->points.at(i).y, pcl_cloud->points.at(i).z));
      //is pixel in bounds of image?
      if (uv.x >= 0 && uv.x < img_vis.cols && uv.y >= 0 && uv.y < img_vis.rows){
	if (img_vis.channels() == 1) {
	  int32_t rgb = extractImgInten(uv, img_vis);
	  pcl_cloud->points[i].rgb_vis = *(float *)(&rgb);
	}
	if (img_vis.channels() == 3) {
	  int32_t rgb = extractImgColor(uv, img_vis);
	  pcl_cloud->points[i].rgb_vis = *(float *)(&rgb);
	}
	fov_indices.indices.push_back(i);
	in_vis_FOV = true;
      }
      //point is not in FOV of the camera
      else{
	pcl_cloud->points[i].rgb_vis = 0;
	// pcl_cloud->points[i].rgb_vis = *(float *)(&color_rgb);
      }
    }
    //point is behind camera
    else{
      pcl_cloud->points[i].rgb_vis = 0;      
      // pcl_cloud->points[i].rgb_vis = *(float *)(&color_rgb);
    }
  
    // for THERM
    if(pcl_cloud_therm->points[i].z>0) {
      uv = cam_model_therm_.project3dToPixel(
  					     cv::Point3d(pcl_cloud_therm->points.at(i).x, pcl_cloud_therm->points.at(i).y, pcl_cloud_therm->points.at(i).z));
      //is pixel in bounds of image?
      if (uv.x >= 0 && uv.x < img_therm.cols && uv.y >= 0 && uv.y < img_therm.rows){
  	if (img_therm.channels() == 1) {
	  int32_t rgb = extractImgInten(uv, img_therm);
	  pcl_cloud->points[i].rgb_therm = *(float *)(&rgb);
  	}
  	if (img_vis.channels() == 3) {
	  int32_t rgb = extractImgColor(uv, img_therm);
	  pcl_cloud->points[i].rgb_therm = *(float *)(&rgb);
  	}
  	fov_indices.indices.push_back(i);
	in_therm_FOV = true;
      }
      //point is not in FOV of the camera
      else{
  	pcl_cloud->points[i].rgb_therm = 0;
  	// pcl_cloud->points[i].rgb_therm = *(float *)(&color_rgb);
      }
    }
    //point is behind camera
    else{
      pcl_cloud->points[i].rgb_therm = 0;
      // pcl_cloud->points[i].rgb_therm = *(float *)(&color_rgb);
    }

   
    // TODO: overlay here
    // rgb values of modality of interest for fusion (with _val tag to prevent name collisions)
    uint32_t rgb_therm_val = *reinterpret_cast<int*>(&pcl_cloud->points[i].rgb_therm);
    uint8_t r_therm_val = (rgb_therm_val >> 16) & 0x0000ff;
    uint8_t g_therm_val = (rgb_therm_val >> 8)  & 0x0000ff;
    uint8_t b_therm_val = (rgb_therm_val)       & 0x0000ff;
    uint32_t rgb_vis_val = *reinterpret_cast<int*>(&pcl_cloud->points[i].rgb_vis);
    uint8_t r_vis_val = (rgb_vis_val >> 16) & 0x0000ff;
    uint8_t g_vis_val = (rgb_vis_val >> 8)  & 0x0000ff;
    uint8_t b_vis_val = (rgb_vis_val)       & 0x0000ff;

    // verified values non zero
    uint8_t inten_therm = (uint8_t)(0.299 * r_therm_val +  0.587 * g_therm_val + 0.114 * b_therm_val);
    uint8_t inten_vis = (uint8_t)(0.299 * r_vis_val +  0.587 * g_vis_val + 0.114 * b_vis_val);
    
    uint8_t threshold = 0;
    double alpha = 1;

    if (inten_therm > threshold) {
  
      uint8_t r_fus = (uint8_t)(alpha * r_therm_val + (1 - alpha) * r_vis_val);
      uint8_t g_fus = (uint8_t)(alpha * g_therm_val + (1 - alpha) * g_vis_val);
      uint8_t b_fus = (uint8_t)(alpha * b_therm_val + (1 - alpha) * b_vis_val);
      int32_t rgb_fus = (r_fus << 16) | (g_fus << 8) | b_fus;

      // cout << "alpha: " << alpha << endl;
      // cout << "r_therm_val: " << unsigned(r_therm_val) << endl;      
      // cout << "r_vis_val: " << unsigned(r_vis_val) << endl;      
      // cout << "r_fus: " << unsigned(r_fus) << endl;

      // uint8_t r_fus = r_therm_val;
      // uint8_t g_fus = g_therm_val;
      // uint8_t b_fus = b_therm_val;
      // int32_t rgb_fus = (r_fus << 16) | (g_fus << 8) | b_fus;

      pcl_cloud->points[i].rgb = *(float *)(&rgb_fus);
    }
    else {
      // default is grayscale visual image
      int32_t rgb = (inten_vis << 16) | (inten_vis << 8) | inten_vis; 
      pcl_cloud->points[i].rgb = *(float *)(&rgb);
    }

    // TODO - only make this happen if keep insiders is checked
    // throw away points that are not in field of view of any camera
    if (in_vis_FOV == false && in_therm_FOV == false) {
      //pcl_cloud.erase(i);

      // iterator it = points.erase (position); 
      // width = static_cast<uint32_t> (points.size ());
      // height = 1;
      // return (it)      
    }
      
  }
}


// TODO: may need function signature to include more variables
void callback(const sensor_msgs::ImageConstPtr &imgMsg_vis, const sensor_msgs::CameraInfoConstPtr &infoMsg_vis, const sensor_msgs::ImageConstPtr &imgMsg_therm, const sensor_msgs::CameraInfoConstPtr &infoMsg_therm, const sensor_msgs::PointCloud2ConstPtr &cloudMsg) {

  // transfer sensor_msgs image ptr to OpenCV image ptr (TODO: make this another function that can be reused for ueye, flir, rad)  
  cv_bridge::CvImagePtr cv_ptr_vis;
  cv_bridge::CvImagePtr cv_ptr_therm;
  try {
    cv_ptr_vis = cv_bridge::toCvCopy(imgMsg_vis);
    cv_ptr_therm = cv_bridge::toCvCopy(imgMsg_therm);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  
  // extract image matrix from ptr
  cv::Mat img_vis = cv_ptr_vis->image;
  cv::Mat img_therm = cv_ptr_therm->image;  
  // if no rgb frame for coloring:
  if (img_vis.data == NULL || img_therm.data == NULL)
    {
      ROS_ERROR("No Color Information in Image");
      return;
    }

  // create blank sensor_msgs pointclouds and transform to image frames  
  sensor_msgs::PointCloud2 camCld_vis;
  sensor_msgs::PointCloud2 camCld_therm;
  try{
    pcl_ros::transformPointCloud(imgMsg_vis->header.frame_id, *cloudMsg, camCld_vis, *listener) ;    
    pcl_ros::transformPointCloud(imgMsg_therm->header.frame_id, *cloudMsg, camCld_therm, *listener) ;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // build camera models from camera info
  cam_model_vis_.fromCameraInfo(infoMsg_vis); 
  cam_model_therm_.fromCameraInfo(infoMsg_therm);


  // create new PCL point clouds and grab metadata and data from trasnformed sensor_msgs
  pcl::PCLPointCloud2* cld_vis = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cld_therm = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(camCld_vis,*cld_vis);
  pcl_conversions::toPCL(camCld_therm,*cld_therm);

  
  // create clouds that will be colorized and grab data from trasformed PCL point clouds
  pcl::PointCloud<multModalCloud>::Ptr pcl_cloud(new pcl::PointCloud<multModalCloud>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_therm(new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::fromPCLPointCloud2(*cld_vis,*pcl_cloud);
  pcl::fromPCLPointCloud2(*cld_therm,*pcl_cloud_therm);

  pcl::PointIndices fov_indices;


  colorPointCloud(pcl_cloud, pcl_cloud_therm, cam_model_vis_, img_vis, cam_model_therm_, img_therm, fov_indices);


  // transform colored PCL cloud to sensor_msgs cloud
  sensor_msgs::PointCloud2 color_cld;
  
  // TODO: will need to do explicit template instantiation for extractIndices
  // http://www.pcl-users.org/custom-point-types-td2611598.html
  if(!keep_outsiders) {
    // //        ROS_INFO("Only Displaying Points in View");
    // pcl::ExtractIndices<pcl::PointCloud<multModalCloud> > extractFOVPoints2;
    // pcl::ExtractIndices<pcl::PointXYZRGB> extractFOVPoints;
    // extractFOVPoints.setIndices(boost::make_shared<const pcl::PointIndices>(fov_indices));
    // extractFOVPoints.setInputCloud(pcl_cloud);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    // extractFOVPoints.filter(*output);
    // pcl::toROSMsg(*output, color_cld);
    
  }
  else
    pcl::toROSMsg(*pcl_cloud, color_cld);

  sensor_msgs::PointCloud2 output_cloud;
  try{
    pcl_ros::transformPointCloud(cloudMsg->header.frame_id, color_cld, output_cloud, *listener) ;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  output_cloud.header=cloudMsg->header;

  pub.publish(output_cloud);
  fov_indices.indices.clear();
}

int main(int argc, char **argv) {

  // initialize node
  ros::init(argc, argv, "colorize_pointcloud");
  ros::NodeHandle nh("~");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  
  // can change node parameters without having to restart node
  dynamic_reconfigure::Server<colorize_pointcloud::ColorizerConfig> server;
  dynamic_reconfigure::Server<colorize_pointcloud::ColorizerConfig>::CallbackType f;

  f = boost::bind(&configCb, _1, _2);
  server.setCallback(f);

  listener = new tf::TransformListener();

  ros::param::get("~keep_outsiders", keep_outsiders);
  ros::param::get("~rect_image_vis", image_topic_vis);
  ros::param::get("~camera_info_vis", info_topic_vis);
  ros::param::get("~rect_image_therm", image_topic_therm);
  ros::param::get("~camera_info_therm", info_topic_therm);
  ros::param::get("~cloud", cloud_topic);
  ros::param::get("~output_cloud", output_topic);
  ros::param::get("~r", color_r);
  ros::param::get("~g", color_g);
  ros::param::get("~b", color_b);

  color_rgb = (((uint8_t)color_r) << 16) | (((uint8_t)color_g) << 8) | ((uint8_t)color_b); 


  // TODO: add rad image message fiters later
  message_filters::Subscriber<sensor_msgs::Image> image_vis_sub(nh, image_topic_vis, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_vis_sub(nh, info_topic_vis, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_therm_sub(nh, image_topic_therm, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_therm_sub(nh, info_topic_therm, 1);
  // message_filters::Subscriber<sensor_msgs::Image> image_rad_sub(nh, image_topic_rad, 1);
  // message_filters::Subscriber<sensor_msgs::CameraInfo> info_rad_sub(nh, info_topic_rad, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, cloud_topic, 1);

  // sync all necessary messages together, and call callback when ready
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_vis_sub, info_vis_sub, image_therm_sub, info_therm_sub, cloud_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

  ros::spin();
  return 0;
}
