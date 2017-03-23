/* #ifndef PCL_COLORIZE_POINTCLOUD_ */
/* #define PCL_COLORIZE_POINTCLOUD_ */

#ifndef PCL_FOO_
#define PCL_FOO_

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

//template <typename PointT>
template <typename PointCloud2>
class pcl::PointCloud<multModalCloud>
//class Foo
{
  public:
    void
    compute (const pcl::PointCloud<PointT> &input,
             pcl::PointCloud<PointT> &output);
};

#endif // PCL_FOO_
