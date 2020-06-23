#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class VelodynePointcloudToDepthimage{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;
		/*publisher*/
		ros::Publisher _pub_img;
		/*image*/
		cv::Mat _img_cv;
		/*point cloud*/
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _rings;
		/*parameter*/
		int _num_ring;
		int _points_per_ring;

	public:
		VelodynePointcloudToDepthimage();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void pcToRings(const sensor_msgs::PointCloud2& pc_msg);
		void ringsToImage(void);
		void publication(std_msgs::Header header);
};

VelodynePointcloudToDepthimage::VelodynePointcloudToDepthimage()
	: _nhPrivate("~")
{
	std::cout << "--- velodyne_pointcloud_to_depthimage ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("num_ring", _num_ring, 32);
	std::cout << "_num_ring = " << _num_ring << std::endl;
	_nhPrivate.param("points_per_ring", _points_per_ring, 1092);
	std::cout << "_points_per_ring = " << _points_per_ring << std::endl;
	/*sub*/
	_sub_pc = _nh.subscribe("/velodyne_points", 1, &VelodynePointcloudToDepthimage::callbackPC, this);
	/*pub*/
	_pub_img = _nh.advertise<sensor_msgs::Image>("/depth_imgae", 1);
	/*initialize*/
	_rings.resize(_num_ring);
	for(size_t i=0 ; i<_rings.size() ; ++i){
		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
		_rings[i] = tmp;
	}
}

void VelodynePointcloudToDepthimage::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "-----" << std::endl;
	for(size_t i=0 ; i<_rings.size() ; ++i){
		_rings[i]->points.clear();
	}
	pcToRings(*msg);
	ringsToImage();
	publication(msg->header);
}

void VelodynePointcloudToDepthimage::pcToRings(const sensor_msgs::PointCloud2& pc_msg)
{
	sensor_msgs::PointCloud2ConstIterator<int> iter_ring(pc_msg,"ring");
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_msg,"x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_msg,"y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_msg,"z");
	sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(pc_msg,"intensity");

	for( ; iter_ring!=iter_ring.end() ; ++iter_ring, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity){
		pcl::PointXYZI tmp;
		tmp.x = *iter_x;
		tmp.y = *iter_y;
		tmp.z = *iter_z;
		tmp.intensity = *iter_intensity;
		_rings[*iter_ring]->points.push_back(tmp);
	}
}

void VelodynePointcloudToDepthimage::ringsToImage(void)
{
	_img_cv = cv::Mat::zeros(_num_ring, _points_per_ring, CV_64FC1);

	double angle_resolution = 2*M_PI/(double)_points_per_ring;
	for(size_t i=0 ; i<_rings.size() ; ++i){
		int row = _rings.size() - i - 1;
		for(size_t j=0 ; j<_rings[i]->points.size() ; ++j){
			double angle = atan2(_rings[i]->points[j].y, _rings[i]->points[j].x);
			int col = (int)((angle + M_PI)/angle_resolution);
			_img_cv.at<double>(row, col) = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);
		}
	}
	std::string save_img_path = "/home/amsl/ozaki/depthimage_example.jpg";
	cv::imwrite(save_img_path, _img_cv);
}

void VelodynePointcloudToDepthimage::publication(std_msgs::Header header)
{
	cv::Mat img_cv_bit;
	_img_cv.convertTo(img_cv_bit, CV_16UC1);
	sensor_msgs::ImagePtr img_ros = cv_bridge::CvImage(header, "mono16", img_cv_bit).toImageMsg();
	_pub_img.publish(img_ros);

	std::cout << "_img_cv = " << _img_cv << std::endl;
	std::cout << "img_cv_bit = " << img_cv_bit << std::endl;
	int row = 0;
	for(int col=0 ; col<1000 ; col+=100){
		std::cout << "_img_cv.at<double>(row, col) = " << _img_cv.at<double>(row, col) << std::endl;
		std::cout << "img_cv_bit.at<unsigned short>(row, col) = " << img_cv_bit.at<unsigned short>(row, col) << std::endl;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_pointcloud_to_depthimage");
	
	VelodynePointcloudToDepthimage velodyne_pointcloud_to_depthimage;

	ros::spin();
}
