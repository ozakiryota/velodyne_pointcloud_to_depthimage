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
		/*file*/
		cv::FileStorage _fs;
		/*image*/
		cv::Mat _img_cv_64fc;
		cv::Mat _img_cv_16uc;
		/*point cloud*/
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _rings;
		/*counter*/
		int _save_counter = 0;
		/*parameter*/
		int _num_ring;
		int _points_per_ring;
		double _depth_resolution;
		int _save_limit;
		std::string _save_root_path;
		std::string _save_img_name;
		std::string _save_yaml_name;
		std::string _save_jpgdir_name;

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
	_nhPrivate.param("depth_resolution", _depth_resolution, 0.1);
	std::cout << "_depth_resolution = " << _depth_resolution << std::endl;
	_nhPrivate.param("save_limit", _save_limit, -1);
	std::cout << "_save_limit = " << _save_limit << std::endl;
	_nhPrivate.param("save_root_path", _save_root_path, std::string("saved"));
	std::cout << "_save_root_path = " << _save_root_path << std::endl;
	_nhPrivate.param("save_img_name", _save_img_name, std::string("depth_"));
	std::cout << "_save_img_name = " << _save_img_name << std::endl;
	_nhPrivate.param("save_yaml_name", _save_yaml_name, std::string("32e_CV64FC1"));
	std::cout << "_save_yaml_name = " << _save_yaml_name << std::endl;
	_nhPrivate.param("save_jpgdir_name", _save_jpgdir_name, std::string("32e_CV16UC1"));
	std::cout << "_save_jpgdir_name = " << _save_jpgdir_name << std::endl;
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
	std::string save_yaml_path = _save_root_path + "/" + _save_yaml_name + ".yml";
	_fs.open(save_yaml_path, cv::FileStorage::WRITE);
	if(!_fs.isOpened()){
		std::cout << save_yaml_path << "cannot be opened" << std::endl;
		exit(1);
	}
}

void VelodynePointcloudToDepthimage::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
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
	/*reset*/
	_img_cv_64fc = cv::Mat::zeros(_num_ring, _points_per_ring, CV_64FC1);
	/*input*/
	double angle_resolution = 2*M_PI/(double)_points_per_ring;
	for(size_t i=0 ; i<_rings.size() ; ++i){
		int row = _rings.size() - i - 1;
		for(size_t j=0 ; j<_rings[i]->points.size() ; ++j){
			double angle = atan2(_rings[i]->points[j].y, _rings[i]->points[j].x);
			int col = (int)((angle + M_PI)/angle_resolution);
			_img_cv_64fc.at<double>(row, col) = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);
		}
	}
	/*convert*/
	_img_cv_64fc.convertTo(_img_cv_16uc, CV_16UC1, 1/_depth_resolution, 0);
	/*save*/
	if(_save_limit > 0 && _save_counter < _save_limit){
		/*CV_64FC1*/
		std::string save_mat_name = _save_img_name + std::to_string(_save_counter);
		_fs << save_mat_name.c_str() << _img_cv_64fc;
		/*CV_16UC1*/
		std::string save_img_path = _save_root_path + "/" + _save_jpgdir_name + "/"  + _save_img_name + std::to_string(_save_counter) + ".jpg";
		cv::imwrite(save_img_path, _img_cv_16uc);
		/*count*/
		++_save_counter;
		/*print*/
		std::cout << "-----" << std::endl;
		std::cout << "_img_cv_64fc: " << _img_cv_64fc.size().height << " x " << _img_cv_64fc.size().width << std::endl;
		std::cout << "_img_cv_16uc: " << _img_cv_16uc.size().height << " x " << _img_cv_16uc.size().width << std::endl;
		for(int row=0 ; row<_img_cv_64fc.size().height ; row+=_img_cv_64fc.size().height/2){
			for(int col=0 ; col<_img_cv_64fc.size().width ; col+=_img_cv_64fc.size().width/3){
				std::cout << "_img_cv_64fc.at<double>(" << row << ", " << col << ") = " << _img_cv_64fc.at<double>(row, col) << std::endl;
				std::cout << "_img_cv_16uc.at<unsigned short>(" << row << ", " << col << ") = " << _img_cv_16uc.at<unsigned short>(row, col) << std::endl;
			}
		}
	}
	else	_fs.release();
}

void VelodynePointcloudToDepthimage::publication(std_msgs::Header header)
{
	sensor_msgs::ImagePtr img_ros = cv_bridge::CvImage(header, "mono16", _img_cv_16uc).toImageMsg();
	_pub_img.publish(*img_ros);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_pointcloud_to_depthimage");
	
	VelodynePointcloudToDepthimage velodyne_pointcloud_to_depthimage;

	ros::spin();
}
