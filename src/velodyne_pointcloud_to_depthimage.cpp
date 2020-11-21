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
		ros::Publisher _pub_img_64f;
		ros::Publisher _pub_img_16u;
		ros::Publisher _pub_img_8u;
		/*file*/
		cv::FileStorage _fs;
		/*image*/
		cv::Mat _img_cv_64f;
		cv::Mat _img_cv_16u;
		cv::Mat _img_cv_8u;
		/*point cloud*/
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _rings;
		/*counter*/
		int _save_counter = 0;
		/*parameter*/
		int _num_ring;
		int _points_per_ring;
		double _depth_resolution;
		double _max_range;
		int _save_limit;
		std::string _save_root_path;
		std::string _save_img_name;

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
	_nhPrivate.param("depth_resolution", _depth_resolution, 0.01);
	std::cout << "_depth_resolution = " << _depth_resolution << std::endl;
	_nhPrivate.param("max_range", _max_range, 100.0);
	std::cout << "_max_range = " << _max_range << std::endl;
	_nhPrivate.param("save_limit", _save_limit, -1);
	std::cout << "_save_limit = " << _save_limit << std::endl;
	_nhPrivate.param("save_root_path", _save_root_path, std::string("saved"));
	std::cout << "_save_root_path = " << _save_root_path << std::endl;
	_nhPrivate.param("save_img_name", _save_img_name, std::string("depth_"));
	std::cout << "_save_img_name = " << _save_img_name << std::endl;
	/*sub*/
	_sub_pc = _nh.subscribe("/velodyne_points", 1, &VelodynePointcloudToDepthimage::callbackPC, this);
	/*pub*/
	_pub_img_64f = _nh.advertise<sensor_msgs::Image>("/depth_image/64fc1", 1);
	_pub_img_16u = _nh.advertise<sensor_msgs::Image>("/depth_image/16uc1", 1);
	_pub_img_8u = _nh.advertise<sensor_msgs::Image>("/depth_image/8uc1", 1);
	/*initialize*/
	_rings.resize(_num_ring);
	for(size_t i=0 ; i<_rings.size() ; ++i){
		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
		_rings[i] = tmp;
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
	sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(pc_msg,"ring");
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
	// _img_cv_64f = cv::Mat::zeros(_num_ring, _points_per_ring, CV_64FC1);
	_img_cv_64f = cv::Mat(_num_ring, _points_per_ring, CV_64FC1, cv::Scalar(-1));
	/*input*/
	double angle_resolution = 2*M_PI/(double)_points_per_ring;
	for(size_t i=0 ; i<_rings.size() ; ++i){
		int row = _rings.size() - i - 1;
		for(size_t j=0 ; j<_rings[i]->points.size() ; ++j){
			double angle = atan2(_rings[i]->points[j].y, _rings[i]->points[j].x);
			int col = _points_per_ring - (int)((angle + M_PI)/angle_resolution) - 1;
			_img_cv_64f.at<double>(row, col) = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);
		}
	}
	/*convert*/
	_img_cv_64f.convertTo(_img_cv_16u, CV_16UC1, 1/_depth_resolution, 0);
	_img_cv_64f.convertTo(_img_cv_8u, CV_8UC1, 255/_max_range, 0);
	/*save*/
	if(_save_limit > 0 && _save_counter < _save_limit){
		/*CV_64FC1*/
		std::string save_mat64f_path = _save_root_path + "/" + _save_img_name + std::to_string(_save_counter) + "_64f.xml";
		cv::FileStorage fs(save_mat64f_path, cv::FileStorage::WRITE);
		if(!fs.isOpened()){
			std::cout << save_mat64f_path << " cannot be opened" << std::endl;
			exit(1);
		}
		fs << "mat" << _img_cv_64f;
		fs.release();
		/*CV_16UC1*/
		std::string save_img16u_path = _save_root_path + "/"  + _save_img_name + std::to_string(_save_counter) + "_16u.jpg";
		cv::imwrite(save_img16u_path, _img_cv_16u);
		/*CV_8UC1*/
		std::string save_img8u_path = _save_root_path + "/"  + _save_img_name + std::to_string(_save_counter) + "_8u.jpg";
		cv::imwrite(save_img8u_path, _img_cv_8u);
		/*count*/
		++_save_counter;
		/*print*/
		std::cout << "----- " << _save_counter << " -----" << std::endl;
		std::cout << "_img_cv_64f: " << _img_cv_64f.size().height << " x " << _img_cv_64f.size().width << std::endl;
		std::cout << "_img_cv_16u: " << _img_cv_16u.size().height << " x " << _img_cv_16u.size().width << std::endl;
		std::cout << "_img_cv_8u: " << _img_cv_8u.size().height << " x " << _img_cv_8u.size().width << std::endl;
		//for(int row=0 ; row<_img_cv_64f.size().height ; row+=_img_cv_64f.size().height/3){
		//	for(int col=0 ; col<_img_cv_64f.size().width ; col+=_img_cv_64f.size().width/3){
		//		std::cout << "_img_cv_64f.at<double>(" << row << ", " << col << ") = " << _img_cv_64f.at<double>(row, col) << std::endl;
		//		std::cout << "_img_cv_16u.at<unsigned short>(" << row << ", " << col << ") = " << _img_cv_16u.at<unsigned short>(row, col) << std::endl;
		//		std::cout << "_img_cv_16u.at<unsigned long int>(" << row << ", " << col << ") = " << _img_cv_16u.at<unsigned long int>(row, col) << std::endl;
		//		std::cout << "(int)_img_cv_8u.at<unsigned char>(" << row << ", " << col << ") = " << (int)_img_cv_8u.at<unsigned char>(row, col) << std::endl;
		//	}
		//}
	}
	else	_fs.release();
}

void VelodynePointcloudToDepthimage::publication(std_msgs::Header header)
{
	sensor_msgs::ImagePtr img_ros_64f = cv_bridge::CvImage(header, "64FC1", _img_cv_64f).toImageMsg();
	sensor_msgs::ImagePtr img_ros_16u = cv_bridge::CvImage(header, "mono16", _img_cv_16u).toImageMsg();
	sensor_msgs::ImagePtr img_ros_8u = cv_bridge::CvImage(header, "mono8", _img_cv_8u).toImageMsg();
	_pub_img_64f.publish(img_ros_64f);
	_pub_img_16u.publish(img_ros_16u);
	_pub_img_8u.publish(img_ros_8u);

	/*check*/
	//cv_bridge::CvImagePtr cv_ptr_64f = cv_bridge::toCvCopy(img_ros_64f, img_ros_64f->encoding);
	//cv_bridge::CvImagePtr cv_ptr_16u = cv_bridge::toCvCopy(img_ros_16u, img_ros_16u->encoding);
	//cv_bridge::CvImagePtr cv_ptr_8u = cv_bridge::toCvCopy(img_ros_8u, img_ros_8u->encoding);
	//for(int row=0 ; row<cv_ptr_64f->image.size().height ; row+=cv_ptr_64f->image.size().height/3){
	//	for(int col=0 ; col<cv_ptr_64f->image.size().width ; col+=cv_ptr_64f->image.size().width/3){
	//		std::cout << "_img_cv_64f.at<double>(" << row << ", " << col << ") = " << _img_cv_64f.at<double>(row, col) << std::endl;
	//		std::cout << "cv_ptr_64f->image.at<double>(" << row << ", " << col << ") = " << cv_ptr_64f->image.at<double>(row, col) << std::endl;
	//		std::cout << "cv_ptr_16u->image.at<unsigned short>(" << row << ", " << col << ") = " << cv_ptr_16u->image.at<unsigned short>(row, col) << std::endl;
	//		std::cout << "(int)cv_ptr_8u->image.at<unsigned char>(" << row << ", " << col << ") = " << (int)cv_ptr_8u->image.at<unsigned char>(row, col) << std::endl;
	//	}
	//}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_pointcloud_to_depthimage");
	
	VelodynePointcloudToDepthimage velodyne_pointcloud_to_depthimage;

	ros::spin();
}
