#include <chrono>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//TODO work on this file is assumed using http://wiki.ros.org/cv_camera for camera node

class WhiteLineDetectionNode
{
private:
	/// Enable the openCv visualization if set by the node param.
	bool enableImshow{};

	/// Set on our first connection to the camera.
	bool connected{};

	/// Publish to pointcloud topics
	ros::Publisher pixelPub, pixelPubPCL2;
	/// Subscribe to the camera feed
	ros::Subscriber cameraInfoSub, imageSub;

	/// The lower and upper bound for what we define 'white' as. TODO make sure this is correct.
	int lowColor, upperColor = 255, kernelSize, nthPixel;

	int lowB = lowColor, lowG = lowColor, lowR = lowColor;
	int highB = upperColor, highG = upperColor, highR = upperColor; // lower and upper limits for HSV slider

	/// Camera resolution as retrived from the camera_info topic.
	int HEIGHT, WIDTH;

	double A, B, C, D; // calibration constants

	//These should be constant matrices
	cv::UMat Uinput, //TODO remove this one
		UhsvImage,
		UbinaryImage,
		Uerosion,
		/// The 3x3 perspective transform matrix.
		Utransmtx,
		/// The warped image. Mutable, but needed for the gui.
		Utransformed,
		Uresize;

	cv::Mat image, temp, transmtx;
	cv::Rect ROI = cv::Rect(112, 12, 1670 - 112, 920 - 12); // TODO: change to params. (x, y, width, height)

	std::vector<cv::Point> pixelCoordinates;

	//empty callback functions but it is the only way to increment the sliders
	static void lowBlueTrackbar(int, void *) {}
	static void highBlueTrackbar(int, void *) {}
	static void lowGreenTrackbar(int, void *) {}
	static void highGreenTrackbar(int, void *) {}
	static void lowRedTrackbar(int, void *) {}
	static void highRedTrackbar(int, void *) {}

public:
	WhiteLineDetectionNode()
	{
		ros::NodeHandle nhPrivate("~");
		ros::NodeHandle nh;

		nhPrivate.param("calibration_constants/A", A, 0.0);
		nhPrivate.param("calibration_constants/B", B, 0.0);
		nhPrivate.param("calibration_constants/C", C, 0.0);
		nhPrivate.param("calibration_constants/D", D, 0.0);
		nhPrivate.param("sample_nth_pixel", nthPixel, 5);
		nhPrivate.param("kernel_size", kernelSize, 5);
		nhPrivate.param("lower_bound_white", lowColor, 160);
		nhPrivate.param("enable_imshow", enableImshow, false);

		//TODO change these feilds to be passed between functions.
		image = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
		Utransformed = cv::UMat(HEIGHT, WIDTH, CV_8UC1);
		UbinaryImage = cv::UMat(HEIGHT, WIDTH, CV_8UC1);
		Uerosion = cv::UMat(HEIGHT, WIDTH, CV_8UC1);

		// Activate main logic loop
		pixelPub = nh.advertise<sensor_msgs::PointCloud>("camera_cloud", 1);
		pixelPubPCL2 = nh.advertise<sensor_msgs::PointCloud2>("camera_cloud_pcl2", 1);

		imageSub = nh.subscribe<sensor_msgs::Image>("image_raw", 5, &WhiteLineDetectionNode::rosImageCallback, this);
		cameraInfoSub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &WhiteLineDetectionNode::rosCameraInfoCallback, this);
	}

	void setupOCL()
	{
		cv::setUseOptimized(true);
		cv::ocl::setUseOpenCL(true);
		if (cv::useOptimized())
		{
			std::cout << "OpenCL optimizations enabled" << std::endl;
		}
		else
		{
			std::cout << "OpenCL optimizations NOT enabled" << std::endl;
		}

		if (!cv::ocl::haveOpenCL())
		{
			std::cout << "no opencl devices detected" << std::endl;
		}

		cv::ocl::Context context;
		if (!context.create(cv::ocl::Device::TYPE_GPU))
		{
			std::cout << "failed to initialize device" << std::endl;
		}
		std::cout << context.ndevices() << " GPU device(s) detected." << std::endl;

		std::cout << "************************" << std::endl;
		for (size_t i = 0; i < context.ndevices(); i++)
		{
			cv::ocl::Device device = context.device(i);
			std::cout << "name: " << device.name() << std::endl;
			std::cout << "available: " << device.available() << std::endl;
			std::cout << "img support: " << device.imageSupport() << std::endl;
			std::cout << device.OpenCL_C_Version() << std::endl;
		}
		std::cout << "************************" << std::endl;

		cv::ocl::Device d = cv::ocl::Device::getDefault();
		std::cout << d.OpenCLVersion() << std::endl;
	}

	/// Sets up the constant perspective transform matrix as defined by node params.
	void setupWarp()
	{
		int tl_x, tl_y;
		int tr_x, tr_y;
		int br_x, br_y;
		int bl_x, bl_y;
		double ratio; // width / height of the actual panel on the ground

		ros::NodeHandle nhPrivate("~");

		nhPrivate.param("pixel_coordinates/top_left/x", tl_x, 0);
		nhPrivate.param("pixel_coordinates/top_left/y", tl_y, 0);
		nhPrivate.param("pixel_coordinates/top_right/x", tr_x, 0);
		nhPrivate.param("pixel_coordinates/top_right/y", tr_y, 0);
		nhPrivate.param("pixel_coordinates/bottom_right/x", br_x, 0);
		nhPrivate.param("pixel_coordinates/bottom_right/y", br_y, 0);
		nhPrivate.param("pixel_coordinates/bottom_left/x", bl_x, 0);
		nhPrivate.param("pixel_coordinates/bottom_left/y", bl_y, 0);
		nhPrivate.param("pixel_coordinates/ratio", ratio, 1.0);

		cv::Point Q1 = cv::Point2f(tl_x, tl_y); //top left pixel coordinate
		cv::Point Q2 = cv::Point2f(tr_x, tr_y); //top right
		cv::Point Q3 = cv::Point2f(br_x, br_y); //bottom right
		cv::Point Q4 = cv::Point2f(bl_x, bl_y); //bottom left

		double boardH = sqrt((Q3.x - Q2.x) * (Q3.x - Q2.x) + (Q3.y - Q2.y) * (Q3.y - Q2.y));
		double boardW = ratio * boardH;

		cv::Rect R(Q1.x, Q1.y, boardW, boardH);

		cv::Point R1 = cv::Point2f(R.x, R.y);
		cv::Point R2 = cv::Point2f(R.x + R.width, R.y);
		cv::Point R3 = cv::Point2f(cv::Point2f(R.x + R.width, R.y + R.height));
		cv::Point R4 = cv::Point2f(cv::Point2f(R.x, R.y + R.height));

		std::vector<cv::Point2d> squarePts{R1, R2, R3, R4};
		std::vector<cv::Point2d> quadPts{Q1, Q2, Q3, Q4};

		//Copy transform to constant feild
		transmtx = cv::getPerspectiveTransform(quadPts, squarePts);
		transmtx.copyTo(Utransmtx);
	}

	/// Converts a raw image to an openCv matrix. This function should decode the image properly automatically.
	void ptgrey2CVMat(const sensor_msgs::ImageConstPtr &imageMsg)
	{
		auto cvImage = cv_bridge::toCvCopy(imageMsg, "CV_8UC3"); // This should decode correctly, but we may need to deal with bayer filters
		cvImage->image.copyTo(Uinput);							 //TODO we can just return this mat when removing feilds.
	}

	void shiftPerspective()
	{
		cv::warpPerspective(Uinput, Utransformed, Utransmtx, Utransformed.size());
		Uresize = Utransformed(ROI);
	}

	void imageFiltering()
	{
		cv::Mat erosionKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
		cv::inRange(Uresize, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), UbinaryImage);
		cv::erode(UbinaryImage, Uerosion, erosionKernel);
	}

	void getPixelPointCloud()
	{
		sensor_msgs::PointCloud msg;
		sensor_msgs::PointCloud2 msg2;
		cv::findNonZero(Uerosion, pixelCoordinates);
		for (size_t i = 0; i < pixelCoordinates.size(); i++)
		{
			if (i % nthPixel == 0)
			{
				geometry_msgs::Point32 pixelLoc;
				//XY distances of each white pixel relative to robot
				pixelLoc.x = (A * pixelCoordinates[i].x) + B;
				pixelLoc.y = (C * pixelCoordinates[i].y) + D;
				msg.points.push_back(pixelLoc);
			}
		}

		sensor_msgs::convertPointCloudToPointCloud2(msg, msg2);

		pixelPub.publish(msg);
		pixelPubPCL2.publish(msg2);
	}

	void createGUI()
	{
		cv::namedWindow("original", cv::WINDOW_FREERATIO);
		cv::namedWindow("erosion", cv::WINDOW_FREERATIO);
		cv::namedWindow("warp", cv::WINDOW_FREERATIO);
		cv::namedWindow("TRACKBARS", cv::WINDOW_FREERATIO);
		//*****************GUI related *********************************
		cv::createTrackbar("Low Blue", "TRACKBARS", &lowB, upperColor, lowBlueTrackbar);
		cv::createTrackbar("Low Green", "TRACKBARS", &lowG, upperColor, lowGreenTrackbar);
		cv::createTrackbar("Low Red", "TRACKBARS", &lowR, upperColor, lowRedTrackbar);
		cv::createTrackbar("High Blue", "TRACKBARS", &highB, upperColor, highBlueTrackbar);
		cv::createTrackbar("High Green", "TRACKBARS", &highG, upperColor, highGreenTrackbar);
		cv::createTrackbar("High Red", "TRACKBARS", &highR, upperColor, highRedTrackbar);
	}

	void display()
	{
		if (enableImshow)
		{
			cv::imshow("original", Uinput);
			cv::imshow("warp", Utransformed);
			cv::imshow("erosion", Uerosion);
		}
	}

	/// Callback passed to the image topic subscription. This produces a pointcloud for every
	/// image sent on the topic.
	void rosImageCallback(const sensor_msgs::ImageConstPtr &imageMsg)
	{
		if (!connected)
		{
			ROS_ERROR("Received image without receiving camera info first. This should not occur, and is a logic error.");
		}
		else
		{
			//Decode image
			ptgrey2CVMat(imageMsg);

			shiftPerspective();
			imageFiltering();

			getPixelPointCloud();
			display();
		}
	}

	/// Callback passed to the camera info sub.
	void rosCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &infoMsg)
	{
		if (!connected) //attempt to prevent this callback from spamming.
		{
			HEIGHT = infoMsg->height;
			WIDTH = infoMsg->height;
			ROS_INFO("Connected to camera");
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "white_line_detection");

	WhiteLineDetectionNode whiteLineDetector;

	//Setup enviornment
	whiteLineDetector.setupOCL();
	whiteLineDetector.createGUI();
	whiteLineDetector.setupWarp(); // 05/15/2019 starting from clockwise tl_x: (662, 315) (1263, 318) (1371, 522) (573, 520) 1.5015

	ros::spin();

	ROS_INFO("Camera node shutting down");

	return 0;
}
