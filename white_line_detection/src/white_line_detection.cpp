#include <chrono>
#include <flycapture/FlyCapture2.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

class CameraNode
{
  private:
	FlyCapture2::Camera camera;
	FlyCapture2::Image rawImage, encodedImage, bgrImage; //Contains the raw and converted frames from the camera.
	FlyCapture2::Error capture;


	bool connected;
	bool enableImshow;

	ros::Publisher pixelPub, pixelPubPCL2;

	int lowColor, upperColor = 255, kernelSize, nthPixel;
	int lowB = lowColor, lowG = lowColor, lowR = lowColor;
	int highB = upperColor, highG = upperColor, highR = upperColor; // lower and upper limits for HSV slider
	int HEIGHT, WIDTH;												//camera resolution retrieved by camera api
	double A, B, C, D;												// calibration constants

	cv::UMat Uinput, UhsvImage, UbinaryImage, Uerosion, Utransmtx, Utransformed, Uresize; //for use on GPU
	cv::Mat image, temp, transmtx;
	cv::Rect ROI = cv::Rect(112, 12, 1670 - 112, 920 - 12); // TODO: change to params. (x, y, width, height)

	std::vector<cv::Point2f> quadPts, squarePts;
	std::vector<cv::Point> pixelCoordinates;

	//empty callback functions but it is the only way to increment the sliders
	static void lowBlueTrackbar(int, void *) {}
	static void highBlueTrackbar(int, void *) {}
	static void lowGreenTrackbar(int, void *) {}
	static void highGreenTrackbar(int, void *) {}
	static void lowRedTrackbar(int, void *) {}
	static void highRedTrackbar(int, void *) {}

  public:
	CameraNode()
	{
		connected = false;
		ros::NodeHandle nhPrivate("~");
		ros::NodeHandle nh;

		pixelPub = nh.advertise<sensor_msgs::PointCloud>("camera_cloud", 1);
		pixelPubPCL2 = nh.advertise<sensor_msgs::PointCloud2>("camera_cloud_pcl2", 1);

		nhPrivate.param("calibration_constants/A", A, 0.0);
		nhPrivate.param("calibration_constants/B", B, 0.0);
		nhPrivate.param("calibration_constants/C", C, 0.0);
		nhPrivate.param("calibration_constants/D", D, 0.0);
		nhPrivate.param("sample_nth_pixel", nthPixel, 5);
		nhPrivate.param("kernel_size", kernelSize, 5);
		nhPrivate.param("lower_bound_white", lowColor, 160);
		nhPrivate.param("enable_imshow", enableImshow, false);

		capture = camera.Connect(0);
		if (capture != FlyCapture2::PGRERROR_OK)
		{
			ROS_ERROR("Could not connect to camera. Please check connection");
		}
		else
		{
			connected = true;
			capture = camera.StartCapture();
			HEIGHT = rawImage.GetRows();
			WIDTH = rawImage.GetCols();
			image = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
			Utransformed = cv::UMat(HEIGHT, WIDTH, CV_8UC1);
			UbinaryImage = cv::UMat(HEIGHT, WIDTH, CV_8UC1);
			Uerosion = cv::UMat(HEIGHT, WIDTH, CV_8UC1);
		}
	}
	~CameraNode()
	{
		capture = camera.StopCapture();
		camera.Disconnect();
	}

	bool ok() { return connected; }

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
  
	void setupWarp()
	{ // TODO: change these parameters to ros params
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

		squarePts = {R1, R2, R3, R4};
		quadPts = {Q1, Q2, Q3, Q4};

		transmtx = cv::getPerspectiveTransform(quadPts, squarePts);
		transmtx.copyTo(Utransmtx);
	}

	void ptgrey2CVMat()
	{
		capture = camera.RetrieveBuffer(&rawImage);
		if (capture != FlyCapture2::PGRERROR_OK)
		{
			ROS_ERROR("Could not retrieve image!, Camera Disconnected"); // TODO: change to ROS ERROR
			connected = false;
		}
		// convert to bgr
		FlyCapture2::Image bgrImage;
		rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &bgrImage);
		// convert to OpenCV Mat
		unsigned int rowBytes =
			(double)bgrImage.GetReceivedDataSize() / (double)bgrImage.GetRows();
		temp = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3,
					   bgrImage.GetData(), rowBytes);
		image = temp.clone();
		image.copyTo(Uinput);
	}

	void shiftPerspective()
	{
		cv::warpPerspective(Uinput, Utransformed, Utransmtx, Utransformed.size());
		//cv::imshow("warp",Utransformed);
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
		cv::namedWindow("original", CV_WINDOW_FREERATIO);
		cv::namedWindow("erosion", CV_WINDOW_FREERATIO);
		cv::namedWindow("warp", CV_WINDOW_FREERATIO);
		cv::namedWindow("TRACKBARS", CV_WINDOW_FREERATIO);
		//*****************GUI related *********************************
		cv::createTrackbar("Low Blue", "TRACKBARS", &lowB, upperColor, lowBlueTrackbar);
		cv::createTrackbar("Low Green", "TRACKBARS", &lowG, upperColor, lowGreenTrackbar);
		cv::createTrackbar("Low Red", "TRACKBARS", &lowR, upperColor, lowRedTrackbar);
		cv::createTrackbar("High Blue", "TRACKBARS", &highB, upperColor, highBlueTrackbar);
		cv::createTrackbar("High Green", "TRACKBARS", &highG, upperColor, highGreenTrackbar);
		cv::createTrackbar("High Red", "TRACKBARS", &highR, upperColor, highRedTrackbar);
	}
  
  void display() // TODO: add param for displaying windows or not
	{
		if (enableImshow)
		{
			cv::imshow("original", Uinput);
			cv::imshow("warp", Utransformed);
			cv::imshow("erosion", Uerosion);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "white_line_detection");

	CameraNode whiteLineDetector;
	
	if (whiteLineDetector.ok())
	{
		whiteLineDetector.setupOCL();
		whiteLineDetector.createGUI();
		whiteLineDetector.setupWarp(); // 05/15/2019 starting from clockwise tl_x: (662, 315) (1263, 318) (1371, 522) (573, 520) 1.5015

		while ((char)cv::waitKey(1) != 'q' && ros::ok())
		{
			whiteLineDetector.ptgrey2CVMat();

			if (whiteLineDetector.ok())
			{
				whiteLineDetector.shiftPerspective();
				whiteLineDetector.imageFiltering();

				whiteLineDetector.getPixelPointCloud();
				whiteLineDetector.display();
			} else {
				break; // there was an error and the camera is disconnected
			}
		}
	}

	ROS_INFO("Camera node shutting down");

	return 0;
}
