#include <thread>

#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
using namespace cv;
using namespace std;
RNG rng(12345);
/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
class Robot: public frc::IterativeRobot {
private:
	static void VisionThread() {
		// Get the USB camera from CameraServer

		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(1);
		// Set the resolution
		camera.SetResolution(640, 480);
		camera.SetFPS(20);
		cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture(0);
					camera2.SetResolution(160, 120);
					camera2.SetFPS(5);
		// Get a CvSink. This will capture Mats from the Camera
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		cs::CvSource outputStream = CameraServer::GetInstance()->
				PutVideo("Rectangle", 640, 480);

		// Mats are very memory expensive. Lets reuse this Mat.
		cv::Mat mat;
		cv::Mat mat2;
		while (true) {
			cv::Mat Erode_Kernel;
			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat.  If there is an error notify the output.
			if (cvSink.GrabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
				// skip the rest of the current iteration
				continue;
			}
			// Put a rectangle on the image
			//rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
					//cv::Scalar(255, 255, 255), 5);
			cv::cvtColor(mat,mat2,cv::COLOR_BGR2HSV);
					cv::inRange(mat2,cv::Scalar(129,16,167),cv::Scalar(174,77,244),mat);
						cv::erode(mat,mat2,Erode_Kernel,cv::Point(-1, -1),2.0,cv::BORDER_CONSTANT,cv::Scalar(-1));

					    	cv::dilate(mat2,mat,Erode_Kernel,cv::Point(-1,-1),3.0, cv::BORDER_CONSTANT,cv::Scalar(-1));
					    	 vector<vector<Point> > contours;
					    	 vector<Vec4i> hierarchy;
					    	findContours(mat,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0, 0));
					    	 Mat drawing = Mat::zeros( mat.size(), CV_8UC3 );
					    	  for( int i = 0; i< contours.size(); i++ )
					    	     {
					    	       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
					    	       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
					    	     }

						    	outputStream.PutFrame(drawing);




		}
	}

	void RobotInit() {
		// We need to run our vision program in a separate Thread.
		// If not, our robot program will not run
		std::thread visionThread(VisionThread);
		visionThread.detach();
	}

};
/*//Step HSV_Threshold0:
	//input
	Mat hsvThresholdInput = source0;
	double hsvThresholdHue[] = {0.0, 180.0};
	double hsvThresholdSaturation[] = {36.690647482014384, 255.0};
	double hsvThresholdValue[] = {197.21223021582733, 255.0};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);

	cvtColor(input, out, COLOR_BGR2HSV);
		inRange(out,Scalar(hue[0], sat[0], val[0]), Scalar(hue[1], sat[1], val[1]), out);


	//Step CV_erode0:
	//input
	Mat cvErodeSrc = hsvThresholdOutput;
	Mat cvErodeKernel;
	Point cvErodeAnchor(-1, -1);
	double cvErodeIterations = 7.0;
	int cvErodeBordertype = BORDER_CONSTANT;
	Scalar cvErodeBordervalue(-1);
	cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, this->cvErodeOutput);
	//Step CV_dilate0:
	//input
	Mat cvDilateSrc = cvErodeOutput;
	Mat cvDilateKernel;
	Point cvDilateAnchor(-1, -1);
	double cvDilateIterations = 11.0;
	int cvDilateBordertype = BORDER_CONSTANT;
	Scalar cvDilateBordervalue(-1);
	cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, this->cvDilateOutput);
	//Step Find_Contours0:
	//input
	Mat findContoursInput = cvDilateOutput;
	bool findContoursExternalOnly = false;
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	//Step Filter_Contours0:
	//input
	vector<vector<Point> > filterContoursContours = findContoursOutput;
	double filterContoursMinArea = 0.0;
	double filterContoursMinPerimeter = 0.0;
	double filterContoursMinWidth = 80.0;
	double filterContoursMaxWidth = 150.0;
	double filterContoursMinHeight = 250.0;
	double filterContoursMaxHeight = 400.0;
	double filterContoursSolidity[] = {0, 100};
	double filterContoursMaxVertices = 1000000.0;
	double filterContoursMinVertices = 0.0;
	double filterContoursMinRatio = 0.0;
	double filterContoursMaxRatio = 1000.0;
	filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->filterContoursOutput);*/

START_ROBOT_CLASS(Robot)
