#include "test_contour.h"
/**
* Initializes a test_contour.
*/
test_contour::test_contour() {
}
/**
* Runs an iteration of the Pipeline and updates outputs.
*
* Sources need to be set before calling this method. 
*
*/
void test_contour::Process(Mat Camera){
	//Step HSV_Threshold0:
	//input

	Mat hsvThresholdInput = Camera;
	double hsvThresholdHue[] = {0.0, 180.0};
	double hsvThresholdSaturation[] = {0.0, 46.12627986348124};
	double hsvThresholdValue[] = {201.79856115107913, 255.0};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
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
	filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->filterContoursOutput);
}

/**
 * This method is a generated setter for source0.
 * @param source the Mat to set
 */
void test_contour::setsource0(Mat &source0){
	source0.copyTo(this->source0);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
Mat* test_contour::gethsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
Mat* test_contour::getcvErodeOutput(){
	return &(this->cvErodeOutput);
}
/**
 * This method is a generated getter for the output of a CV_dilate.
 * @return Mat output from CV_dilate.
 */
Mat* test_contour::getcvDilateOutput(){
	return &(this->cvDilateOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
vector<vector<Point> >* test_contour::getfindContoursOutput(){
	return &(this->findContoursOutput);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
vector<vector<Point> >* test_contour::getfilterContoursOutput(){
	return &(this->filterContoursOutput);
}
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void test_contour::hsvThreshold(Mat &input, double hue[], double sat[], double val[], Mat &out) {
		cvtColor(input, out, COLOR_BGR2HSV);
		inRange(out,Scalar(hue[0], sat[0], val[0]), Scalar(hue[1], sat[1], val[1]), out);
	}
	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void test_contour::cvErode(Mat &src, Mat &kernel, Point &anchor, double iterations, int borderType, Scalar &borderValue, Mat &dst) {
		erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}
	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void test_contour::cvDilate(Mat &src, Mat &kernel, Point &anchor, double iterations, int borderType, Scalar &borderValue, Mat &dst) {
		dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}
	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void test_contour::findContours(Mat &input, bool externalOnly, vector<vector<Point> > &contours) {
		vector<Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? RETR_EXTERNAL : RETR_LIST;
		int method = CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Filters through contours.
	 * @param inputContours is the input vector of contours.
	 * @param minArea is the minimum area of a contour that will be kept.
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
	 * @param minWidth minimum width of a contour.
	 * @param maxWidth maximum width.
	 * @param minHeight minimum height.
	 * @param maxHeight  maximimum height.
	 * @param solidity the minimum and maximum solidity of a contour.
	 * @param minVertexCount minimum vertex Count of the contours.
	 * @param maxVertexCount maximum vertex Count.
	 * @param minRatio minimum ratio of width to height.
	 * @param maxRatio maximum ratio of width to height.
	 * @param output vector of filtered contours.
	 */
	void test_contour::filterContours(vector<vector<Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, vector<vector<Point> > &output) {
		vector<Point> hull;
		output.clear();
		for (vector<Point> contour: inputContours) {
			Rect bb = boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			double area = contourArea(contour);
			if (area < minArea) continue;
			if (arcLength(contour, true) < minPerimeter) continue;
			convexHull(Mat(contour, true), hull);
			double solid = 100 * area / contourArea(hull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
			double ratio = bb.width / bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.push_back(contour);
		}
	}

