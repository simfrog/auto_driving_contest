#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define IMG_SIZE 100

using namespace std;
using namespace cv;

class SignDetector{
public:
/*
    int test() //Test Code
    {
        char chBuf[255];

        ////// Load Template //////
        vector<Mat> vMatAB(2), vMatOTH(3);
        vMatAB[0] = imread("/home/autoware/shared_dir/pic/template/00.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatAB[1] = imread("/home/autoware/shared_dir/pic/template/01.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatOTH[0] = imread("/home/autoware/shared_dir/pic/template/02.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatOTH[1] = imread("/home/autoware/shared_dir/pic/template/03.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatOTH[2] = imread("/home/autoware/shared_dir/pic/template/04.jpg", CV_LOAD_IMAGE_GRAYSCALE);

        Rect rectABROI(10, 10, 50, 80), rectOTHROI(40, 10, 50, 80);
        
        for (int i = 0; i < 53; i++) {
            sprintf(chBuf, "/home/autoware/shared_dir/pic/imgs/%02d.jpg", i);
            cout << i << endl;
            string strFilename = chBuf;
            Mat matCurImage = imread(strFilename);
            Mat matResizeImage; resize(matCurImage, matResizeImage, Size(IMG_SIZE, IMG_SIZE));
            Mat matGrayImage; cvtColor(matResizeImage, matGrayImage, COLOR_BGR2GRAY);
            Mat matBinaryImage; threshold(matGrayImage, matBinaryImage, 128, 255, THRESH_OTSU);

            ////// Match Template //////
            int nABMaxIdx, nOTHMaxIdx;
            double dABMaxVal = DBL_MIN, dOTHMaxVal = DBL_MIN;
            vector<double> vAccABVal(2), vAccOTHVal(3);
            Mat matResponseMap, matABImage = matBinaryImage(rectABROI), matOTHImage = matBinaryImage(rectOTHROI);
            imshow("matABImage",matABImage);
            imshow("matOTHImage",matOTHImage);
            waitKey(0);
            for (int j = 0; j < vMatAB.size(); j++) {
                for (int k = -2; k < 3; k++) {
                    Mat matCurABImage; resize(matABImage, matCurABImage, Size(IMG_SIZE * (1.0 - 0.2 * k), IMG_SIZE * (1.0 - 0.2 * k)));

                    matchTemplate(matCurABImage, vMatAB[j], matResponseMap, TM_CCOEFF_NORMED);

                    ////// Check Response //////
                    //Mat matNormResponseMap;  normalize(matResponseMap, matNormResponseMap, 0, 255, NORM_MINMAX, CV_8U);
                    //imshow("response", matNormResponseMap);
                    //waitKey(0);

                    double dCurMinVal, dCurMaxVal;
                    minMaxLoc(matResponseMap, &dCurMinVal, &dCurMaxVal);
                    if (dCurMaxVal > dABMaxVal) {
                        nABMaxIdx = j;
                        dABMaxVal = dCurMaxVal;
                    }

                    //cout << j << " " << dCurMaxVal << endl;
                }
            }

            for (int j = 0; j < vMatOTH.size(); j++) {
                for (int k = -2; k < 3; k++) { 
                    Mat matCurOTHImage; resize(matOTHImage, matCurOTHImage, Size(IMG_SIZE * (1.0 - 0.2 * k), IMG_SIZE * (1.0 - 0.2 * k)));
                    matchTemplate(matCurOTHImage, vMatOTH[j], matResponseMap, TM_CCOEFF_NORMED);

                    ////// Check Response //////
                    //Mat matNormResponseMap;  normalize(matResponseMap, matNormResponseMap, 0, 255, NORM_MINMAX, CV_8U);
                    //imshow("response", matNormResponseMap);
                    //waitKey(0);

                    double dCurMinVal, dCurMaxVal;
                    minMaxLoc(matResponseMap, &dCurMinVal, &dCurMaxVal);
                    if (dCurMaxVal > dOTHMaxVal){
                        nOTHMaxIdx = j;
                        dOTHMaxVal = dCurMaxVal;
                    }
                }
            }

            switch (nABMaxIdx){
            case 0:
                cout << "A ";
                break;
            case 1:
                cout << "B ";
                break;
            default:
                break;
            }

            switch (nOTHMaxIdx){
            case 0:
                cout << "1 ";
                break;
            case 1:
                cout << "2 ";
                break;
            case 2:
                cout << "3 ";
                break;
            }

            cout << dABMaxVal << " " << dOTHMaxVal << endl;

            imshow("curImage", matCurImage);
            // waitKey(0);
            destroyAllWindows();
        }
        return 0;
    }
*/
    void delivery_sign(int maxIdx, Rect roi, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
    {
        string sign;
        switch(maxIdx)
        {
            case 0:
                sign = "A1";
                // cout << "A1" << endl;
                break;
            case 1:
                sign = "A2";
                // cout << "A2" << endl;
                break;
            case 2:
                sign = "A3";
                // cout << "A3" << endl;
                break;
            case 3:
                sign = "B1";
                // cout << "B1" << endl;
                break;
            case 4:
                sign = "B2";
                // cout << "B2" << endl;
                break;
            case 5:
                sign = "B3";
                // cout << "B3" << endl; 
                break;
        }

        sign_msg.header = in_objects->header;
        sign_msg.objects = in_objects->objects;
        for(auto& obj : sign_msg.objects){
            if (false == IsObjectValid(obj)) continue;
            if (obj.label == "traffic sign")
            {
                if(obj.x == roi.x && obj.y == roi.y && obj.width == roi.width && obj.height == roi.height)
                {
                    obj.label = sign;
                    break;
                }
            }
        }

        traffic_sign_pub.publish(sign_msg);
    }

    int publish_sign(cv_bridge::CvImagePtr& cv_ptr, Rect roi, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
    {
        // char chBuf[255];
        
        if(cv_ptr == nullptr) 
            return 0;
        Mat matOriginImage = cv_ptr->image;
        if(roi.x + roi.width > matOriginImage.cols || roi.y + roi.height > matOriginImage.rows)
        {
            return 0;
        }
        
        Mat matCurImage = matOriginImage(roi);
       
        // sprintf(chBuf, "/home/autoware/shared_dir/pic/trafficSign/%d.jpg", i);
        // imwrite(chBuf, matCurImage);
        // i++;
        // imshow("matCurImage",matCurImage);
        // waitKey(0);

        ////// Load Template //////
        ///Memory
        vector<Mat> vMatAB(2), vMatOTH(3);
        vMatAB[0] = imread("/home/autoware/shared_dir/pic/template/00.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatAB[1] = imread("/home/autoware/shared_dir/pic/template/01.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatOTH[0] = imread("/home/autoware/shared_dir/pic/template/02.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatOTH[1] = imread("/home/autoware/shared_dir/pic/template/03.jpg", CV_LOAD_IMAGE_GRAYSCALE);
        vMatOTH[2] = imread("/home/autoware/shared_dir/pic/template/04.jpg", CV_LOAD_IMAGE_GRAYSCALE);

        // for (int i = 0; i < 53; i++) {
        //     sprintf(chBuf, "/home/autoware/shared_dir/pic/imgs/%02d.jpg", i);
        //     cout << i << endl;
        //     string strFilename = chBuf;
        //     Mat matCurImage = imread(strFilename);

        Mat matResizeImage; resize(matCurImage, matResizeImage, Size(IMG_SIZE, IMG_SIZE));
        Mat matGrayImage; cvtColor(matResizeImage, matGrayImage, COLOR_BGR2GRAY);
        Mat matBinaryImage; threshold(matGrayImage, matBinaryImage, 128, 255, THRESH_OTSU);

        Rect bounds(0,0,matBinaryImage.cols-10,matBinaryImage.rows);
        Rect rAB(10,10,matBinaryImage.cols/2,matBinaryImage.rows-20);
        Rect rOTH(matBinaryImage.cols/2,10,matBinaryImage.cols,matBinaryImage.rows-20);
        Rect rectOTHSmallROI(60, 20, 30, 60);

        ////// Match Template //////
        uchar ucPixelVal;
		int nMinX, nMaxX;
		int nABMaxIdx, nOTHMaxIdx, nCharWidth, nMinCharWidth = INT_MAX;
		double dABMaxVal = DBL_MIN, dOTHMaxVal = DBL_MIN;
        vector<double> vAccABVal(2), vAccOTHVal(3);
        Mat matResponseMap, matRotOTHImg, matOTHSmall, matABImage, matOTHImage;
        matBinaryImage(rAB & bounds).copyTo(matABImage);
		matBinaryImage(rOTH & bounds).copyTo(matOTHImage);
		matBinaryImage(rectOTHSmallROI).copyTo(matOTHSmall);
        // imshow("matABImage",matABImage);
        // imshow("matOTHImage",matOTHImage);
        // waitKey(0);

        int nMinWidthAngle, nInitAngle = 15;
		float fOneLine;
		for (int nAngle = -nInitAngle; nAngle < nInitAngle; nAngle+=1) {
			
			Mat matTransf = getRotationMatrix2D(Point2f(matOTHSmall.cols/2, matOTHSmall.rows/2), nAngle, 1.0);
			warpAffine(matOTHSmall, matRotOTHImg, matTransf, Size(matOTHSmall.cols, matOTHSmall.rows));

			int nCheckHeight = IMG_SIZE / 2, nWidthStep = matRotOTHImg.step1();
			vector<float> vProjX(matRotOTHImg.cols);
			for (int j = 0; j < matRotOTHImg.cols; j++) {
				fOneLine = 0.;
				for (int k = 0; k < matRotOTHImg.rows; k++) {
					ucPixelVal = matRotOTHImg.data[k * nWidthStep + j];
					fOneLine += (float)ucPixelVal;
				}
				vProjX[j] = fOneLine;
			}

			//for (int j = vProjX.size() / 2; j > -1; j--) {
			//	if (vProjX[j + 1] - vProjX[j] > 0.) {
			//		nMinX = j;
			//	}
			//}


			for (int j = vProjX.size() / 2; j < vProjX.size(); j++) {
				fOneLine = vProjX[j];
				if (vProjX[j-1] - vProjX[j] > 0.) {
					nMaxX = j;
				}
			}

			//if (nMinX == nMaxX) 
			//	continue;

			//nCharWidth = nMaxX - nMinX;
			if (nMaxX <= nMinCharWidth) {
				nMinCharWidth = nMaxX;
				nMinWidthAngle = nAngle;
			}

			////cout << nCharWidth << endl;
			//Mat matColorRotImg;
			//cvtColor(matRotOTHImg, matColorRotImg, CV_GRAY2BGR);
			////line(matColorRotImg, Point(nMinX, 0), Point(nMinX, matColorRotImg.rows - 1), Scalar(255, 0, 0), 2);
			//line(matColorRotImg, Point(nMaxX, 0), Point(nMaxX, matColorRotImg.rows - 1), Scalar(0, 0, 255), 2);
			//imshow("aaa", matColorRotImg);
			//waitKey(0);
		}
		Mat matTransf = getRotationMatrix2D(Point2f(matOTHImage.cols / 2, matOTHImage.rows / 2), nMinWidthAngle, 1.0);
		warpAffine(matABImage, matABImage, matTransf, Size(matABImage.cols, matABImage.rows));
		warpAffine(matOTHImage, matOTHImage, matTransf, Size(matOTHImage.cols, matOTHImage.rows));

        for (int j = 0; j < vMatAB.size(); j++) {
            for (int k = -2; k < 3; k++) {
                Mat matCurABImage; 
                resize(matABImage, matCurABImage, Size(IMG_SIZE * (1.0 + 0.2 * k), IMG_SIZE * (1.0 + 0.2 * k)));

                matchTemplate(matCurABImage, vMatAB[j], matResponseMap, TM_CCOEFF_NORMED);

                ////// Check Response //////
                //Mat matNormResponseMap;  normalize(matResponseMap, matNormResponseMap, 0, 255, NORM_MINMAX, CV_8U);
                //imshow("response", matNormResponseMap);
                //waitKey(0);

                double dCurMinVal, dCurMaxVal;
                minMaxLoc(matResponseMap, &dCurMinVal, &dCurMaxVal);
                if(dCurMaxVal > 0.6) {
                    if (dCurMaxVal > dABMaxVal) {
                        nABMaxIdx = j;
                        dABMaxVal = dCurMaxVal;
                    }
                }

                //cout << j << " " << dCurMaxVal << endl;
            }
        }

        for (int j = 0; j < vMatOTH.size(); j++) {
            for (int k = -2; k < 3; k++) { 
                Mat matCurOTHImage;
                resize(matOTHImage, matCurOTHImage, Size(IMG_SIZE * (1.0 + 0.2 * k), IMG_SIZE * (1.0 + 0.2 * k)));
                matchTemplate(matCurOTHImage, vMatOTH[j], matResponseMap, TM_CCOEFF_NORMED);

                ////// Check Response //////
                //Mat matNormResponseMap;  normalize(matResponseMap, matNormResponseMap, 0, 255, NORM_MINMAX, CV_8U);
                //imshow("response", matNormResponseMap);
                //waitKey(0);

                double dCurMinVal, dCurMaxVal;
                minMaxLoc(matResponseMap, &dCurMinVal, &dCurMaxVal);
                if(dCurMaxVal > 0.6) {
                    if (dCurMaxVal > dOTHMaxVal){
                        nOTHMaxIdx = j;
                        dOTHMaxVal = dCurMaxVal;
                    }
                }
            }
        }

        switch (nABMaxIdx)
        {
            case 0:
                cout << "A ";
                break;
            case 1:
                cout << "B ";
                break;
            default:
                break;
        }

        switch (nOTHMaxIdx)
        {
            case 0:
                cout << "1 ";
                break;
            case 1:
                cout << "2 ";
                break;
            case 2:
                cout << "3 ";
                break;
        }

        int nCount[6] = {0};
        if(nABMaxIdx == 0)
        {
            if(nOTHMaxIdx == 0)
                nCount[0] += 1;
            else if(nOTHMaxIdx == 1)
                nCount[1] += 1;
            else if(nOTHMaxIdx == 2)
                nCount[2] += 1;
        } 
        else 
        {
            if(nOTHMaxIdx == 0)
                nCount[3] += 1;
            else if(nOTHMaxIdx == 1)
                nCount[4] += 1;
            else if(nOTHMaxIdx == 2)
                nCount[5] += 1;
        }

        int maxIdx, maxVal = 0;
        for(int i=0; i < 6; i++)
        {
            if(maxVal < nCount[i])
            {
                maxVal = nCount[i];
                maxIdx = i;
            }
        }

        delivery_sign(maxIdx, roi, in_objects);

        cout << dABMaxVal << " " << dOTHMaxVal << endl;

        // imshow("curImage", matCurImage);
        // waitKey(0);
        destroyAllWindows();
        //}
        return 0;
    }

    bool IsObjectValid(const autoware_msgs::DetectedObject &in_object)
    {
        if (!in_object.valid ||
            in_object.width <= 0 ||
            in_object.height <= 0 ||
            in_object.x <= 0 ||
            in_object.y <= 0)
            {
                return false;
            }
        return true;
    }

    std::pair<Rect, bool> findROI(cv_bridge::CvImagePtr& cv_ptr,const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
    {
        Rect roi;
              
        if (0 == in_objects->objects.size()) return make_pair(roi, false);
        
        //extract traffic sign roi
        std::vector<autoware_msgs::DetectedObject> trafficSignCandidateVec;
        for(const auto& obj : in_objects->objects){
            if (false == IsObjectValid(obj)) continue;  //invalid roi
            if (obj.label != "traffic sign") continue;  //not traffic sign
            trafficSignCandidateVec.push_back(obj);
        }
        if(trafficSignCandidateVec.size() == 0) return make_pair(roi, false);

        int max = 0;
        for(const auto& trafficSignObj: trafficSignCandidateVec){
            int max_area = trafficSignObj.width * trafficSignObj.width;

            if (max_area > max){
                max = max_area;
                // roi.x = trafficSignObj.x+5;
                // roi.y = trafficSignObj.y+5;
                // roi.width = trafficSignObj.width-7;
                // roi.height = trafficSignObj.height-7;
                roi.x = trafficSignObj.x;
                roi.y = trafficSignObj.y;
                roi.width = trafficSignObj.width;
                roi.height = trafficSignObj.height;
                if(debug)
                    cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;
            }
        }

        return make_pair(roi, true);
    }

    void SyncedDetectionsCallback(
        const sensor_msgs::Image::ConstPtr &in_image_msg,
        const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
    {
        //convert ros img to cv img
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
            // if(debug){
            //     Mat img = cv_ptr->image;
            //     imshow("ros->cv",img);
            //     waitKey(0);
            // }
            // cout << "1" << endl;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // if(debug){
        //     cout << "DetectedObjectArray size:" << in_objects->objects.size() << endl;
        //     cout << "section:" << section << endl;
        //     for(int i=0; i < in_objects->objects.size(); i++)
        //         cout << in_objects->objects[i] << " ";
        //     cout << endl;
        // }
        auto roi_pair = findROI(cv_ptr,in_objects);
        /*if(debug){
            Mat img = cv_ptr->image;
            Mat img_roi;
            img.copyTo(img_roi);
            cout << roi_pair.first.width << endl;
            rectangle(img_roi, Rect(roi_pair.first.x, roi_pair.first.y, roi_pair.first.width, roi_pair.first.height), Scalar(0,0,255), 1, 8, 0);
            imshow("img_roi",img_roi);
            waitKey(0);
        }*/

        if (false == roi_pair.second) return;
        else publish_sign(cv_ptr, roi_pair.first, in_objects);
    }

    SignDetector()
    {
        ros::NodeHandle nh;
    
        //get namespace from topic
        // image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "usb_cam/image_raw", 1);
        image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "camera/image_raw", 1);
        detection_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
            (nh,"/detection/image_detector/objects",1);

        detections_synchronizer_ =  new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(1000),
                                                   *image_filter_subscriber_,
                                                   *detection_filter_subscriber_);
        detections_synchronizer_->registerCallback(
            boost::bind(&SignDetector::SyncedDetectionsCallback, this, _1, _2));
        
        nh.param<bool>("sm_sign_detection/debug", debug, false);
        ROS_INFO("debug : %d", debug);
        traffic_sign_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/image_detector/traffic_sign", 1);
    }

private:
    // ros::Subscriber image_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

    message_filters::Synchronizer<SyncPolicyT> *detections_synchronizer_;

    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detection_filter_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *image_filter_subscriber_;

    bool debug;
    autoware_msgs::DetectedObjectArray sign_msg;
    ros::Publisher traffic_sign_pub;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sm_sign_detection");
    SignDetector d;
    ros::spin();
}