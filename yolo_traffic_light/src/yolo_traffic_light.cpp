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
#include <traffic_light_msgs/Light.h>
#include <std_msgs/Int32.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <traffic_area_msgs/traffic_area.h>

#include <fstream>
#include <string>

using namespace std;
using namespace cv;
int i = 0;
int count1 = 0;

class LightDetector{
public:
    void setTightROI(bool bTightROI) { m_bTightROI = bTightROI; }
    void publish_color(int light)
    {
        cout << "publish_light : " << light << endl;
        light_msg.header.stamp = ros::Time::now(); 
        
        switch (light)
        {
        case 1:
            cout << "3_RED" << endl;
            light_msg.light = light_msg.RED;
            break;
        case 2:
            cout << "3_YELLOW" << endl;
            light_msg.light = light_msg.YELLOW;
            break;
        case 3:
            cout << "3_GREEN" << endl;
            light_msg.light = light_msg.STRAIGHT;            
            break;
        case 4:
            cout << "4_RED" << endl;
            light_msg.light = light_msg.RED;
            break;
        case 5:
            cout << "4_YELLOW" << endl;
            light_msg.light = light_msg.YELLOW;
            break;
        case 6:
            cout << "4_GREEN" << endl;
            light_msg.light = light_msg.STRAIGHT;
            break;
        case 7:
            cout << "4_LEFT&RED" << endl;
            light_msg.light = light_msg.LEFT;
            break;
        case 8:
            cout << "4_LEFT&GREEN" << endl;
            light_msg.light = light_msg.STRAIGHT_AND_LEFT;
            break;
        default:
            count1++;
            ROS_ERROR("color light dafault");
            break;
        }
        traffic_light_pub.publish(light_msg);
        cout << count1 << endl;
    }

    int confirm_light(string color)
    { 
        cout << "light : " << color << endl;
        int light = 0;

        if(color == "FAILED")
            light = 0;
        else if(color == "3_RED")
            light = 1;
        else if(color == "3_RED&YELLOW")
            light = 2;
        else if(color == "3_YELLOW&GREEN")
            light = 2;
        else if(color == "3_GREEN")
            light = 3;
        else if(color == "4_RED")
            light = 4;
        else if(color == "4_RED&YELLOW")
            light = 5;
        else if(color == "4_YELLOW")
            light = 5;
        else if(color == "4_GREEN")
            light = 6;
        else if(color == "4_READ&LEFT")
            light = 7;
        else if(color == "4_GREEN&LEFT")
            light = 8; 

        return light;
    }

    string findColor(cv_bridge::CvImagePtr& cv_ptr, Rect rectRoi)
    {
        if(cv_ptr == nullptr) 
            return 0;
        Mat matSrcImg = cv_ptr->image;

        if(rectRoi.x + rectRoi.width > matSrcImg.cols || rectRoi.y + rectRoi.height > matSrcImg.rows)
        {
            return 0;
        }

        Mat matROIImg;
        Mat matExpROIImg_test;

		if (m_bTightROI) {
			//////Expanding ROI
			double dExpWHalfRatio = 0.05, dExpHHalfRatio = 0.1;
			int nExpHalfWidth = round(dExpWHalfRatio * rectRoi.width), nExpHalfHeight = round(dExpHHalfRatio * rectRoi.height);
			//rectRoi.x -= nExpHalfWidth; rectRoi.y += nExpHalfHeight; rectRoi.width += 2 * nExpHalfWidth; rectRoi.height += 2 * nExpHalfHeight;
			rectRoi.x -= nExpHalfWidth; rectRoi.y += nExpHalfHeight; rectRoi.width += 2 * nExpHalfWidth;
			if (rectRoi.x < 0) rectRoi.x = 0;
			if (rectRoi.y < 0) rectRoi.y = 0;
			if (rectRoi.x + rectRoi.width > matSrcImg.cols) rectRoi.width = matSrcImg.cols - rectRoi.x;
			if (rectRoi.y + rectRoi.height > matSrcImg.rows) rectRoi.height = matSrcImg.rows - rectRoi.y;

            ///////Finding Dark Region
			Mat matExpROIImg;
			vector<Mat> vMatGR;
			matSrcImg(rectRoi).copyTo(matExpROIImg);
            matSrcImg(rectRoi).copyTo(matExpROIImg_test);
			split(matExpROIImg, vMatGR);
			vMatGR.erase(vMatGR.begin());


			for (int i = 0; i < 2; i++)
				threshold(vMatGR[i], vMatGR[i], 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

			Mat matThreImg, matTemp;
			vMatGR[0].convertTo(matThreImg, CV_32F);
			vMatGR[1].convertTo(matTemp, CV_32F);
			Mat(matThreImg + matTemp).convertTo(matThreImg, CV_8U, 0.5);


			Mat matContourImg; matThreImg.copyTo(matContourImg);
			vector<Vec4i> vHierarchy;
			vector<vector<Point>> vContours;
			findContours(matContourImg, vContours, vHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

			///////Finding Angle
			double dMinDist = DBL_MAX;
			RotatedRect rrectROI(Point2f(-1., -1.), Size2f(-1., -1.), -1);
			for (int i = 0; i < vContours.size(); i++) {
				if ((int)vContours[i].size() < 5) continue;
				RotatedRect rrectOneROI = fitEllipse(vContours[i]);
				if (rrectOneROI.center.x < 0) continue;

				double dDiffX = matThreImg.cols / 2 - rrectOneROI.center.x, dDiffY = matThreImg.rows / 2 - rrectOneROI.center.y;
				double dDist = dDiffX * dDiffX + dDiffY * dDiffY;

				if (dMinDist > dDist) {
					dMinDist = dDist;
					rrectROI = rrectOneROI;
				}
			}

			///////Rotating ROI
			double dThresAngle = 5.0, dThresWHRatio = 1.5;
			if (rrectROI.center.x > 0.0 &&
				abs(rrectROI.angle - 90.0) < dThresAngle &&
				rrectROI.size.height / rrectROI.size.width > dThresWHRatio) {
				Mat matTransf = getRotationMatrix2D(rrectROI.center, rrectROI.angle - 90, 1.0);
				warpAffine(matExpROIImg, matExpROIImg, matTransf, Size(matExpROIImg.cols, matExpROIImg.rows));
				warpAffine(matThreImg, matThreImg, matTransf, Size(matThreImg.cols, matThreImg.rows));
			}

			//////Projecting to X-Y Axis
			uchar ucPixelVal;
			int nWidthStep = matThreImg.step1();
			int nROISize = matThreImg.cols * matThreImg.rows;
			float fMinVal = 9999, fMaxVal = 0;
			float fOneLine;
			vector<float> vProjX(matThreImg.cols), vProjY(matThreImg.rows);
			for (int i = 0; i < matThreImg.cols; i++) {
				fOneLine = 0.;
				for (int j = 0; j < matThreImg.rows; j++) {
					ucPixelVal = matThreImg.data[j * nWidthStep + i];
					vProjY[j] += (float)ucPixelVal;
					fOneLine += (float)ucPixelVal;
				}
				vProjX[i] = fOneLine / nROISize;
			}
			for (int i = 0; i < vProjY.size(); i++) vProjY[i] /= nROISize;

			//////Finding Tight ROI
			float fThreX = 1.5, fThreY = 1.0;
			Point ptLT(matThreImg.cols, matThreImg.rows), ptRB(-1, -1);
			for (int i = vProjX.size() / 2; i > -1; i--) {
				if (vProjX[i] > fThreX) ptLT.x = i;
			}
			for (int i = vProjY.size() / 2; i > -1; i--) {
				if (vProjY[i] > fThreY) ptLT.y = i;
				else break;
			}
			for (int i = vProjX.size() / 2; i < vProjX.size(); i++) {
				if (vProjX[i] > fThreX) ptRB.x = i;
			}
			for (int i = vProjY.size() / 2; i < vProjY.size(); i++) {
				if (vProjY[i] > fThreY) ptRB.y = i;
				else break;
			}

			if(ptRB.x > 0 && ptRB.y > 0 &&
				ptLT.x < matExpROIImg.cols && ptLT.y < matExpROIImg.rows)
				matExpROIImg(Rect(ptLT, ptRB)).copyTo(matROIImg);
			else {
				if (rectRoi.x < 0) rectRoi.x = 0;
				if (rectRoi.y < 0) rectRoi.y = 0;
				if (rectRoi.x + rectRoi.width > matSrcImg.cols) rectRoi.width = matSrcImg.cols - rectRoi.x;
				if (rectRoi.y + rectRoi.height > matSrcImg.rows) rectRoi.height = matSrcImg.rows - rectRoi.y;
				matSrcImg(rectRoi).copyTo(matROIImg);
			}
		}
        else {
			if (rectRoi.x < 0) rectRoi.x = 0;
			if (rectRoi.y < 0) rectRoi.y = 0;
			if (rectRoi.x + rectRoi.width > matSrcImg.cols) rectRoi.width = matSrcImg.cols - rectRoi.x;
			if (rectRoi.y + rectRoi.height > matSrcImg.rows) rectRoi.height = matSrcImg.rows - rectRoi.y;
			matSrcImg(rectRoi).copyTo(matROIImg);
		}
   
        Size sizeROI(300, 100);
		Mat matResizedROI; resize(matROIImg, matResizedROI, sizeROI);

        int nCurHoles = m_nhole; if (m_nhole < 0) nCurHoles = 3;
		int nCenterROIHalfSize = 20, nCenterROIWSize = nCenterROIHalfSize * 2.5,  nCenterROIHSize = nCenterROIHalfSize * 2;
		double dValB, dValG, dValR, dValGRSum;
		double dColorGRRatio, dColorGBRatio , dColorRatioThres = 0.40, dColorValThres = 80.0, dColorSumThres = 70.0;
		vector<int> vLightGR(nCurHoles); // 0: Black, 1: Green, 2:Red
		Mat matBlue, matGreen, matRed;
		vector<Mat> vMatBGR;
		split(matResizedROI, vMatBGR);

		for (int i = 0; i < nCurHoles; i++) {
			Rect rectCenter( sizeROI.width * (i + 0.5) / nCurHoles - nCenterROIHalfSize, sizeROI.height / 2 - nCenterROIHalfSize, nCenterROIWSize, nCenterROIHSize);

			matBlue = vMatBGR[0](rectCenter);
			matGreen = vMatBGR[1](rectCenter);
			matRed = vMatBGR[2](rectCenter);

			dValB = mean(matBlue)[0];
			dValG = mean(matGreen)[0];
			dValR = mean(matRed)[0];
			dValGRSum = dValG + dValR;
			//rectangle(matResizedROI, rectCenter, Scalar(0, 0, 255), 2);

			dColorGRRatio = abs(1.0 - dValG / dValR);
			dColorGBRatio = abs(1.0 - dValG / dValB);
			if (dValGRSum > dColorSumThres) {
				if (dColorGRRatio > dColorRatioThres) {
					if (dValG > dValR) vLightGR[i] = 1;
					else vLightGR[i] = 2;
				} 
				else if (dValG > dColorValThres && dValB < dColorValThres)
					vLightGR[i] = 2;
				else
					vLightGR[i] = 0;
			}
			else
				vLightGR[i] = 0;
		}


        ///////for K-CITY Signal Light
		///////////// 200: Red, 220: Red&Yellow, 020: Yellow, 021: Yellow&Green, 001: Green
		///////////// 2000: Red, 2200: Red&Yellow, 0200: Yellow, 2010: Red&Left, 0011: Green&Left, 0001: Green
		bool bLight = false;
		string strText;
		if (m_nhole == 3) {
			strText = "3_";
			if (vLightGR[0] > 0) {
				strText += "RED";
				bLight = true;
				if (vLightGR[2] > 0) 
                {
                    strText = "FAILED";
                    char chBuf[255];
                    // Mat matCurImage = img(roi);
                    sprintf(chBuf, "/home/autoware/shared_dir/pic/light_img/%d.jpg", i);
                    imwrite(chBuf, matExpROIImg_test);
                    i++;
                }
			}
			else if (vLightGR[2] > 0) {
				strText += "GREEN";
				bLight = true;
			}

			if (vLightGR[1] > 0) {
				if (bLight)
					strText += "&";
				strText += "YELLOW";
				bLight = true;
			}
			
			if (!bLight)
            {
                strText = "FAILED";
                char chBuf[255];
                // Mat matCurImage = img(roi);
                sprintf(chBuf, "/home/autoware/shared_dir/pic/light_img/%d.jpg", i);
                imwrite(chBuf, matExpROIImg_test);
                i++;
            }   
                
		}
		else
		{
			strText = "4_";
			if (vLightGR[0] > 0) {
				strText += "RED";
				bLight = true;
				if (vLightGR[3] > 0)
				{
                    strText = "FAILED";
                    char chBuf[255];
                    // Mat matCurImage = img(roi);
                    sprintf(chBuf, "/home/autoware/shared_dir/pic/light_img/%d.jpg", i);
                    imwrite(chBuf, matExpROIImg_test);
                i++;
                }
			}
			else if (vLightGR[3] > 0) {
				strText += "GREEN";
				bLight = true;
			}
			
			if (vLightGR[1] > 0) {
				if (bLight)
					strText += "&";
				strText += "YELLOW";
				bLight = true;
				if (vLightGR[2] > 0)
				{
                    strText = "FAILED";
                    char chBuf[255];
                    // Mat matCurImage = img(roi);
                    sprintf(chBuf, "/home/autoware/shared_dir/pic/light_img/%d.jpg", i);
                    imwrite(chBuf, matExpROIImg_test);
                    i++;
                }
			}
			else if (vLightGR[2] > 0) {
				if (bLight)
					strText += "&";
				strText += "LEFT";
				bLight = true;
			}

			if (!bLight)
				{
                    strText = "FAILED";
                    char chBuf[255];
                    // Mat matCurImage = img(roi);
                    sprintf(chBuf, "/home/autoware/shared_dir/pic/light_img/%d.jpg", i);
                    imwrite(chBuf, matExpROIImg_test);
                    i++;
                }

		}
        cout << "6" << endl;
        // cout << "light : " << strText << endl;

        if (debug) {
			for (int i = 0; i < vLightGR.size(); i++)
				cout << vLightGR[i] << " ";
			cout << endl;

			putText(matResizedROI, strText, Point(20, matResizedROI.rows - 50), cv::FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 255, 0), 2);
			imshow("ROI", matROIImg);
			imshow("resizeROI", matResizedROI);
		}
        return strText;
    }

    //sm modify
    bool IsObjectValid(const autoware_msgs::DetectedObject &in_object){
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

    std::pair<Rect, bool> findROI(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects, int section){
        Rect roi;
              
        if (0 == in_objects->objects.size()) return make_pair(roi, false);
        
        //extract traffic light roi
        std::vector<autoware_msgs::DetectedObject> trafficLightCandidateVec;
        for(const auto& obj : in_objects->objects){
            if (false == IsObjectValid(obj)) continue;  //invalid roi
            if (obj.label != "traffic light") continue; //not traffic light
            if (obj.width < 1.5 * obj.height) continue; //vertical traffic light
            trafficLightCandidateVec.push_back(obj);
        }
        if(trafficLightCandidateVec.size() == 0) return make_pair(roi, false);

        //sm modify
        int maxW = 0;
        for(const auto& trafficLightObj: trafficLightCandidateVec){
            int max_width = trafficLightObj.width;
            if(debug)
                cout << "max_width:" << max_width << endl;

            // if (max_cur > maxC && max_width > maxW){
            if (max_width > maxW && trafficLightObj.x > 99){ //sm
                maxW = max_width;
                roi.x = trafficLightObj.x;
                roi.y = trafficLightObj.y;
                roi.width = trafficLightObj.width;
                roi.height = trafficLightObj.height;
                if(debug)
                    cout << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;
            }
        }
        //sm
        if(roi.height < 10)
            return make_pair(roi, false);
        else
            return make_pair(roi, true);
        if(debug)
            cout << "final:" << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << endl;

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
            if(cv_ptr == nullptr) 
                return;
            // if(debug){
            //     Mat img = cv_ptr->image;
            //     imshow("ros->cv",img);
            //     waitKey(0);
            // }
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
        auto roi_pair = findROI(in_objects, section);

        /*if(debug){
            Mat img = cv_ptr->image;
            Mat img_roi;
            img.copyTo(img_roi);
            cout << roi_pair.first.width << endl;
            rectangle(img_roi, Rect(roi_pair.first.x, roi_pair.first.y, roi_pair.first.width, roi_pair.first.height), Scalar(0,0,255), 1, 8, 0);
            imshow("img_roi",img_roi);
            waitKey(0);
        }*/

        string str_light;
        if (false == roi_pair.second) 
            return;
        else {
            setTightROI(true);
            str_light = findColor(cv_ptr, roi_pair.first);
        }

        int int_light;
        int_light = confirm_light(str_light);

        publish_color(int_light);
    }

    LightDetector() : m_nhole(4){
        ros::NodeHandle nh;

        //get namespace from topic
        //image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "usb_cam/image_raw", 1);
        image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "camera/image_raw", 1);
        detection_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
            (nh,"/detection/image_detector/objects",1);

        detections_synchronizer_ =  new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(1000),
                                                   *image_filter_subscriber_,
                                                   *detection_filter_subscriber_);
        detections_synchronizer_->registerCallback(
            boost::bind(&LightDetector::SyncedDetectionsCallback, this, _1, _2));
        
        nh.param<bool>("sm_test_yolo/debug", debug, false);
        ROS_INFO("debug : %d", debug);
        traffic_light_pub = nh.advertise<traffic_light_msgs::Light>("traffic_light_erp42", 10);
        traffic_area_sub = nh.subscribe("traffic_area_info", 10, &LightDetector::trafficAreaCB, this);
    }

    void trafficAreaCB(const traffic_area_msgs::traffic_areaConstPtr& ptr){
        m_nhole = ptr->n_hole;
        section = ptr->section;
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

    message_filters::Synchronizer<SyncPolicyT> *detections_synchronizer_;

    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detection_filter_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *image_filter_subscriber_;

    bool debug, m_bTightROI = false;
    traffic_light_msgs::Light light_msg;
    ros::Publisher traffic_light_pub;
    ros::Subscriber traffic_area_sub;
    int m_nhole, section = 1;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "yolo_traffic_light");
    LightDetector d;
    ros::spin();

}
