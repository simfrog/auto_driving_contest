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

class LightDetector{
public:
    void color_light(int light, string& light_c)
    {
        light_msg.header.stamp = ros::Time::now(); 
        
        switch (light)
        {
        case 1:
            cout << "3_red" << endl;
            light_c = "3_RED";
            light_msg.light = light_msg.RED;
            break;
        case 2:
        cout << "3_yellow" << endl;
            light_c = "3_YELLOW";
            light_msg.light = light_msg.YELLOW;
            break;
        case 3:
        cout << "3_green" << endl;
            light_c = "3_GREEN";
            light_msg.light = light_msg.STRAIGHT;            
            break;
        case 4:
        cout << "4_red" << endl;
            light_c = "4_RED";
            light_msg.light = light_msg.RED;
            break;
        case 5:
        cout << "4_yellow" << endl;
            light_c = "4_YELLOW";
            light_msg.light = light_msg.YELLOW;
            break;
        case 6:
        cout << "4_green" << endl;
            light_c = "4_GREEN";
            light_msg.light = light_msg.STRAIGHT;
            break;
        case 7:
        cout << "4_LEFT&RED" << endl;
            light_c = "4_LEFT&RED";
            light_msg.light = light_msg.LEFT;
            break;
        case 8:
        cout << "4_LEFT&GREEN" << endl;
            light_c = "4_LEFT&GREEN";
            light_msg.light = light_msg.STRAIGHT_AND_LEFT;
            break;
        default:
            ROS_ERROR("color light dafault");
            break;
        }
        traffic_light_pub.publish(light_msg);
    }

    int confirm_light(Mat src, int tlightnum, int max_node)
    { 
        int light = 0;

        if (tlightnum == 3) //3hole
        {
            if (max_node == 0)
            {
                light = 1;
            }else if (max_node == 1)
            {
                light = 2;
            }else if (max_node == 2)
            {
                light = 3;
            }
        }
        else //4hole
        {
            if (max_node == 0)
            {
                light = 4;
            }else if (max_node == 1)
            {
                light = 5;
            }
            else if (max_node == 3)
            {
                light = 6;
            }
            else if (max_node == 4)
            {
                light = 7;
            }else if (max_node == 5)
		    {
		    	light = 8;
		    }
        }

        return light;
    }

    int max_node(int num[4])
    {
        int max = -1;
        int max_n = 0;
        for (int i = 0; i < 4; i++)
        {
            if (num[i] > max)
            {
                max = num[i];
                max_n = i;
            }
        }  
        // if(num[max_n] < thresh)
        // {
        //     max_n = -1;
        // }
        return max_n;
    }

    void projection(Mat roi_img, Mat morphology_img, Mat thresh_img, Rect &traffic, bool &traffic_rect_flg)
    {
        Mat find = Mat::zeros(Size(morphology_img.size()), CV_8UC1);
        Mat proj_y = Mat::zeros(Size(morphology_img.size()), CV_8UC1);

        int perPixelValue;

        int sta_pixelx = 0;
        int fin_pixelx = 0;

        int sta_pixely = 0;
        int fin_pixely = 0;

        int width = morphology_img.cols;
        int height = morphology_img.rows;

        int* projectValArry = new int[width];
        memset(projectValArry, 0, width * 4);

        for (int col = 0; col < width; col++)
        {
            for (int row = 0; row < height; row++)
            {
                perPixelValue = morphology_img.at<uchar>(row, col);
                if (perPixelValue == 255) {
                    projectValArry[col]++;
                }
            }
            if (projectValArry[col] < morphology_img.rows / 3.5)
            {
                projectValArry[col] = 0;
            }
        } 
        projectValArry[width-1] = 0;

        Mat verticalProjectionMat(height, width, CV_8U, Scalar(0));
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                perPixelValue = 0;
                verticalProjectionMat.at<uchar>(i, j) = perPixelValue;
            }
        }

        if(debug){
            imshow("verticalProjectionMat1", verticalProjectionMat);
        }

        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < projectValArry[i]; j++)
            {
                perPixelValue = 255;
                verticalProjectionMat.at<uchar>(j, i) = perPixelValue;
            }
        }

        if(debug){
            imshow("roi",roi_img);
            imshow("morhopolgy",morphology_img);
            imshow("verticalProjectionMat2", verticalProjectionMat);
        }


        bool flgg = false;
        bool retu = true;

        int tmp_sta_pixelx = -1;
        int tmp_fin_pixelx = -1;

        int max1 = -1;
        int max_n1 = -1;
        for (int i = 0; i < (verticalProjectionMat.cols / 4); i++)
        {
            if (projectValArry[i] > max1)
            {
                max1 = projectValArry[i];
                max_n1 = i;
            }
        }
        
        int temp_start1 = 0;
        for (int j = 1; j < morphology_img.cols - 1; j++)
        {
            int pixel = projectValArry[j];
            int prev_pixel = projectValArry[j - 1];
            int next_pixel = projectValArry[j + 1];
         
            if(j<morphology_img.cols-3)
            {
                int next_pixel1 = projectValArry[j + 2];

                if (prev_pixel == 0 && pixel != 0 && next_pixel != 0 && next_pixel1 != 0) {
                    if (retu == true)
                    {
                        temp_start1 = j;
                        
                        while ((projectValArry[temp_start1 + 1] > projectValArry[temp_start1]) && 
                            (projectValArry[temp_start1 + 1] - projectValArry[temp_start1] > 0))
                        {
                            temp_start1++;
                            
                        }
                        tmp_sta_pixelx = temp_start1;
                    }
                }
            }
           
            if (projectValArry[0] != 0 || projectValArry[1] != 0 || projectValArry[2] != 0)
            {
                tmp_sta_pixelx = max_n1;
            }

            if (tmp_sta_pixelx != -1 && pixel != 0 && next_pixel == 0)
            {
                tmp_fin_pixelx = j; 
                retu = false;
                flgg = true;
            }
        }	

        if (flgg == true)
        {
            if (tmp_fin_pixelx - tmp_sta_pixelx > 20)
            {
                sta_pixelx = tmp_sta_pixelx;
                fin_pixelx = tmp_fin_pixelx;
            }
        }

        Mat fin_y = Mat::zeros(Size(morphology_img.size()), CV_8UC1);
        if (sta_pixelx >= 0 && sta_pixelx < morphology_img.cols && fin_pixelx > sta_pixelx && fin_pixelx < morphology_img.cols)
        {
        }
        else {
            sta_pixelx = 0;
            fin_pixelx = morphology_img.cols;
        }

        Rect roi(sta_pixelx, 0, (fin_pixelx - sta_pixelx), morphology_img.rows);
        fin_y = thresh_img(roi);

        ////yÃà Åõ¿µ
        int* projectHorArry = new int[fin_y.rows];
        memset(projectHorArry, 0, fin_y.rows * 4);

        for (int row = 0; row < fin_y.rows; row++)
        {
            for (int col = 0; col < fin_y.cols; col++)
            {
                perPixelValue = fin_y.at<uchar>(row, col);
                if (perPixelValue == 255) {
                    projectHorArry[row]++;
                }
            }
            if (projectHorArry[row] < fin_y.cols / 3)
            {
                projectHorArry[row] = 0;
            }
        }
        projectHorArry[0] = 0;


        Mat horizonProjectionMat(fin_y.rows, fin_y.cols, CV_8U, Scalar(0));
        for (int i = 0; i < fin_y.rows; i++)
        {
            for (int j = 0; j < fin_y.cols; j++)
            {
                perPixelValue = 0;
                horizonProjectionMat.at<uchar>(i, j) = perPixelValue;
            }
        }

        if(debug){
            imshow("horizionProjectionMat1-1",horizonProjectionMat);
        }

        for (int i = 0; i < fin_y.rows; i++)
        {
            for (int j = 0; j < projectHorArry[i]; j++)
            {
                perPixelValue = 255;
                horizonProjectionMat.at<uchar>(i, j) = perPixelValue;
            }
        }

        if(debug){
            imshow("thresh",thresh_img);
            imshow("fin_y",fin_y);
            imshow("horizonProjectionMat1-2", horizonProjectionMat);
            //waitKey(0);
        }

        int tmp_sta_pixely = -1;
        int tmp_fin_pixely = -1;

        bool retu_y = true;
        bool flg_y = false;
        int temp_start = 0;

        vector <Point2i> tmp_pixely;

        int max = -1;
        int max_n = -1;
        for (int i = 0; i < (fin_y.rows/2); i++)
        {
            if (projectHorArry[i] > max)
            {
                max = projectHorArry[i];
                max_n = i;
            }
        }
        
        for (int i = 1; i < fin_y.rows - 3; i++)
        {
            int pixel = projectHorArry[i];
            int prev_pixel = projectHorArry[i - 1];
            int next_pixel = projectHorArry[i + 1];
            
            if (prev_pixel == 0 && pixel != 0) {

                temp_start = i;
                while ((projectHorArry[temp_start + 1] > projectHorArry[temp_start]) && (projectHorArry[temp_start + 1] - projectHorArry[temp_start] > 0))
                {
                    temp_start++;
                }
                tmp_sta_pixely = temp_start;
            }
            if (i == fin_y.rows / 3) {
                if (tmp_sta_pixely == -1) {
                    tmp_sta_pixely = max_n;
                }
            }

            if (tmp_sta_pixely != -1 && pixel != 0 && next_pixel == 0) {
                tmp_fin_pixely = i;

                tmp_pixely.push_back(Point2i(tmp_sta_pixely, tmp_fin_pixely)); 
            }
        }

        for (int q = 0; q < tmp_pixely.size(); q++)
        {
            if (abs(tmp_pixely[q].x - tmp_pixely[q].y) > 10)
            {
                sta_pixely = tmp_sta_pixely;
                fin_pixely = tmp_fin_pixely;
            }
        }

        tmp_pixely.clear();

        Mat fin_x = Mat::zeros(Size(horizonProjectionMat.size()), CV_8UC1);

        if (sta_pixely >= 0 && sta_pixely < fin_y.rows && fin_pixely > sta_pixely && fin_pixely < fin_y.rows)
        {
        }
        else {
            sta_pixely = 0;
            fin_pixely = fin_y.rows;
        }
        Rect roi2(0, sta_pixely, fin_y.cols, (fin_pixely - sta_pixely));
        fin_x = fin_y(roi2);

        if(debug){
            imshow("fin_x",fin_x);
            //waitKey(0);
        }


        Rect traffic_temp;
        bool traffic_temp_flg = false;
        if (fin_x.cols >= fin_x.rows*1.5)
        {
            traffic_temp.x = sta_pixelx;
            traffic_temp.y = sta_pixely;
            traffic_temp.width = fin_pixelx - sta_pixelx;
            traffic_temp.height = fin_pixely - sta_pixely;
            traffic_temp_flg = true;
        }
        else {
            traffic_temp.x = 0;
            traffic_temp.y = 0;
            traffic_temp.width = 0;
            traffic_temp.height = 0;
            traffic_temp_flg = false;
        }

        if(traffic_temp_flg == true)
        {
            Mat traffic_temp_img = roi_img(traffic_temp);
            Mat traffic_temp_gray = Mat::zeros(Size(traffic_temp_img.size()), CV_8UC1);
            cvtColor(traffic_temp_img, traffic_temp_gray, CV_BGR2GRAY);
            threshold(traffic_temp_gray, traffic_temp_gray, 0, 255, THRESH_BINARY | THRESH_OTSU);
            dilate(traffic_temp_gray, traffic_temp_gray, cv::Mat());

            if(debug){
                imshow("traffic_temp_img",traffic_temp_img);
                imshow("traffic_temp_gray", traffic_temp_gray);
                //waitKey(0);
            }

            traffic.x = traffic_temp.x;
            traffic.y = traffic_temp.y;
            traffic.width = traffic_temp.width;
            traffic.height = traffic_temp.height;
          
            traffic_rect_flg = true;
        }
        else
        {
            traffic_rect_flg = false;
        }
        delete projectValArry;
        delete projectHorArry;
    }

    int publish_color(cv_bridge::CvImagePtr& cv_ptr, Rect roi)
    {
        if(cv_ptr == nullptr) 
            return 0;
        Mat img = cv_ptr->image;

        // if(debug){
        //     imshow("img",img);
        //     imshow("awb_img",awb_img);
        //     waitKey(0);
        // }  

        char out_line[256];
        char buf[255];
        sprintf(out_line, "%d %d %d %d", roi.x, roi.y, roi.width, roi.height);
        sprintf(buf, "/home/autoware/shared_dir/pic/light_txt/%d.txt", i); 
        ofstream writeFile;
        writeFile.open(buf);
        writeFile.write(out_line,255);
        writeFile.close();

        char chBuf[255];
        // Mat matCurImage = img(roi);
        sprintf(chBuf, "/home/autoware/shared_dir/pic/light_img/%d.jpg", i);
        imwrite(chBuf, img);
        i++;

        int x_padding = 7, y_padding = -3, w_padding =7, h_padding = -5;
        if(roi.x > x_padding && roi.y > y_padding)
        {
            roi.x = roi.x;// - x_padding;  //x-20
            roi.y = roi.y;// - y_padding;  //y-20
            roi.width = roi.width;// + w_padding;  //width+50
            roi.height = roi.height;// + h_padding;  //height+30
        }

        Mat roi_img;
     
        if (roi.x < 0) roi.x = 0;
        if (roi.y < 0) roi.y = 0;
        if (roi.x + roi.width > img.cols) roi.width = img.cols - roi.x;
        if (roi.y + roi.height > img.rows) roi.height = img.rows - roi.y;
        roi_img = img(roi);

        // if(debug){
        //     imshow("img",img);
        //     imshow("awb_img",awb_img);
        //     imshow("padding",roi_img);
        //     waitKey(0);
        // }
        
        Mat thresh_img;
        cvtColor(roi_img, thresh_img, CV_BGR2GRAY);
        threshold(thresh_img, thresh_img, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

        // if(debug){
        //     imshow("thresh_img", thresh_img);
        //     waitKey(0);
        // }
   
        Mat morphology_img;
        morphology_img = thresh_img.clone();
        cv::Mat element5(10, 10, CV_8U, cv::Scalar(1));
        cv::morphologyEx(morphology_img, morphology_img, MORPH_CLOSE, element5);
        
        // if(debug){
        //     imshow("padding",roi_img);
        //     imshow("morphology_img", morphology_img);
        //     imshow("thresh_img", thresh_img);
        //     waitKey(0);
        // }

        bool traffic_rect_flg = false;
        Rect traffic_temp;
        projection(roi_img, morphology_img, thresh_img, traffic_temp, traffic_rect_flg);
        
        morphology_img.release(); //tw
        if (traffic_rect_flg == false)
        {
            ROS_ERROR("traffic_rect_flg failed");
            return 0;
        }

        Mat traffic_img = roi_img(traffic_temp);
        // if(debug)
        //     cout << "img_size:" << traffic_temp.width << "*" << traffic_temp.height << "=" << traffic_temp.width * traffic_temp.height << endl;

        Mat draw = roi_img.clone();
        Mat gray = Mat::zeros(Size(traffic_img.size()), CV_8UC1);
        cvtColor(traffic_img, gray, CV_BGR2GRAY);
        threshold(gray, gray, 0, 255, THRESH_BINARY | THRESH_OTSU);
        dilate(gray, gray, cv::Mat());

        // if(debug){
        //     namedWindow("gray",0);
        //     imshow("gray", gray);
        //     waitKey(0);
        // }

        Mat gcolor = Mat::zeros(Size(gray.size()), CV_8UC3);
        cvtColor(gray, gcolor, CV_GRAY2BGR);

        int tlightnum = 0;
        int tnum[4] = { 0, };

        int max_n = 0;

        if(debug)
            cout << "n_hole:" << n_hole << endl; 

        //3 hole
        if (n_hole == 3)
        {
            tlightnum = 3;
            line(gcolor, Point(traffic_img.cols / 3, 0), Point(traffic_img.cols / 3, traffic_img.rows), Scalar(255, 0, 0));
            line(gcolor, Point(traffic_img.cols / 3*2, 0), Point(traffic_img.cols / 3*2, traffic_img.rows), Scalar(255, 0, 0));
            int ncount1 = 0;
            int ncount2 = 0;
            int ncount3 = 0;
            for (int y = 0; y < traffic_img.rows; y++) {
                for (int x = 0; x <  traffic_img.cols/3; x++) {

                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount1++;
                    }
                }for (int x = traffic_img.cols / 3; x < traffic_img.cols / 3*2; x++) {

                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount2++;
                    }
                }for (int x = traffic_img.cols / 3*2; x < traffic_img.cols; x++) {

                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount3++;
                    }
                }
            }
            tnum[0] = ncount1;
            tnum[1] = ncount2;
            tnum[2] = ncount3;
            tnum[3] = 0;
            if(debug)
                cout << "3±ž =  " << tnum[0] << "  " << tnum[1] << "  " << tnum[2] << "  " << tnum[3] << endl;
        }
        //4 hole
        else if (n_hole == 4)
        {
            tlightnum = 4;
            line(gcolor, Point(traffic_img.cols / 4, 0), Point(traffic_img.cols / 4, traffic_img.rows), Scalar(255, 0, 0));
            line(gcolor, Point(traffic_img.cols / 4 * 2, 0), Point(traffic_img.cols / 4 * 2, traffic_img.rows), Scalar(255, 0, 0));
            line(gcolor, Point(traffic_img.cols / 4 * 3, 0), Point(traffic_img.cols / 4 * 3, traffic_img.rows), Scalar(255, 0, 0));
            int ncount1 = 0;
            int ncount2 = 0;
            int ncount3 = 0;
            int ncount4 = 0;
            for (int y = 0; y < traffic_img.rows; y++) {
                for (int x = 0; x < traffic_img.cols / 4; x++) {
                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount1++;
                    }
                }for (int x = traffic_img.cols / 4; x < traffic_img.cols / 4 * 2; x++) {
                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount2++;
                    }
                }for (int x = traffic_img.cols / 4 * 2; x < traffic_img.cols / 4 * 3; x++) {
                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount3++;
                    }
                }for (int x = traffic_img.cols / 4 * 3; x < traffic_img.cols; x++) {
                    int a = gray.at<uchar>(y, x);
                    if (a != 0)
                    {
                        ncount4++;
                    }
                }
            }
            tnum[0] = ncount1;
            tnum[1] = ncount2;
            tnum[2] = ncount3;
            tnum[3] = ncount4;
            cout << "4±ž =  "  << tnum[0] << "  " << tnum[1] << "  " << tnum[2] << "  " << tnum[3] << endl;
            // if(debug)
                // cout << "4±ž =  "  << tnum[0] << "  " << tnum[1] << "  " << tnum[2] << "  " << tnum[3] << endl;
        }
        else{
            ROS_ERROR("impossible situation happened : n_hole == %d", n_hole);
        }

        // int thresh = 0; //sm
        // int thresh = (traffic_img.cols / tlightnum) * 4;
        max_n = max_node(tnum);
        
        // if(max_n < 0)
        // {
        //     light_msg.header.stamp = ros::Time::now();
        //     light_msg.light = light_msg.STRANGE; 
        //     traffic_light_pub.publish(light_msg);
        //     return 0;
        // }

        cout << "hole3 left : " << traffic_img.cols*traffic_img.rows / n_hole / 4 << endl;
        if ((n_hole == 3) && (tnum[2] > traffic_img.cols*traffic_img.rows / n_hole / 4)) // red&left threshold
        {
            if (max_n == 0)
            {
                max_n = 2;
            }
        }
        
        cout << "hole4 left : " << traffic_img.cols*traffic_img.rows / n_hole / 7 << endl;
        if ((n_hole == 4) && (tnum[2] > traffic_img.cols*traffic_img.rows / n_hole / 7)) // red&left threshold
        {
            if (max_n == 0)
            {
                max_n = 4;
            }
            else if (max_n == 3)
            {
                max_n = 5;
            }
        }

        // if(debug)
            // cout << "max node = " << max_n << endl;

        int light = confirm_light(traffic_img, tlightnum, max_n);

        string light_color;
        color_light(light, light_color);
        cout << "light = case " << light << "  /  "  << light_color << endl;
        // if(debug)
        //     cout << "light = case " << light << "  /  "  << light_color << endl;

        Mat reduce_img;
        resize(img, reduce_img, Size(img.cols / 3, img.rows / 3));

        char number[100];
        sprintf(number, "light = case %d", light);
        putText(reduce_img, number, Point(50, 20), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 0, 0), 1);
        putText(reduce_img, light_color, Point(50, 50), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 0, 0), 1);

        if(debug) {
            // namedWindow("gray",0);
            // imshow("gray", gray);
            // imshow("morphology_img", morphology_img);
            // imshow("thresh_img", thresh_img);
            namedWindow("gc",0);
            imshow("gc", gcolor);
            namedWindow("traffic_img",0);
            imshow("traffic_img", traffic_img);
            namedWindow("roi_img_af",0);
            imshow("roi_img_af", roi_img);
            imshow("reduce_img", reduce_img);

            waitKey(0);
        }
        return 0;

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

        if (false == roi_pair.second) return;
        else publish_color(cv_ptr, roi_pair.first);
    }

    LightDetector() : n_hole(4){
        ros::NodeHandle nh;

        //get namespace from topic
        //image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "usb_cam/image_raw", 1);
        image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "camera/image_raw", 1);
        detection_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>
            (nh,"/detection/image_detector/objects",1);

        detections_synchronizer_ =  new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
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
        n_hole = ptr->n_hole;
        section = ptr->section;
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

    message_filters::Synchronizer<SyncPolicyT> *detections_synchronizer_;

    message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detection_filter_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *image_filter_subscriber_;

    bool debug;
    traffic_light_msgs::Light light_msg;
    ros::Publisher traffic_light_pub;
    ros::Subscriber traffic_area_sub;
    int n_hole, section = 1;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "yolo_traffic_light");
    LightDetector d;
    ros::spin();

}
