#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;

// Function calculates distance 
// between two points 
long dist(cv::Point p1,
          cv::Point p2) 
{ 
    long x0 = p1.x - p2.x; 
    long y0 = p1.y - p2.y; 
    return x0 * x0 + y0 * y0; 
} 

// Function calculates distance 
// between a point and a line defined by two points 
double point_line_distance(cv::Point l1, cv::Point l2, cv::Point p0)
{
    return abs((l2.x - l1.x)*(l1.y-p0.y) - (l1.x - p0.x)*(l2.y-l1.y)) / (sqrt((l2.x - l1.x)*(l2.x - l1.x) + (l2.y-l1.y)*(l2.y-l1.y)));

}

//check if a point stays on a line defined by two points
void check_line_point(std::vector<cv::Point> pointList, cv::Point l1, cv::Point l2){
    double equation = 0.0;

    for(int i = 0; i < pointList.size(); i++)
    { 
        equation = ((l2.y - l1.y) * (pointList[i].x - l1.x) -  (pointList[i].y - l1.y) * (l2.x - l1.x));
        std::cout<<"belonging check:"<<equation<<std::endl;
    }


}

// Function to find the maximum 
// distance between a list of point and a line
void point_line_dist(std::vector<cv::Point> pointList, cv::Point l1, cv::Point l2) 
{ 
 
    // Iterate over all possible point -line 
    for(int i = 0; i < pointList.size(); i++)
    { 
        // actual_min = min(minimum, (double)point_line_distance(pointList[i],l1,l2)); 

        std::cout<<"point line distance: "<<(double)point_line_distance(pointList[i],l1,l2)<<std::endl;
        
    } 
 
    // Return actual distance

}

// Function to find the maximum 
// distance between any two points 
std::vector<cv::Point> maxDist(std::vector<cv::Point> pointList) 
{ 
    double Max = 0; 
    double actual_Max = 0;
    std::vector<cv::Point> max_dist_points (2);
 
    // Iterate over all possible pairs 
    for(int i = 0; i < pointList.size(); i++)
    { 
        for(int j = i + 1; j < pointList.size(); j++)
        { 
             
            // Update max 
            actual_Max = max(Max, (double)dist(pointList[i], 
                                        pointList[j])); 

            if(actual_Max > Max)
            {
                Max = actual_Max;

                max_dist_points.at(0) = pointList[i];
                max_dist_points.at(1) = pointList[j];         
            }
        } 
    } 
 
    // Return actual distance 
    return max_dist_points; 
} 

int main()
{
    cv::Mat resized_img;

    cv::Mat color = cv::imread("./circles/dhtf_00002.jpg");
    
    cv::resize(color, resized_img, cv::Size(), 0.30, 0.30);
    cv::namedWindow("input"); cv::imshow("input", resized_img);

    cv::Mat canny;

    cv::Mat gray;
    /// Convert it to gray
    cv::cvtColor( color, gray, CV_BGR2GRAY );

    // compute canny (don't blur with that image quality!!)
    cv::Canny(gray, canny, 200,20);
    
    cv::resize(canny, resized_img, cv::Size(), 0.30, 0.30);
    cv::namedWindow("canny2"); cv::imshow("canny2", resized_img>0);

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 30, 100, 12, 15, 20 );

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ ) 
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( color, center, 3, Scalar(0,255,255), -1);       //draw the center as a small circle
        cv::circle( color, center, radius, Scalar(0,0,255), 1 );    //draw founded circles
    }

    //compute distance transform:
    cv::Mat dt;
    cv::distanceTransform(255-(canny>0), dt, CV_DIST_L2 ,3);
    
    cv::resize(dt, resized_img, cv::Size(), 0.30, 0.30);
    cv::namedWindow("distance transform"); cv::imshow("distance transform", resized_img/255.0f);

    // test for semi-circles:
    float minInlierDist = 2.0f;
    for( size_t i = 0; i < circles.size(); i++ ) 
    {
        // test inlier percentage:
        // sample the circle and check for distance to the next edge
        unsigned int counter = 0;
        unsigned int inlier = 0;

        cv::Point2f center((circles[i][0]), (circles[i][1]));
        float radius = (circles[i][2]);

        // maximal distance of inlier might depend on the size of the circle
        float maxInlierDist = radius/25.0f;
        if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;

        // store the 2 inliers and outliers
        std::vector<cv::Point> inliersList;
        std::vector<cv::Point> outliersList;

        //TODO: maybe paramter incrementation might depend on circle size!
        for(float t =0; t<2*3.14159265359f; t+= 0.1f) 
        {
            counter++;
            float cX = radius*cos(t) + circles[i][0];
            float cY = radius*sin(t) + circles[i][1];

            if(dt.at<float>(cY,cX) < maxInlierDist) 
            {
                inlier++;
                cv::circle(color, cv::Point2i(cX,cY),3, cv::Scalar(0,255,0));     //draw inliers with green
                inliersList.push_back(cv::Point (cX, cY));
            } 
            else
            {
                cv::circle(color, cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));     //draw outliers with blue
                outliersList.push_back(cv::Point (cX, cY));    
            }
        }

        
        // for(cv::Point p : inliersList)
        // {
        //     std::cout<<"Inliers:\t" << p.x << '\t' << p.y << '\n';
        // }
        // std::cout<<std::endl;

        std::vector<cv::Point> max_dist_point = maxDist(inliersList); 
        // for(auto elem : max_dist_point)
        // {
        //     std::cout<<"Points:\t" << elem.x << '\t' << elem.y << '\n';
        //     cv::circle(color, cv::Point2i(elem.x, elem.y),5, cv::Scalar(255,255,255));
        // }
        // std::cout<<std::endl;

        // diameter line
        //cv::line(color, max_dist_point.at(0), max_dist_point.at(1), cv::Scalar(255,255,255),5);
        
        
        
        double dir_x = max_dist_point.at(0).x-max_dist_point.at(1).x;
        double dir_y = max_dist_point.at(0).y-max_dist_point.at(1).y;
        // double mag = sqrt(dir_x*dir_x + dir_y*dir_y);
        
        //std::cout<<"Inliers:\n";
        // point_line_dist(inliersList, center, cv::Point(dir_y+center.x, -dir_x+center.y));
        //std::cout<<"\nOutliers:\n";
        // point_line_dist(outliersList, center, cv::Point(dir_y+center.x, -dir_x+center.y));

   
        
        // Vec3b & pixel_color = color.at<Vec3b>(perp_dir_v.y-2.0*radius,perp_dir_v.x-2.0*radius);
        // std::cout<<"Colour on board:"<<pixel_color<<std::endl;
        // cv::circle(color, cv::Point2i(perp_dir_v.x-radius,perp_dir_v.y-radius),8, cv::Scalar(0,0,255));

        double mean_x = 0.0;
        double mean_y = 0.0;
        for(int i=0; i<outliersList.size(); i++)
        {
            mean_x += outliersList[i].x;
            mean_y += outliersList[i].y;
        }
        mean_x /= outliersList.size();
        mean_y /= outliersList.size();

        double theta= atan2(center.y-mean_y, center.x-mean_x);
        double pt_x = center.x - 30.0 * cos(theta);
        double pt_y = center.y - 30.0 * sin(theta);
        // cv::circle(color, cv::Point2i(mean_x,mean_y),5, cv::Scalar(0,0,255), -1);
        cv::line(color, center, cv::Point2i(pt_x,pt_y), cv::Scalar(255,255,255),5);
        // circle(InputOutputArray img, Point center, int radius,
        //                const Scalar& color, int thickness = -1,
        //                int lineType = LINE_8, int shift = 0);

        cv::Point perp_dir_v;
        if(true)
        {
            perp_dir_v = cv::Point(-dir_y+center.x, +dir_x+center.y);
        }
        else
        {
            perp_dir_v = cv::Point(dir_y+center.x, -dir_x+center.y);
        }
        
        //QUESTA È LA LINEA CHE USERAI SEMPRE
        //cv::line(color, center, perp_dir_v, cv::Scalar(255,255,255),5);

        //check_line_point(inliersList, center, perp_dir_v);
        // cv::line(color, center, ort_point, cv::Scalar(255,255,255),5);
        std::cout<<"Circle center:"<<center.x<<'\t'<<center.y<<std::endl;
        std::cout << 100.0f*(float)inlier/(float)counter << " % of a circle with radius " << radius << " detected" << std::endl;
    
    }

    cv::resize(color, resized_img, cv::Size(), 0.30, 0.30);
    cv::namedWindow("output"); cv::imshow("output", resized_img);
    cv::imwrite("houghLinesComputed.png", color);

    cv::waitKey(-1);

    
    return 0;
}