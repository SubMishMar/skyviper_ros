#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ocam_header.h"
int main(int argc, char** argv)
{

  //Reading parameter file
  std::string cfg_filename = argv[1];
  cv::FileStorage fs(cfg_filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
   std::cerr << "Failed to open " << cfg_filename << std::endl;
   return 1;
  }
 
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat r_cw, T_cw, T_wc_transformed, rt_cw; 
    fs["intrinsics"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    int fd;
 
    fd = open(argv[2], O_RDWR);//accepts the name of the device as a string argument
    if (fd == -1)
    {
     perror("Opening video device");
     return 1;
    }
    if(print_caps(fd))
     return 1;
        
    if(init_mmap(fd))
     return 1;
   
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat image;
    std::vector< cv::Vec3d > rvecs, tvecs;
    std::vector<int> ids;
    std::vector<double> dists;
    std::vector<std::vector<cv::Point2f> > corners;


    //Clock Variables
    clock_t begin, end;
    double dt;
    
    double data[] = {0, 0, 0, 1}; 
    //double data_R[4][4] = {{0, 1, 0, 0}, {1, 0, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1}};
    double data_R[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    cv::Mat last_row = cv::Mat(1,4, CV_64F, data);
    cv::Mat transformation = cv::Mat(4, 4, CV_64F, data_R);
    cv::namedWindow("Image View");
    cv::moveWindow("Image View", 20,20);
    double c_y, c_x;
    double mean_x, mean_y, d_fm_ctr;
    while (true) 
    {
     image = capture_image(fd); 
     begin = clock();
     c_y = (image.rows - 1)/2;
     c_x = (image.cols - 1)/2;
     cv::aruco::detectMarkers(image, dictionary, corners, ids);
     // if at least one marker detected
     if (ids.size() > 0)
     {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
        /*
        cv::aruco::estimatePoseSingleMarkers(corners, 0.141, cameraMatrix, distCoeffs, rvecs, tvecs);

        for(int i=0; i<ids.size(); i++)
        {
         
         mean_x = mean_y = 0;
         for(int j =0; j < 4; j++)
         {
          mean_x = mean_x + corners[i][j].x;
          mean_y = mean_y + corners[i][j].y;
         }
         mean_x = mean_x/4;
         mean_y = mean_y/4;

         
         d_fm_ctr = sqrt((mean_x - c_x)*(mean_x - c_x) + (mean_y - c_y)*(mean_y - c_y));
         //std::cout << d_fm_ctr_2 << std::endl;
         dists.push_back(d_fm_ctr * norm(tvecs[i])); // dist metric = distance from center times dist from camera.
        }

        int hi, lo;    
        hi = lo = 0;   
     
        for (int i = 0;i < dists.size();i++) 
        {
         if(dists[i] < dists[lo]) 
         {
          lo = i;
         }
         else if(dists[i] > dists[hi]) 
         {
          hi = i;
         }
        }
        //std::cout << corners[lo] << std::endl;
        cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[lo], tvecs[lo], 0.1);

        cv::Rodrigues(rvecs[lo], r_cw, cv::noArray());
        cv::hconcat(r_cw, tvecs[lo], rt_cw);
        cv::vconcat(rt_cw, last_row, T_cw);
        T_wc_transformed = transformation * T_cw.inv();
        */
     }
     dists.clear();
     end = clock();
     dt = double(end - begin) / CLOCKS_PER_SEC;
     //std::cout << 1/dt << std::endl;
     cv::imshow("Image View", image);
     char key = (char) cv::waitKey(1);
     if (key == 27)
         break;
    }
    close(fd);

    cv::destroyWindow("Image View");
    return 0;
}
