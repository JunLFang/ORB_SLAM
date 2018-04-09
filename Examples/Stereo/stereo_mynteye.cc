/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<System.h>


#include "camera.h"
#include "utility.h"


using namespace std;
using namespace mynteye;

void Draw2dMap(Mat &pose, Mat &background, double timestamp);


int main(int argc, char **argv)
{
    Mat background(1200,1200,CV_32F);
    int slam_flag=1;
    std::uint32_t timestamp;
    float time;
    vector<IMUData> imudatas;
    // Open camera
    std::cout << "Open Camera: "<< std::endl;
    stringstream device;
    device << argv[3];
    mynteye::Camera cam;
   // cam.SetMode(MODE_GPU);
//    cout<<"device :"<<device.str()<<"/n"<<endl;

    mynteye::InitParameters params(device.str());
    std::cout << "Open Camera: "<< device.str()<< std::endl;

    cam.Open(params);
    std::cout << "Open Camera: "  << std::endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    while(cam.Grab() != ErrorCode::SUCCESS) {
        std::cout << "Warning: Grab failed <" << ">" << std::endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;


    cv::Mat img_left;
    cv::Mat img_right;
    int n=1;
    while(slam_flag!=1000)
    {

        cam.Grab();
        cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED);
        cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED);
        cam.RetrieveIMUData(imudatas,timestamp);
        time = static_cast<float>(static_cast<float>(timestamp)/10000.0);

        //SLAM.TrackMonocular(img_left, time);
        Mat each_pose;
        each_pose=SLAM.TrackStereo(img_left, img_right,time);
        if(!each_pose.empty())
        {
            Draw2dMap(each_pose,background,time);
        }

        slam_flag++;
//        if(slam_flag==2000)
//            break;

        if(slam_flag==100*n) {
            n++;
            cv::imwrite("/home/jimmy/VSLAM/ORB_SLAM2/map2d.jpg", background);
            cout << "imwrite success" << endl;
        }
        

    }
    cv::imwrite("/home/tony_jun/Git/ORB_SLAM2/map2d.jpg", background);
    SLAM.Shutdown();
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    SLAM.SaveMap("./mynt.map.jpg");
    cam.Close();
    cv::destroyAllWindows();

    return 0;

}

void Draw2dMap(Mat &pose, Mat &background, double timestamp) {
    Mat t(3, 1, CV_32F);
    Mat R(3, 3, CV_32F);
    Mat tw(3, 1, CV_32F);

    t = pose.rowRange(0, 3).col(3);
    R = pose.rowRange(0, 3).colRange(0, 3);

    tw = (-1) * R.inv() * t;
    cv::circle(background, cv::Point2f(tw.at<float>(0, 0)*10 + 600, tw.at<float>(0, 2)*10 + 600), 0, 255);
    cout << "<<<<<<<<<Pose output world: " << timestamp << "\n" << tw << endl;
}

