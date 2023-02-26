/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <ctime>
#include <chrono>
#include <sstream>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
   cout << "Finishing session" << endl;
   b_continue_session = false;

}

int main(int argc, char **argv)
{
    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: ./mono_webcam path_to_vocabulary path_to_settings (trajectory_file_name)" << endl;
        return 1;
    }
    cout.precision(17);

    string file_name;
    bool bFileName = false;

    if (argc == 4)
    {
        file_name = string(argv[argc-1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Open an OpenCV webcam input stream
    const int cameraIndex = 0;
    cv::VideoCapture cap(cameraIndex);
    if (!cap.isOpened())
    {
        cerr << "ERROR: Could not open camera " << cameraIndex << endl;
        return 1;
    }
    cv::Mat imCV; // input frame buffer

    // get image size from webcam:
    const int width_img = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    const int height_img = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    const int fps_reported = cap.get(cv::CAP_PROP_FPS);
    cout << "Camera open. Image size: " << width_img << "x" << height_img << " FPS: " << fps_reported << endl;
    int framesReadTotal = 0; // total number of frames read from webcam
    int framesReadLastSecond = 0; // total number of frames read from webcam
    int cur_fps = 0; // frame read in the last second
    std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    Sophus::SE3f pose;
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    while(b_continue_session)
    {
        // Get a timestamp in milliseconds
        const double timestamp_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // cout << "timestamp: " << timestamp_ms << endl;

        if(cap.read(imCV) == false) // Retrieve image via OpenCV camera capture interface
        {
            cerr << "ERROR: Could not read from camera " << cameraIndex << endl;
            b_continue_session = false;
            break;
        }

        // imCV = cv::Mat(cv::Size(width_img, height_img), CV_8UC1, (void*)(frame.get_data()), cv::Mat::AUTO_STEP);
        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
            int width = imCV.cols * imageScale;
            int height = imCV.rows * imageScale;
            cv::resize(imCV, imCV, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
#endif
        // Pass the image to the SLAM system
        pose = SLAM.TrackMonocular(imCV, timestamp_ms);

        framesReadTotal++;
        framesReadLastSecond++;
        if (std::chrono::steady_clock::now() - tStart >= std::chrono::seconds(1))
        {
            cur_fps = framesReadLastSecond;
            framesReadLastSecond = 0;
            tStart = std::chrono::steady_clock::now();
            cout << "FPS: " << cur_fps << endl;
        }

#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
