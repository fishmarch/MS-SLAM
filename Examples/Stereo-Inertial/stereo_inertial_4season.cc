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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

size_t mem_usage() {
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != nullptr) {
        if (strncmp(line, "VmRSS:", 6) == 0) {
            int len = strlen(line);

            const char* p = line;
            for (; std::isdigit(*p) == false; ++p) {}

            line[len - 3] = 0;
            result = atoi(p);

            break;
        }
    }

    fclose(file);
    return result;
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./stereo_inertial_4season path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    const int num_seq = 1;
    cout << "num_seq = " << num_seq << endl;

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[3]), vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(string(argv[3]), vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true);

    vector<float> vMem;
    vMem.reserve(tot_images);
    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++) {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;
        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);

            if (imLeft.empty()) {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if (imRight.empty()) {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if (ni > 0)
                while (vTimestampsImu[seq][first_imu[seq]] <=
                       vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x, vAcc[seq][first_imu[seq]].y,
                                                             vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x, vGyro[seq][first_imu[seq]].y,
                                                             vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1];

            if (ttrack < T)
                usleep((T - ttrack) * 1e6);


            size_t vm = mem_usage();
            vMem.push_back((float) vm / 1024.0);
//            cout << "#" << " " << ni << " " << (float)vm / 1024.0 << " "
//            << SLAM.CompressedMapPointsInMap() << " " << SLAM.MapPointsInMap() << endl;
        }
    }
    // Stop all threads
    SLAM.Shutdown();

    float totaltime = 0;
    for(int ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;

    float memMax = 0.0;
    for (int i = 0; i < vMem.size(); ++i) {
        if(vMem[i] > memMax)
            memMax = vMem[i];
    }
    cout <<"----------Memory Usage----------" << endl;
    cout << "Max Memory Consumption: " << memMax  << " MB, increasing " << memMax-vMem[10] << " MB" << endl;
//    // Save camera trajectory
//    if (bFileName)
//    {
//        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
//        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
//        SLAM.SaveTrajectoryEuRoC(f_file);
//        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
//    }
//    else
//    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt", false);

//    }

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    string strPrefixLeft = strPathToSequence + "/undistorted_images/cam0/";
    string strPrefixRight = strPathToSequence + "/undistorted_images/cam1/";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string file;
            ss >> file;
            vstrImageLeft.push_back(strPrefixLeft + file + ".png");
            vstrImageRight.push_back(strPrefixRight + file + ".png");
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath + "/imu.txt");
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
            double wx, wy, wz;
            ss >> wx >> wy >> wz;
            vGyro.push_back(cv::Point3f(wx, wy, wz));
            double ax, ay, az;
            ss >> ax >> ay >> az;
            vAcc.push_back(cv::Point3f(ax, ay, az));
        }
    }
}
