/**
* This file is a modified version of ORB-SLAM2.<https://github.com/raulmur/ORB_SLAM2>
*
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/bertabescos/DynaSLAM>.
*
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <sys/un.h>
#include <stdlib.h>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>

#include <memory>

#include<System.h>

#include "Frame.h"
#include "Object.h"

#include "Geometry.h"
#include "MaskNet.h"
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadBoundingBox(const string& strPathToDetectionResult, vector<std::pair<vector<double>, int>>& detect_result);

void LoadBoundingBoxFromPython(const string& resultFromPython, std::pair<vector<double>, int>& detect_result);
void MakeDetect_result(vector<std::pair<vector<double>, int>>& detect_result, int sockfd);

int main(int argc, char **argv)
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    if(argc != 5 && argc != 6 && argc != 7)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association (path_to_masks) (path_to_output)" << endl;
        return 1;
    }

    //for yolov5
    int sockfd;
	int len;
	struct sockaddr_un address;
	int result;
	int i,byte;
	char send_buf[128],ch_recv[1024];
 
	if((sockfd = socket(AF_UNIX, SOCK_STREAM, 0))==-1)//创建socket，指定通信协议为AF_UNIX,数据方式SOCK_STREAM
	{
		perror("socket");
		exit(EXIT_FAILURE);
	}
	
	//配置server_address
	address.sun_family = AF_UNIX;
	strcpy(address.sun_path, "/home/yakai/SLAM/orbslam_addsemantic-main/yolov5_RemoveDynamic/detect_speedup_send");
	len = sizeof(address);
 
	result = connect(sockfd, (struct sockaddr *)&address, len);
 
	if(result == -1) 
	{
		printf("ensure the server is up\n");
        	perror("connect");
        	exit(EXIT_FAILURE);
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    std::cout << "nImages: " << nImages << std::endl;

    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Initialize Mask R-CNN
    DynaSLAM::SegmentDynObject *MaskNet;
    if (argc==6 || argc==7)
    {
        cout << "Loading Mask R-CNN. This could take a while..." << endl;
        MaskNet = new DynaSLAM::SegmentDynObject();
        cout << "Mask R-CNN loaded!" << endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Dilation settings
    int dilation_size = 15;
    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           cv::Point( dilation_size, dilation_size ) );

    if (argc==7)
    {
        std::string dir = string(argv[6]);
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir = string(argv[6]) + "/rgb/";
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir = string(argv[6]) + "/depth/";
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir = string(argv[6]) + "/mask/";
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat imRGBOut, imDOut,maskOut;
    vector<std::pair<vector<double>, int>> detect_result,detect_result_test2;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	    //yolov5
        if (argc == 5)
        {
            cout << "********new********** " << ni+2 << endl;
            MakeDetect_result(detect_result,sockfd);
        }
  
        // Segment out the images
        if (argc == 6 || argc == 7)
        {
            cv::Mat maskRCNN;
            maskRCNN = MaskNet->GetSegmentation(imRGB,string(argv[5]),vstrImageFilenamesRGB[ni].replace(0,4,""));
            cv::Mat maskRCNNdil = maskRCNN.clone();
            cv::dilate(maskRCNN,maskRCNNdil, kernel);
            mask = mask - maskRCNNdil;
        }
        // Pass the image to the SLAM system
        if (argc == 7){SLAM.TrackRGBD(imRGB,imD,mask,tframe,imRGBOut,imDOut,maskOut);}
        if (argc == 6){SLAM.TrackRGBD(imRGB,imD,mask,tframe);}
        if (argc == 5){SLAM.TrackRGBD(imRGB,imD,tframe,detect_result); //for yolov5
            detect_result.clear();}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        if (argc == 7)
        {
            cv::imwrite(string(argv[6]) + "/rgb/" + vstrImageFilenamesRGB[ni],imRGBOut);
            vstrImageFilenamesD[ni].replace(0,6,"");
            cv::imwrite(string(argv[6]) + "/depth/" + vstrImageFilenamesD[ni],imDOut);
            cv::imwrite(string(argv[6]) + "/mask/" + vstrImageFilenamesRGB[ni],maskOut);
        }

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::duration<double> >(start - end).count();
    std::cout << total_time << std::endl;

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void LoadBoundingBox(const string& strPathToDetectionResult, vector<std::pair<vector<double>, int>>& detect_result){
    ifstream infile;
    infile.open(strPathToDetectionResult);
    if (!infile.is_open()) {
        cout<<"yolo_detection file open fail"<<endl;
        exit(233);
    }
    vector<double> result_parameter;
    string line;
    while (getline(infile, line)){
        int sum = 0, num_bit = 0;
        for (char c : line) {//读取数字.    例如读取"748",先读7,再7*10+8=78,再78*10+4,最后读到空格结束
            if (c >= '0' && c <= '9') {
                num_bit = c - '0';
                sum = sum * 10 + num_bit;
            } else if (c == ' ') {
                result_parameter.push_back(sum);
                sum = 0;
                num_bit = 0;
            }
        }

        string idx_begin = "class:";//读取物体类别
        int idx = line.find(idx_begin);
        string idx_end = "0.";
        int idx2 = line.find(idx_end);
        string class_label;
        for (int j = idx + 6; j < idx2-1; ++j){
            class_label += line[j];
        }
        // cout << "**" << class_label << "**";

        int class_id = -1;//存入识别物体的种类
        if (class_label == "person") { //高动态物体:人,动物等
            class_id = 3;
        }

        if (class_label == "tv" ||   //低动态物体(在程序中可以假设为一直静态的物体):tv,refrigerator
            class_label == "refrigerator" || 
            class_label == "teddy bear") {
            class_id = 1;
        }

        if (class_label == "chair" || //中动态物体,在程序中不做先验动态静态判断
            class_label == "car"){
            class_id =2;
        }

        detect_result.emplace_back(result_parameter,class_id);
        result_parameter.clear();
        line.clear();
    }
    infile.close();

}

//在一句话中提取出四个边框值和物体类别,such as: left:1 top:134 right:269 bottom:478 class:person 0.79
void LoadBoundingBoxFromPython(const string& resultFromPython, std::pair<vector<double>, int>& detect_result){
    
    if(resultFromPython.empty())
    {
        cerr << "no string from python! " << endl;
    }
    // cout << "here is LoadBoundingBoxFromPython " << endl;
    vector<double> result_parameter;
    int sum = 0, num_bit = 0;

    for (char c : resultFromPython) {//读取数字.    例如读取"748",先读7,再7*10+8=78,再78*10+4,最后读到空格结束
        if (c >= '0' && c <= '9') {
            num_bit = c - '0';
            sum = sum * 10 + num_bit;
        } else if (c == ' ') {
            result_parameter.push_back(sum);
            sum = 0;
            num_bit = 0;
        }
    }

    detect_result.first = result_parameter;
    // cout << "detect_result.first size is : " << detect_result.first.size() << endl;

    string idx_begin = "class:";//读取物体类别
    int idx = resultFromPython.find(idx_begin);
    string idx_end = "0.";
    int idx2 = resultFromPython.find(idx_end);
    string class_label;
    for (int j = idx + 6; j < idx2-1; ++j){
        class_label += resultFromPython[j];
    }

    int class_id = -1;//存入识别物体的种类

    if (class_label == "tv" ||   //低动态物体(在程序中可以假设为一直静态的物体):tv,refrigerator
        class_label == "refrigerator" || 
        class_label == "teddy bear"||
        class_label == "laptop") {
        class_id = 1;
    }

    if (class_label == "chair" || //中动态物体,在程序中不做先验动态静态判断
        class_label == "car"){
        class_id =2;
    } 

    if (class_label == "person") { //高动态物体:人,动物等
        class_id = 3;
    }

    detect_result.second = class_id;
    // cout << "LoadBoundingBoxFromPython class id is: " << class_id << endl;

}

//通过UNIX的协议,从python进程中获取一帧图像的物体框
void MakeDetect_result(vector<std::pair<vector<double>, int>>& detect_result , int sockfd){
    detect_result.clear();

	std::pair<vector<double>, int> detect_result_str;
    int byte;
	char send_buf[128],ch_recv[1024];

    sprintf(send_buf,"ok");//用sprintf事先把消息写到send_buf
	if((byte=write(sockfd, send_buf, sizeof(send_buf)))==-1)
    {
		perror("write");
		exit(EXIT_FAILURE);
	}

    if((byte=read(sockfd,&ch_recv,1000))==-1)
	{
		perror("read");
		exit(EXIT_FAILURE);
	}
    // cout << "**ch_recv is : \n" << ch_recv << endl;

    char *ptr;//char[]可读可写,可以修改字符串的内容。char*可读不可写，写入就会导致段错误
    ptr = strtok(ch_recv, "*");//字符串分割函数
    while(ptr != NULL){
        printf("ptr=%s\n",ptr);

        if ( strlen(ptr)>20 ){//试图去除乱码,乱码原因未知...好像并不能去除,留着吧,心理安慰下
            // cout << strlen(ptr) << endl;
            string ptr_str = ptr;
            LoadBoundingBoxFromPython(ptr_str,detect_result_str);
        } 

        detect_result.emplace_back(detect_result_str);
        // cout << "hh: " << ptr_str << endl;  
        ptr = strtok(NULL, "*");
    }
    // cout << "detect_result size is : " << detect_result.size() << endl;
    // for (int k=0; k<detect_result.size(); ++k)
        // cout << "detect_result is : \n " << detect_result[k].second << endl;
}
