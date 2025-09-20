/**
* This file is part of ORB-SLAM2.
* 
* This is a modified example for ESP32 camera live streaming
* Based on mono_tum.cc but adapted for IP camera streaming
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_esp32cam_live path_to_vocabulary path_to_settings camera_stream_url" << endl;
        cerr << "Example: ./mono_esp32cam_live Vocabulary/ORBvoc.txt Examples/Monocular/ESP32CAM.yaml http://192.168.43.36:81" << endl;
        return 1;
    }

    string strVocFile = argv[1];
    string strSettingsFile = argv[2]; 
    string strStreamURL = argv[3];

    cout << endl << "-------" << endl;
    cout << "Starting ORB-SLAM2 with ESP32 Camera" << endl;
    cout << "Vocabulary: " << strVocFile << endl;
    cout << "Settings: " << strSettingsFile << endl;
    cout << "Stream URL: " << strStreamURL << endl;
    cout << "-------" << endl << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strVocFile, strSettingsFile, ORB_SLAM2::System::MONOCULAR, true);

    cout << "Initializing camera stream..." << endl;
    
    // Initialize camera capture with multiple backend attempts
    cv::VideoCapture cap;
    
    // Try different backends for network streaming
    std::vector<int> backends = {
        cv::CAP_FFMPEG,
        cv::CAP_GSTREAMER, 
        cv::CAP_OPENCV_MJPEG,
        cv::CAP_ANY
    };
    
    bool stream_opened = false;
    for(int backend : backends) {
        cout << "Trying backend: " << backend << endl;
        if(cap.open(strStreamURL, backend)) {
            cout << "Successfully opened stream with backend: " << backend << endl;
            stream_opened = true;
            break;
        }
    }
    
    if(!stream_opened) {
        cerr << "ERROR: Cannot open camera stream with any backend: " << strStreamURL << endl;
        cerr << "Trying alternative approach..." << endl;
        
        // Alternative: Try opening as MJPEG stream 
        string mjpeg_url = strStreamURL;
        if(cap.open(mjpeg_url)) {
            cout << "Stream opened with default method" << endl;
            stream_opened = true;
        }
    }
    
    if(!stream_opened) {
        cerr << "ERROR: Failed to open stream. Please check:" << endl;
        cerr << "1. Camera is accessible at: " << strStreamURL << endl;
        cerr << "2. Stream format is supported" << endl;
        cerr << "3. Network connectivity" << endl;
        return -1;
    }
    
    // Set camera properties for better performance
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Reduce buffer size to minimize latency
    cap.set(cv::CAP_PROP_FPS, 30);        // Set desired FPS
    
    if(!cap.isOpened()) {
        cerr << "ERROR: Camera stream is not opened!" << endl;
        return -1;
    }

    cout << "Camera stream initialized successfully!" << endl;
    cout << "Press ESC to stop tracking" << endl;

    // Main loop
    cv::Mat im;
    double frame_time = 0.0;
    auto start_time = chrono::steady_clock::now();
    int frame_count = 0;
    
    for(;;)
    {
        // Capture frame
        if(!cap.read(im)) {
            cerr << "Warning: Failed to read frame from camera" << endl;
            continue;
        }
        
        if(im.empty()) {
            cerr << "Warning: Empty frame received" << endl;
            continue;
        }

        frame_count++;
        auto current_time = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count();
        
        // Calculate timestamp (in seconds since start)
        double timestamp = elapsed / 1000.0;

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, timestamp);
        
        // Display the image with SLAM overlay
        cv::Mat im_with_info = im.clone();
        
        // Add FPS info
        if(elapsed > 0) {
            double fps = frame_count * 1000.0 / elapsed;
            cv::putText(im_with_info, 
                       "FPS: " + to_string((int)fps) + " | Frame: " + to_string(frame_count), 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::putText(im_with_info, "ESC to quit", 
                   cv::Point(10, im_with_info.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        cv::imshow("ORB-SLAM2: ESP32 Camera", im_with_info);
        
        char key = cv::waitKey(1);
        if(key == 27) { // ESC key
            cout << "\nESC pressed. Stopping..." << endl;
            break;
        }
        
        // Optional: limit frame rate to prevent overwhelming the system
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS max
    }

    // Stop all threads
    cout << "Shutting down SLAM system..." << endl;
    SLAM.Shutdown();

    // Save camera trajectory
    cout << "Saving camera trajectory to CameraTrajectory.txt" << endl;
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    
    cout << "Saving keyframe trajectory to KeyFrameTrajectory.txt" << endl;
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cv::destroyAllWindows();
    cap.release();
    
    cout << "Program finished successfully!" << endl;
    return 0;
}