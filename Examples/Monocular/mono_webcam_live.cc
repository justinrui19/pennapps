/*
 * Webcam Live SLAM
 * Modified from mono_tum.cc for live webcam input
 * Real-time ORB-SLAM2 with computer camera
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/opencv.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_webcam_live path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Starting webcam live SLAM" << endl;
    cout << "Press ESC to stop" << endl;
    cout << "-------" << endl;

    // Initialize webcam
    cv::VideoCapture cap(0); // Try camera index 0
    
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open webcam" << endl;
        return -1;
    }
    
    // Set camera properties for better SLAM
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    cout << "Webcam initialized" << endl;
    cout << "Camera resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "Camera FPS: " << cap.get(cv::CAP_PROP_FPS) << endl;

    // Main loop
    cv::Mat im;
    int frame_count = 0;
    auto start_time = chrono::steady_clock::now();
    
    while(true)
    {
        // Capture frame
        cap >> im;
        
        if(im.empty())
        {
            cerr << "WARNING: Empty frame captured" << endl;
            continue;
        }
        
        frame_count++;
        
        // Get timestamp (current time)
        auto current_time = chrono::steady_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(current_time.time_since_epoch()).count() / 1000.0;
        
        // Convert to grayscale if needed
        cv::Mat im_gray;
        if(im.channels() == 3)
        {
            cv::cvtColor(im, im_gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            im_gray = im.clone();
        }
        
        // Pass the image to the SLAM system
        cv::Mat pose = SLAM.TrackMonocular(im_gray, timestamp);
        
        // Display some info
        if(frame_count % 30 == 0) // Every 30 frames
        {
            auto elapsed = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
            double fps = frame_count / (double)elapsed;
            cout << "Frame " << frame_count << ", FPS: " << fps << ", Tracking: " << (pose.empty() ? "LOST" : "OK") << endl;
        }
        
        // Check for ESC key
        int key = cv::waitKey(1) & 0xFF;
        if(key == 27) // ESC key
        {
            cout << "ESC pressed, stopping SLAM" << endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("webcam_trajectory.txt");
    
    cout << "Camera trajectory saved to webcam_trajectory.txt" << endl;
    cout << "Processed " << frame_count << " frames" << endl;

    return 0;
}