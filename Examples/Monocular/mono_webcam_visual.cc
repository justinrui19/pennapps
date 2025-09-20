/*
 * Enhanced Webcam Live SLAM with Visual Feedback
 * Shows camera feed with tracked ORB features, trajectory, and mapping
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/opencv.hpp>

#include<System.h>

using namespace std;

// Helper function to draw ORB features on image
void drawFeatures(cv::Mat& img, const vector<cv::KeyPoint>& keypoints, cv::Scalar color = cv::Scalar(0, 255, 0))
{
    for(const auto& kp : keypoints)
    {
        cv::circle(img, kp.pt, 3, color, 1);
    }
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_webcam_visual path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system with visualization enabled
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    cout << endl << "-------" << endl;
    cout << "Starting Enhanced Webcam SLAM with Visual Feedback" << endl;
    cout << "You will see:" << endl;
    cout << "• Camera feed with tracked ORB features (green circles)" << endl;
    cout << "• Real-time tracking status" << endl;
    cout << "• 3D viewer window (if available)" << endl;
    cout << "Press ESC to stop" << endl;
    cout << "-------" << endl;

    // Initialize webcam
    cv::VideoCapture cap(0);
    
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open webcam" << endl;
        return -1;
    }
    
    // Set camera properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    cout << "Webcam initialized" << endl;
    cout << "Resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;

    // ORB feature detector for visualization
    cv::Ptr<cv::ORB> orb_detector = cv::ORB::create(1000); // Match SLAM settings

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
        
        // Get timestamp
        auto current_time = chrono::steady_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(current_time.time_since_epoch()).count() / 1000.0;
        
        // Convert to grayscale for SLAM
        cv::Mat im_gray;
        if(im.channels() == 3)
        {
            cv::cvtColor(im, im_gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            im_gray = im.clone();
        }
        
        // Detect ORB features for visualization
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb_detector->detectAndCompute(im_gray, cv::Mat(), keypoints, descriptors);
        
        // Pass the image to the SLAM system
        cv::Mat pose = SLAM.TrackMonocular(im_gray, timestamp);
        
        // Create enhanced display frame
        cv::Mat display_frame = im.clone();
        
        // Draw ORB features
        if(!keypoints.empty())
        {
            drawFeatures(display_frame, keypoints, cv::Scalar(0, 255, 0)); // Green circles
        }
        
        // Add status overlay
        std::string status = pose.empty() ? "TRACKING: LOST" : "TRACKING: OK";
        cv::Scalar status_color = pose.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        
        auto elapsed = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
        double fps = frame_count / (double)(elapsed + 1);
        
        // Create info panel background
        cv::rectangle(display_frame, cv::Point(5, 5), cv::Point(400, 150), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(display_frame, cv::Point(5, 5), cv::Point(400, 150), cv::Scalar(255, 255, 255), 1);
        
        // Overlay information
        cv::putText(display_frame, status, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);
        cv::putText(display_frame, "Frame: " + std::to_string(frame_count), cv::Point(10, 55), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(display_frame, "FPS: " + std::to_string((int)fps), cv::Point(10, 75), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(display_frame, "Features: " + std::to_string(keypoints.size()), cv::Point(10, 95), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(display_frame, "ESC to quit", cv::Point(10, 115), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        // Add trajectory info if tracking
        if(!pose.empty())
        {
            cv::putText(display_frame, "Building map...", cv::Point(10, 135), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        
        // Show the enhanced camera feed
        cv::imshow("ORB-SLAM2: Live Camera with Features", display_frame);
        
        // Console output
        if(frame_count % 30 == 0)
        {
            cout << "Frame " << frame_count 
                 << ", FPS: " << (int)fps 
                 << ", Features: " << keypoints.size()
                 << ", Tracking: " << (pose.empty() ? "LOST" : "OK") << endl;
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

    // Save trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("webcam_visual_trajectory.txt");
    
    cout << endl << "Session Summary:" << endl;
    cout << "Trajectory saved to: webcam_visual_trajectory.txt" << endl;
    cout << "Total frames processed: " << frame_count << endl;
    
    cv::destroyAllWindows();
    return 0;
}