//=============================================================================
// Copyright (c) 2001-2019 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================



#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <filesystem> // C++17 feature
#include <string>
#include "direct.h"
#include <fstream>
#include <cmath>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "opencv2/opencv.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;


#define VERBOSE 

#define ALERT_INVERSION 50
#define SAVE_IMAGE "FALSE"
#define PRINT_IMAGE "TRUE"
#define PATH_SAVE_IMG "Img"

#define IMAGE_NUMBER 100000
#define ZOOM_FACTOR 5.0

#define IR_FILTER "TRUE"





bool SetAcquisitionMode(INodeMap& nodeMap)
{
#ifdef VERBOSE
    cout << endl << endl << "*** SETTING ACQUISITION MODE ***" << endl << endl;
#endif // VERBOSE
    try
    {
        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
#ifdef VERBOSE
            cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
#endif // VERBOSE
            return false;
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
#ifdef VERBOSE
            cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
#endif // VERBOSE
            return false;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
#ifdef VERBOSE
        cout << "Acquisition mode set to continuous..." << endl;
#endif // VERBOSE

        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return false;
    }
}


// This function returns the camera serial number
gcstring GetDeviceSerialNumber(INodeMap& nodeMapTLDevice)
{
    gcstring deviceSerialNumber("");
    CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
    {
        deviceSerialNumber = ptrStringSerial->GetValue();
#ifdef VERBOSE
        cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
#endif // VERBOSE
    }
#ifdef VERBOSE
    cout << endl;
#endif // VERBOSE
    return deviceSerialNumber;
}

int PrintDeviceInfo(INodeMap& nodeMap)
{
#ifdef VERBOSE
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;
#endif // VERBOSE

    try
    {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            for (auto it = features.begin(); it != features.end(); ++it)
            {
                const CNodePtr pfeatureNode = *it;
#ifdef VERBOSE
                cout << pfeatureNode->GetName() << " : ";
#endif // VERBOSE
                CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
#ifdef VERBOSE
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
#endif // VERBOSE
            }
        }
        else
        {
#ifdef VERBOSE
            cout << "Device control information not available." << endl;
#endif // VERBOSE
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}


// This function creates a folder
bool CreateFolder(const char* folderPath)
{
    if (_mkdir(folderPath) == 0) {
#ifdef VERBOSE
        std::cout << "Folder created successfully: " << folderPath << std::endl;
#endif // VERBOSE
        return true;
    }
    else {
#ifdef VERBOSE
        std::cout << "Folder already exists or failed to create: " << folderPath << std::endl;
#endif // VERBOSE
        return false;
    }
}


void SaveImage(const std::string& folderPath, const gcstring& deviceSerialNumber, const Mat& image, unsigned int imageCnt) {
    ostringstream filename_ROI;

    filename_ROI << folderPath;
    filename_ROI << "/Acquisition_-";
    if (!deviceSerialNumber.empty()) {
        filename_ROI << deviceSerialNumber.c_str() << "-";
    }
    filename_ROI << imageCnt << ".jpg";

    // Save the current image using imwrite
    imwrite(filename_ROI.str(), image);

    cout << "Image saved at " << filename_ROI.str() << endl;
}


//Get the bounding box of the largest contour in the image
Rect Get_bounding_box(const Mat& image, double zoomFactor) {
    // Pre-process the image: Blur and edge detection
    Mat blurredImage, edges;
    if (IR_FILTER != "TRUE") {
        GaussianBlur(image, blurredImage, Size(5, 5), 1.5); // Not necessary with the filter
    }
    Canny(image, edges, 100, 200);

    // Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        cout << "No contours found in the image." << endl;
        return Rect();
    }

    // Find the largest contour
    double maxArea = 0.0;
    int largestContourIndex = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            largestContourIndex = static_cast<int>(i);
        }
    }

    if (largestContourIndex == -1) {
        cout << "Failed to find the largest contour." << endl;
        return Rect();
    }
    // Get the bounding rectangle of the largest contour
    Rect boundingBox = boundingRect(contours[largestContourIndex]);

    return boundingBox;
}

Mat zoom_into_bouding_rect(const Mat& image, double zoomFactor, Rect boundingBox) {

    // Crop the ROI
    Mat croppedROI = image(boundingBox);

    // Resize the cropped ROI to zoom in
    Mat zoomedImage;
    Size newSize(static_cast<int>(croppedROI.cols * zoomFactor),
        static_cast<int>(croppedROI.rows * zoomFactor));
    try {
        resize(croppedROI, zoomedImage, newSize, 0, 0, INTER_LINEAR);
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        zoomedImage = croppedROI;
    }

    return zoomedImage;
}


int FindClosestContour(const Mat& zoomedImage, const vector<vector<Point>>& contours)
{
    Point2f imageCenter(zoomedImage.cols / 2.0f, zoomedImage.rows / 2.0f);
    double minDistance = std::numeric_limits<double>::max();
    int closestContourIndex = -1;

    for (size_t i = 0; i < contours.size(); ++i) {
        Moments mu = moments(contours[i], false);
        Point2f contourCenter(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
        double distance = norm(contourCenter - imageCenter);
        if (distance < minDistance) {
            minDistance = distance;
            closestContourIndex = static_cast<int>(i);
        }
    }

    if (closestContourIndex == -1) {
        cerr << "Failed to find the closest contour." << endl;
    }

    return closestContourIndex;
}

void CalculateAverageDistances(const vector<Point>& contour, RotatedRect ellipse, double& averageDistance_1, double& averageDistance_2, Point2f point_1, Point2f point_2, Mat& contourImage)
{
    
    double totalDistance_1 = 0.0, totalDistance_2 = 0.0;
    int count_1 = 0, count_2 = 0;

    // Calculer l'angle de rotation pour aligner l'axe de l'ellipse sur l'axe des y
    //double angle = atan2(point_2.x - point_1.x, point_2.y - point_1.y) + CV_PI / 2; // Inverser x et y pour aligner sur l'axe des Y
    double angle = ellipse.angle * CV_PI / 180.0;
    double cosAngle = cos(-angle);
    double sinAngle = sin(-angle);

    for (const Point& point : contour) {
        // Appliquer la rotation au point de contour
        Point2f rotatedPoint(
            static_cast<float>(cosAngle * (point.x - ellipse.center.x) - sinAngle * (point.y - ellipse.center.y) + ellipse.center.x),
            static_cast<float>(sinAngle * (point.x - ellipse.center.x) + cosAngle * (point.y - ellipse.center.y) + ellipse.center.y)
        );

        // Tracer les points après rotation
        //circle(contourImage, rotatedPoint, 2, Scalar(0, 255, 0), -1); // Vert pour les points de contour

        // Calculer la distance entre le point de contour et le centre de l'ellipse
        double distance = norm(rotatedPoint - ellipse.center);

        // Tracer le centre de l'ellipse en violet
        circle(contourImage, ellipse.center, 5, Scalar(255, 0, 255), -1); // Violet pour le centre de l'ellipse
        // Vérifier à quel demi-ellipse appartient le point
        if (rotatedPoint.y < ellipse.center.y) {
            totalDistance_1 += distance;
            count_1++;
        }
        else {
            totalDistance_2 += distance;
            count_2++;
        }
    }

    // Calculer les distances moyennes
    averageDistance_1 = (count_1 > 0) ? totalDistance_1 / count_1 : std::numeric_limits<double>::max();
    averageDistance_2 = (count_2 > 0) ? totalDistance_2 / count_2 : std::numeric_limits<double>::max();


}


// Draw a reper axis (line through the center of the ellipse) and calculate the angle between the head and the reper axis
void DrawReperAxisAndCalculateAngle(Mat& contourImage, const RotatedRect& ellipse, Point2f& head_point, Point2f& tail_point, double& headingAngle, double& previous_headingAngle, int first_passage, double& averageDistance_1, double& averageDistance_2, Point2f& point_1, Point2f& point_2, int& alert_inversion)
{

	Point2f point_conversion;

    // Déterminer quel demi-ellipse a les points de contour les plus proches du centre   
#ifdef VERBOSE
    cout << "averageDistance_1  =  " <<  averageDistance_1 << endl;
    cout << "averageDistance_2  =  " << averageDistance_2 <<  endl;
#endif // VERBOSE

    if (averageDistance_1 < averageDistance_2) {
        head_point = point_1;
        tail_point = point_2;
    }
    else {
        head_point = point_2;
        tail_point = point_1;
    }
    // Calculate the angle between the head and reper axis. substract 90 degrees (CV_PI /2) to get the angle between the major axis and the y-axis                
    headingAngle = (atan2(tail_point.y - head_point.y, tail_point.x - head_point.x) - CV_PI / 2) * 180.0 / CV_PI; // Convert to degrees
    if (headingAngle < 0) headingAngle += 360; // Ensure the angle is positive
    // Check if we have a 90 degrees angle change between 2 capture
    if (first_passage != 0) { // At first passage, no need to check
        if (abs(previous_headingAngle - headingAngle) > 90 && alert_inversion > 0) { // We set Alert inversion alarm to avoid being stuck upside down
            headingAngle = fmod(headingAngle + 180.0, 360.0); //We invert the angle modulo 360
            point_conversion = head_point;
			head_point = tail_point;
            tail_point = point_conversion;
            alert_inversion--;
        }
    }
    if (alert_inversion >= 0) {
        alert_inversion = ALERT_INVERSION; // We reset alert inversion 
    }
    previous_headingAngle = headingAngle;

    // Draw a reper axis (line through the center of the ellipse)
    Point2f refAxisStart_y(ellipse.center.x, ellipse.center.y - 150);
    Point2f refAxisEnd_y(ellipse.center.x, ellipse.center.y + 150);
    Point2f refAxisStart_x(ellipse.center.x - 150, ellipse.center.y);
    Point2f refAxisEnd_x(ellipse.center.x + 150, ellipse.center.y);
    line(contourImage, refAxisStart_y, refAxisEnd_y, Scalar(0, 127, 255), 1); // line for reference axis
    line(contourImage, refAxisStart_x, refAxisEnd_x, Scalar(0, 127, 255), 1); // line for reference axis

    // Déterminer quel demi-ellipse a les points de contour les plus proches du centre
    //if (averageDistance_1 < averageDistance_2) {
    if (head_point == point_1) {
        putText(contourImage, "Head", point_1, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        putText(contourImage, "Tail", point_2, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
        circle(contourImage, point_1, 5, Scalar(0, 0, 255), -1); // Rouge pour la tête
        circle(contourImage, point_2, 5, Scalar(255, 0, 0), -1); // Bleu pour la queue
    }
    else {
        putText(contourImage, "Head", point_2 + Point2f(15, 0), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2); // Décale le texte de 10 pixels à droite
        putText(contourImage, "Tail", point_1 + Point2f(30, 0), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2); // Décale le texte de 10 pixels à droite
        circle(contourImage, point_2, 5, Scalar(0, 0, 255), -1); // Rouge pour la tête
        circle(contourImage, point_1, 5, Scalar(255, 0, 0), -1); // Bleu pour la queue
    }
}

// test if the folder can be created
int test_create_folder() {
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == nullptr)
    {
#ifdef VERBOSE
        cout << "Failed to create file in current folder.  Please check "
            "permissions."
            << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
#endif // VERBOSE
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");
	return 0;
}



// Function to detect wings in the tail region
bool DetectWings(const Mat& binaryImage, const Point2f& tail_point, Mat& contourImage) {
    // Define a region of interest (ROI) around the tail
    int roiSize = 50; // Size of the ROI
    Rect tailROI(static_cast<int>(tail_point.x - roiSize / 2), static_cast<int>(tail_point.y - roiSize / 2), static_cast<int>(roiSize), static_cast<int>(roiSize));

    // Ensure the ROI is within the image bounds
    tailROI &= Rect(0, 0, contourImage.cols, contourImage.rows);

    // Extract the ROI from the binary image
    Mat tailRegion = binaryImage(tailROI);

    // Find contours in the tail region
    vector<vector<Point>> tailContours;
    vector<Vec4i> tailHierarchy;
    findContours(tailRegion, tailContours, tailHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Check if significant contours are found in the tail region
    bool wingsDetected = false;
    for (const auto& contour : tailContours) {
        if (contourArea(contour) > 100) { // Adjust the threshold based on your needs
            wingsDetected = true;
            break;
        }
    }

    // Draw the ROI on the contour image
    rectangle(contourImage, tailROI, Scalar(0, 255, 255), 2); // Yellow rectangle for ROI

    // Display the result of wing detection
    ostringstream wingsText;
    wingsText << "Wings: " << (wingsDetected ? "Detected" : "Not Detected");
    putText(contourImage, wingsText.str(), Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);

    return wingsDetected;
}



// Create a new image with a width equal to the sum of the widths of the two images and a height equal to the maximum height of the two images
Mat CombineImages(const Mat& img1, const Mat& img2) {

    int rows = max(img1.rows, img2.rows);
    int cols = img1.cols + img2.cols;
    Mat converted_image;
    Mat combinedImage(rows, cols, img1.type());
    cvtColor(img2, converted_image, COLOR_GRAY2BGR);

	// Copy the first image to the left part of the combined image
    Mat left(combinedImage, Rect(0, 0, img1.cols, img1.rows));
    img1.copyTo(left);

	// Copy the second image to the right part of the combined image
    Mat right(combinedImage, Rect(img1.cols, 0, converted_image.cols, converted_image.rows));
    converted_image.copyTo(right);

    return combinedImage;
}

std::ofstream CreateCSVFile() {
    // Obtenez la date et l'heure actuelles
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    // Formatez la date et l'heure dans une chaîne de caractères
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = oss.str();

    // Créez le nom du fichier avec la date et l'heure
    std::string filename = "data_" + timestamp + ".csv";

    // Ouvrez le fichier CSV en mode écriture
    std::ofstream csvFile(filename);
    if (!csvFile.is_open()) {
        std::cerr << "Error opening CSV file." << std::endl;
        throw std::runtime_error("Failed to create CSV file");
    }

    // Écrivez les en-têtes de colonne
    csvFile << "iteration, datetime, FrameRate,HeadingAngle\n";

    return csvFile;
}




int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
    double zoomFactor = ZOOM_FACTOR; // Scale factor
    const unsigned int k_numImages = IMAGE_NUMBER;
    double headingAngle;
    double previous_headingAngle;
	double iteration = 0;
    Rect boundingBox;
    int first_passage = 0;
    int result = 0;
	Mat display_image;
    int alert_inversion = ALERT_INVERSION;
    double frame_rate = 0;
    auto start = std::chrono::high_resolution_clock::now();

	// Open csv file in wrtting mode  
    std::ofstream csvFile;
    try {
        csvFile = CreateCSVFile();
    }
    catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
  
    // ***  Set Acquisition mode for run  *** //
    if (!SetAcquisitionMode(nodeMap)){
        return -1;
    }


    // ***  Start big loop  *** //
    try
    {

        // ***  Begin acquiring images  *** //
        pCam->BeginAcquisition();
#ifdef VERBOSE
        cout << "Acquiring images..." << endl;
#endif // VERBOSE




		// ***  Retrieve device serial number for filename if save mode  *** //
        gcstring deviceSerialNumber;
        const char* folderPath = PATH_SAVE_IMG;
        if (SAVE_IMAGE == "TRUE") {
            // ***  Retrieve device serial number for filename  *** //
            gcstring deviceSerialNumber = GetDeviceSerialNumber(nodeMapTLDevice);
            // ***  Create Folder for image  *** //
            CreateFolder(folderPath);
        }




        // Loop ont image number
        for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
        {
            try
            {
                // Retrieve next received image
                ImagePtr pResultImage = pCam->GetNextImage();

                // Ensure image completion
                if (pResultImage->IsIncomplete())
                {
#ifdef VERBOSE
                    cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())<< "..." << endl << endl;
#endif // VERBOSE
                }
                else
                {
                    
                    // ***  Get the frame rate between two frames  *** //
					if (first_passage != 0) { // At first Passage, no measure
                        auto end = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> elapsed = end - start;
                        frame_rate = elapsed.count();
                        start = std::chrono::high_resolution_clock::now();
					}


                    // Print image information; height and width recorded in pixels
                    const size_t width = pResultImage->GetWidth();
                    const size_t height = pResultImage->GetHeight();
\


                    // ** DATA CONVERSION TO OPENCV FORMAT STEP ** //
                    if (pResultImage->GetHeight() > std::numeric_limits<unsigned int>::max()) {
                        throw std::overflow_error("size_t value is too large to fit in unsigned int");
                    }
                    unsigned int rows = static_cast<unsigned int>(pResultImage->GetHeight());

                    if (pResultImage->GetWidth() > std::numeric_limits<unsigned int>::max()) {
                        throw std::overflow_error("size_t value is too large to fit in unsigned int");
                    }
                    unsigned int cols = static_cast<unsigned int>(pResultImage->GetWidth());

                    if (pResultImage->GetNumChannels() > std::numeric_limits<unsigned int>::max()) {
                        throw std::overflow_error("size_t value is too large to fit in unsigned int");
                    }
                    unsigned int num_channels = static_cast<unsigned int>(pResultImage->GetNumChannels());

                    void *image_data = pResultImage->GetData();

                    if (pResultImage->GetStride() > std::numeric_limits<unsigned int>::max()) {
                        throw std::overflow_error("size_t value is too large to fit in unsigned int");
                    }
                    unsigned int stride = static_cast<unsigned int>(pResultImage->GetStride());

                    Mat current_frame = cv::Mat(rows, cols, (num_channels == 3) ? CV_8UC3 : CV_8UC1, image_data, stride);






                    // ** IMAGE ANALYSE STEP ** //
                    
                    // Zoom into the ROI
                    if (first_passage == 0) { // At first passage needed only
                        boundingBox = Get_bounding_box(current_frame, zoomFactor);
                        if (boundingBox.height < 50) {
                            boundingBox.width = 62;
                            boundingBox.x = 329;
                            boundingBox.y = 207;
                            boundingBox.height = 61;
							//boundingBox = Get_bounding_box(current_frame, zoomFactor); // Try again if the bounding box is too small
                        }
                    }
                    Mat zoomedImage = zoom_into_bouding_rect(current_frame, zoomFactor, boundingBox);
                    if (zoomedImage.empty()) return -1;

                    // Step 1: Threshold the image
                    Mat binaryImage;
                    double thresholdValue = 100; // Adjust based on image intensity
                    if (IR_FILTER != "TRUE") {
                        thresholdValue = 200;
                    }
                    threshold(zoomedImage, binaryImage, thresholdValue, 255, THRESH_BINARY);

                    // Step 2: Find contours
                    vector<vector<Point>> contours;
                    vector<Vec4i> hierarchy;
                    findContours(binaryImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                    // Step 3: Find the contour closest to the center
                    int closestContourIndex = FindClosestContour(zoomedImage, contours);

                    //If no coutour is found, we skip the frame TANT PIS
                    if (closestContourIndex != -1) {
                                         
                    // Step 4: Filter and draw the closest contour
                    Mat contourImage = Mat::zeros(zoomedImage.size(), CV_8UC3); // Create an output image
                    drawContours(contourImage, vector<vector<Point>>{contours[closestContourIndex]}, -1, Scalar(255, 255, 255), 2); // Draw in white

                    // Step 5: Fit an ellipse around the closest contour and determine head and tail sides
                    if (contours[closestContourIndex].size() >= 5) { // fitEllipse requires at least 5 points

                        try {

                        // Fit the ellipse
                        RotatedRect ellipse = fitEllipse(contours[closestContourIndex]);
                        cv::ellipse(contourImage, ellipse, Scalar(0, 255, 0), 2); // Draw the ellipse in green

                        // Calculate the heading of the fly based on the head-tail symmetry from the ellipse
                        Point2f head_point, tail_point;
                        Point2f vertices[4];
                        ellipse.points(vertices);

                        // Calculate the major axis of the ellipse
                        Point2f majorAxis = vertices[1] - vertices[0];
                        Point2f minorAxis = vertices[2] - vertices[1];

                        // Determine the sommets ellipse points
                        Point2f point_1 = ellipse.center - 0.5f * majorAxis;
                        Point2f point_2 = ellipse.center + 0.5f * majorAxis;

                        // Split the ellipse into two parts along the minor axis
                        Point2f minorAxisStart = ellipse.center - 0.5f * minorAxis;
                        Point2f minorAxisEnd = ellipse.center + 0.5f * minorAxis;

						// Calculate mean distances of contour points from the ellipse center for each half-ellipse
                        double averageDistance_1, averageDistance_2;
                        CalculateAverageDistances(contours[closestContourIndex], ellipse, averageDistance_1, averageDistance_2, point_1, point_2, contourImage);

						// Draw a reper axis and calculate the angle headingAngle between the head and the reper axis
                        DrawReperAxisAndCalculateAngle(contourImage, ellipse, head_point, tail_point, headingAngle, previous_headingAngle, first_passage, averageDistance_1, averageDistance_2, point_1, point_2, alert_inversion);

                        // Detect wings in the tail region and display the result of wing detection
                        DetectWings(binaryImage, tail_point, contourImage);
                        }
                        catch (const std::exception& e) {
                            headingAngle = -1;
                            std::cerr << "Exception: " << e.what() << std::endl;
                        }

                    }
                    
                    // Display the angle on the image and the frame rate
                    ostringstream angleText;
                    ostringstream frame_rate_draw;
                    angleText << headingAngle << " deg";
                    frame_rate_draw << frame_rate << " sec";
                    putText(contourImage, angleText.str(), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
                    putText(contourImage, frame_rate_draw.str(), Point(10, contourImage.rows - 10), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

                    // Combine contourImage and zoomedImage side by side 
                    display_image = CombineImages(contourImage, zoomedImage);


					}
					else {
						cout << "No contour found in the image." << endl;
                        display_image = CombineImages(zoomedImage, zoomedImage);
						frame_rate = -1;
						headingAngle = -1;
					}

                    // Display results
                    std::vector<uchar> encodedBuffer; // Buffer to store encoded data
                    Mat display_image_trash;
                    cv::resize(display_image, display_image_trash, cv::Size(display_image.cols, display_image.rows));
                    cv::imencode(".jpg", display_image_trash, encodedBuffer); // Encode the Mat as JPEG

                    if (PRINT_IMAGE == "TRUE") {
                        imshow("Analyse_de_la_MOUCHE", display_image);
                        waitKey(0);
                    }
#ifdef VERBOSE                 
                    cout << "Fequence =  " << frame_rate << endl;
                    cout << "Heading  =  " << headingAngle << endl;
#endif // VERBOSE


                    // ** CSV WRITING STEP ** //
                    std::string timestamp = []() {
                        auto t = std::time(nullptr);
                        auto tm = *std::localtime(&t);
                        std::ostringstream oss;
                        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
                        return oss.str();
                        }();
                    csvFile << iteration << "," << timestamp << "," << frame_rate << "," << headingAngle << "\n";

                    // ** SAVING STEP ** //
                    if (SAVE_IMAGE == "TRUE") {
                        SaveImage(PATH_SAVE_IMG, deviceSerialNumber, display_image, imageCnt);
                    }

                
}


                // ** CLOSING STEP ** //
                pResultImage->Release();
                pResultImage = nullptr;
                first_passage = 1;
				iteration++;

                cout << endl;
            }
            catch (Spinnaker::Exception &e)
            {
                cout << "Error: " << e.what() << endl;
                result = -1;
            }
        }

        // ** End acquisition ** //
        pCam->EndAcquisition();
        // Close the CSV file
        csvFile.close();


    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return result;
}





// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int run_camera()
{
    int result;
    CameraPtr pCam = nullptr;

    // Test if the folder can be created
    test_create_folder();

    // ** Get Camera List ** //
// Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();
#ifdef VERBOSE
    cout << "Number of cameras detected: " << numCameras << endl << endl;
#endif // VERBOSE
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();
#ifdef VERBOSE
        cout << "Not enough cameras! Done! Press Enter to exit..." << endl;
        getchar();
#endif // VERBOSE
        return -1;
    }


    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);
#ifdef VERBOSE
        cout << endl << "Running example for camera " << i << "..." << endl;
#endif // VERBOSE

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        INodeMap & nodeMap = pCam->GetNodeMap();

        // Acquire images
        AcquireImages(pCam, nodeMap, nodeMapTLDevice);

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

#ifdef VERBOSE
    cout << "Camera " << i << " example complete..." << endl << endl;
#endif // VERBOSE
    }

    // Release reference to the camera
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
#ifdef VERBOSE
    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();
#endif // VERBOSE

    return result;
}






int main(int /*argc*/, char** /*argv*/)
{


   // Run
   run_camera();

    return 0;
}



