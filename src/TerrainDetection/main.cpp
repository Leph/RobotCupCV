#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <opencv.hpp>
#include <highgui/highgui.hpp>
#include <unistd.h>

using namespace cv;
using namespace std;

//Variables globales
int event_mode = 0;
vector<Vec3b> points;
Mat* current_frame = NULL;

//Gestionnaire Sourie
void onMouseCallback(int event, int x, int y, int flags, void* data)
{
    (void) flags;
    (void) data;
    if (event == CV_EVENT_LBUTTONDOWN && event_mode == 1 && current_frame != NULL) {
        points.push_back(current_frame->at<Vec3b>(y,x));
    }
    if (event == CV_EVENT_LBUTTONDOWN && event_mode == 2 && current_frame != NULL) {
        cout << "[" << x << "," << y << "] " << (int)current_frame->at<uchar>(y,x) << endl;
    }
}

//Détection de contour avec couleur
void colorEdgeDetector(Mat& in)
{
    Mat out = in.clone();
    if (in.channels() > 1) {
        return;
    }
    Size s = in.size();
    for (long i=0;i<s.width;i++) {
        for (long j=0;j<s.height;j++) {
            
        }
    }
}

int main()
{
    size_t maxFrame = 500;

    //Création de la fenêtre graphique
    namedWindow("win", CV_WINDOW_KEEPRATIO);
    cvResizeWindow("win", 800, 600);
    namedWindow("win2", CV_WINDOW_KEEPRATIO);

    //Gestionnaire de la sourie
    event_mode = 0;
    setMouseCallback("win", onMouseCallback);

    //Ouverture de la video
    char* videoname = (char*)"../../media/video-bruno.mp4";
    //char* videoname = (char*)"../../media/video-tribot.mp4";
    VideoCapture video(videoname);

    //Extraction des frames de la video
    cout << "Loading video..." << endl;
    vector<Mat> frames;
    Mat frame;
    video >> frame;
    while (frame.data != NULL && frames.size() < maxFrame) {
        frames.push_back(frame.clone());
        video >> frame;
        cout << "." << flush;
    }
    cout << endl << "Frames count : " << frames.size() << endl;

    //Test extraction du terrain
    /*
    event_mode = 1;
    points.clear();
    for (size_t i=0;i<frames.size();i++) {
        Mat tmp = frames[i].clone();
        //cvtColor(tmp, tmp, CV_BGR2HLS);
        current_frame = &tmp;
        imshow("win", tmp);
        waitKey();
        current_frame = NULL;
    }
    event_mode = 0;
    Vec3b min(255, 255, 255);
    Vec3b max(0, 0, 0);
    Vec3i moy(0, 0, 0);
    for (size_t j=0;j<points.size();j++) {
        if (points[j][0] < min[0]) min[0] = points[j][0];
        if (points[j][0] > max[0]) max[0] = points[j][0];
        if (points[j][1] < min[1]) min[1] = points[j][1];
        if (points[j][1] > max[1]) max[1] = points[j][1];
        if (points[j][2] < min[2]) min[2] = points[j][2];
        if (points[j][2] > max[2]) max[2] = points[j][2];
        moy += points[j];
    }
    moy[0] = moy[0] / points.size();
    moy[1] = moy[1] / points.size();
    moy[2] = moy[2] / points.size();
    cout << "min " << (int)min[0] << " " << (int)min[1] << " " << (int)min[2] << endl;
    cout << "max " << (int)max[0] << " " << (int)max[1] << " " << (int)max[2] << endl;
    cout << "moy " << (int)moy[0] << " " << (int)moy[1] << " " << (int)moy[2] << endl;
    */
   
    /*
    event_mode = 2;
    for (size_t i=0;i<frames.size();i++) {
        Mat imgBGR = frames[i].clone();
        Mat imgHLS = frames[i].clone();
        cvtColor(imgBGR, imgHLS, CV_BGR2HLS);
        vector<Mat> imgHLS_channels;
        split(imgHLS, imgHLS_channels);
        
        current_frame = &(imgHLS_channels[2]);
        imshow("win", imgHLS_channels[2]);
        imshow("win2", imgHLS);
        waitKey();
    }
    event_mode = 0;
    return 0;
    */

    Vec3b minLine(6, 50, 13);
    Vec3b maxLine(185, 217, 212);
    Vec3b minTerrain(47, 17, 200);
    Vec3b maxTerrain(62, 52, 255);
    for (size_t i=0;i<frames.size();i++) {
        Mat imgBGR = frames[i].clone();
        Mat imgHLS = frames[i].clone();
        cvtColor(imgBGR, imgHLS, CV_BGR2HLS);

        vector<Mat> imgBGR_channels;
        split(imgBGR, imgBGR_channels);
        vector<Mat> imgHLS_channels;
        split(imgHLS, imgHLS_channels);

        Mat edges = imgHLS_channels[1];
        GaussianBlur(edges, edges, Size(5,5), 10.0);
        Canny(edges, edges, 7.0, 30.0, 3, true);

        inRange(imgBGR, Scalar(minLine[0], minLine[1], minLine[2]), Scalar(maxLine[0], maxLine[1], maxLine[2]), imgBGR);
        inRange(imgHLS, Scalar(minTerrain[0], minTerrain[1], minTerrain[2]), Scalar(maxTerrain[0], maxTerrain[1], maxTerrain[2]), imgHLS);

        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(10,10));
        dilate(imgBGR, imgBGR, kernel);
        dilate(imgHLS, imgHLS, kernel);

        edges = edges & imgBGR & imgHLS;
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1.0, CV_PI/180, 80, 20, 8);
        Mat terrain = Mat::zeros(imgBGR.rows, imgBGR.cols, CV_8UC3);
        for(size_t j=0;j<lines.size();j++) {
            line(terrain, Point(lines[j][0], lines[j][1]),
                Point(lines[j][2], lines[j][3]), Scalar(0,0,255), 1);
        }

        imshow("win", terrain);
        imshow("win2", edges);
        waitKey(10);
    }
    
    //Libération de la fenêtre
    destroyWindow("win");
    destroyWindow("win2");
    return EXIT_SUCCESS;
}

