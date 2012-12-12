#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <opencv.hpp>
#include <unistd.h>

using namespace cv;
using namespace std;

int main()
{
    //Création de la fenêtre graphique
    namedWindow("win", CV_WINDOW_KEEPRATIO);
    resizeWindow("win", 800, 600);

    //Ouverture de la video
    char* videoname = (char*)"../../media/video-tribot.mp4";
    VideoCapture video(videoname);

    //Extraction des frames de la video
    cout << "Loading video..." << endl;
    vector<Mat> frames;
    Mat frame;
    video >> frame;
    while (frame.data != NULL) {
        frames.push_back(frame.clone());
        cout << frames.size() << endl;
        video >> frame;
    }
    cout << "Frames count : " << frames.size() << endl;

    //Affichage des images de la video
    for (size_t i=0;i<frames.size();i++) {
        imshow("win", frames[i]);
        waitKey(5);
    }

    //Libération de la fenêtre
    destroyWindow("win");
    return EXIT_SUCCESS;
}

