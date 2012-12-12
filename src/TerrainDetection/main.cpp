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
    size_t maxFrame = 150;

    //Création de la fenêtre graphique
    namedWindow("win", CV_WINDOW_KEEPRATIO);
    resizeWindow("win", 800, 600);

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
        cout << frames.size() << endl;
        video >> frame;
    }
    cout << "Frames count : " << frames.size() << endl;

    //Affichage des images de la video
    /*
    for (size_t i=0;i<frames.size();i++) {
        imshow("win", frames[i]);
        waitKey(5);
    }
    */

    //Extraction des lignes
    for (size_t i=0;i<frames.size();i++) {
        Mat tmp = frames[i].clone();
        cvtColor(tmp, tmp, CV_BGR2HLS);
        vector<Mat> tmp_channels;
        split(tmp, tmp_channels);
        tmp = tmp_channels[1];
        imshow("win", tmp);
        waitKey(50);
    }

    //Extraction du terrain
    for (size_t i=0;i<frames.size();i++) {
        Mat tmp = frames[i].clone();
        //Sépare les cannaux RGB pour en
        //extraire le vert
        vector<Mat> tmp_channels;
        split(tmp, tmp_channels);
        tmp = tmp_channels[1] - (0.5*tmp_channels[0]+0.5*tmp_channels[2]);
        //Calcul de l'histograme pour fixer
        //la valeur du seuil
        /*
        Mat hist;
        const int channels[] = {0};
        const int histSize[] = {256};
        const float range[] = {0, 256};
        const float* ranges[] = {range};
        calcHist((const Mat*)(&tmp), 1, channels, Mat(), hist, 1, histSize, ranges);
        unsigned long long sum = 0;
        unsigned long long count = 0;
        for (size_t i=0;i<256;i++) {
            sum += i * hist.at<unsigned long>(i, 0);
            count += hist.at<unsigned long>(i, 0);
            cout << i << " : " << hist.at<unsigned long>(i, 0) << endl;
        }
        assert(count > 0);
        unsigned long long mean = sum/count;
        cout << sum << " " << count << " -> " << mean << endl;
        */
        //GaussianBlur(tmp, tmp, Size(21, 21), 1.0);
        //threshold(tmp, tmp, 40, 255, THRESH_BINARY);
        imshow("win", tmp);
        waitKey(50);
    }

    //Libération de la fenêtre
    destroyWindow("win");
    return EXIT_SUCCESS;
}

