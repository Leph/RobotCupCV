#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <opencv.hpp>
#include <highgui/highgui.hpp>
#include <unistd.h>
#include <cmath>
#include <assert.h>

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
}

//Renvoi la distance entière entre les deux pixels
int distance(long x1, long y1, long x2, long y2)
{
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return dx > dy ? dx : dy;
}

//Renvoi la distance d'orientation entre deux orientations
double directionDist(double dir1, double dir2)
{
    assert(dir1 >= 0.0 && dir1 <= CV_PI);
    assert(dir2 >= 0.0 && dir2 <= CV_PI);
    double dist = abs(dir1-dir2);
    if (dist < CV_PI/2.0) {
        return dist;
    } else {
        return CV_PI-dist;
    }
}

//Renvoi la direction entre deux point
double direction(double x1, double y1, double x2, double y2)
{
    Vec2i dir(0, 0);
    if (y1 > y2) {
        dir[0] = x1-x2;
        dir[1] = y1-y2;
    } else {
        dir[0] = x2-x1;
        dir[1] = y2-y1;
    }
    return atan2((double)dir[1], (double)dir[0]);
}

//Renvoi la direction d'un point de la ligne entre 0 et PI
double directionPath(vector<Vec2i> path, size_t index, int radius)
{
    Vec2i pos = path[index];
    Vec2i dir(0, 0);
    int count = 0;
    for (size_t i=0;i<path.size();i++) {
        Vec2i pos2 = path[i];
        int dist = distance(pos[0], pos[1], pos2[0], pos2[1]);
        if (i != index && dist <= radius && dist > 1) {
            int coef = dist;
            if (pos[1] > pos2[1]) {
                dir[0] += coef*(pos[0]-pos2[0]);
                dir[1] += coef*(pos[1]-pos2[1]);
            } else {
                dir[0] += coef*(pos2[0]-pos[0]);
                dir[1] += coef*(pos2[1]-pos[1]);
            }
            count += coef;
        }
    }
    assert(count > 0);
    double x = (double)dir[0]/(double)count;
    double y = (double)dir[1]/(double)count;
    
    double val = atan2(y, x);
    assert(val >= 0.0 && val <= CV_PI);
    return val;
}

//Détecte et créer les lignes de contours
void buildTerrainPaths(Mat& img, vector< vector<Vec2i> >& paths, int radius, int minNb)
{
    assert(img.channels() == 1);
    assert(paths.empty());

    //Initialisation
    Mat tmp = img.clone();
    Size s = img.size();
    for (long i=0;i<s.width;i++) {
        for (long j=0;j<s.height;j++) {
            uchar val = tmp.at<uchar>(j,i);
            if (val > 0) {
                tmp.at<uchar>(j,i) = 1;
            }
        }
    }

    int continued = 1;
    while (continued == 1) {
        continued = 0;
        int founded = 0;
        for (long i=0;i<s.width && !founded;i++) {
            for (long j=0;j<s.height && !founded;j++) {
                uchar val = tmp.at<uchar>(j,i);
                if (val == 1) {
                    tmp.at<uchar>(j,i) = 2;
                    paths.push_back(vector<Vec2i>());
                    founded = 1;
                    continued = 1;
                }
            }
        }
        founded = 1;
        while (founded == 1) {
            founded = 0;
            for (long i=0;i<s.width;i++) {
                for (long j=0;j<s.height;j++) {
                    uchar val = tmp.at<uchar>(j,i);
                    if (val == 2) {
                        tmp.at<uchar>(j,i) = 0;
                        paths[paths.size()-1].push_back(Vec2i(i,j));
                        founded = 1;
                        for (long k=i-radius;k<=i+radius;k++) {
                            for (long l=j-radius;l<=j+radius;l++) {
                                if (k>=0 && l>=0 && k<s.width && l<s.height && tmp.at<uchar>(l,k) == 1) {
                                    tmp.at<uchar>(l,k) = 2;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    continued = 1;
    while (continued == 1) {
        continued = 0;
        for (size_t i=0;i<paths.size();i++) {
            if (paths[i].size() < (unsigned int)minNb) {
                paths.erase(paths.begin()+i);
                continued = 1;
            }
        }
    }
}

//Dessine les lignes
Mat drawTerrainPaths(Size size, vector< vector<Vec2i> >& paths)
{
    Mat img = Mat::zeros(size, CV_8UC3);
    for (size_t i=0;i<paths.size();i++) {
        Vec3b color(rand()%256, rand()%256, rand()%256);
        for (size_t j=0;j<paths[i].size();j++) {
            img.at<Vec3b>(paths[i][j][1], paths[i][j][0]) = color;
        }
    }

    return img;
}

//Filtre des lignes
void filterTerrainPaths(vector< vector<Vec2i> >& paths, double threshold1, double threshold2, double threshold3)
{
    vector< vector<double> > directions;
    vector< vector<double> > distances;
    for (size_t i=0;i<paths.size();i++) {
        directions.push_back(vector<double>());
        for (size_t j=0;j<paths[i].size();j++) {
            double dir = directionPath(paths[i], j, 3);
            directions[i].push_back(dir);
        }
    }
    for (size_t i=0;i<paths.size();i++) {
        distances.push_back(vector<double>());
        for (size_t j=0;j<paths[i].size();j++) {
            double dist = 1000000;
            for (size_t k=0;k<paths.size();k++) {
                for (size_t l=0;l<paths[k].size();l++) {
                    double tmp_dist = distance(paths[i][j][0], paths[i][j][1], paths[k][l][0], paths[k][l][1]);
                    double tmp_dir = direction(paths[i][j][0], paths[i][j][1], paths[k][l][0], paths[k][l][1]);
                    double align = directionDist(directions[i][j], directions[k][l]);
                    double cross = CV_PI/2.0 - directionDist(directions[i][j], tmp_dir);
                    if (tmp_dist < dist && tmp_dist > 3 && align < threshold1 && cross < threshold2) {
                        dist = tmp_dist;
                    }
                }
            }
            distances[i].push_back(dist);
        }
    }
   
    int founded = 1;
    while (founded == 1) {
        founded = 0;
        for (size_t i=0;i<paths.size();i++) {
            for (size_t j=0;j<paths[i].size();j++) {
                if (distances[i][j] > threshold3) {
                    paths[i].erase(paths[i].begin()+j);
                    founded = 1;
                }
            }
        }
    }
}

//Renvoi true si les deux lignes sont collinéaire
bool isCollinear(Vec4i& l1, Vec4i& l2)
{
    Vec2i v1(l1[2]-l1[0], l1[3]-l1[1]);
    Vec2i v2(l2[2]-l2[0], l2[3]-l2[1]);
    Vec2i v3(l2[0]-l1[0], l2[1]-l1[1]);
    double cos1 = (v1[0]*v2[0]+v1[1]*v2[1])/(sqrt(v1[0]*v1[0]+v1[1]*v1[1])*sqrt(v2[0]*v2[0]+v2[1]*v2[1]));
    double cos2 = (v1[0]*v3[0]+v1[1]*v3[1])/(sqrt(v1[0]*v1[0]+v1[1]*v1[1])*sqrt(v3[0]*v3[0]+v3[1]*v3[1]));

    return (abs(cos1) >= 0.999 && abs(cos2) >= 0.9999);
}

//Traite les lignes renvoyés par HoughLinesP
void linesFilter(Mat& img, vector<Vec4i>& lines)
{
    vector< vector<Vec4i> > groups;
    for (size_t i=0;i<lines.size();i++) {
        int founded = 0;
        for (size_t k=0;k<groups.size();k++) { 
            if (isCollinear(lines[i], groups[k][0])) {
                groups[k].push_back(lines[i]);
                founded = 1;
                break;
            }
        }
        if (founded == 0) {
            groups.push_back(vector<Vec4i>());
            groups[groups.size()-1].push_back(lines[i]);
        }
    }
    
    for (size_t i=0;i<groups.size();i++) {
        Scalar color(rand()%256, rand()%256, rand()%256);
        for(size_t j=0;j<groups[i].size();j++) {
            line(img, Point(groups[i][j][0], groups[i][j][1]),
                Point(groups[i][j][2], groups[i][j][3]), color, 1);
        }

        long min_x = 1000;
        long max_x = 0;
        long min_y;
        long max_y;
        for (size_t j=0;j<groups[i].size();j++) {
            if (groups[i][j][0] < min_x) {
                min_x = groups[i][j][0];
                min_y = groups[i][j][1];
            }
            if (groups[i][j][0] > max_x) {
                max_x = groups[i][j][0];
                max_y = groups[i][j][1];
            }
            if (groups[i][j][2] < min_x) {
                min_x = groups[i][j][2];
                min_y = groups[i][j][3];
            }
            if (groups[i][j][2] > max_x) {
                max_x = groups[i][j][2];
                max_y = groups[i][j][3];
            }
        }
        Vec4i line;
        line[0] = min_x;
        line[1] = min_y;
        line[2] = max_x;
        line[3] = max_y;
        //lines.push_back(line); 
    }
}

int main(int argc, char* argv[])
{
    int main_mode = 1;
    if (argc > 1) {
        if (argv[1][0] == '1') {
            main_mode = 1;
        } else if (argv[1][0] == '2') {
            main_mode = 2;
        } else if (argv[1][0] == '3') {
            main_mode = 3;
        } else if (argv[1][0] == '4') {
            main_mode = 4;
        } else if (argv[1][0] == '5') {
            main_mode = 5;
        } else if (argv[1][0] == '6') {
            main_mode = 6;
        } else {
            cout << "Bad usage" << endl;
            exit(1);
        }
    }

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

    //Sélection de la couleur des zones à détecter
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

        if (main_mode == 1) {
            imshow("win", imgHLS);
            imshow("win2", imgBGR);
            waitKey(10);
            continue;
        }

        Mat edges = imgHLS_channels[1].clone();
        GaussianBlur(edges, edges, Size(5,5), 10.0);
        Canny(edges, edges, 7.0, 30.0, 3, true);

        inRange(imgBGR, Scalar(minLine[0], minLine[1], minLine[2]), Scalar(maxLine[0], maxLine[1], maxLine[2]), imgBGR);
        inRange(imgHLS, Scalar(minTerrain[0], minTerrain[1], minTerrain[2]), Scalar(maxTerrain[0], maxTerrain[1], maxTerrain[2]), imgHLS);

        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(10,10));
        dilate(imgBGR, imgBGR, kernel);
        dilate(imgHLS, imgHLS, kernel);

        if (main_mode == 2) {
            imshow("win", imgHLS);
            imshow("win2", imgBGR);
            waitKey(10);
            continue;
        }

        if (main_mode == 3) {
            imshow("win", edges);
            imshow("win2", imgHLS_channels[1]);
            waitKey(10);
            continue;
        }

        edges = edges & imgBGR & imgHLS;

        if (main_mode == 4) {
            imshow("win", edges);
            imshow("win2", imgBGR & imgHLS);
            waitKey(10);
            continue;
        }

        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1.0, CV_PI/180, 80, 20, 8);
        Mat terrain = Mat::zeros(imgBGR.rows, imgBGR.cols, CV_8UC3);
        //linesFilter(terrain, lines);
        for(size_t j=0;j<lines.size();j++) {
            line(terrain, Point(lines[j][0], lines[j][1]),
                Point(lines[j][2], lines[j][3]), Scalar(0,0,255), 1);
        }

        if (main_mode == 5) {
            imshow("win", terrain);
            imshow("win2", edges);
            waitKey(10);
            continue;
        }
        
        vector< vector<Vec2i> > paths;
        buildTerrainPaths(edges, paths, 1, 20);
        filterTerrainPaths(paths, CV_PI/6.0, CV_PI/6.0, 100);
        Mat terrainPaths = drawTerrainPaths(edges.size(), paths);

        if (main_mode == 6) {
            imshow("win", terrainPaths);
            imshow("win2", edges);
            waitKey(10);
            continue;
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

