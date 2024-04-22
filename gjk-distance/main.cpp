#include "gjk.hpp"
#include "tool/generate_convexhull.cpp"
#include "tool/save_points.cpp"

#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <x1> <y1> <x1> <y1>" << std::endl;
        return 1;
    }

    std::vector<cv::Point2f> s1 = GenerateHull(std::atof(argv[1]), std::atof(argv[2]));
    std::vector<cv::Point2f> s2 = GenerateHull(std::atof(argv[3]), std::atof(argv[4]));

    // std::vector<cv::Point2f> s1;
    // cv::Point2f p; 
    // p.x = -4.33304; p.y = 6.12644; s1.push_back(p); 
    // p.x = -5.13327; p.y = 9.21342; s1.push_back(p); 
    // p.x = -12.3496; p.y = 6.6309;  s1.push_back(p); 
    // p.x = -13.9803; p.y = 5.25541; s1.push_back(p); 
    // p.x = -13.5747; p.y = 1.5156;  s1.push_back(p); 
    // p.x = -9.76489; p.y = 1.64857; s1.push_back(p); 
    // p.x = -7.48916; p.y = 2.36117; s1.push_back(p); 

    // std::vector<cv::Point2f> s2;
    // p.x = 17.7267; p.y = -5.72086; s2.push_back(p); 
    // p.x = 10.2123; p.y = -1.58326; s2.push_back(p); 
    // p.x = 5.71667; p.y = -2.27288; s2.push_back(p); 
    // p.x = 12.2122; p.y = -7.12906; s2.push_back(p); 


    printf("s1 num: %i\n", s1.size());
    for (int i = 0; i < s1.size(); i++)
    {
        Writetxt(s1[i].x, s1[i].y, "s1_hull.txt");
    }

    printf("s2 num: %i\n", s2.size());
    for (int i = 0; i < s2.size(); i++)
    {
        Writetxt(s2[i].x, s2[i].y, "s2_hull.txt");
    }

    GJK TestGjk(s1, s2);
    std::cout << TestGjk.Distance() << std::endl;

    return 0;
}