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
    // p.x = -5.9881;  p.y = 2.98947; s1.push_back(p);
    // p.x = -8.76046; p.y = 10.272;  s1.push_back(p);
    // p.x = -10.2307; p.y = 10.2049; s1.push_back(p);
    // p.x = -13.9854; p.y = 6.26672; s1.push_back(p);
    // p.x = -12.742;  p.y = 3.05959; s1.push_back(p);
    // p.x = -11.9017; p.y = 2.84836; s1.push_back(p);

    // std::vector<cv::Point2f> s2;
    // p.x = 17.9686; p.y = 7.28632;  s2.push_back(p);
    // p.x = 15.8011; p.y = 11.5456;  s2.push_back(p);
    // p.x = 15.3843; p.y = 11.7826;  s2.push_back(p);
    // p.x = 8.9584;  p.y = 8.72296;  s2.push_back(p);
    // p.x = 11.5057; p.y = 1.72528;  s2.push_back(p);
    // p.x = 13.3438; p.y = 0.685664; s2.push_back(p); 
    
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
    std::cout << TestGjk.CollisionCheck() << std::endl;

    return 0;
}