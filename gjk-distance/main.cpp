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