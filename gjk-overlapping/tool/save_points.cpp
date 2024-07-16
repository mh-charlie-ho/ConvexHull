#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/core.hpp>
using namespace std;

int Writetxt(float x, float y, string filename = "output.txt")
{
    ofstream ofs(filename, ios::app);

    if (!ofs.is_open())
    {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }

    ofs << x << " " << y << "\n";
    ofs.close();
    return 0;
}

int Readtxt(string fileName, vector<cv::Point2f> &pts)
{
    ifstream file(fileName);
    if (!file.is_open())
    {
        cerr << "Failed to open " << fileName << std::endl;
        return 1;
    }

    string line;
    while (getline(file, line))
    {
        std::stringstream ss(line);
        double num1, num2;
        if (ss >> num1 >> num2)
        {
            cv::Point2f p;
            p.x = num1;
            p.y = num2;
            pts.push_back(p);
        }
        else
        {
            std::cerr << "Failed to analyze: " << line << std::endl;
        }
    }
    file.close();

    return 0;
}