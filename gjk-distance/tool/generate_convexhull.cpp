#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core.hpp>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include <random>

static float GenerateRandom(float mean = 0, float variance = 3.0)
{
    std::random_device rd;
    std::mt19937 generator(rd());

    std::normal_distribution<float> norm(mean, variance);

    return norm(generator);
}

std::vector<cv::Point2f> GenerateHull(float xMean,float yMean, int num = 20)
{
    std::vector<cv::Point2f> pointSet;

    for (int i = 0; i < num; i++)
    {
        cv::Point2f p;
        p.x = GenerateRandom(xMean);
        p.y = GenerateRandom(yMean);

        pointSet.push_back(p);
    }

    std::vector<cv::Point2f> hull;
    cv::convexHull(pointSet, hull);

    return hull;
}
