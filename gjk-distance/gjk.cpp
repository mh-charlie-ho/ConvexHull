#include "gjk.hpp"
#include <cmath>
#include <iostream>

using namespace std;

static Eigen::Vector2f prod(Eigen::Vector2f a, Eigen::Vector2f b)
{
    return (1 - (a.dot(b) / (a.norm() * b.norm()))) * b;
}

GJK::GJK(const ConvexHull &s1, const ConvexHull &s2)
    : mS1(s1), mS2(s2), mOrigin(0, 0)
{
}

GJK::GJK(const std::vector<cv::Point2f> &s1, const std::vector<cv::Point2f> &s2)
    : mOrigin(0, 0)
{
    mS1 = ConvertFormat(s1);
    mS2 = ConvertFormat(s2);
}

ConvexHull GJK::ConvertFormat(const std::vector<cv::Point2f> &points)
{
    ConvexHull outputConvexHull;

    float xSum = 0;
    float ySum = 0;
    for (int i = 0; i < points.size(); i++)
    {
        xSum += points[i].x;
        ySum += points[i].y;
        Eigen::Vector2f convertPt(points[i].x, points[i].y);
        outputConvexHull.vertex.push_back(convertPt);
    }

    outputConvexHull.center << xSum / points.size(), ySum / points.size();
    return outputConvexHull;
}

Eigen::Vector2f GJK::SupportFunciton(
    const ConvexHull &s, const Eigen::Vector2f &d)
{
    float tempdot = d[0] * s.vertex[0][0] + d[1] * s.vertex[0][1];
    int supportPtId = 0;
    for (int i = 1; i < s.vertex.size(); i++)
    {
        float dot = d[0] * s.vertex[i][0] + d[1] * s.vertex[i][1];
        if (tempdot < dot)
        {
            tempdot = dot;
            supportPtId = i;
        }
    }
    return s.vertex[supportPtId];
}

Eigen::Vector2f GJK::Support(const Eigen::Vector2f &d)
{
    cout << "s1: " << SupportFunciton(mS1, d)[0] << ", " << SupportFunciton(mS1, d)[1] << endl;
    cout << "s2: " << SupportFunciton(mS2, -d)[0] << ", " << SupportFunciton(mS2, -d)[1] << endl;

    return (SupportFunciton(mS1, d) - SupportFunciton(mS2, -d));
}

Eigen::Vector2f GJK::NearestVector(
    const Eigen::Vector2f &sPt,
    const Eigen::Vector2f &ePt)
{
    // the vertical vector
    Eigen::Vector2f AO(mOrigin - sPt);
    Eigen::Vector2f AB(ePt - sPt);

    if (AO.dot(AB) < 0)
    {
        return AO;
    }
    else if ((AO.dot(AB)) / AB.norm() > AB.norm())
    {
        Eigen::Vector2f BO(mOrigin - ePt);
        return BO;
    }
    else
    {
        return prod(AB, AO);
    }
}

void GJK::Remover(ConvexHull &simplex)
{
    if (simplex.vertex[0].norm() > simplex.vertex[1].norm())
    {
        simplex.vertex[0] = simplex.vertex[1];
        simplex.vertex.pop_back();
    }
    else
    {
        simplex.vertex.pop_back();
    }
}

void GJK::PrintSimplex(const ConvexHull &simplex)
{
    for (int i = 0; i < simplex.vertex.size(); i++)
    {
        printf(
            "simplex vertex (x, y): %f, %f\n",
            simplex.vertex[i][0], simplex.vertex[i][1]);
    }
}

float GJK::Distance()
{
    ConvexHull simplex;

    // initialization of the search vector
    Eigen::Vector2f d(mS2.center - mS1.center);

    simplex.vertex.push_back(Support(d)); // first pt
    PrintSimplex(simplex);

    d = mOrigin - simplex.vertex[0];
    Eigen::Vector2f p(Support(d)); // second pt

    if (p == simplex.vertex[0])
    {
        cout << "HERE" << endl;
        return d.norm();
    }
    else
    {
        simplex.vertex.push_back(p);
        PrintSimplex(simplex);
    }

    while (true)
    {
        d = NearestVector(simplex.vertex[0], simplex.vertex[1]);
        p = Support(d);

        if (p == simplex.vertex[simplex.vertex.size() - 1])
        {
            cout << "THERE" << endl;
            return d.norm();
        }
        else
        {
            Remover(simplex);
            simplex.vertex.push_back(p);
            PrintSimplex(simplex);
        }
    }
}
