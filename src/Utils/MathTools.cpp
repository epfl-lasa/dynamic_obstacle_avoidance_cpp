#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"

std::vector<double> MathTools::linspace(const double& start, const double& ed, const int& num) {
    // catch rarely, throw often
    if (num < 2) {
        throw new std::exception();
    }
    int partitions = num - 1;
    std::vector<double> pts;
    // length of each segment    
    double length = (ed - start) / partitions; 
    // first, not to change
    pts.push_back(start);
    for (int i = 1; i < num - 1; i ++) {
        pts.push_back(start + i * length);
    }
    // last, not to change
    pts.push_back(ed);
    return pts;
}

double MathTools::rand_float(const double& a, const double& b) 
{
    return ((double)rand() / RAND_MAX) * (a - b) + b;
}
