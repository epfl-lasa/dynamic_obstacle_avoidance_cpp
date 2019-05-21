#include "DynamicObstacleAvoidance/Utils/VectorTools.hpp"

std::vector<double> VectorTools::linspace(const double& start, const double& ed, const int& num) {
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