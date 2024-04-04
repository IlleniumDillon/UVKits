#include "bspline.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() 
{

    vector<Vector3d> points = {{10,10,0}, {20, 80,0},{40,50,0}}; // 贝塞尔曲线控制点，给定控制点的数量决定贝塞尔曲线的阶数
    
    // cout << "size:" << points.size() << endl;
    std::vector<double> x, y;
    const float step = 0.02; // 步长
    for (float t = 0; t <= 1; t += step) {
        Vector3d p = bezier_curve(points, t);
        x.push_back(p.x());
        y.push_back(p.y());
        // cout << p.x() << ", " << p.y() <<", " <<p.z()<< endl;
    }
    plt::plot(x,y);
    plt::grid(true);
    plt::show();

    return 0;
}