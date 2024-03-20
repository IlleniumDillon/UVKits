#include "rrt.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main()
{
    int map_x = 4,map_y = 6;

    UV::RRT solver;
    solver.SetMap(1,map_x,map_y,1,Eigen::Vector3d(0.0,0.0,0.0));
    solver.mapData = new uint8_t[map_x * map_y]{0};
    solver.mapData[0*map_y + 2] = 1;
    solver.mapData[1*map_y + 2] = 1;
    solver.mapData[2*map_y + 2] = 1;
    solver.mapData[2*map_y + 3] = 1;
    solver.mapData[2*map_y + 4] = 1;
    solver.ResetRRT(Eigen::Vector3d(0.5,0.5,0.0),Eigen::Vector3d(1.5,3.5,0.0));
    auto path = solver.Planning();

    int n = path.size();
    std::vector<double> x(n), y(n);
    for(int i=0; i<n; ++i) {
        x.at(i) = path[i].x();
        y.at(i) = path[i].y();
    }
 
    // 设置输出图像的大小为1200x780像素
    // plt::figure_size(1200, 780);
    // 绘制给定x和y数据的折线图，颜色自动选择
    plt::plot(x, y);


    return 0;
}