#include "bspline.hpp"

// 计算组合数
int binomial(int n, int i) 
{
    int res = 1;
    for (int j = 1; j <= i; ++j) {
        res *= (n - j + 1) / (double)j; //(double)十分关键，不然j=i=n时，j为分数=0；
    }
    return res;
}

// 计算n次贝塞尔曲线上的点
Vector3d bezier_curve(const vector<Vector3d>& Vector3ds, float t) 
{
    t = t<0 ? 0 : t>1 ? 1 : t;
    int n = Vector3ds.size() - 1;
    Vector3d res(0,0,0);
    for (int i = 0; i <= n; ++i) {
        // cout << "p:" << Vector3ds[i].x() << "," << Vector3ds[i].y() << endl;
        float b =  binomial(n, i)* pow(t, i) * pow(1 - t, n - i);
        // cout << "bino=" << binomial(n, i) << endl;
        res.x() = res.x() + Vector3ds[i].x() * b;
        res.y() = res.y() + Vector3ds[i].y() * b;
        res.z() = res.z() + Vector3ds[i].z() * b;
    }
    return res;
}
