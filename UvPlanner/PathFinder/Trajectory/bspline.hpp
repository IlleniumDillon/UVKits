#include <iostream>
#include <vector>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;

int binomial(int n, int i);
Vector3d bezier_curve(const vector<Vector3d>& Vector3ds, float t);
