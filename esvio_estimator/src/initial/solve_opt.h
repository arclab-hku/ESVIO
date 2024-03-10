#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "initial_sfm.h"
using namespace Eigen;
using namespace std;

class OptSolver 
{
public:
	OptSolver();

	// optimize to solve [R, t]
	bool solveCeres(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rji, Vector3d &tv);

	// entrance function, estimate [R,t] given intial R 
	bool solveHybrid(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rji, Vector3d &tv);

	// only solve t in [R; t]
	bool solveTCeres(const vector<pair<Vector3d, Vector3d>> &corres, const Matrix3d &Rji, Vector3d &tji);

	// camera pose 
	double m_tji[1][3]; 
	double m_qji[1][4]; 

	double para_T[1][3]; // for translation 
	double para_s[1][0]; // for scale 

	double para_pt[1000][3]; 

	int feature_num;
};