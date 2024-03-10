#pragma once


#include <ceres/ceres.h>
#include <Eigen/Dense>


class TranslateFactor : public ceres::SizedCostFunction<2, 3>
{
public:
	TranslateFactor(const Eigen::Matrix3d& R, const Eigen::Vector3d& xi, const Eigen::Vector2d& xnj, 
			const Eigen::Matrix3d& cov_xi);
	virtual bool Evaluate(double const* const *params, double *residuals, double**jacobians) const; 

	Eigen::Vector3d pt_xi; 
	Eigen::Vector2d pt_xnj; 
	Eigen::Matrix3d Rji; 
	Eigen::Matrix3d cov_pt_xi; 
	// Eigen::Matrix2d cov_pt_xnj; 
	
};

class TranslateScaleFactor : public ceres::SizedCostFunction<2, 1>
{
public:
	TranslateScaleFactor(const Eigen::Matrix3d& R, const Eigen::Vector3d& nt, const Eigen::Vector3d& xi, const Eigen::Vector2d& xnj, 
			const Eigen::Matrix3d& cov_xi);
	virtual bool Evaluate(double const* const *params, double *residuals, double**jacobians) const; 

	Eigen::Vector3d pt_xi; 
	Eigen::Vector2d pt_xnj; 
	Eigen::Matrix3d Rji; 
	Eigen::Vector3d ntji; // normalized tji 
	Eigen::Matrix3d cov_pt_xi; 
};

class TranslateWithPtFactor : public ceres::SizedCostFunction<2, 3, 3>
{
public:
	TranslateWithPtFactor(const Eigen::Matrix3d& R, const Eigen::Vector2d& xnj, 
			const Eigen::Matrix3d& cov_xi);
	virtual bool Evaluate(double const* const *params, double *residuals, double**jacobians) const; 

	Eigen::Vector2d pt_xnj; 
	Eigen::Matrix3d Rji; 
	Eigen::Matrix3d cov_pt_xi; 
};

class SinglePtFactor : public ceres::SizedCostFunction<3, 3>
{
public:
	SinglePtFactor(const Eigen::Vector3d& pt, const Eigen::Matrix3d& cov); 

	virtual bool Evaluate(double const* const *params, double *residuals, double**jacobians) const; 

	Eigen::Vector3d pt_xi; 
	Eigen::Matrix3d inf_pt_xi; 
};