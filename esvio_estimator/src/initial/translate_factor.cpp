#include "translate_factor.h"
using namespace Eigen; 

SinglePtFactor::SinglePtFactor(const Eigen::Vector3d& pt, const Eigen::Matrix3d& C):
	pt_xi(pt)
{
	Matrix3d INF = C.inverse(); 
	LLT<Matrix3d> lltOfA(INF); // compute the Cholesky decomposition of A
	inf_pt_xi = lltOfA.matrixL(); // retrieve factor L  in the decomposition
	// The previous two lines can also be written as "L = A.llt().matrixL()"
}

bool SinglePtFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	Eigen::Vector3d pt(parameters[0][0], parameters[0][1], parameters[0][2]); 
	Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);

	residual = pt - pt_xi; 
	residual = inf_pt_xi * residual; 

	if(jacobians){
		if(jacobians[0]){
			Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_t(jacobians[0]); 
			jacobian_t = inf_pt_xi;
		}
	}
	return true; 
}


TranslateWithPtFactor::TranslateWithPtFactor(const Eigen::Matrix3d& R, const Eigen::Vector2d& xnj, 
			const Eigen::Matrix3d& cov_xi):
			pt_xnj(xnj), Rji(R), cov_pt_xi(cov_xi)
{}

bool TranslateWithPtFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	Eigen::Vector3d T(parameters[0][0], parameters[0][1], parameters[0][2]); 
	Eigen::Vector3d pt_xi(parameters[1][0], parameters[1][1], parameters[1][2]); 
	Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residuals); 

	Eigen::Matrix<double, 1, 3> R1 = Rji.row(0); 
	Eigen::Matrix<double, 1, 3> R2 = Rji.row(1); 
	Eigen::Matrix<double, 1, 3> R3 = Rji.row(2); 

	Eigen::Matrix<double, 2, 3> A; 
	A << 1, 0, -pt_xnj(0),
		 0, 1, -pt_xnj(1); 
	Eigen::Matrix<double, 2, 1> b; 
	Eigen::Matrix<double, 1, 3> dr_dx = (R1 - pt_xnj(0)*R3);
	Eigen::Matrix<double, 1, 3> dr_dy = (R2 - pt_xnj(1)*R3); 
	b(0) = (pt_xnj(0)*R3 - R1)*pt_xi; 
	b(1) = (pt_xnj(1)*R3 - R2)*pt_xi;

	// weight 
	Eigen::Matrix2d W = Eigen::Matrix2d::Identity(); 
	W(0,0) = 1./sqrt(dr_dx * cov_pt_xi * dr_dx.transpose()); 
	W(1,1) = 1./sqrt(dr_dy * cov_pt_xi * dr_dy.transpose()); 

	residual = A*T - b; 
	residual = W * residual;

	if(jacobians){
		if(jacobians[0]){
			Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_t(jacobians[0]); 

			jacobian_t = W*A;
		}
		if(jacobians[1]){
			Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_t(jacobians[1]);

			jacobian_t.row(0) = dr_dx; 
			jacobian_t.row(1) = dr_dy; 
			jacobian_t = W * jacobian_t; 
		}
	}

	return true; 
}

TranslateFactor::TranslateFactor(const Eigen::Matrix3d& R, const Eigen::Vector3d& xi, const Eigen::Vector2d& xnj, 
			const Eigen::Matrix3d& cov_xi):
pt_xi(xi), pt_xnj(xnj), Rji(R), cov_pt_xi(cov_xi)
{}

bool TranslateFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	Eigen::Vector3d T(parameters[0][0], parameters[0][1], parameters[0][2]); 
	Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residuals); 

	Eigen::Matrix<double, 1, 3> R1 = Rji.row(0); 
	Eigen::Matrix<double, 1, 3> R2 = Rji.row(1); 
	Eigen::Matrix<double, 1, 3> R3 = Rji.row(2); 

	Eigen::Matrix<double, 2, 3> A; 
	A << 1, 0, -pt_xnj(0),
		 0, 1, -pt_xnj(1); 
	Eigen::Matrix<double, 2, 1> b; 
	Eigen::Matrix<double, 1, 3> dr_dx = (R1 - pt_xnj(0)*R3);
	Eigen::Matrix<double, 1, 3> dr_dy = (R2 - pt_xnj(1)*R3); 
	b(0) = (pt_xnj(0)*R3 - R1)*pt_xi; 
	b(1) = (pt_xnj(1)*R3 - R2)*pt_xi;

	// weight 
	Eigen::Matrix2d W = Eigen::Matrix2d::Identity(); 
	W(0,0) = 1./sqrt(dr_dx * cov_pt_xi * dr_dx.transpose()); 
	W(1,1) = 1./sqrt(dr_dy * cov_pt_xi * dr_dy.transpose()); 

	residual = A*T - b; 
	residual = W * residual;

	if(jacobians){
		if(jacobians[0]){
			Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_t(jacobians[0]); 

			jacobian_t = W*A;

		}
	}

	return true; 
}


TranslateScaleFactor::TranslateScaleFactor(const Eigen::Matrix3d& R, const Eigen::Vector3d& nt, const Eigen::Vector3d& xi, const Eigen::Vector2d& xnj, 
			const Eigen::Matrix3d& cov_xi):
pt_xi(xi), pt_xnj(xnj), Rji(R), ntji(nt), cov_pt_xi(cov_xi)
{}

bool TranslateScaleFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
	double scale = parameters[0][0]; 
	Eigen::Vector3d T = scale * ntji; 
	Eigen::Map<Eigen::Matrix<double, 2, 1>> residual(residuals); 

	Eigen::Matrix<double, 1, 3> R1 = Rji.row(0); 
	Eigen::Matrix<double, 1, 3> R2 = Rji.row(1); 
	Eigen::Matrix<double, 1, 3> R3 = Rji.row(2); 

	Eigen::Matrix<double, 2, 3> A; 
	A << 1, 0, -pt_xnj(0),
		 0, 1, -pt_xnj(1); 
	Eigen::Matrix<double, 2, 1> b; 
	Eigen::Matrix<double, 1, 3> dr_dx = (R1 - pt_xnj(0)*R3);
	Eigen::Matrix<double, 1, 3> dr_dy = (R2 - pt_xnj(1)*R3); 
	b(0) = (pt_xnj(0)*R3 - R1)*pt_xi; 
	b(1) = (pt_xnj(1)*R3 - R2)*pt_xi;

	// weight 
	Eigen::Matrix2d W = Eigen::Matrix2d::Identity(); 
	W(0,0) = 1./sqrt(dr_dx * cov_pt_xi * dr_dx.transpose()); 
	W(1,1) = 1./sqrt(dr_dy * cov_pt_xi * dr_dy.transpose()); 

	residual = A*T - b; 
	residual = W * residual;

	if(jacobians){
		if(jacobians[0]){
			Eigen::Map<Eigen::Matrix<double, 2, 1>> jacobian_t(jacobians[0]); 

			jacobian_t = W*A*ntji;

		}
	}

	return true; 
}