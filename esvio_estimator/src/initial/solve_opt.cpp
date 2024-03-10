#include "solve_opt.h"
#include "translate_factor.h"
#include "../feature_manager.h"

OptSolver::OptSolver(){}

	// notice here corres must be inliers 
bool OptSolver::solveHybrid(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rji, Vector3d &tji)
{
	// first solve for initial translation 
	solveTCeres(corres, Rji, tji); 
	
	// TODO:  remove outliers? 

	// optimization the result 
	solveCeres(corres, Rji, tji); 
	return true; 
}

bool OptSolver::solveTCeres(const vector<pair<Vector3d, Vector3d>> &corres, const Matrix3d &Rji, Vector3d &tji)
{
	para_T[0][0] = tji(0); 
	para_T[0][1] = tji(1); 
	para_T[0][2] = tji(2); 

  	ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::CauchyLoss(1.0);    
    loss_function = new ceres::HuberLoss(1.0);

    problem.AddParameterBlock(para_T[0], 3); 

 
    Matrix3d cov_pti = Matrix3d::Identity(); 

    double ceres_sum_err = 0; 
    for(int i=0; i<corres.size(); i++){

    	Vector3d pti = corres[i].first; 
    	Vector3d ptj = corres[i].second; 

    	pti.x() *= pti.z(); 
    	pti.y() *= pti.z();
    	Vector2d ptj_n(ptj(0), ptj(1)); 

    	double depth = pti.z(); 
    	double weight = (depth - 1)*(depth-1)/8.1; 

    	Matrix3d cov_pti_ext = cov_pti*weight; 

    	TranslateFactor *f = new TranslateFactor(Rji, pti, ptj_n, cov_pti_ext); 

    	ceres::ResidualBlockId fid = problem.AddResidualBlock(f, loss_function, para_T[0]); 
    }

    // cout<<"ini_sum_err: "<<ini_sum_err<<" ceres_sum_err: "<<ceres_sum_err<<endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 20;
    // options.minimizer_progress_to_stdout = true;
    // options.max_solver_time_in_seconds = SOLVER_TIME; 
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;

    // Vector3d tji(para_T[0][0], para_T[0][1], para_T[0][2]); 
    tji = Vector3d(para_T[0][0], para_T[0][1], para_T[0][2]);
	return true ; 

}

bool OptSolver::solveCeres(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rji, Vector3d &tji)
{
	Quaterniond qji(Rji); 

	m_tji[0][0] = tji.x(); 
	m_tji[0][1] = tji.y(); 
	m_tji[0][2] = tji.z(); 

	m_qji[0][0] = qji.w(); 
	m_qji[0][1] = qji.x(); 
	m_qji[0][2] = qji.y(); 
	m_qji[0][3] = qji.z(); 
 	
 
	//full BA
	ceres::Problem problem;
	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
	problem.AddParameterBlock(m_qji[0], 4, local_parameterization);
	problem.AddParameterBlock(m_tji[0], 3);

	int N = corres.size(); 
	for(int i=0; i<N; i++){
		Vector3d pti = corres[i].first; 
		pti.x() *= pti.z(); 
    	pti.y() *= pti.z();
    	para_pt[i][0] = pti.x(); 
    	para_pt[i][1] = pti.y(); 
    	para_pt[i][2] = pti.z(); 
    	problem.AddParameterBlock(para_pt[i], 3);
    	problem.SetParameterBlockConstant(para_pt[i]);
	}


	for(int i=0; i<corres.size(); i++){


		ceres::CostFunction* cost_function = ReprojectionError3D::Create(
											corres[i].second.x(),
											corres[i].second.y());
		problem.AddResidualBlock(cost_function, NULL, m_qji[0], m_tji[0], para_pt[i]); 

	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	// options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = 0.2;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	// cout << summary.BriefReport() << endl;

	qji.w() = m_qji[0][0]; 
	qji.x() = m_qji[0][1]; 
	qji.y() = m_qji[0][2]; 
	qji.z() = m_qji[0][3]; 
	Rji = qji.toRotationMatrix(); 

	tji.x() = m_tji[0][0]; 
	tji.y() = m_tji[0][1]; 
	tji.z() = m_tji[0][2]; 
	
	return true; 
}
