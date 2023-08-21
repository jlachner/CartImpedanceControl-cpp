/*
 * A Cartesian Impedance Controller based on Elastic Potentials.
 *
 * Copyright (c) 2023
 * Authors: Johannes Lachner  	<jlachner@mit.edu>	
 */


#include <Eigen/Dense>

/*
	@brief A class a Cartesian Impedance Controller
*/
class CartImpController
{
	public:

		char type; 
		const int nq;

		Eigen::VectorXd dq;

		Eigen::VectorXd p;
		Eigen::VectorXd p_des;
		Eigen::VectorXd del_p;

		Eigen::MatrixXd H;
		Eigen::MatrixXd J;
		Eigen::MatrixXd M;

		Eigen::MatrixXd R_0_des;
		Eigen::MatrixXd R_0_b;
		Eigen::VectorXd epsilon;
		double eta;

		Eigen::MatrixXd K_p;
		Eigen::MatrixXd K_e;

		double Zeta;

	public:

		/* 
			Default Constructor
		*/
		CartImpController( ) {};

		/*
			Constructor
		*/
		CartImpController( const int type, const int nq );

		void setKinematics( const Eigen::VectorXd &dq, const Eigen::MatrixXd &H, const Eigen::MatrixXd &J, const Eigen::MatrixXd &M );
		void setStiffness( const Eigen::MatrixXd &K );
		void setZFT( const Eigen::MatrixXd &x_des );
		void setDampingRatio( const double &Xi );

		Eigen::VectorXd getElasticWrench( );	
		Eigen::Matrix3d getDampingMatrix( );
		Eigen::VectorXd getDampingWrench( );
		Eigen::Matrix3d getLambdaLeastSquares( );
		Eigen::Matrix3d vec2SkewSym( const Eigen::Vector3d &v );
		double getQuatAngle( Eigen::Matrix3d R );
		Eigen::Vector3d getQuatAxis( Eigen::Matrix3d R );

};
