#include <cmath>
#include <iostream>
#include <Eigen/Dense>

/*********************************************************************/
/****** CARTESIAN IMPEDANCE CONTROL BASED ON ELASTIC POTENTIALS ******/
/*********************************************************************/

CartImpController::CartImpController( char type, const int nq )
{
	this->type = type;
	// Assertion that type is either T (Translational) or R (Rotational)
	assert( type == 'T' || type == 'R' );

	this->nq = nq;

	this->dq = Eigen::VectorXd::Zero( this->nq, 1 );

	this->p = Eigen::VectorXd::Zero( 3, 1 );
	this->p_des = Eigen::VectorXd::Zero( 3, 1 );
	this->del_p = Eigen::VectorXd::Zero( 3, 1 );

	this->H	= Eigen::MatrixXd::Zero( 4,  4 );
	this->J	= Eigen::MatrixXd::Zero( 6,  this->nq );
	this->M	= Eigen::MatrixXd::Zero( this->nq,  this->nq );

	this->R_0_des = Eigen::MatrixXd::Zero( 3,  3 );
	this->R_0_b = Eigen::MatrixXd::Zero( 3,  3 );
	this->epsilon = Eigen::VectorXd::Zero( 3, 1 );
	this->eta = 0;	

	this->K_p = Eigen::MatrixXd::Zero( 3, 3 );
	this->K_e = Eigen::MatrixXd::Zero( 3, 3 );

	this->Zeta = 0;	
}

void CartImpController::setKinematics( const Eigen::VectorXd &dq, const Eigen::MatrixXd &H, const Eigen::MatrixXd &J, const Eigen::MatrixXd &M )
{
	this->dq = dq;
	assert( this->nq == dq.size( ) );

	this->M = M;
	assert( M.rows() == this->nq && M.cols() == this->nq );

	this->H = H;
	assert( H.rows() == 4 && H.cols() == 4 );
	this->p = H.block< 3, 1 >( 0, 3 );
	this->R_0_b = H.block< 3, 3 >( 0, 0 );

	assert( J.rows() == 6 && J.cols() == this->nq );
	switch ( this->type ) 
	{
  	case 'T':
    	this->J.topRows(3) = J;
    	break;
  	case 'R':
    	this->J.bottomRows(3) = J;
    	break;
	}

}

void CartImpController::setStiffness( const Eigen::MatrixXd &K )
{
	assert( K.rows() == 3 && K.cols() == 3 );

	switch ( this->type ) 
	{
  	case 'T':
    	this->K_p = K;
    	break;
  	case 'R':
    	this->K_e = K;
    	break;
	}

}

void CartImpController::setZFT( const Eigen::MatrixXd &x_des )
{

	switch ( this->type ) 
	{
  	case 'T':
		assert( x_des.size( ) == 3 );
    	this->p_des = x_des;
    	break;
  	case 'R':
		assert( x_des.rows() == 3 && x_des.cols() == 3 );
    	this->R_0_des = x_des;
    	break;
	}

}

Eigen::VectorXd CartImpController::getElasticWrench( )
{
	switch ( this->type ) 
	{
  	case 'T':

		assert( !this->p_des.isZero( 0 ) );
		assert( !this->K_p.isZero( 0 ) );
		assert( !this->p.isZero( 0 ) );
  
		this->del_p = this->p_des - this->p;
		Eigen::Matrix3d del_p_mat = this->vec2SkewSym( this->del_p )

		Eigen::VectorXd F_elas = Eigen::VectorXd::Zero( 6,  1 );
		F_elas.block< 3, 1 >( 0, 0 ) = this->R_0_b * this->K_p * this->R_0_b.transpose() * this->del_p;
		F_elas.block< 3, 1 >( 3, 0 ) = this->R_0_b * del_p_mat * this->K_p * this->R_0_b.transpose() * this->del_p;

    	break;

  	case 'R':

		assert( !this->R_0_des.isZero( 0 ) );
		assert( !this->K_e.isZero( 0 ) );
		assert( !this->R_0_b.isZero( 0 ) );
  
		Eigen::Matrix3d R_b_des = this->R_0_b.transpose() * this->R_0_des;
		this->epsilon = this->getQuatAxis( R_b_des );
		this->eta = this->getQuatAngle( R_b_des );	
		
		Eigen::Matrix3d eps_b_0_tilde = this->vec2SkewSym( this->epsilon )
    	Eigen::Matrix3d E = this->eta * Eigen::Matrix3d::Identity( 3, 3 ) - eps_b_0_tilde;
  
		Eigen::VectorXd F_elas = Eigen::VectorXd::Zero( 6,  1 );
		F_elas.block< 3, 1 >( 3, 0 ) = this->R_0_b * 2 * E.transpose() * this->K_e * this->epsilon;

    	break;
	}
	return F_elas;

}

void CartImpController::setDampingRatio( const double &Xi )
{
	assert( Xi.size( ) == 1 );
    this->Zeta = Xi;

}

Eigen::Matrix3d CartImpController::getDampingMatrix( )
{
	assert( !this->Zeta.isZero( 0 ) );
	assert( !this->R_0_b.isZero( 0 ) );

	Eigen::Matrix3d Kd1 = Eigen::Matrix3d::Zero( 3,  3 );
	switch ( this->type ) 
	{
  	case 'T':
	    for( int i=0; i<3; i++ )
		{
            for ( int j=0; j<3; j++ )
			{
                if( this->K_p( i, j ) < 0 )
				{
                    this->K_p( i, j ) = 0.0;
                }
                Kd1( i, j ) = sqrt( this->K_p );
            }
        }
    	break;
  	case 'R':
		for( int i=0; i<3; i++ )
		{
            for ( int j=0; j<3; j++ )
			{
                if( this->K_e( i, j ) < 0)
				{
                    this->K_e( i, j ) = 0.0;
                }
                Kd1( i, j ) = sqrt( this->K_e );
            }
        }
    	break;
	}

	Eigen::Matrix3d Lambda = this->getLambdaLeastSquares( );
	Eigen::Matrix3d Lambda_sqrt = Eigen::Matrix3d::Zero( 3,  3 );

    for( int i=0; i<3; i++ )
	{
        for ( int j=0; j<3; j++ )
		{
            if( Lambda( i, j ) < 0)
			{
                    Lambda( i, j ) = 0.0;
            }
            Lambda_sqrt( i, j ) = sqrt( Lambda( i, j ) );
        }
    }

	Eigen::Matrix3d D_zeta = this->Zeta * MatrixNd::Identity( 3, 3 );
	Eigen::Matrix3d Dx_b = Lambda_sqrt * D_zeta * Kd1 + Kd1 * D_zeta * Lambda_sqrt;
    Eigen::Matrix3d Dx_0 = this->R_0_b * Dx_b * this->R_0_b.transpose();
	return Dx_0;

}

Eigen::VectorXd CartImpController::getDampingWrench( )
{
	Eigen::Matrix3d B = this->getDampingMatrix();
	
	Eigen::VectorXd twist = this->J * this->dq;
	
	Eigen::VectorXd F_damp = B * twist;
	switch ( this->type ) 
	{
  	case 'T':
		F_damp.< 3, 1 >( 0, 0 ) = Eigen::VectorXd::Zero( 3, 1 );
    	break;
  	case 'R':
		F_damp.< 3, 1 >( 3, 0 ) = Eigen::VectorXd::Zero( 3, 1 );
    	break;
	}
	return F_damp;

}

Eigen::Matrix3d CartImpController::getLambdaLeastSquares( )
{
	k = 0.01;
	Eigen::Matrix3d J_3D = Eigen::Matrix3d::Zero( 3,  this->nq );
	switch ( this->type ) 
	{
  	case 'T':
		J_3D = this->J.topRows(3);
    	break;
  	case 'R':
		J_3D = this->J.bottomRows(3);
    	break;
	}

    Eigen::Matrix3d Lambda_Inv = J_3D * this->M.inverse() * J_3D.transpose() + ( k * k ) * MatrixNd::Identity( 3, 3 );
    Eigen::Matrix3d Lambda = Lambda_Inv.inverse();
    return Lambda;

}

Eigen::Matrix3d CartImpController::vec2SkewSym( const Eigen::Vector3d &v )
{
	Eigen::Matrix3d Mat { 
		{ 	    0, -v( 2 ),  v( 1 ) },
		{  v( 2 ),       0, -v( 0 ) },
		{ -v( 1 ),  v( 0 ),       0 }
	  };

	return Mat;

}

double CartImpController::getQuatAngle( Eigen::Matrix3d R ){
    double r11 = R( 0, 0 );
    double r12 = R( 0, 1 );
    double r13 = R( 0, 2 );
    double r21 = R( 1, 0 );
    double r22 = R( 1, 1 );
    double r23 = R( 1, 2 );
    double r31 = R( 2, 0 );
    double r32 = R( 2, 1 );
    double r33 = R( 2, 2 );
    double eta = 0;
    eta = 0.5 * sqrt(r11 + r22 + r33 + 1);
    return eta;

}

Eigen::Vector3d CartImpController::getQuatAxis( Eigen::Matrix3d R ){
    double r11 = R( 0, 0 );
    double r12 = R( 0, 1 );
    double r13 = R( 0, 2 );
    double r21 = R( 1, 0 );
    double r22 = R( 1, 1 );
    double r23 = R( 1, 2 );
    double r31 = R( 2, 0 );
    double r32 = R( 2, 1 );
    double r33 = R( 2, 2 );

    Eigen::Vector3d epsilon = Eigen::Vector3d::Zero( 3, 1 );
    double r32_23 = r32 - r23;
    if( r32_23 > 0 )
	{
        r32_23 = 1;
    }
    if( r32_23 == 0 )
	{
        r32_23 = 0;
    }
    if( r32_23 < 0 )
	{
        r32_23 = -1;
    }
    double r13_31 = r13 - r31;
    if( r13_31 > 0 )
	{
        r13_31 = 1;
    }
    if( r13_31 == 0 )
	{
        r13_31 = 0;
    }
    if( r13_31 < 0 )
	{
        r13_31 = -1;
    }
    double r21_12 = r21 - r12;
    if( r21_12 > 0 )
	{
        r21_12 = 1;
    }
    if( r21_12 == 0 )
	{
        r21_12 = 0;
    }
    if( r21_12 < 0 )
	{
        r21_12 = -1;
    }

    if( sqrt(r11 - r22 - r33 + 1 ) >= 0 )
	{
        epsilon( 0 ) = 0.5 * r32_23 * sqrt( r11 - r22 - r33 + 1 );
    }else{
        epsilon( 0 ) = 0;
    }
    if( sqrt(-r11 + r22 - r33 + 1) >= 0 )
	{
        epsilon( 1 ) = 0.5 * r13_31 * sqrt( -r11 + r22 - r33 + 1 );
    }else{
        epsilon( 1 ) = 0;
    }
    if( sqrt( -r11 - r22 + r33 + 1 ) >= 0 )
	{
        epsilon( 2 ) = 0.5 * r21_12 * sqrt( -r11 - r22 + r33 + 1 );
    }else{
        epsilon( 2 ) = 0;
    }
    return epsilon;

}







