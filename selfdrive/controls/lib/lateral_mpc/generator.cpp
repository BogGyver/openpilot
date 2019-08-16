#include <acado_code_generation.hpp>

#define PI 3.1415926536
#define deg2rad(d) (d/180.0*PI)

const int controlHorizon = 50;

using namespace std;

int main( )
{
  USING_NAMESPACE_ACADO


  DifferentialEquation f;

  DifferentialState xx; // x position
  DifferentialState yy; // y position
  DifferentialState psi; // vehicle heading
  DifferentialState delta;

  OnlineData curvature_factor;
  OnlineData v_ref; // m/s
  OnlineData l_poly_r0, l_poly_r1, l_poly_r2, l_poly_r3;
  OnlineData r_poly_r0, r_poly_r1, r_poly_r2, r_poly_r3;
  OnlineData d_poly_r0, d_poly_r1, d_poly_r2, d_poly_r3;
  OnlineData l_prob, r_prob;
  OnlineData lane_width;

  Control t;

  // Equations of motion
  f << dot(xx) == v_ref * cos(psi);
  f << dot(yy) == v_ref * sin(psi);
  f << dot(psi) == v_ref * delta * curvature_factor;
  f << dot(delta) == t;

  auto lr_prob = l_prob + r_prob - l_prob * r_prob;

  auto poly_l = l_poly_r0*(xx*xx*xx) + l_poly_r1*(xx*xx) + l_poly_r2*xx + l_poly_r3;
  auto poly_r = r_poly_r0*(xx*xx*xx) + r_poly_r1*(xx*xx) + r_poly_r2*xx + r_poly_r3;
  auto poly_d = d_poly_r0*(xx*xx*xx) + d_poly_r1*(xx*xx) + d_poly_r2*xx + d_poly_r3;

  auto angle_d = atan(3*d_poly_r0*xx*xx + 2*d_poly_r1*xx + d_poly_r2);

  // When the lane is not visible, use an estimate of its position
  auto weighted_left_lane = l_prob * poly_l + (1 - l_prob) * (poly_d + lane_width/2.0);
  auto weighted_right_lane = r_prob * poly_r + (1 - r_prob) * (poly_d - lane_width/2.0);

  auto c_left_lane = exp(-(weighted_left_lane - yy));
  auto c_right_lane = exp(weighted_right_lane - yy);

  // Running cost
  Function h;

  // Distance errors
  h << poly_d - yy;
  h << lr_prob * c_left_lane;
  h << lr_prob * c_right_lane;

  // Heading error
  h << (v_ref + 1.0 ) * (angle_d - psi);

  // Angular rate error
  h << (v_ref + 1.0 ) * t;

  BMatrix Q(5,5); Q.setAll(true);
  // Q(0,0) = 1.0;
  // Q(1,1) = 1.0;
  // Q(2,2) = 1.0;
  // Q(3,3) = 1.0;
  // Q(4,4) = 2.0;

  // Terminal cost
  Function hN;

  // Distance errors
  hN << poly_d - yy;
  hN << l_prob * c_left_lane;
  hN << r_prob * c_right_lane;

  // Heading errors
  hN << (2.0 * v_ref + 1.0 ) * (angle_d - psi);

  BMatrix QN(4,4); QN.setAll(true);
  // QN(0,0) = 1.0;
  // QN(1,1) = 1.0;
  // QN(2,2) = 1.0;
  // QN(3,3) = 1.0;

  // Non uniform time grid
  // First 5 timesteps are 0.05, after that it's 0.15
  DMatrix numSteps(20, 1);
  for (int i = 0; i < 5; i++){
    numSteps(i) = 1;
  }
  for (int i = 5; i < 20; i++){
    numSteps(i) = 3;
  }

  // Setup Optimal Control Problem
  const double tStart = 0.0;
  const double tEnd   = 2.5;

  OCP ocp( tStart, tEnd, numSteps);
  ocp.subjectTo(f);

  ocp.minimizeLSQ(Q, h);
  ocp.minimizeLSQEndTerm(QN, hN);

  // car can't go backward to avoid "circles"
  ocp.subjectTo( deg2rad(-90) <= psi <= deg2rad(90));
  // more than absolute max steer angle
  ocp.subjectTo( deg2rad(-50) <= delta <= deg2rad(50));
  ocp.setNOD(17);

  OCPexport mpc(ocp);
  mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
  mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
  mpc.set( INTEGRATOR_TYPE, INT_RK4 );
  mpc.set( NUM_INTEGRATOR_STEPS, 1 * controlHorizon);
  mpc.set( MAX_NUM_QP_ITERATIONS, 500);
  mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);

  mpc.set( SPARSE_QP_SOLUTION, CONDENSING );
  mpc.set( QP_SOLVER, QP_QPOASES );
  mpc.set( HOTSTART_QP, YES );
  mpc.set( GENERATE_TEST_FILE, NO);
  mpc.set( GENERATE_MAKE_FILE, NO );
  mpc.set( GENERATE_MATLAB_INTERFACE, NO );
  mpc.set( GENERATE_SIMULINK_INTERFACE, NO );

  if (mpc.exportCode( "lib_mpc_export" ) != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );

  mpc.printDimensionsQP( );

  return EXIT_SUCCESS;
}
