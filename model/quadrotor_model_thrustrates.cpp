/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
    const bool CODE_GEN = true;
  // const bool CODE_GEN = false;

  // System variables
  DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  DifferentialState     w_x, w_y, w_z;
  Control               T_1, T_2, T_3, T_4;
  // Control               T, w_x, w_y, w_z;
  DifferentialEquation  f;
  Function              h, hN;
  // OnlineData            p_F_x, p_F_y, p_F_z;
  // OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  // OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double T_min = 2 / 4.0;         // Minimal thrust [N]
  const double T_max = 20 / 4.0;        // Maximal thrust [N]

  // Bias to prevent division by zero.
  const double epsilon = 0.1;     // Camera projection recover bias [m]

  // Intertia
  DMatrix I_uav(3, 3);
  DMatrix I_uav_inv(3, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j){
      I_uav(i, j) = 0.0;
      I_uav_inv(i, j) = 0.0;
    }
  I_uav(0, 0) = 0.007;
  I_uav(1, 1) = 0.007;
  I_uav(2, 2) = 0.012;
  I_uav_inv(0, 0) = 1.0 / 0.007;
  I_uav_inv(1, 1) = 1.0 / 0.007;
  I_uav_inv(2, 2) = 1.0 / 0.012;
  const double moment_constant = 0.016;
  const double arm_length = 0.17;
  double uav_mass = 0.68;


  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * (T_1 + T_2 + T_3 + T_4);
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * (T_1 + T_2 + T_3 + T_4);
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * (T_1 + T_2 + T_3 + T_4) - g_z;
  IntermediateState torque[3];
  torque[0] = uav_mass * (arm_length * T_2 - arm_length * T_4);
  torque[1] = uav_mass * (-arm_length * T_1 + arm_length * T_3);
  torque[2] = uav_mass * (moment_constant * T_1 - moment_constant * T_2
                          + moment_constant * T_3 - moment_constant * T_4);
  IntermediateState Iw[3];
  Iw[0] = I_uav(0, 0) * w_x + I_uav(0, 1) * w_y + I_uav(0, 2) * w_z;
  Iw[1] = I_uav(1, 0) * w_x + I_uav(1, 1) * w_y + I_uav(1, 2) * w_z;
  Iw[2] = I_uav(2, 0) * w_x + I_uav(2, 1) * w_y + I_uav(2, 2) * w_z;
  IntermediateState wxIw[3];
  wxIw[0] = w_y * Iw[2] - w_z * Iw[1];
  wxIw[1] = w_z * Iw[0] - w_x * Iw[2];
  wxIw[2] = w_x * Iw[1] - w_y * Iw[0];
  f << dot(w_x) == I_uav_inv(0, 0) * (torque[0] - wxIw[0])
    + I_uav_inv(0, 1) * (torque[1] - wxIw[1])
    + I_uav_inv(0, 2) * (torque[2] - wxIw[2]);
  f << dot(w_y) == I_uav_inv(1, 0) * (torque[0] - wxIw[0])
    + I_uav_inv(1, 1) * (torque[1] - wxIw[1])
    + I_uav_inv(1, 2) * (torque[2] - wxIw[2]);
  f << dot(w_z) == I_uav_inv(2, 0) * (torque[0] - wxIw[0])
    + I_uav_inv(2, 1) * (torque[1] - wxIw[1])
    + I_uav_inv(2, 2) * (torque[2] - wxIw[2]);

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << w_x << w_y << w_z
    << T_1 << T_2 << T_3 << T_4;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
     << w_x << w_y << w_z;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 100;   // x
  Q(1,1) = 100;   // y
  Q(2,2) = 100;   // z
  Q(3,3) = 100;   // qw
  Q(4,4) = 100;   // qx
  Q(5,5) = 100;   // qy
  Q(6,6) = 100;   // qz
  Q(7,7) = 10;   // vx
  Q(8,8) = 10;   // vy
  Q(9,9) = 10;   // vz
  Q(10,10) = 1;   // wx
  Q(11,11) = 1;   // wy
  Q(12,12) = 1;   // wz
  Q(13,13) = 1;   // T_1
  Q(14,14) = 1;   // T_2
  Q(15,15) = 1;   // T_3
  Q(16,16) = 1;   // T_4

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = Q(0,0);   // x
  QN(1,1) = Q(1,1);   // y
  QN(2,2) = Q(2,2);   // z
  QN(3,3) = Q(3,3);   // qw
  QN(4,4) = Q(4,4);   // qx
  QN(5,5) = Q(5,5);   // qy
  QN(6,6) = Q(6,6);   // qz
  QN(7,7) = Q(7,7);   // vx
  QN(8,8) = Q(8,8);   // vy
  QN(9,9) = Q(9,9);   // vz
  QN(10,10) = Q(10,10);   // wx
  QN(11,11) = Q(11,11);   // wy
  QN(12,12) = Q(12,12);   // wz

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(0) = 2.0;
  r(3) = 1.0;

  DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(3) = r(3);


  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );
  if(!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );
  }else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
  }

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo( T_min <= T_1 <= T_max);
  ocp.subjectTo( T_min <= T_2 <= T_max);
  ocp.subjectTo( T_min <= T_3 <= T_max);
  ocp.subjectTo( T_min <= T_4 <= T_max);

  // ocp.setNOD(10);


  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, p_x ==  0.0 );
    ocp.subjectTo( AT_START, p_y ==  0.0 );
    ocp.subjectTo( AT_START, p_z ==  0.0 );
    ocp.subjectTo( AT_START, q_w ==  1.0 );
    ocp.subjectTo( AT_START, q_x ==  0.0 );
    ocp.subjectTo( AT_START, q_y ==  0.0 );
    ocp.subjectTo( AT_START, q_z ==  0.0 );
    ocp.subjectTo( AT_START, v_x ==  0.0 );
    ocp.subjectTo( AT_START, v_y ==  0.0 );
    ocp.subjectTo( AT_START, v_z ==  0.0 );
    ocp.subjectTo( AT_START, w_x ==  0.0 );
    ocp.subjectTo( AT_START, w_y ==  0.0 );
    ocp.subjectTo( AT_START, w_z ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( p_x,"position x" );
    window1.addSubplot( p_y,"position y" );
    window1.addSubplot( p_z,"position z" );
    window1.addSubplot( v_x,"verlocity x" );
    window1.addSubplot( v_y,"verlocity y" );
    window1.addSubplot( v_z,"verlocity z" );
    window1.addSubplot( w_x,"rotation-acc x" );
    window1.addSubplot( w_y,"rotation-acc y" );
    window1.addSubplot( w_z,"rotation-acc z" ); 

    GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
    window3.addSubplot( T_1,"Thrust 1" );
    window3.addSubplot( T_2,"Thrust 2" );
    window3.addSubplot( T_3,"Thrust 3" );
    window3.addSubplot( T_4,"Thrust 4" );


    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );
    algorithm << window1;
    algorithm << window3;
    algorithm.solve();

  }else{
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,   N);
    mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,            YES);
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}
