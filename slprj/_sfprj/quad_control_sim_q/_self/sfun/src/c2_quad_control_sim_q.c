/* Include files */

#include <stddef.h>
#include "blas.h"
#include "quad_control_sim_q_sfun.h"
#include "c2_quad_control_sim_q.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "quad_control_sim_q_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[75] = { "m", "g", "l", "Jxx", "Jyy",
  "Jzz", "k1", "k2", "k3", "k4", "b1", "b2", "b3", "b4", "d", "w1", "w2", "w3",
  "w4", "x", "x_dot", "y", "y_dot", "z", "z_dot", "q0", "q0_dot", "q1", "q1_dot",
  "q2", "q2_dot", "q3", "q3_dot", "upphi_x", "upphi_y", "upphi_z", "T_e2E",
  "force_mult_1", "force_mult_2", "force_mult_1_E", "force_mult_2_E", "dx1_dq0",
  "dx2_dq0", "upphi_q0", "dx1_dq1", "dx2_dq1", "upphi_q1", "dx1_dq2", "dx2_dq2",
  "upphi_q2", "dx1_dq3", "dx2_dq3", "upphi_q3", "delta1", "delta2", "delta3",
  "delta4", "delta5", "delta6", "delta7", "q_ddot", "nargin", "nargout",
  "states", "params", "w", "x_ddot", "y_ddot", "z_ddot", "q0_ddot", "q1_ddot",
  "q2_ddot", "q3_ddot", "quaternion_multipliers", "quaternion_deltas" };

/* Function Declarations */
static void initialize_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void initialize_params_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void enable_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void disable_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct *
  chartInstance);
static void c2_update_debugger_state_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void set_sim_state_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void sf_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void c2_chartstep_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void initSimStructsc2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void registerMessagesc2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_quaternion_deltas, const char_T
  *c2_identifier, real_T c2_y[5]);
static void c2_b_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[5]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_quaternion_multipliers, const char_T
  *c2_identifier, real_T c2_y[20]);
static void c2_d_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[20]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_e_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_q3_ddot, const char_T *c2_identifier);
static real_T c2_f_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_g_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_i_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[209]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[209]);
static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[209]);
static void c2_d_info_helper(c2_ResolvedFunctionInfo c2_info[209]);
static real_T c2_mpower(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_a);
static void c2_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static real_T c2_sqrt(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c2_x);
static void c2_eml_error(SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static real_T c2_dot(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
                     real_T c2_a[3], real_T c2_b[3]);
static void c2_c_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void c2_mldivide(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_A[20], real_T c2_B[5], real_T c2_Y[4]);
static void c2_d_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void c2_eps(SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static real_T c2_eml_xnrm2(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_x[20], int32_T c2_ix0);
static void c2_realmin(SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static void c2_check_forloop_overflow_error
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance, boolean_T c2_overflow);
static int32_T c2_eml_ixamax(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T c2_x[4], int32_T c2_ix0);
static void c2_eml_xswap(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_x[20], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[20]);
static void c2_below_threshold(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);
static void c2_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T c2_alpha1, real_T c2_x[20], int32_T
  c2_ix0, real_T *c2_b_alpha1, real_T c2_b_x[20], real_T *c2_tau);
static real_T c2_b_eml_xnrm2(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T c2_x[20], int32_T c2_ix0);
static void c2_eml_xscal(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_a, real_T c2_x[20], int32_T c2_ix0, real_T c2_b_x[20]);
static void c2_b_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, real_T c2_alpha1, real_T c2_x, real_T *c2_b_alpha1, real_T
  *c2_b_x, real_T *c2_tau);
static void c2_c_eml_xnrm2(SFc2_quad_control_sim_qInstanceStruct *chartInstance);
static real_T c2_b_eml_xscal(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, real_T c2_x);
static void c2_eml_matlab_zlarf(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_iv0, real_T c2_tau,
  real_T c2_C[20], int32_T c2_ic0, real_T c2_work[4], real_T c2_b_C[20], real_T
  c2_b_work[4]);
static void c2_eml_xgemv(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_A[20], int32_T c2_ia0, real_T c2_x[20],
  int32_T c2_ix0, real_T c2_y[4], real_T c2_b_y[4]);
static void c2_eml_xger(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, real_T c2_y[4],
  real_T c2_A[20], int32_T c2_ia0, real_T c2_b_A[20]);
static void c2_eml_warning(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_varargin_2, char_T c2_varargin_3[14]);
static void c2_j_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_sprintf, const char_T *c2_identifier, char_T
  c2_y[14]);
static void c2_k_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  char_T c2_y[14]);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_l_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_m_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_quad_control_sim_q, const
  char_T *c2_identifier);
static uint8_T c2_n_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sqrt(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T *c2_x);
static void c2_b_eml_xswap(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_x[20], int32_T c2_ix0, int32_T c2_iy0);
static real_T c2_c_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T *c2_alpha1, real_T c2_x[20], int32_T
  c2_ix0);
static void c2_c_eml_xscal(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_a, real_T c2_x[20], int32_T c2_ix0);
static real_T c2_d_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, real_T *c2_alpha1, real_T *c2_x);
static void c2_d_eml_xscal(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T *c2_x);
static void c2_b_eml_matlab_zlarf(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_iv0, real_T c2_tau,
  real_T c2_C[20], int32_T c2_ic0, real_T c2_work[4]);
static void c2_b_eml_xgemv(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_A[20], int32_T c2_ia0, real_T c2_x[20],
  int32_T c2_ix0, real_T c2_y[4]);
static void c2_b_eml_xger(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, real_T c2_y[4],
  real_T c2_A[20], int32_T c2_ia0);
static void init_dsm_address_info(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_quad_control_sim_q = 0U;
}

static void initialize_params_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void enable_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct *
  chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  int32_T c2_i0;
  real_T c2_e_u[5];
  const mxArray *c2_f_y = NULL;
  int32_T c2_i1;
  real_T c2_f_u[20];
  const mxArray *c2_g_y = NULL;
  real_T c2_e_hoistedGlobal;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_f_hoistedGlobal;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_g_hoistedGlobal;
  real_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  uint8_T c2_h_hoistedGlobal;
  uint8_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  real_T *c2_q0_ddot;
  real_T *c2_q1_ddot;
  real_T *c2_q2_ddot;
  real_T *c2_q3_ddot;
  real_T *c2_x_ddot;
  real_T *c2_y_ddot;
  real_T *c2_z_ddot;
  real_T (*c2_quaternion_multipliers)[20];
  real_T (*c2_quaternion_deltas)[5];
  c2_quaternion_deltas = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S,
    9);
  c2_quaternion_multipliers = (real_T (*)[20])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c2_q3_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_q2_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_q1_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_q0_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_z_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_y_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(10), FALSE);
  c2_hoistedGlobal = *c2_q0_ddot;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_q1_ddot;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *c2_q2_ddot;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = *c2_q3_ddot;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  for (c2_i0 = 0; c2_i0 < 5; c2_i0++) {
    c2_e_u[c2_i0] = (*c2_quaternion_deltas)[c2_i0];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", c2_e_u, 0, 0U, 1U, 0U, 1, 5), FALSE);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  for (c2_i1 = 0; c2_i1 < 20; c2_i1++) {
    c2_f_u[c2_i1] = (*c2_quaternion_multipliers)[c2_i1];
  }

  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", c2_f_u, 0, 0U, 1U, 0U, 2, 5, 4),
                FALSE);
  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_e_hoistedGlobal = *c2_x_ddot;
  c2_g_u = c2_e_hoistedGlobal;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_f_hoistedGlobal = *c2_y_ddot;
  c2_h_u = c2_f_hoistedGlobal;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_g_hoistedGlobal = *c2_z_ddot;
  c2_i_u = c2_g_hoistedGlobal;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 8, c2_j_y);
  c2_h_hoistedGlobal = chartInstance->c2_is_active_c2_quad_control_sim_q;
  c2_j_u = c2_h_hoistedGlobal;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 9, c2_k_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[5];
  int32_T c2_i2;
  real_T c2_dv1[20];
  int32_T c2_i3;
  real_T *c2_q0_ddot;
  real_T *c2_q1_ddot;
  real_T *c2_q2_ddot;
  real_T *c2_q3_ddot;
  real_T *c2_x_ddot;
  real_T *c2_y_ddot;
  real_T *c2_z_ddot;
  real_T (*c2_quaternion_deltas)[5];
  real_T (*c2_quaternion_multipliers)[20];
  c2_quaternion_deltas = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S,
    9);
  c2_quaternion_multipliers = (real_T (*)[20])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c2_q3_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_q2_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_q1_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_q0_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_z_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_y_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_q0_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 0)), "q0_ddot");
  *c2_q1_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 1)), "q1_ddot");
  *c2_q2_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 2)), "q2_ddot");
  *c2_q3_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 3)), "q3_ddot");
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
                      "quaternion_deltas", c2_dv0);
  for (c2_i2 = 0; c2_i2 < 5; c2_i2++) {
    (*c2_quaternion_deltas)[c2_i2] = c2_dv0[c2_i2];
  }

  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 5)),
                        "quaternion_multipliers", c2_dv1);
  for (c2_i3 = 0; c2_i3 < 20; c2_i3++) {
    (*c2_quaternion_multipliers)[c2_i3] = c2_dv1[c2_i3];
  }

  *c2_x_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 6)), "x_ddot");
  *c2_y_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 7)), "y_ddot");
  *c2_z_ddot = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 8)), "z_ddot");
  chartInstance->c2_is_active_c2_quad_control_sim_q = c2_m_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 9)),
     "is_active_c2_quad_control_sim_q");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_quad_control_sim_q(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static void sf_c2_quad_control_sim_q(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  real_T *c2_x_ddot;
  real_T *c2_y_ddot;
  real_T *c2_z_ddot;
  real_T *c2_q0_ddot;
  real_T *c2_q1_ddot;
  real_T *c2_q2_ddot;
  real_T *c2_q3_ddot;
  real_T (*c2_quaternion_deltas)[5];
  real_T (*c2_quaternion_multipliers)[20];
  real_T (*c2_w)[4];
  real_T (*c2_params)[15];
  real_T (*c2_states)[14];
  c2_quaternion_deltas = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S,
    9);
  c2_quaternion_multipliers = (real_T (*)[20])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c2_w = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 2);
  c2_params = (real_T (*)[15])ssGetInputPortSignal(chartInstance->S, 1);
  c2_q3_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_q2_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_q1_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_q0_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_z_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_y_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_states = (real_T (*)[14])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i4 = 0; c2_i4 < 14; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_states)[c2_i4], 0U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_x_ddot, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_y_ddot, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_z_ddot, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_q0_ddot, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_q1_ddot, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_q2_ddot, 6U);
  _SFD_DATA_RANGE_CHECK(*c2_q3_ddot, 7U);
  for (c2_i5 = 0; c2_i5 < 15; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*c2_params)[c2_i5], 8U);
  }

  for (c2_i6 = 0; c2_i6 < 4; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((*c2_w)[c2_i6], 9U);
  }

  for (c2_i7 = 0; c2_i7 < 20; c2_i7++) {
    _SFD_DATA_RANGE_CHECK((*c2_quaternion_multipliers)[c2_i7], 10U);
  }

  for (c2_i8 = 0; c2_i8 < 5; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((*c2_quaternion_deltas)[c2_i8], 11U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_quad_control_sim_q(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_quad_control_sim_qMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
  int32_T c2_i9;
  real_T c2_states[14];
  int32_T c2_i10;
  real_T c2_params[15];
  int32_T c2_i11;
  real_T c2_w[4];
  uint32_T c2_debug_family_var_map[75];
  real_T c2_m;
  real_T c2_g;
  real_T c2_l;
  real_T c2_Jxx;
  real_T c2_Jyy;
  real_T c2_Jzz;
  real_T c2_k1;
  real_T c2_k2;
  real_T c2_k3;
  real_T c2_k4;
  real_T c2_b1;
  real_T c2_b2;
  real_T c2_b3;
  real_T c2_b4;
  real_T c2_d;
  real_T c2_w1;
  real_T c2_w2;
  real_T c2_w3;
  real_T c2_w4;
  real_T c2_x;
  real_T c2_x_dot;
  real_T c2_y;
  real_T c2_y_dot;
  real_T c2_z;
  real_T c2_z_dot;
  real_T c2_q0;
  real_T c2_q0_dot;
  real_T c2_q1;
  real_T c2_q1_dot;
  real_T c2_q2;
  real_T c2_q2_dot;
  real_T c2_q3;
  real_T c2_q3_dot;
  real_T c2_upphi_x;
  real_T c2_upphi_y;
  real_T c2_upphi_z;
  real_T c2_T_e2E[9];
  real_T c2_force_mult_1[3];
  real_T c2_force_mult_2[3];
  real_T c2_force_mult_1_E[3];
  real_T c2_force_mult_2_E[3];
  real_T c2_dx1_dq0[3];
  real_T c2_dx2_dq0[3];
  real_T c2_upphi_q0;
  real_T c2_dx1_dq1[3];
  real_T c2_dx2_dq1[3];
  real_T c2_upphi_q1;
  real_T c2_dx1_dq2[3];
  real_T c2_dx2_dq2[3];
  real_T c2_upphi_q2;
  real_T c2_dx1_dq3[3];
  real_T c2_dx2_dq3[3];
  real_T c2_upphi_q3;
  real_T c2_delta1;
  real_T c2_delta2;
  real_T c2_delta3;
  real_T c2_delta4;
  real_T c2_delta5;
  real_T c2_delta6;
  real_T c2_delta7;
  real_T c2_q_ddot[4];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 9.0;
  real_T c2_x_ddot;
  real_T c2_y_ddot;
  real_T c2_z_ddot;
  real_T c2_q0_ddot;
  real_T c2_q1_ddot;
  real_T c2_q2_ddot;
  real_T c2_q3_ddot;
  real_T c2_quaternion_multipliers[20];
  real_T c2_quaternion_deltas[5];
  real_T c2_a;
  real_T c2_b;
  real_T c2_b_y;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_c_y;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_d_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_e_y;
  real_T c2_e_b;
  real_T c2_f_y;
  real_T c2_e_a;
  real_T c2_f_b;
  real_T c2_g_y;
  real_T c2_f_a;
  real_T c2_g_b;
  real_T c2_h_y;
  real_T c2_g_a;
  real_T c2_h_b;
  real_T c2_i_y;
  real_T c2_h_a;
  real_T c2_i_b;
  real_T c2_j_y;
  real_T c2_i_a;
  real_T c2_j_b;
  real_T c2_k_y;
  real_T c2_j_a;
  real_T c2_k_b;
  real_T c2_l_y;
  real_T c2_k_a;
  real_T c2_l_b;
  real_T c2_m_y;
  real_T c2_m_b;
  real_T c2_n_y;
  real_T c2_l_a;
  real_T c2_n_b;
  real_T c2_o_y;
  real_T c2_m_a;
  real_T c2_o_b;
  real_T c2_p_y;
  real_T c2_n_a;
  real_T c2_p_b;
  real_T c2_q_y;
  real_T c2_o_a;
  real_T c2_q_b;
  real_T c2_r_y;
  real_T c2_p_a;
  real_T c2_r_b;
  real_T c2_s_y;
  real_T c2_s_b;
  real_T c2_t_y;
  real_T c2_q_a;
  real_T c2_t_b;
  real_T c2_u_y;
  real_T c2_r_a;
  real_T c2_u_b;
  real_T c2_v_y;
  real_T c2_s_a;
  real_T c2_v_b;
  real_T c2_w_y;
  real_T c2_t_a;
  real_T c2_w_b;
  real_T c2_x_y;
  real_T c2_u_a;
  real_T c2_x_b;
  real_T c2_y_y;
  real_T c2_v_a;
  real_T c2_y_b;
  real_T c2_ab_y;
  real_T c2_w_a;
  real_T c2_ab_b;
  real_T c2_bb_y;
  real_T c2_x_a;
  real_T c2_bb_b;
  real_T c2_cb_y;
  real_T c2_cb_b;
  real_T c2_db_y;
  real_T c2_y_a;
  real_T c2_db_b;
  real_T c2_eb_y;
  real_T c2_ab_a;
  real_T c2_eb_b;
  real_T c2_fb_y;
  real_T c2_bb_a;
  real_T c2_fb_b;
  real_T c2_gb_y;
  real_T c2_cb_a;
  real_T c2_gb_b;
  real_T c2_hb_y;
  real_T c2_db_a;
  real_T c2_hb_b;
  real_T c2_ib_y;
  real_T c2_eb_a;
  real_T c2_ib_b;
  real_T c2_jb_y;
  real_T c2_fb_a;
  real_T c2_jb_b;
  real_T c2_kb_y;
  real_T c2_kb_b;
  real_T c2_lb_y;
  real_T c2_gb_a;
  real_T c2_lb_b;
  real_T c2_mb_y;
  real_T c2_hb_a;
  real_T c2_mb_b;
  real_T c2_nb_y;
  real_T c2_ib_a;
  real_T c2_nb_b;
  real_T c2_ob_y;
  real_T c2_jb_a;
  real_T c2_ob_b;
  real_T c2_pb_y;
  real_T c2_kb_a;
  real_T c2_pb_b;
  real_T c2_qb_y;
  real_T c2_lb_a;
  real_T c2_qb_b;
  real_T c2_rb_y;
  real_T c2_mb_a;
  real_T c2_rb_b;
  real_T c2_sb_y;
  real_T c2_sb_b;
  real_T c2_tb_y;
  real_T c2_nb_a;
  real_T c2_tb_b;
  real_T c2_ub_y;
  real_T c2_ob_a;
  real_T c2_ub_b;
  real_T c2_vb_y;
  real_T c2_pb_a;
  real_T c2_vb_b;
  real_T c2_wb_y;
  real_T c2_qb_a;
  real_T c2_wb_b;
  real_T c2_xb_y;
  real_T c2_rb_a;
  real_T c2_xb_b;
  real_T c2_yb_y;
  real_T c2_sb_a;
  real_T c2_yb_b;
  real_T c2_ac_y;
  real_T c2_ac_b;
  real_T c2_bc_y;
  real_T c2_tb_a;
  real_T c2_bc_b;
  real_T c2_cc_y;
  real_T c2_ub_a;
  real_T c2_cc_b;
  real_T c2_dc_y;
  real_T c2_vb_a;
  real_T c2_dc_b;
  real_T c2_ec_y;
  real_T c2_wb_a;
  real_T c2_ec_b;
  real_T c2_fc_y;
  real_T c2_xb_a;
  real_T c2_fc_b;
  real_T c2_gc_y;
  real_T c2_gc_b;
  real_T c2_hc_y;
  real_T c2_yb_a;
  real_T c2_hc_b;
  real_T c2_ic_y;
  real_T c2_ac_a;
  real_T c2_ic_b;
  real_T c2_jc_y;
  real_T c2_bc_a;
  real_T c2_jc_b;
  real_T c2_kc_y;
  real_T c2_cc_a;
  real_T c2_kc_b;
  real_T c2_lc_y;
  real_T c2_dc_a;
  real_T c2_lc_b;
  real_T c2_mc_y;
  real_T c2_ec_a;
  real_T c2_mc_b;
  real_T c2_nc_y;
  real_T c2_fc_a;
  real_T c2_nc_b;
  real_T c2_oc_y;
  real_T c2_oc_b;
  real_T c2_pc_y;
  real_T c2_gc_a;
  real_T c2_pc_b;
  real_T c2_qc_y;
  real_T c2_hc_a;
  real_T c2_qc_b;
  real_T c2_rc_y;
  real_T c2_ic_a;
  real_T c2_rc_b;
  real_T c2_sc_y;
  real_T c2_jc_a;
  real_T c2_sc_b;
  real_T c2_tc_y;
  real_T c2_tc_b;
  real_T c2_uc_y;
  real_T c2_kc_a;
  real_T c2_uc_b;
  real_T c2_vc_y;
  real_T c2_lc_a;
  real_T c2_vc_b;
  real_T c2_wc_y;
  real_T c2_wc_b;
  real_T c2_xc_y;
  real_T c2_mc_a;
  real_T c2_xc_b;
  real_T c2_yc_y;
  real_T c2_nc_a;
  real_T c2_yc_b;
  real_T c2_ad_y;
  real_T c2_ad_b;
  real_T c2_bd_y;
  real_T c2_oc_a;
  real_T c2_bd_b;
  real_T c2_cd_y;
  real_T c2_pc_a;
  real_T c2_cd_b;
  real_T c2_dd_y;
  real_T c2_dd_b;
  real_T c2_ed_y;
  real_T c2_ed_b;
  real_T c2_fd_y;
  real_T c2_qc_a;
  real_T c2_fd_b;
  real_T c2_gd_y;
  real_T c2_rc_a;
  real_T c2_gd_b;
  real_T c2_hd_y;
  real_T c2_hd_b;
  real_T c2_id_y;
  real_T c2_sc_a;
  real_T c2_id_b;
  real_T c2_jd_y;
  real_T c2_tc_a;
  real_T c2_jd_b;
  real_T c2_kd_y;
  real_T c2_kd_b;
  real_T c2_ld_y;
  real_T c2_uc_a;
  real_T c2_ld_b;
  real_T c2_md_y;
  real_T c2_vc_a;
  real_T c2_md_b;
  real_T c2_nd_y;
  real_T c2_nd_b;
  real_T c2_od_y;
  real_T c2_od_b;
  real_T c2_pd_y;
  real_T c2_wc_a;
  real_T c2_pd_b;
  real_T c2_qd_y;
  real_T c2_xc_a;
  real_T c2_qd_b;
  real_T c2_rd_y;
  real_T c2_yc_a;
  real_T c2_rd_b;
  real_T c2_sd_y;
  real_T c2_ad_a;
  real_T c2_sd_b;
  real_T c2_td_y;
  real_T c2_bd_a;
  real_T c2_td_b;
  real_T c2_ud_y;
  real_T c2_cd_a;
  real_T c2_ud_b;
  real_T c2_vd_y;
  real_T c2_dd_a;
  real_T c2_vd_b;
  real_T c2_wd_y;
  real_T c2_ed_a;
  real_T c2_wd_b;
  real_T c2_xd_y;
  real_T c2_fd_a;
  real_T c2_xd_b;
  real_T c2_yd_y;
  real_T c2_gd_a;
  real_T c2_yd_b;
  real_T c2_ae_y;
  real_T c2_hd_a;
  real_T c2_ae_b;
  real_T c2_be_y;
  real_T c2_id_a;
  real_T c2_be_b;
  real_T c2_ce_y;
  int32_T c2_i12;
  real_T c2_jd_a[9];
  int32_T c2_i13;
  real_T c2_ce_b[3];
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  real_T c2_C[3];
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  int32_T c2_i32;
  int32_T c2_i33;
  real_T c2_A;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_de_y;
  real_T c2_de_b;
  real_T c2_ee_y;
  real_T c2_ee_b;
  real_T c2_fe_y;
  real_T c2_fe_b;
  real_T c2_ge_y;
  real_T c2_ge_b;
  real_T c2_he_y;
  real_T c2_kd_a;
  int32_T c2_i34;
  real_T c2_b_A;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_ie_y;
  real_T c2_he_b;
  real_T c2_je_y;
  real_T c2_ie_b;
  real_T c2_ke_y;
  real_T c2_je_b;
  real_T c2_le_y;
  real_T c2_ke_b;
  real_T c2_me_y;
  real_T c2_ld_a;
  int32_T c2_i35;
  int32_T c2_i36;
  real_T c2_b_force_mult_1_E[3];
  int32_T c2_i37;
  real_T c2_b_dx1_dq0[3];
  int32_T c2_i38;
  real_T c2_b_force_mult_2_E[3];
  int32_T c2_i39;
  real_T c2_b_dx2_dq0[3];
  real_T c2_c_A;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_ne_y;
  real_T c2_le_b;
  real_T c2_oe_y;
  real_T c2_me_b;
  real_T c2_pe_y;
  real_T c2_ne_b;
  real_T c2_qe_y;
  real_T c2_oe_b;
  real_T c2_re_y;
  real_T c2_pe_b;
  real_T c2_se_y;
  real_T c2_md_a;
  int32_T c2_i40;
  real_T c2_d_A;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_te_y;
  real_T c2_qe_b;
  real_T c2_ue_y;
  real_T c2_re_b;
  real_T c2_ve_y;
  real_T c2_se_b;
  real_T c2_we_y;
  real_T c2_te_b;
  real_T c2_xe_y;
  real_T c2_ue_b;
  real_T c2_ye_y;
  real_T c2_nd_a;
  int32_T c2_i41;
  int32_T c2_i42;
  real_T c2_c_force_mult_1_E[3];
  int32_T c2_i43;
  real_T c2_b_dx1_dq1[3];
  int32_T c2_i44;
  real_T c2_c_force_mult_2_E[3];
  int32_T c2_i45;
  real_T c2_b_dx2_dq1[3];
  real_T c2_e_A;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_af_y;
  real_T c2_ve_b;
  real_T c2_bf_y;
  real_T c2_we_b;
  real_T c2_cf_y;
  real_T c2_xe_b;
  real_T c2_df_y;
  real_T c2_ye_b;
  real_T c2_ef_y;
  real_T c2_af_b;
  real_T c2_ff_y;
  real_T c2_od_a;
  int32_T c2_i46;
  real_T c2_f_A;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_gf_y;
  real_T c2_bf_b;
  real_T c2_hf_y;
  real_T c2_cf_b;
  real_T c2_if_y;
  real_T c2_df_b;
  real_T c2_jf_y;
  real_T c2_ef_b;
  real_T c2_kf_y;
  real_T c2_ff_b;
  real_T c2_lf_y;
  real_T c2_pd_a;
  int32_T c2_i47;
  int32_T c2_i48;
  real_T c2_d_force_mult_1_E[3];
  int32_T c2_i49;
  real_T c2_b_dx1_dq2[3];
  int32_T c2_i50;
  real_T c2_d_force_mult_2_E[3];
  int32_T c2_i51;
  real_T c2_b_dx2_dq2[3];
  real_T c2_g_A;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_mf_y;
  real_T c2_gf_b;
  real_T c2_nf_y;
  real_T c2_hf_b;
  real_T c2_of_y;
  real_T c2_if_b;
  real_T c2_pf_y;
  real_T c2_jf_b;
  real_T c2_qf_y;
  real_T c2_kf_b;
  real_T c2_rf_y;
  real_T c2_lf_b;
  real_T c2_sf_y;
  real_T c2_qd_a;
  int32_T c2_i52;
  real_T c2_h_A;
  real_T c2_p_x;
  real_T c2_q_x;
  real_T c2_tf_y;
  real_T c2_mf_b;
  real_T c2_uf_y;
  real_T c2_nf_b;
  real_T c2_vf_y;
  real_T c2_of_b;
  real_T c2_wf_y;
  real_T c2_pf_b;
  real_T c2_xf_y;
  real_T c2_qf_b;
  real_T c2_yf_y;
  real_T c2_rf_b;
  real_T c2_ag_y;
  real_T c2_rd_a;
  int32_T c2_i53;
  int32_T c2_i54;
  real_T c2_e_force_mult_1_E[3];
  int32_T c2_i55;
  real_T c2_b_dx1_dq3[3];
  int32_T c2_i56;
  real_T c2_e_force_mult_2_E[3];
  int32_T c2_i57;
  real_T c2_b_dx2_dq3[3];
  real_T c2_sf_b;
  real_T c2_bg_y;
  real_T c2_sd_a;
  real_T c2_tf_b;
  real_T c2_cg_y;
  real_T c2_td_a;
  real_T c2_uf_b;
  real_T c2_dg_y;
  real_T c2_ud_a;
  real_T c2_vf_b;
  real_T c2_eg_y;
  real_T c2_wf_b;
  real_T c2_fg_y;
  real_T c2_vd_a;
  real_T c2_xf_b;
  real_T c2_gg_y;
  real_T c2_wd_a;
  real_T c2_yf_b;
  real_T c2_hg_y;
  real_T c2_xd_a;
  real_T c2_ag_b;
  real_T c2_ig_y;
  real_T c2_bg_b;
  real_T c2_jg_y;
  real_T c2_yd_a;
  real_T c2_cg_b;
  real_T c2_kg_y;
  real_T c2_ae_a;
  real_T c2_dg_b;
  real_T c2_lg_y;
  real_T c2_be_a;
  real_T c2_eg_b;
  real_T c2_mg_y;
  real_T c2_fg_b;
  real_T c2_ng_y;
  real_T c2_ce_a;
  real_T c2_gg_b;
  real_T c2_og_y;
  real_T c2_de_a;
  real_T c2_hg_b;
  real_T c2_pg_y;
  real_T c2_ig_b;
  real_T c2_qg_y;
  real_T c2_ee_a;
  real_T c2_jg_b;
  real_T c2_rg_y;
  real_T c2_fe_a;
  real_T c2_kg_b;
  real_T c2_sg_y;
  real_T c2_lg_b;
  real_T c2_tg_y;
  real_T c2_ge_a;
  real_T c2_mg_b;
  real_T c2_ug_y;
  real_T c2_he_a;
  real_T c2_ng_b;
  real_T c2_vg_y;
  real_T c2_og_b;
  real_T c2_wg_y;
  real_T c2_pg_b;
  real_T c2_xg_y;
  real_T c2_ie_a;
  real_T c2_qg_b;
  real_T c2_yg_y;
  real_T c2_je_a;
  real_T c2_rg_b;
  real_T c2_ah_y;
  real_T c2_ke_a;
  real_T c2_sg_b;
  real_T c2_bh_y;
  real_T c2_tg_b;
  real_T c2_ch_y;
  real_T c2_ug_b;
  real_T c2_dh_y;
  real_T c2_le_a;
  real_T c2_vg_b;
  real_T c2_eh_y;
  real_T c2_me_a;
  real_T c2_wg_b;
  real_T c2_fh_y;
  real_T c2_ne_a;
  real_T c2_xg_b;
  real_T c2_gh_y;
  real_T c2_yg_b;
  real_T c2_hh_y;
  real_T c2_ah_b;
  real_T c2_ih_y;
  real_T c2_oe_a;
  real_T c2_bh_b;
  real_T c2_jh_y;
  real_T c2_pe_a;
  real_T c2_ch_b;
  real_T c2_kh_y;
  real_T c2_qe_a;
  real_T c2_dh_b;
  real_T c2_lh_y;
  real_T c2_eh_b;
  real_T c2_mh_y;
  real_T c2_re_a;
  real_T c2_fh_b;
  real_T c2_nh_y;
  real_T c2_se_a;
  real_T c2_gh_b;
  real_T c2_oh_y;
  real_T c2_te_a;
  real_T c2_hh_b;
  real_T c2_ph_y;
  real_T c2_ih_b;
  real_T c2_qh_y;
  real_T c2_ue_a;
  real_T c2_jh_b;
  real_T c2_rh_y;
  real_T c2_ve_a;
  real_T c2_kh_b;
  real_T c2_sh_y;
  real_T c2_we_a;
  real_T c2_lh_b;
  real_T c2_th_y;
  real_T c2_mh_b;
  real_T c2_uh_y;
  real_T c2_xe_a;
  real_T c2_nh_b;
  real_T c2_vh_y;
  real_T c2_ye_a;
  real_T c2_oh_b;
  real_T c2_wh_y;
  real_T c2_af_a;
  real_T c2_ph_b;
  real_T c2_xh_y;
  real_T c2_qh_b;
  real_T c2_yh_y;
  real_T c2_rh_b;
  real_T c2_ai_y;
  real_T c2_bf_a;
  real_T c2_sh_b;
  real_T c2_bi_y;
  real_T c2_cf_a;
  real_T c2_th_b;
  real_T c2_ci_y;
  real_T c2_df_a;
  real_T c2_uh_b;
  real_T c2_di_y;
  real_T c2_vh_b;
  real_T c2_ei_y;
  real_T c2_wh_b;
  real_T c2_fi_y;
  real_T c2_ef_a;
  real_T c2_xh_b;
  real_T c2_gi_y;
  real_T c2_ff_a;
  real_T c2_yh_b;
  real_T c2_hi_y;
  real_T c2_gf_a;
  real_T c2_ai_b;
  real_T c2_ii_y;
  real_T c2_bi_b;
  real_T c2_ji_y;
  real_T c2_ci_b;
  real_T c2_ki_y;
  real_T c2_hf_a;
  real_T c2_di_b;
  real_T c2_li_y;
  real_T c2_if_a;
  real_T c2_ei_b;
  real_T c2_mi_y;
  real_T c2_jf_a;
  real_T c2_fi_b;
  real_T c2_ni_y;
  real_T c2_gi_b;
  real_T c2_oi_y;
  real_T c2_kf_a;
  real_T c2_hi_b;
  real_T c2_pi_y;
  real_T c2_lf_a;
  real_T c2_ii_b;
  real_T c2_qi_y;
  real_T c2_ji_b;
  real_T c2_ri_y;
  real_T c2_mf_a;
  real_T c2_ki_b;
  real_T c2_si_y;
  real_T c2_nf_a;
  real_T c2_li_b;
  real_T c2_ti_y;
  real_T c2_mi_b;
  real_T c2_ui_y;
  real_T c2_of_a;
  real_T c2_ni_b;
  real_T c2_vi_y;
  real_T c2_pf_a;
  real_T c2_oi_b;
  real_T c2_wi_y;
  real_T c2_pi_b;
  real_T c2_xi_y;
  real_T c2_qf_a;
  real_T c2_qi_b;
  real_T c2_yi_y;
  real_T c2_rf_a;
  real_T c2_ri_b;
  real_T c2_aj_y;
  real_T c2_sf_a;
  real_T c2_si_b;
  real_T c2_bj_y;
  real_T c2_ti_b;
  real_T c2_cj_y;
  real_T c2_tf_a;
  real_T c2_ui_b;
  real_T c2_dj_y;
  real_T c2_uf_a;
  real_T c2_vi_b;
  real_T c2_ej_y;
  real_T c2_vf_a;
  real_T c2_wi_b;
  real_T c2_fj_y;
  real_T c2_xi_b;
  real_T c2_gj_y;
  real_T c2_wf_a;
  real_T c2_yi_b;
  real_T c2_hj_y;
  real_T c2_xf_a;
  real_T c2_aj_b;
  real_T c2_ij_y;
  real_T c2_yf_a;
  real_T c2_bj_b;
  real_T c2_jj_y;
  real_T c2_cj_b;
  real_T c2_kj_y;
  real_T c2_ag_a;
  real_T c2_dj_b;
  real_T c2_lj_y;
  real_T c2_bg_a;
  real_T c2_ej_b;
  real_T c2_mj_y;
  real_T c2_fj_b;
  real_T c2_nj_y;
  real_T c2_cg_a;
  real_T c2_gj_b;
  real_T c2_oj_y;
  real_T c2_dg_a;
  real_T c2_hj_b;
  real_T c2_pj_y;
  real_T c2_ij_b;
  real_T c2_qj_y;
  real_T c2_eg_a;
  real_T c2_jj_b;
  real_T c2_rj_y;
  real_T c2_fg_a;
  real_T c2_kj_b;
  real_T c2_sj_y;
  real_T c2_lj_b;
  real_T c2_tj_y;
  real_T c2_gg_a;
  real_T c2_mj_b;
  real_T c2_uj_y;
  real_T c2_hg_a;
  real_T c2_nj_b;
  real_T c2_vj_y;
  real_T c2_ig_a;
  real_T c2_oj_b;
  real_T c2_wj_y;
  real_T c2_pj_b;
  real_T c2_xj_y;
  real_T c2_jg_a;
  real_T c2_qj_b;
  real_T c2_yj_y;
  real_T c2_kg_a;
  real_T c2_rj_b;
  real_T c2_ak_y;
  real_T c2_lg_a;
  real_T c2_sj_b;
  real_T c2_bk_y;
  real_T c2_tj_b;
  real_T c2_ck_y;
  real_T c2_mg_a;
  real_T c2_uj_b;
  real_T c2_dk_y;
  real_T c2_ng_a;
  real_T c2_vj_b;
  real_T c2_ek_y;
  real_T c2_og_a;
  real_T c2_wj_b;
  real_T c2_fk_y;
  real_T c2_xj_b;
  real_T c2_gk_y;
  real_T c2_pg_a;
  real_T c2_yj_b;
  real_T c2_hk_y;
  real_T c2_qg_a;
  real_T c2_ak_b;
  real_T c2_ik_y;
  real_T c2_rg_a;
  real_T c2_bk_b;
  real_T c2_jk_y;
  real_T c2_ck_b;
  real_T c2_kk_y;
  real_T c2_sg_a;
  real_T c2_dk_b;
  real_T c2_lk_y;
  real_T c2_tg_a;
  real_T c2_ek_b;
  real_T c2_mk_y;
  real_T c2_ug_a;
  real_T c2_fk_b;
  real_T c2_nk_y;
  real_T c2_gk_b;
  real_T c2_ok_y;
  real_T c2_vg_a;
  real_T c2_hk_b;
  real_T c2_pk_y;
  real_T c2_wg_a;
  real_T c2_ik_b;
  real_T c2_qk_y;
  real_T c2_xg_a;
  real_T c2_jk_b;
  real_T c2_rk_y;
  real_T c2_kk_b;
  real_T c2_sk_y;
  real_T c2_yg_a;
  real_T c2_lk_b;
  real_T c2_tk_y;
  real_T c2_ah_a;
  real_T c2_mk_b;
  real_T c2_uk_y;
  real_T c2_nk_b;
  real_T c2_vk_y;
  real_T c2_bh_a;
  real_T c2_ok_b;
  real_T c2_wk_y;
  real_T c2_ch_a;
  real_T c2_pk_b;
  real_T c2_xk_y;
  real_T c2_qk_b;
  real_T c2_yk_y;
  real_T c2_dh_a;
  real_T c2_rk_b;
  real_T c2_al_y;
  real_T c2_eh_a;
  real_T c2_sk_b;
  real_T c2_bl_y;
  real_T c2_tk_b;
  real_T c2_cl_y;
  real_T c2_uk_b;
  real_T c2_dl_y;
  real_T c2_fh_a;
  real_T c2_vk_b;
  real_T c2_el_y;
  real_T c2_gh_a;
  real_T c2_wk_b;
  real_T c2_fl_y;
  real_T c2_hh_a;
  real_T c2_xk_b;
  real_T c2_gl_y;
  real_T c2_yk_b;
  real_T c2_hl_y;
  real_T c2_al_b;
  real_T c2_il_y;
  real_T c2_ih_a;
  real_T c2_bl_b;
  real_T c2_jl_y;
  real_T c2_jh_a;
  real_T c2_cl_b;
  real_T c2_kl_y;
  real_T c2_kh_a;
  real_T c2_dl_b;
  real_T c2_ll_y;
  real_T c2_el_b;
  real_T c2_ml_y;
  real_T c2_fl_b;
  real_T c2_nl_y;
  real_T c2_lh_a;
  real_T c2_gl_b;
  real_T c2_ol_y;
  real_T c2_mh_a;
  real_T c2_hl_b;
  real_T c2_pl_y;
  real_T c2_nh_a;
  real_T c2_il_b;
  real_T c2_ql_y;
  real_T c2_d0;
  real_T c2_d1;
  real_T c2_d2;
  real_T c2_d3;
  real_T c2_jl_b;
  real_T c2_rl_y;
  real_T c2_oh_a;
  real_T c2_kl_b;
  real_T c2_sl_y;
  real_T c2_ll_b;
  real_T c2_tl_y;
  real_T c2_ph_a;
  real_T c2_ml_b;
  real_T c2_ul_y;
  real_T c2_nl_b;
  real_T c2_vl_y;
  real_T c2_qh_a;
  real_T c2_ol_b;
  real_T c2_wl_y;
  real_T c2_pl_b;
  real_T c2_xl_y;
  real_T c2_rh_a;
  real_T c2_ql_b;
  real_T c2_yl_y;
  real_T c2_sh_a;
  real_T c2_rl_b;
  real_T c2_am_y;
  real_T c2_sl_b;
  real_T c2_bm_y;
  real_T c2_th_a;
  real_T c2_tl_b;
  real_T c2_cm_y;
  real_T c2_uh_a;
  real_T c2_ul_b;
  real_T c2_dm_y;
  real_T c2_vl_b;
  real_T c2_em_y;
  real_T c2_vh_a;
  real_T c2_wl_b;
  real_T c2_fm_y;
  real_T c2_wh_a;
  real_T c2_xl_b;
  real_T c2_gm_y;
  real_T c2_yl_b;
  real_T c2_hm_y;
  real_T c2_xh_a;
  real_T c2_am_b;
  real_T c2_im_y;
  real_T c2_yh_a;
  real_T c2_bm_b;
  real_T c2_jm_y;
  real_T c2_cm_b;
  real_T c2_km_y;
  real_T c2_ai_a;
  real_T c2_dm_b;
  real_T c2_lm_y;
  real_T c2_bi_a;
  real_T c2_em_b;
  real_T c2_mm_y;
  real_T c2_fm_b;
  real_T c2_nm_y;
  real_T c2_ci_a;
  real_T c2_gm_b;
  real_T c2_om_y;
  real_T c2_di_a;
  real_T c2_hm_b;
  real_T c2_pm_y;
  real_T c2_im_b;
  real_T c2_qm_y;
  real_T c2_ei_a;
  real_T c2_jm_b;
  real_T c2_rm_y;
  real_T c2_fi_a;
  real_T c2_km_b;
  real_T c2_sm_y;
  real_T c2_lm_b;
  real_T c2_tm_y;
  real_T c2_gi_a;
  real_T c2_mm_b;
  real_T c2_um_y;
  real_T c2_hi_a;
  real_T c2_nm_b;
  real_T c2_vm_y;
  real_T c2_om_b;
  real_T c2_wm_y;
  real_T c2_ii_a;
  real_T c2_pm_b;
  real_T c2_xm_y;
  real_T c2_ji_a;
  real_T c2_qm_b;
  real_T c2_ym_y;
  real_T c2_rm_b;
  real_T c2_an_y;
  real_T c2_ki_a;
  real_T c2_sm_b;
  real_T c2_bn_y;
  real_T c2_li_a;
  real_T c2_tm_b;
  real_T c2_cn_y;
  real_T c2_um_b;
  real_T c2_dn_y;
  real_T c2_mi_a;
  real_T c2_vm_b;
  real_T c2_en_y;
  real_T c2_ni_a;
  real_T c2_wm_b;
  real_T c2_fn_y;
  real_T c2_xm_b;
  real_T c2_gn_y;
  real_T c2_oi_a;
  real_T c2_ym_b;
  real_T c2_hn_y;
  real_T c2_pi_a;
  real_T c2_an_b;
  real_T c2_in_y;
  real_T c2_bn_b;
  real_T c2_jn_y;
  real_T c2_qi_a;
  real_T c2_cn_b;
  real_T c2_kn_y;
  real_T c2_dn_b;
  real_T c2_ln_y;
  real_T c2_ri_a;
  real_T c2_en_b;
  real_T c2_mn_y;
  real_T c2_fn_b;
  real_T c2_nn_y;
  real_T c2_si_a;
  real_T c2_gn_b;
  real_T c2_on_y;
  real_T c2_hn_b;
  real_T c2_pn_y;
  real_T c2_ti_a;
  real_T c2_in_b;
  real_T c2_qn_y;
  real_T c2_ui_a;
  real_T c2_jn_b;
  real_T c2_rn_y;
  real_T c2_kn_b;
  real_T c2_sn_y;
  real_T c2_vi_a;
  real_T c2_ln_b;
  real_T c2_tn_y;
  real_T c2_wi_a;
  real_T c2_mn_b;
  real_T c2_un_y;
  real_T c2_nn_b;
  real_T c2_vn_y;
  real_T c2_xi_a;
  real_T c2_on_b;
  real_T c2_wn_y;
  real_T c2_yi_a;
  real_T c2_pn_b;
  real_T c2_xn_y;
  real_T c2_qn_b;
  real_T c2_yn_y;
  real_T c2_aj_a;
  real_T c2_rn_b;
  real_T c2_ao_y;
  real_T c2_bj_a;
  real_T c2_sn_b;
  real_T c2_bo_y;
  real_T c2_tn_b;
  real_T c2_co_y;
  real_T c2_cj_a;
  real_T c2_un_b;
  real_T c2_do_y;
  real_T c2_dj_a;
  real_T c2_vn_b;
  real_T c2_eo_y;
  real_T c2_wn_b;
  real_T c2_fo_y;
  real_T c2_ej_a;
  real_T c2_xn_b;
  real_T c2_go_y;
  real_T c2_fj_a;
  real_T c2_yn_b;
  real_T c2_ho_y;
  real_T c2_ao_b;
  real_T c2_io_y;
  real_T c2_gj_a;
  real_T c2_bo_b;
  real_T c2_jo_y;
  real_T c2_hj_a;
  real_T c2_co_b;
  real_T c2_ko_y;
  real_T c2_do_b;
  real_T c2_lo_y;
  real_T c2_ij_a;
  real_T c2_eo_b;
  real_T c2_mo_y;
  real_T c2_jj_a;
  real_T c2_fo_b;
  real_T c2_no_y;
  real_T c2_go_b;
  real_T c2_oo_y;
  real_T c2_kj_a;
  real_T c2_ho_b;
  real_T c2_po_y;
  real_T c2_lj_a;
  real_T c2_io_b;
  real_T c2_qo_y;
  real_T c2_jo_b;
  real_T c2_ro_y;
  real_T c2_mj_a;
  real_T c2_ko_b;
  real_T c2_so_y;
  real_T c2_nj_a;
  real_T c2_lo_b;
  real_T c2_to_y;
  real_T c2_mo_b;
  real_T c2_uo_y;
  real_T c2_oj_a;
  real_T c2_no_b;
  real_T c2_vo_y;
  real_T c2_pj_a;
  real_T c2_oo_b;
  real_T c2_wo_y;
  real_T c2_po_b;
  real_T c2_xo_y;
  real_T c2_qj_a;
  real_T c2_qo_b;
  real_T c2_yo_y;
  real_T c2_rj_a;
  real_T c2_ro_b;
  real_T c2_ap_y;
  real_T c2_so_b;
  real_T c2_bp_y;
  real_T c2_sj_a;
  real_T c2_to_b;
  real_T c2_cp_y;
  real_T c2_uo_b;
  real_T c2_dp_y;
  real_T c2_tj_a;
  real_T c2_vo_b;
  real_T c2_ep_y;
  real_T c2_wo_b;
  real_T c2_fp_y;
  real_T c2_uj_a;
  real_T c2_xo_b;
  real_T c2_gp_y;
  real_T c2_yo_b;
  real_T c2_hp_y;
  real_T c2_vj_a;
  real_T c2_ap_b;
  real_T c2_ip_y;
  real_T c2_wj_a;
  real_T c2_bp_b;
  real_T c2_jp_y;
  real_T c2_cp_b;
  real_T c2_kp_y;
  real_T c2_xj_a;
  real_T c2_dp_b;
  real_T c2_lp_y;
  real_T c2_yj_a;
  real_T c2_ep_b;
  real_T c2_mp_y;
  real_T c2_fp_b;
  real_T c2_np_y;
  real_T c2_ak_a;
  real_T c2_gp_b;
  real_T c2_op_y;
  real_T c2_bk_a;
  real_T c2_hp_b;
  real_T c2_pp_y;
  real_T c2_ip_b;
  real_T c2_qp_y;
  real_T c2_ck_a;
  real_T c2_jp_b;
  real_T c2_rp_y;
  real_T c2_dk_a;
  real_T c2_kp_b;
  real_T c2_sp_y;
  real_T c2_lp_b;
  real_T c2_tp_y;
  real_T c2_ek_a;
  real_T c2_mp_b;
  real_T c2_up_y;
  real_T c2_fk_a;
  real_T c2_np_b;
  real_T c2_vp_y;
  real_T c2_op_b;
  real_T c2_wp_y;
  real_T c2_gk_a;
  real_T c2_pp_b;
  real_T c2_xp_y;
  real_T c2_hk_a;
  real_T c2_qp_b;
  real_T c2_yp_y;
  real_T c2_rp_b;
  real_T c2_aq_y;
  real_T c2_ik_a;
  real_T c2_sp_b;
  real_T c2_bq_y;
  real_T c2_jk_a;
  real_T c2_tp_b;
  real_T c2_cq_y;
  real_T c2_up_b;
  real_T c2_dq_y;
  real_T c2_kk_a;
  real_T c2_vp_b;
  real_T c2_eq_y;
  real_T c2_lk_a;
  real_T c2_wp_b;
  real_T c2_fq_y;
  real_T c2_xp_b;
  real_T c2_gq_y;
  real_T c2_mk_a;
  real_T c2_yp_b;
  real_T c2_hq_y;
  real_T c2_nk_a;
  real_T c2_aq_b;
  real_T c2_iq_y;
  real_T c2_bq_b;
  real_T c2_jq_y;
  real_T c2_ok_a;
  real_T c2_cq_b;
  real_T c2_kq_y;
  real_T c2_pk_a;
  real_T c2_dq_b;
  real_T c2_lq_y;
  real_T c2_eq_b;
  real_T c2_mq_y;
  real_T c2_qk_a;
  real_T c2_fq_b;
  real_T c2_nq_y;
  real_T c2_rk_a;
  real_T c2_gq_b;
  real_T c2_oq_y;
  real_T c2_hq_b;
  real_T c2_pq_y;
  real_T c2_sk_a;
  real_T c2_iq_b;
  real_T c2_qq_y;
  real_T c2_tk_a;
  real_T c2_jq_b;
  real_T c2_rq_y;
  real_T c2_kq_b;
  real_T c2_sq_y;
  real_T c2_uk_a;
  real_T c2_lq_b;
  real_T c2_tq_y;
  real_T c2_mq_b;
  real_T c2_uq_y;
  real_T c2_vk_a;
  real_T c2_nq_b;
  real_T c2_vq_y;
  real_T c2_oq_b;
  real_T c2_wq_y;
  real_T c2_wk_a;
  real_T c2_pq_b;
  real_T c2_xq_y;
  int32_T c2_i58;
  real_T c2_b_quaternion_multipliers[20];
  int32_T c2_i59;
  real_T c2_b_quaternion_deltas[5];
  real_T c2_dv2[4];
  int32_T c2_i60;
  real_T c2_i_A;
  real_T c2_B;
  real_T c2_r_x;
  real_T c2_yq_y;
  real_T c2_s_x;
  real_T c2_ar_y;
  real_T c2_j_A;
  real_T c2_b_B;
  real_T c2_t_x;
  real_T c2_br_y;
  real_T c2_u_x;
  real_T c2_cr_y;
  real_T c2_k_A;
  real_T c2_c_B;
  real_T c2_v_x;
  real_T c2_dr_y;
  real_T c2_w_x;
  real_T c2_er_y;
  real_T c2_fr_y;
  int32_T c2_i61;
  int32_T c2_i62;
  real_T *c2_b_x_ddot;
  real_T *c2_b_y_ddot;
  real_T *c2_b_z_ddot;
  real_T *c2_b_q0_ddot;
  real_T *c2_b_q1_ddot;
  real_T *c2_b_q2_ddot;
  real_T *c2_b_q3_ddot;
  real_T (*c2_c_quaternion_multipliers)[20];
  real_T (*c2_c_quaternion_deltas)[5];
  real_T (*c2_b_w)[4];
  real_T (*c2_b_params)[15];
  real_T (*c2_b_states)[14];
  c2_c_quaternion_deltas = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S,
    9);
  c2_c_quaternion_multipliers = (real_T (*)[20])ssGetOutputPortSignal
    (chartInstance->S, 8);
  c2_b_w = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_params = (real_T (*)[15])ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_q3_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_b_q2_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_b_q1_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_b_q0_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_b_z_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_b_y_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_x_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_states = (real_T (*)[14])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i9 = 0; c2_i9 < 14; c2_i9++) {
    c2_states[c2_i9] = (*c2_b_states)[c2_i9];
  }

  for (c2_i10 = 0; c2_i10 < 15; c2_i10++) {
    c2_params[c2_i10] = (*c2_b_params)[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 4; c2_i11++) {
    c2_w[c2_i11] = (*c2_b_w)[c2_i11];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 75U, 75U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_m, 0U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_g, 1U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_l, 2U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Jxx, 3U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Jyy, 4U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Jzz, 5U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_k1, 6U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_k2, 7U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_k3, 8U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_k4, 9U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b1, 10U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b2, 11U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b3, 12U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b4, 13U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d, 14U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w1, 15U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w2, 16U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w3, 17U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w4, 18U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x, 19U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x_dot, 20U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y, 21U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y_dot, 22U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_z, 23U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_z_dot, 24U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q0, 25U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q0_dot, 26U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q1, 27U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q1_dot, 28U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q2, 29U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q2_dot, 30U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q3, 31U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q3_dot, 32U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_x, 33U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_y, 34U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_z, 35U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_T_e2E, 36U, c2_h_sf_marshallOut,
    c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_force_mult_1, 37U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_force_mult_2, 38U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_force_mult_1_E, 39U,
    c2_g_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_force_mult_2_E, 40U,
    c2_g_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx1_dq0, 41U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx2_dq0, 42U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_q0, 43U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx1_dq1, 44U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx2_dq1, 45U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_q1, 46U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx1_dq2, 47U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx2_dq2, 48U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_q2, 49U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx1_dq3, 50U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_dx2_dq3, 51U, c2_g_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_upphi_q3, 52U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta1, 53U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta2, 54U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta3, 55U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta4, 56U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta5, 57U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta6, 58U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_delta7, 59U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_q_ddot, 60U, c2_d_sf_marshallOut,
    c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 61U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 62U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_states, 63U, c2_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_params, 64U, c2_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_w, 65U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x_ddot, 66U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y_ddot, 67U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_z_ddot, 68U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q0_ddot, 69U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q1_ddot, 70U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q2_ddot, 71U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q3_ddot, 72U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_quaternion_multipliers, 73U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_quaternion_deltas, 74U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  c2_m = c2_params[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_g = c2_params[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_l = c2_params[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_Jxx = c2_params[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_Jyy = c2_params[4];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  c2_Jzz = c2_params[5];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_k1 = c2_params[6];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_k2 = c2_params[7];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_k3 = c2_params[8];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_k4 = c2_params[9];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
  c2_b1 = c2_params[10];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  c2_b2 = c2_params[11];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  c2_b3 = c2_params[12];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
  c2_b4 = c2_params[13];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_d = c2_params[14];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  c2_w1 = c2_w[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 24);
  c2_w2 = c2_w[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
  c2_w3 = c2_w[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  c2_w4 = c2_w[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_x = c2_states[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_x_dot = c2_states[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_y = c2_states[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_y_dot = c2_states[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_z = c2_states[4];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_z_dot = c2_states[5];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  c2_q0 = c2_states[6];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  c2_q0_dot = c2_states[7];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  c2_q1 = c2_states[8];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 30);
  c2_q1_dot = c2_states[9];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_q2 = c2_states[10];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_q2_dot = c2_states[11];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_q3 = c2_states[12];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
  c2_q3_dot = c2_states[13];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 33);
  c2_a = c2_k1;
  c2_b = c2_mpower(chartInstance, c2_w1);
  c2_b_y = c2_a * c2_b;
  c2_b_a = c2_k2;
  c2_b_b = c2_mpower(chartInstance, c2_w2);
  c2_c_y = c2_b_a * c2_b_b;
  c2_c_a = c2_k3;
  c2_c_b = c2_mpower(chartInstance, c2_w3);
  c2_d_y = c2_c_a * c2_c_b;
  c2_d_a = c2_k4;
  c2_d_b = c2_mpower(chartInstance, c2_w4);
  c2_e_y = c2_d_a * c2_d_b;
  c2_e_b = ((c2_b_y + c2_c_y) + c2_d_y) + c2_e_y;
  c2_f_y = 2.0 * c2_e_b;
  c2_e_a = c2_q2;
  c2_f_b = c2_q0;
  c2_g_y = c2_e_a * c2_f_b;
  c2_f_a = c2_q1;
  c2_g_b = c2_q3;
  c2_h_y = c2_f_a * c2_g_b;
  c2_g_a = c2_f_y;
  c2_h_b = c2_g_y + c2_h_y;
  c2_i_y = c2_g_a * c2_h_b;
  c2_h_a = -c2_b1;
  c2_i_b = c2_mpower(chartInstance, c2_w1);
  c2_j_y = c2_h_a * c2_i_b;
  c2_i_a = c2_b2;
  c2_j_b = c2_mpower(chartInstance, c2_w2);
  c2_k_y = c2_i_a * c2_j_b;
  c2_j_a = c2_b3;
  c2_k_b = c2_mpower(chartInstance, c2_w3);
  c2_l_y = c2_j_a * c2_k_b;
  c2_k_a = c2_b4;
  c2_l_b = c2_mpower(chartInstance, c2_w4);
  c2_m_y = c2_k_a * c2_l_b;
  c2_m_b = c2_mpower(chartInstance, c2_q2) + c2_mpower(chartInstance, c2_q3);
  c2_n_y = 2.0 * c2_m_b;
  c2_l_a = ((c2_j_y + c2_k_y) + c2_l_y) - c2_m_y;
  c2_n_b = 1.0 - c2_n_y;
  c2_o_y = c2_l_a * c2_n_b;
  c2_m_a = c2_b1;
  c2_o_b = c2_mpower(chartInstance, c2_w1);
  c2_p_y = c2_m_a * c2_o_b;
  c2_n_a = c2_b2;
  c2_p_b = c2_mpower(chartInstance, c2_w2);
  c2_q_y = c2_n_a * c2_p_b;
  c2_o_a = c2_b3;
  c2_q_b = c2_mpower(chartInstance, c2_w3);
  c2_r_y = c2_o_a * c2_q_b;
  c2_p_a = c2_b4;
  c2_r_b = c2_mpower(chartInstance, c2_w4);
  c2_s_y = c2_p_a * c2_r_b;
  c2_s_b = ((c2_p_y + c2_q_y) - c2_r_y) - c2_s_y;
  c2_t_y = 2.0 * c2_s_b;
  c2_q_a = c2_q1;
  c2_t_b = c2_q2;
  c2_u_y = c2_q_a * c2_t_b;
  c2_r_a = c2_q3;
  c2_u_b = c2_q0;
  c2_v_y = c2_r_a * c2_u_b;
  c2_s_a = c2_t_y;
  c2_v_b = c2_u_y - c2_v_y;
  c2_w_y = c2_s_a * c2_v_b;
  c2_t_a = c2_d;
  c2_w_b = c2_x_dot;
  c2_x_y = c2_t_a * c2_w_b;
  c2_upphi_x = ((c2_i_y + c2_o_y) + c2_w_y) - c2_x_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  c2_u_a = c2_k1;
  c2_x_b = c2_mpower(chartInstance, c2_w1);
  c2_y_y = c2_u_a * c2_x_b;
  c2_v_a = c2_k2;
  c2_y_b = c2_mpower(chartInstance, c2_w2);
  c2_ab_y = c2_v_a * c2_y_b;
  c2_w_a = c2_k3;
  c2_ab_b = c2_mpower(chartInstance, c2_w3);
  c2_bb_y = c2_w_a * c2_ab_b;
  c2_x_a = c2_k4;
  c2_bb_b = c2_mpower(chartInstance, c2_w4);
  c2_cb_y = c2_x_a * c2_bb_b;
  c2_cb_b = ((c2_y_y + c2_ab_y) + c2_bb_y) + c2_cb_y;
  c2_db_y = 2.0 * c2_cb_b;
  c2_y_a = c2_q3;
  c2_db_b = c2_q2;
  c2_eb_y = c2_y_a * c2_db_b;
  c2_ab_a = c2_q1;
  c2_eb_b = c2_q0;
  c2_fb_y = c2_ab_a * c2_eb_b;
  c2_bb_a = c2_db_y;
  c2_fb_b = c2_eb_y - c2_fb_y;
  c2_gb_y = c2_bb_a * c2_fb_b;
  c2_cb_a = -c2_b1;
  c2_gb_b = c2_mpower(chartInstance, c2_w1);
  c2_hb_y = c2_cb_a * c2_gb_b;
  c2_db_a = c2_b2;
  c2_hb_b = c2_mpower(chartInstance, c2_w2);
  c2_ib_y = c2_db_a * c2_hb_b;
  c2_eb_a = c2_b3;
  c2_ib_b = c2_mpower(chartInstance, c2_w3);
  c2_jb_y = c2_eb_a * c2_ib_b;
  c2_fb_a = c2_b4;
  c2_jb_b = c2_mpower(chartInstance, c2_w4);
  c2_kb_y = c2_fb_a * c2_jb_b;
  c2_kb_b = ((c2_hb_y + c2_ib_y) + c2_jb_y) - c2_kb_y;
  c2_lb_y = 2.0 * c2_kb_b;
  c2_gb_a = c2_q3;
  c2_lb_b = c2_q0;
  c2_mb_y = c2_gb_a * c2_lb_b;
  c2_hb_a = c2_q1;
  c2_mb_b = c2_q2;
  c2_nb_y = c2_hb_a * c2_mb_b;
  c2_ib_a = c2_lb_y;
  c2_nb_b = c2_mb_y + c2_nb_y;
  c2_ob_y = c2_ib_a * c2_nb_b;
  c2_jb_a = c2_b1;
  c2_ob_b = c2_mpower(chartInstance, c2_w1);
  c2_pb_y = c2_jb_a * c2_ob_b;
  c2_kb_a = c2_b2;
  c2_pb_b = c2_mpower(chartInstance, c2_w2);
  c2_qb_y = c2_kb_a * c2_pb_b;
  c2_lb_a = c2_b3;
  c2_qb_b = c2_mpower(chartInstance, c2_w3);
  c2_rb_y = c2_lb_a * c2_qb_b;
  c2_mb_a = c2_b4;
  c2_rb_b = c2_mpower(chartInstance, c2_w4);
  c2_sb_y = c2_mb_a * c2_rb_b;
  c2_sb_b = c2_mpower(chartInstance, c2_q1) + c2_mpower(chartInstance, c2_q3);
  c2_tb_y = 2.0 * c2_sb_b;
  c2_nb_a = ((c2_pb_y + c2_qb_y) - c2_rb_y) - c2_sb_y;
  c2_tb_b = 1.0 - c2_tb_y;
  c2_ub_y = c2_nb_a * c2_tb_b;
  c2_ob_a = c2_d;
  c2_ub_b = c2_y_dot;
  c2_vb_y = c2_ob_a * c2_ub_b;
  c2_upphi_y = ((c2_gb_y + c2_ob_y) + c2_ub_y) - c2_vb_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 41);
  c2_pb_a = c2_k1;
  c2_vb_b = c2_mpower(chartInstance, c2_w1);
  c2_wb_y = c2_pb_a * c2_vb_b;
  c2_qb_a = c2_k2;
  c2_wb_b = c2_mpower(chartInstance, c2_w2);
  c2_xb_y = c2_qb_a * c2_wb_b;
  c2_rb_a = c2_k3;
  c2_xb_b = c2_mpower(chartInstance, c2_w3);
  c2_yb_y = c2_rb_a * c2_xb_b;
  c2_sb_a = c2_k4;
  c2_yb_b = c2_mpower(chartInstance, c2_w4);
  c2_ac_y = c2_sb_a * c2_yb_b;
  c2_ac_b = c2_mpower(chartInstance, c2_q1) + c2_mpower(chartInstance, c2_q2);
  c2_bc_y = 2.0 * c2_ac_b;
  c2_tb_a = ((c2_wb_y + c2_xb_y) + c2_yb_y) + c2_ac_y;
  c2_bc_b = 1.0 - c2_bc_y;
  c2_cc_y = c2_tb_a * c2_bc_b;
  c2_ub_a = -c2_b1;
  c2_cc_b = c2_mpower(chartInstance, c2_w1);
  c2_dc_y = c2_ub_a * c2_cc_b;
  c2_vb_a = c2_b2;
  c2_dc_b = c2_mpower(chartInstance, c2_w2);
  c2_ec_y = c2_vb_a * c2_dc_b;
  c2_wb_a = c2_b3;
  c2_ec_b = c2_mpower(chartInstance, c2_w3);
  c2_fc_y = c2_wb_a * c2_ec_b;
  c2_xb_a = c2_b4;
  c2_fc_b = c2_mpower(chartInstance, c2_w4);
  c2_gc_y = c2_xb_a * c2_fc_b;
  c2_gc_b = ((c2_dc_y + c2_ec_y) + c2_fc_y) - c2_gc_y;
  c2_hc_y = 2.0 * c2_gc_b;
  c2_yb_a = c2_q3;
  c2_hc_b = c2_q1;
  c2_ic_y = c2_yb_a * c2_hc_b;
  c2_ac_a = c2_q2;
  c2_ic_b = c2_q0;
  c2_jc_y = c2_ac_a * c2_ic_b;
  c2_bc_a = c2_hc_y;
  c2_jc_b = c2_ic_y - c2_jc_y;
  c2_kc_y = c2_bc_a * c2_jc_b;
  c2_cc_a = c2_b1;
  c2_kc_b = c2_mpower(chartInstance, c2_w1);
  c2_lc_y = c2_cc_a * c2_kc_b;
  c2_dc_a = c2_b2;
  c2_lc_b = c2_mpower(chartInstance, c2_w2);
  c2_mc_y = c2_dc_a * c2_lc_b;
  c2_ec_a = c2_b3;
  c2_mc_b = c2_mpower(chartInstance, c2_w3);
  c2_nc_y = c2_ec_a * c2_mc_b;
  c2_fc_a = c2_b4;
  c2_nc_b = c2_mpower(chartInstance, c2_w4);
  c2_oc_y = c2_fc_a * c2_nc_b;
  c2_oc_b = ((c2_lc_y + c2_mc_y) - c2_nc_y) - c2_oc_y;
  c2_pc_y = 2.0 * c2_oc_b;
  c2_gc_a = c2_q1;
  c2_pc_b = c2_q0;
  c2_qc_y = c2_gc_a * c2_pc_b;
  c2_hc_a = c2_q3;
  c2_qc_b = c2_q2;
  c2_rc_y = c2_hc_a * c2_qc_b;
  c2_ic_a = c2_pc_y;
  c2_rc_b = c2_qc_y + c2_rc_y;
  c2_sc_y = c2_ic_a * c2_rc_b;
  c2_jc_a = c2_d;
  c2_sc_b = c2_z_dot;
  c2_tc_y = c2_jc_a * c2_sc_b;
  c2_upphi_z = ((c2_cc_y + c2_kc_y) + c2_sc_y) - c2_tc_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 45);
  c2_tc_b = c2_mpower(chartInstance, c2_q2) + c2_mpower(chartInstance, c2_q3);
  c2_uc_y = 2.0 * c2_tc_b;
  c2_kc_a = c2_q1;
  c2_uc_b = c2_q2;
  c2_vc_y = c2_kc_a * c2_uc_b;
  c2_lc_a = c2_q3;
  c2_vc_b = c2_q0;
  c2_wc_y = c2_lc_a * c2_vc_b;
  c2_wc_b = c2_vc_y - c2_wc_y;
  c2_xc_y = 2.0 * c2_wc_b;
  c2_mc_a = c2_q2;
  c2_xc_b = c2_q0;
  c2_yc_y = c2_mc_a * c2_xc_b;
  c2_nc_a = c2_q1;
  c2_yc_b = c2_q3;
  c2_ad_y = c2_nc_a * c2_yc_b;
  c2_ad_b = c2_yc_y + c2_ad_y;
  c2_bd_y = 2.0 * c2_ad_b;
  c2_oc_a = c2_q3;
  c2_bd_b = c2_q0;
  c2_cd_y = c2_oc_a * c2_bd_b;
  c2_pc_a = c2_q1;
  c2_cd_b = c2_q2;
  c2_dd_y = c2_pc_a * c2_cd_b;
  c2_dd_b = c2_cd_y + c2_dd_y;
  c2_ed_y = 2.0 * c2_dd_b;
  c2_ed_b = c2_mpower(chartInstance, c2_q1) + c2_mpower(chartInstance, c2_q3);
  c2_fd_y = 2.0 * c2_ed_b;
  c2_qc_a = c2_q3;
  c2_fd_b = c2_q2;
  c2_gd_y = c2_qc_a * c2_fd_b;
  c2_rc_a = c2_q1;
  c2_gd_b = c2_q0;
  c2_hd_y = c2_rc_a * c2_gd_b;
  c2_hd_b = c2_gd_y - c2_hd_y;
  c2_id_y = 2.0 * c2_hd_b;
  c2_sc_a = c2_q3;
  c2_id_b = c2_q1;
  c2_jd_y = c2_sc_a * c2_id_b;
  c2_tc_a = c2_q2;
  c2_jd_b = c2_q0;
  c2_kd_y = c2_tc_a * c2_jd_b;
  c2_kd_b = c2_jd_y - c2_kd_y;
  c2_ld_y = 2.0 * c2_kd_b;
  c2_uc_a = c2_q1;
  c2_ld_b = c2_q0;
  c2_md_y = c2_uc_a * c2_ld_b;
  c2_vc_a = c2_q3;
  c2_md_b = c2_q2;
  c2_nd_y = c2_vc_a * c2_md_b;
  c2_nd_b = c2_md_y + c2_nd_y;
  c2_od_y = 2.0 * c2_nd_b;
  c2_od_b = c2_mpower(chartInstance, c2_q1) + c2_mpower(chartInstance, c2_q2);
  c2_pd_y = 2.0 * c2_od_b;
  c2_T_e2E[0] = 1.0 - c2_uc_y;
  c2_T_e2E[3] = c2_xc_y;
  c2_T_e2E[6] = c2_bd_y;
  c2_T_e2E[1] = c2_ed_y;
  c2_T_e2E[4] = 1.0 - c2_fd_y;
  c2_T_e2E[7] = c2_id_y;
  c2_T_e2E[2] = c2_ld_y;
  c2_T_e2E[5] = c2_od_y;
  c2_T_e2E[8] = 1.0 - c2_pd_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 50);
  c2_wc_a = -c2_b1;
  c2_pd_b = c2_mpower(chartInstance, c2_w1);
  c2_qd_y = c2_wc_a * c2_pd_b;
  c2_xc_a = c2_b3;
  c2_qd_b = c2_mpower(chartInstance, c2_w3);
  c2_rd_y = c2_xc_a * c2_qd_b;
  c2_yc_a = c2_b1;
  c2_rd_b = c2_mpower(chartInstance, c2_w1);
  c2_sd_y = c2_yc_a * c2_rd_b;
  c2_ad_a = c2_b3;
  c2_sd_b = c2_mpower(chartInstance, c2_w3);
  c2_td_y = c2_ad_a * c2_sd_b;
  c2_bd_a = c2_k1;
  c2_td_b = c2_mpower(chartInstance, c2_w1);
  c2_ud_y = c2_bd_a * c2_td_b;
  c2_cd_a = c2_k3;
  c2_ud_b = c2_mpower(chartInstance, c2_w3);
  c2_vd_y = c2_cd_a * c2_ud_b;
  c2_force_mult_1[0] = c2_qd_y - c2_rd_y;
  c2_force_mult_1[1] = c2_sd_y + c2_td_y;
  c2_force_mult_1[2] = c2_ud_y - c2_vd_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
  c2_dd_a = c2_b2;
  c2_vd_b = c2_mpower(chartInstance, c2_w2);
  c2_wd_y = c2_dd_a * c2_vd_b;
  c2_ed_a = c2_b4;
  c2_wd_b = c2_mpower(chartInstance, c2_w4);
  c2_xd_y = c2_ed_a * c2_wd_b;
  c2_fd_a = c2_b2;
  c2_xd_b = c2_mpower(chartInstance, c2_w2);
  c2_yd_y = c2_fd_a * c2_xd_b;
  c2_gd_a = c2_b4;
  c2_yd_b = c2_mpower(chartInstance, c2_w4);
  c2_ae_y = c2_gd_a * c2_yd_b;
  c2_hd_a = c2_k2;
  c2_ae_b = c2_mpower(chartInstance, c2_w2);
  c2_be_y = c2_hd_a * c2_ae_b;
  c2_id_a = c2_k4;
  c2_be_b = c2_mpower(chartInstance, c2_w4);
  c2_ce_y = c2_id_a * c2_be_b;
  c2_force_mult_2[0] = c2_wd_y + c2_xd_y;
  c2_force_mult_2[1] = c2_yd_y + c2_ae_y;
  c2_force_mult_2[2] = c2_be_y - c2_ce_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
  for (c2_i12 = 0; c2_i12 < 9; c2_i12++) {
    c2_jd_a[c2_i12] = c2_T_e2E[c2_i12];
  }

  for (c2_i13 = 0; c2_i13 < 3; c2_i13++) {
    c2_ce_b[c2_i13] = c2_force_mult_1[c2_i13];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i14 = 0; c2_i14 < 3; c2_i14++) {
    c2_force_mult_1_E[c2_i14] = 0.0;
  }

  for (c2_i15 = 0; c2_i15 < 3; c2_i15++) {
    c2_force_mult_1_E[c2_i15] = 0.0;
  }

  for (c2_i16 = 0; c2_i16 < 3; c2_i16++) {
    c2_C[c2_i16] = c2_force_mult_1_E[c2_i16];
  }

  for (c2_i17 = 0; c2_i17 < 3; c2_i17++) {
    c2_force_mult_1_E[c2_i17] = c2_C[c2_i17];
  }

  for (c2_i18 = 0; c2_i18 < 3; c2_i18++) {
    c2_C[c2_i18] = c2_force_mult_1_E[c2_i18];
  }

  for (c2_i19 = 0; c2_i19 < 3; c2_i19++) {
    c2_force_mult_1_E[c2_i19] = c2_C[c2_i19];
  }

  for (c2_i20 = 0; c2_i20 < 3; c2_i20++) {
    c2_force_mult_1_E[c2_i20] = 0.0;
    c2_i21 = 0;
    for (c2_i22 = 0; c2_i22 < 3; c2_i22++) {
      c2_force_mult_1_E[c2_i20] += c2_jd_a[c2_i21 + c2_i20] * c2_ce_b[c2_i22];
      c2_i21 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 54);
  for (c2_i23 = 0; c2_i23 < 9; c2_i23++) {
    c2_jd_a[c2_i23] = c2_T_e2E[c2_i23];
  }

  for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
    c2_ce_b[c2_i24] = c2_force_mult_2[c2_i24];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i25 = 0; c2_i25 < 3; c2_i25++) {
    c2_force_mult_2_E[c2_i25] = 0.0;
  }

  for (c2_i26 = 0; c2_i26 < 3; c2_i26++) {
    c2_force_mult_2_E[c2_i26] = 0.0;
  }

  for (c2_i27 = 0; c2_i27 < 3; c2_i27++) {
    c2_C[c2_i27] = c2_force_mult_2_E[c2_i27];
  }

  for (c2_i28 = 0; c2_i28 < 3; c2_i28++) {
    c2_force_mult_2_E[c2_i28] = c2_C[c2_i28];
  }

  for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
    c2_C[c2_i29] = c2_force_mult_2_E[c2_i29];
  }

  for (c2_i30 = 0; c2_i30 < 3; c2_i30++) {
    c2_force_mult_2_E[c2_i30] = c2_C[c2_i30];
  }

  for (c2_i31 = 0; c2_i31 < 3; c2_i31++) {
    c2_force_mult_2_E[c2_i31] = 0.0;
    c2_i32 = 0;
    for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
      c2_force_mult_2_E[c2_i31] += c2_jd_a[c2_i32 + c2_i31] * c2_ce_b[c2_i33];
      c2_i32 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 56);
  c2_A = c2_l;
  c2_b_x = c2_A;
  c2_c_x = c2_b_x;
  c2_de_y = c2_c_x / 1.4142135623730951;
  c2_de_b = c2_q3;
  c2_ee_y = -2.0 * c2_de_b;
  c2_ee_b = c2_q3;
  c2_fe_y = 2.0 * c2_ee_b;
  c2_fe_b = c2_q1;
  c2_ge_y = 2.0 * c2_fe_b;
  c2_ge_b = c2_q2;
  c2_he_y = 2.0 * c2_ge_b;
  c2_kd_a = c2_de_y;
  c2_ce_b[0] = c2_ee_y;
  c2_ce_b[1] = c2_fe_y;
  c2_ce_b[2] = c2_ge_y - c2_he_y;
  for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
    c2_dx1_dq0[c2_i34] = c2_kd_a * c2_ce_b[c2_i34];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 57);
  c2_b_A = c2_l;
  c2_d_x = c2_b_A;
  c2_e_x = c2_d_x;
  c2_ie_y = c2_e_x / 1.4142135623730951;
  c2_he_b = c2_q3;
  c2_je_y = -2.0 * c2_he_b;
  c2_ie_b = c2_q3;
  c2_ke_y = -2.0 * c2_ie_b;
  c2_je_b = c2_q1;
  c2_le_y = 2.0 * c2_je_b;
  c2_ke_b = c2_q2;
  c2_me_y = 2.0 * c2_ke_b;
  c2_ld_a = c2_ie_y;
  c2_ce_b[0] = c2_je_y;
  c2_ce_b[1] = c2_ke_y;
  c2_ce_b[2] = c2_le_y + c2_me_y;
  for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
    c2_dx2_dq0[c2_i35] = c2_ld_a * c2_ce_b[c2_i35];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 58);
  for (c2_i36 = 0; c2_i36 < 3; c2_i36++) {
    c2_b_force_mult_1_E[c2_i36] = c2_force_mult_1_E[c2_i36];
  }

  for (c2_i37 = 0; c2_i37 < 3; c2_i37++) {
    c2_b_dx1_dq0[c2_i37] = c2_dx1_dq0[c2_i37];
  }

  for (c2_i38 = 0; c2_i38 < 3; c2_i38++) {
    c2_b_force_mult_2_E[c2_i38] = c2_force_mult_2_E[c2_i38];
  }

  for (c2_i39 = 0; c2_i39 < 3; c2_i39++) {
    c2_b_dx2_dq0[c2_i39] = c2_dx2_dq0[c2_i39];
  }

  c2_upphi_q0 = c2_dot(chartInstance, c2_b_force_mult_1_E, c2_b_dx1_dq0) +
    c2_dot(chartInstance, c2_b_force_mult_2_E, c2_b_dx2_dq0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 60);
  c2_c_A = c2_l;
  c2_f_x = c2_c_A;
  c2_g_x = c2_f_x;
  c2_ne_y = c2_g_x / 1.4142135623730951;
  c2_le_b = c2_q2;
  c2_oe_y = 2.0 * c2_le_b;
  c2_me_b = c2_q2;
  c2_pe_y = 2.0 * c2_me_b;
  c2_ne_b = c2_q1;
  c2_qe_y = 4.0 * c2_ne_b;
  c2_oe_b = c2_q0;
  c2_re_y = 2.0 * c2_oe_b;
  c2_pe_b = c2_q3;
  c2_se_y = 2.0 * c2_pe_b;
  c2_md_a = c2_ne_y;
  c2_ce_b[0] = c2_oe_y;
  c2_ce_b[1] = c2_pe_y - c2_qe_y;
  c2_ce_b[2] = c2_re_y + c2_se_y;
  for (c2_i40 = 0; c2_i40 < 3; c2_i40++) {
    c2_dx1_dq1[c2_i40] = c2_md_a * c2_ce_b[c2_i40];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 61);
  c2_d_A = c2_l;
  c2_h_x = c2_d_A;
  c2_i_x = c2_h_x;
  c2_te_y = c2_i_x / 1.4142135623730951;
  c2_qe_b = c2_q2;
  c2_ue_y = 2.0 * c2_qe_b;
  c2_re_b = c2_q1;
  c2_ve_y = -4.0 * c2_re_b;
  c2_se_b = c2_q2;
  c2_we_y = 2.0 * c2_se_b;
  c2_te_b = c2_q0;
  c2_xe_y = 2.0 * c2_te_b;
  c2_ue_b = c2_q3;
  c2_ye_y = 2.0 * c2_ue_b;
  c2_nd_a = c2_te_y;
  c2_ce_b[0] = c2_ue_y;
  c2_ce_b[1] = c2_ve_y - c2_we_y;
  c2_ce_b[2] = c2_xe_y - c2_ye_y;
  for (c2_i41 = 0; c2_i41 < 3; c2_i41++) {
    c2_dx2_dq1[c2_i41] = c2_nd_a * c2_ce_b[c2_i41];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 62);
  for (c2_i42 = 0; c2_i42 < 3; c2_i42++) {
    c2_c_force_mult_1_E[c2_i42] = c2_force_mult_1_E[c2_i42];
  }

  for (c2_i43 = 0; c2_i43 < 3; c2_i43++) {
    c2_b_dx1_dq1[c2_i43] = c2_dx1_dq1[c2_i43];
  }

  for (c2_i44 = 0; c2_i44 < 3; c2_i44++) {
    c2_c_force_mult_2_E[c2_i44] = c2_force_mult_2_E[c2_i44];
  }

  for (c2_i45 = 0; c2_i45 < 3; c2_i45++) {
    c2_b_dx2_dq1[c2_i45] = c2_dx2_dq1[c2_i45];
  }

  c2_upphi_q1 = c2_dot(chartInstance, c2_c_force_mult_1_E, c2_b_dx1_dq1) +
    c2_dot(chartInstance, c2_c_force_mult_2_E, c2_b_dx2_dq1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 64);
  c2_e_A = c2_l;
  c2_j_x = c2_e_A;
  c2_k_x = c2_j_x;
  c2_af_y = c2_k_x / 1.4142135623730951;
  c2_ve_b = c2_q1;
  c2_bf_y = 2.0 * c2_ve_b;
  c2_we_b = c2_q2;
  c2_cf_y = 4.0 * c2_we_b;
  c2_xe_b = c2_q1;
  c2_df_y = 2.0 * c2_xe_b;
  c2_ye_b = c2_q3;
  c2_ef_y = 2.0 * c2_ye_b;
  c2_af_b = c2_q0;
  c2_ff_y = 2.0 * c2_af_b;
  c2_od_a = c2_af_y;
  c2_ce_b[0] = c2_bf_y - c2_cf_y;
  c2_ce_b[1] = c2_df_y;
  c2_ce_b[2] = c2_ef_y - c2_ff_y;
  for (c2_i46 = 0; c2_i46 < 3; c2_i46++) {
    c2_dx1_dq2[c2_i46] = c2_od_a * c2_ce_b[c2_i46];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 65);
  c2_f_A = c2_l;
  c2_l_x = c2_f_A;
  c2_m_x = c2_l_x;
  c2_gf_y = c2_m_x / 1.4142135623730951;
  c2_bf_b = c2_q1;
  c2_hf_y = 2.0 * c2_bf_b;
  c2_cf_b = c2_q2;
  c2_if_y = 4.0 * c2_cf_b;
  c2_df_b = c2_q1;
  c2_jf_y = -2.0 * c2_df_b;
  c2_ef_b = c2_q3;
  c2_kf_y = 2.0 * c2_ef_b;
  c2_ff_b = c2_q0;
  c2_lf_y = 2.0 * c2_ff_b;
  c2_pd_a = c2_gf_y;
  c2_ce_b[0] = c2_hf_y + c2_if_y;
  c2_ce_b[1] = c2_jf_y;
  c2_ce_b[2] = c2_kf_y + c2_lf_y;
  for (c2_i47 = 0; c2_i47 < 3; c2_i47++) {
    c2_dx2_dq2[c2_i47] = c2_pd_a * c2_ce_b[c2_i47];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 66);
  for (c2_i48 = 0; c2_i48 < 3; c2_i48++) {
    c2_d_force_mult_1_E[c2_i48] = c2_force_mult_1_E[c2_i48];
  }

  for (c2_i49 = 0; c2_i49 < 3; c2_i49++) {
    c2_b_dx1_dq2[c2_i49] = c2_dx1_dq2[c2_i49];
  }

  for (c2_i50 = 0; c2_i50 < 3; c2_i50++) {
    c2_d_force_mult_2_E[c2_i50] = c2_force_mult_2_E[c2_i50];
  }

  for (c2_i51 = 0; c2_i51 < 3; c2_i51++) {
    c2_b_dx2_dq2[c2_i51] = c2_dx2_dq2[c2_i51];
  }

  c2_upphi_q2 = c2_dot(chartInstance, c2_d_force_mult_1_E, c2_b_dx1_dq2) +
    c2_dot(chartInstance, c2_d_force_mult_2_E, c2_b_dx2_dq2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 68);
  c2_g_A = c2_l;
  c2_n_x = c2_g_A;
  c2_o_x = c2_n_x;
  c2_mf_y = c2_o_x / 1.4142135623730951;
  c2_gf_b = c2_q0;
  c2_nf_y = -2.0 * c2_gf_b;
  c2_hf_b = c2_q3;
  c2_of_y = 4.0 * c2_hf_b;
  c2_if_b = c2_q0;
  c2_pf_y = 2.0 * c2_if_b;
  c2_jf_b = c2_q3;
  c2_qf_y = 4.0 * c2_jf_b;
  c2_kf_b = c2_q1;
  c2_rf_y = 2.0 * c2_kf_b;
  c2_lf_b = c2_q2;
  c2_sf_y = 2.0 * c2_lf_b;
  c2_qd_a = c2_mf_y;
  c2_ce_b[0] = c2_nf_y - c2_of_y;
  c2_ce_b[1] = c2_pf_y - c2_qf_y;
  c2_ce_b[2] = c2_rf_y + c2_sf_y;
  for (c2_i52 = 0; c2_i52 < 3; c2_i52++) {
    c2_dx1_dq3[c2_i52] = c2_qd_a * c2_ce_b[c2_i52];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 69);
  c2_h_A = c2_l;
  c2_p_x = c2_h_A;
  c2_q_x = c2_p_x;
  c2_tf_y = c2_q_x / 1.4142135623730951;
  c2_mf_b = c2_q0;
  c2_uf_y = -2.0 * c2_mf_b;
  c2_nf_b = c2_q3;
  c2_vf_y = 4.0 * c2_nf_b;
  c2_of_b = c2_q3;
  c2_wf_y = -4.0 * c2_of_b;
  c2_pf_b = c2_q0;
  c2_xf_y = 2.0 * c2_pf_b;
  c2_qf_b = c2_q2;
  c2_yf_y = 2.0 * c2_qf_b;
  c2_rf_b = c2_q1;
  c2_ag_y = 2.0 * c2_rf_b;
  c2_rd_a = c2_tf_y;
  c2_ce_b[0] = c2_uf_y + c2_vf_y;
  c2_ce_b[1] = c2_wf_y - c2_xf_y;
  c2_ce_b[2] = c2_yf_y - c2_ag_y;
  for (c2_i53 = 0; c2_i53 < 3; c2_i53++) {
    c2_dx2_dq3[c2_i53] = c2_rd_a * c2_ce_b[c2_i53];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 70);
  for (c2_i54 = 0; c2_i54 < 3; c2_i54++) {
    c2_e_force_mult_1_E[c2_i54] = c2_force_mult_1_E[c2_i54];
  }

  for (c2_i55 = 0; c2_i55 < 3; c2_i55++) {
    c2_b_dx1_dq3[c2_i55] = c2_dx1_dq3[c2_i55];
  }

  for (c2_i56 = 0; c2_i56 < 3; c2_i56++) {
    c2_e_force_mult_2_E[c2_i56] = c2_force_mult_2_E[c2_i56];
  }

  for (c2_i57 = 0; c2_i57 < 3; c2_i57++) {
    c2_b_dx2_dq3[c2_i57] = c2_dx2_dq3[c2_i57];
  }

  c2_upphi_q3 = c2_dot(chartInstance, c2_e_force_mult_1_E, c2_b_dx1_dq3) +
    c2_dot(chartInstance, c2_e_force_mult_2_E, c2_b_dx2_dq3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 73);
  c2_delta1 = c2_upphi_x;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 75);
  c2_delta2 = c2_upphi_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 77);
  c2_delta3 = c2_upphi_z;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 80);
  c2_sf_b = c2_Jxx;
  c2_bg_y = 8.0 * c2_sf_b;
  c2_sd_a = c2_bg_y;
  c2_tf_b = c2_q0_dot;
  c2_cg_y = c2_sd_a * c2_tf_b;
  c2_td_a = c2_cg_y;
  c2_uf_b = c2_q1_dot;
  c2_dg_y = c2_td_a * c2_uf_b;
  c2_ud_a = c2_dg_y;
  c2_vf_b = c2_q1;
  c2_eg_y = c2_ud_a * c2_vf_b;
  c2_wf_b = c2_Jyy;
  c2_fg_y = 8.0 * c2_wf_b;
  c2_vd_a = c2_fg_y;
  c2_xf_b = c2_q0_dot;
  c2_gg_y = c2_vd_a * c2_xf_b;
  c2_wd_a = c2_gg_y;
  c2_yf_b = c2_q2_dot;
  c2_hg_y = c2_wd_a * c2_yf_b;
  c2_xd_a = c2_hg_y;
  c2_ag_b = c2_q2;
  c2_ig_y = c2_xd_a * c2_ag_b;
  c2_bg_b = c2_Jzz;
  c2_jg_y = 8.0 * c2_bg_b;
  c2_yd_a = c2_jg_y;
  c2_cg_b = c2_q0_dot;
  c2_kg_y = c2_yd_a * c2_cg_b;
  c2_ae_a = c2_kg_y;
  c2_dg_b = c2_q3_dot;
  c2_lg_y = c2_ae_a * c2_dg_b;
  c2_be_a = c2_lg_y;
  c2_eg_b = c2_q3;
  c2_mg_y = c2_be_a * c2_eg_b;
  c2_fg_b = c2_Jxx;
  c2_ng_y = 8.0 * c2_fg_b;
  c2_ce_a = c2_ng_y;
  c2_gg_b = c2_mpower(chartInstance, c2_q1_dot);
  c2_og_y = c2_ce_a * c2_gg_b;
  c2_de_a = c2_og_y;
  c2_hg_b = c2_q0;
  c2_pg_y = c2_de_a * c2_hg_b;
  c2_ig_b = c2_Jyy;
  c2_qg_y = 8.0 * c2_ig_b;
  c2_ee_a = c2_qg_y;
  c2_jg_b = c2_mpower(chartInstance, c2_q2_dot);
  c2_rg_y = c2_ee_a * c2_jg_b;
  c2_fe_a = c2_rg_y;
  c2_kg_b = c2_q0;
  c2_sg_y = c2_fe_a * c2_kg_b;
  c2_lg_b = c2_Jzz;
  c2_tg_y = 8.0 * c2_lg_b;
  c2_ge_a = c2_tg_y;
  c2_mg_b = c2_mpower(chartInstance, c2_q3_dot);
  c2_ug_y = c2_ge_a * c2_mg_b;
  c2_he_a = c2_ug_y;
  c2_ng_b = c2_q0;
  c2_vg_y = c2_he_a * c2_ng_b;
  c2_og_b = c2_Jxx;
  c2_wg_y = 8.0 * c2_og_b;
  c2_pg_b = c2_Jyy;
  c2_xg_y = 8.0 * c2_pg_b;
  c2_ie_a = c2_wg_y - c2_xg_y;
  c2_qg_b = c2_q1_dot;
  c2_yg_y = c2_ie_a * c2_qg_b;
  c2_je_a = c2_yg_y;
  c2_rg_b = c2_q2_dot;
  c2_ah_y = c2_je_a * c2_rg_b;
  c2_ke_a = c2_ah_y;
  c2_sg_b = c2_q3;
  c2_bh_y = c2_ke_a * c2_sg_b;
  c2_tg_b = c2_Jzz;
  c2_ch_y = 8.0 * c2_tg_b;
  c2_ug_b = c2_Jxx;
  c2_dh_y = 8.0 * c2_ug_b;
  c2_le_a = c2_ch_y - c2_dh_y;
  c2_vg_b = c2_q1_dot;
  c2_eh_y = c2_le_a * c2_vg_b;
  c2_me_a = c2_eh_y;
  c2_wg_b = c2_q3_dot;
  c2_fh_y = c2_me_a * c2_wg_b;
  c2_ne_a = c2_fh_y;
  c2_xg_b = c2_q2;
  c2_gh_y = c2_ne_a * c2_xg_b;
  c2_yg_b = c2_Jyy;
  c2_hh_y = 8.0 * c2_yg_b;
  c2_ah_b = c2_Jzz;
  c2_ih_y = 8.0 * c2_ah_b;
  c2_oe_a = c2_hh_y - c2_ih_y;
  c2_bh_b = c2_q2_dot;
  c2_jh_y = c2_oe_a * c2_bh_b;
  c2_pe_a = c2_jh_y;
  c2_ch_b = c2_q3_dot;
  c2_kh_y = c2_pe_a * c2_ch_b;
  c2_qe_a = c2_kh_y;
  c2_dh_b = c2_q1;
  c2_lh_y = c2_qe_a * c2_dh_b;
  c2_delta4 = ((((((((c2_upphi_q0 - c2_eg_y) - c2_ig_y) - c2_mg_y) + c2_pg_y) +
                  c2_sg_y) + c2_vg_y) + c2_bh_y) + c2_gh_y) + c2_lh_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 85);
  c2_eh_b = c2_Jxx;
  c2_mh_y = 8.0 * c2_eh_b;
  c2_re_a = c2_mh_y;
  c2_fh_b = c2_q0_dot;
  c2_nh_y = c2_re_a * c2_fh_b;
  c2_se_a = c2_nh_y;
  c2_gh_b = c2_q1_dot;
  c2_oh_y = c2_se_a * c2_gh_b;
  c2_te_a = c2_oh_y;
  c2_hh_b = c2_q0;
  c2_ph_y = c2_te_a * c2_hh_b;
  c2_ih_b = c2_Jyy;
  c2_qh_y = 8.0 * c2_ih_b;
  c2_ue_a = c2_qh_y;
  c2_jh_b = c2_q1_dot;
  c2_rh_y = c2_ue_a * c2_jh_b;
  c2_ve_a = c2_rh_y;
  c2_kh_b = c2_q3_dot;
  c2_sh_y = c2_ve_a * c2_kh_b;
  c2_we_a = c2_sh_y;
  c2_lh_b = c2_q3;
  c2_th_y = c2_we_a * c2_lh_b;
  c2_mh_b = c2_Jzz;
  c2_uh_y = 8.0 * c2_mh_b;
  c2_xe_a = c2_uh_y;
  c2_nh_b = c2_q1_dot;
  c2_vh_y = c2_xe_a * c2_nh_b;
  c2_ye_a = c2_vh_y;
  c2_oh_b = c2_q2_dot;
  c2_wh_y = c2_ye_a * c2_oh_b;
  c2_af_a = c2_wh_y;
  c2_ph_b = c2_q2;
  c2_xh_y = c2_af_a * c2_ph_b;
  c2_qh_b = c2_Jyy;
  c2_yh_y = 8.0 * c2_qh_b;
  c2_rh_b = c2_Jzz;
  c2_ai_y = 8.0 * c2_rh_b;
  c2_bf_a = c2_yh_y - c2_ai_y;
  c2_sh_b = c2_q2_dot;
  c2_bi_y = c2_bf_a * c2_sh_b;
  c2_cf_a = c2_bi_y;
  c2_th_b = c2_q3_dot;
  c2_ci_y = c2_cf_a * c2_th_b;
  c2_df_a = c2_ci_y;
  c2_uh_b = c2_q0;
  c2_di_y = c2_df_a * c2_uh_b;
  c2_vh_b = c2_Jxx;
  c2_ei_y = 8.0 * c2_vh_b;
  c2_wh_b = c2_Jyy;
  c2_fi_y = 8.0 * c2_wh_b;
  c2_ef_a = c2_ei_y - c2_fi_y;
  c2_xh_b = c2_q0_dot;
  c2_gi_y = c2_ef_a * c2_xh_b;
  c2_ff_a = c2_gi_y;
  c2_yh_b = c2_q3_dot;
  c2_hi_y = c2_ff_a * c2_yh_b;
  c2_gf_a = c2_hi_y;
  c2_ai_b = c2_q2;
  c2_ii_y = c2_gf_a * c2_ai_b;
  c2_bi_b = c2_Jzz;
  c2_ji_y = 8.0 * c2_bi_b;
  c2_ci_b = c2_Jxx;
  c2_ki_y = 8.0 * c2_ci_b;
  c2_hf_a = c2_ji_y - c2_ki_y;
  c2_di_b = c2_q0_dot;
  c2_li_y = c2_hf_a * c2_di_b;
  c2_if_a = c2_li_y;
  c2_ei_b = c2_q2_dot;
  c2_mi_y = c2_if_a * c2_ei_b;
  c2_jf_a = c2_mi_y;
  c2_fi_b = c2_q3;
  c2_ni_y = c2_jf_a * c2_fi_b;
  c2_gi_b = c2_Jxx;
  c2_oi_y = 8.0 * c2_gi_b;
  c2_kf_a = c2_oi_y;
  c2_hi_b = c2_mpower(chartInstance, c2_q0_dot);
  c2_pi_y = c2_kf_a * c2_hi_b;
  c2_lf_a = c2_pi_y;
  c2_ii_b = c2_q1;
  c2_qi_y = c2_lf_a * c2_ii_b;
  c2_ji_b = c2_Jyy;
  c2_ri_y = 8.0 * c2_ji_b;
  c2_mf_a = c2_ri_y;
  c2_ki_b = c2_mpower(chartInstance, c2_q3_dot);
  c2_si_y = c2_mf_a * c2_ki_b;
  c2_nf_a = c2_si_y;
  c2_li_b = c2_q1;
  c2_ti_y = c2_nf_a * c2_li_b;
  c2_mi_b = c2_Jzz;
  c2_ui_y = 8.0 * c2_mi_b;
  c2_of_a = c2_ui_y;
  c2_ni_b = c2_mpower(chartInstance, c2_q2_dot);
  c2_vi_y = c2_of_a * c2_ni_b;
  c2_pf_a = c2_vi_y;
  c2_oi_b = c2_q1;
  c2_wi_y = c2_pf_a * c2_oi_b;
  c2_delta5 = ((((((((c2_upphi_q1 - c2_ph_y) - c2_th_y) - c2_xh_y) + c2_di_y) +
                  c2_ii_y) + c2_ni_y) + c2_qi_y) + c2_ti_y) + c2_wi_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 90);
  c2_pi_b = c2_Jxx;
  c2_xi_y = 8.0 * c2_pi_b;
  c2_qf_a = c2_xi_y;
  c2_qi_b = c2_q2_dot;
  c2_yi_y = c2_qf_a * c2_qi_b;
  c2_rf_a = c2_yi_y;
  c2_ri_b = c2_q3_dot;
  c2_aj_y = c2_rf_a * c2_ri_b;
  c2_sf_a = c2_aj_y;
  c2_si_b = c2_q3;
  c2_bj_y = c2_sf_a * c2_si_b;
  c2_ti_b = c2_Jyy;
  c2_cj_y = 8.0 * c2_ti_b;
  c2_tf_a = c2_cj_y;
  c2_ui_b = c2_q0_dot;
  c2_dj_y = c2_tf_a * c2_ui_b;
  c2_uf_a = c2_dj_y;
  c2_vi_b = c2_q2_dot;
  c2_ej_y = c2_uf_a * c2_vi_b;
  c2_vf_a = c2_ej_y;
  c2_wi_b = c2_q0;
  c2_fj_y = c2_vf_a * c2_wi_b;
  c2_xi_b = c2_Jzz;
  c2_gj_y = 8.0 * c2_xi_b;
  c2_wf_a = c2_gj_y;
  c2_yi_b = c2_q1_dot;
  c2_hj_y = c2_wf_a * c2_yi_b;
  c2_xf_a = c2_hj_y;
  c2_aj_b = c2_q2_dot;
  c2_ij_y = c2_xf_a * c2_aj_b;
  c2_yf_a = c2_ij_y;
  c2_bj_b = c2_q1;
  c2_jj_y = c2_yf_a * c2_bj_b;
  c2_cj_b = c2_Jxx;
  c2_kj_y = 8.0 * c2_cj_b;
  c2_ag_a = c2_kj_y;
  c2_dj_b = c2_mpower(chartInstance, c2_q3_dot);
  c2_lj_y = c2_ag_a * c2_dj_b;
  c2_bg_a = c2_lj_y;
  c2_ej_b = c2_q2;
  c2_mj_y = c2_bg_a * c2_ej_b;
  c2_fj_b = c2_Jyy;
  c2_nj_y = 8.0 * c2_fj_b;
  c2_cg_a = c2_nj_y;
  c2_gj_b = c2_mpower(chartInstance, c2_q0_dot);
  c2_oj_y = c2_cg_a * c2_gj_b;
  c2_dg_a = c2_oj_y;
  c2_hj_b = c2_q2;
  c2_pj_y = c2_dg_a * c2_hj_b;
  c2_ij_b = c2_Jzz;
  c2_qj_y = 8.0 * c2_ij_b;
  c2_eg_a = c2_qj_y;
  c2_jj_b = c2_mpower(chartInstance, c2_q1_dot);
  c2_rj_y = c2_eg_a * c2_jj_b;
  c2_fg_a = c2_rj_y;
  c2_kj_b = c2_q2;
  c2_sj_y = c2_fg_a * c2_kj_b;
  c2_lj_b = c2_Jxx - c2_Jyy;
  c2_tj_y = 8.0 * c2_lj_b;
  c2_gg_a = c2_tj_y;
  c2_mj_b = c2_q0_dot;
  c2_uj_y = c2_gg_a * c2_mj_b;
  c2_hg_a = c2_uj_y;
  c2_nj_b = c2_q3_dot;
  c2_vj_y = c2_hg_a * c2_nj_b;
  c2_ig_a = c2_vj_y;
  c2_oj_b = c2_q1;
  c2_wj_y = c2_ig_a * c2_oj_b;
  c2_pj_b = c2_Jyy - c2_Jzz;
  c2_xj_y = 8.0 * c2_pj_b;
  c2_jg_a = c2_xj_y;
  c2_qj_b = c2_q0_dot;
  c2_yj_y = c2_jg_a * c2_qj_b;
  c2_kg_a = c2_yj_y;
  c2_rj_b = c2_q1_dot;
  c2_ak_y = c2_kg_a * c2_rj_b;
  c2_lg_a = c2_ak_y;
  c2_sj_b = c2_q3;
  c2_bk_y = c2_lg_a * c2_sj_b;
  c2_tj_b = c2_Jzz - c2_Jxx;
  c2_ck_y = 8.0 * c2_tj_b;
  c2_mg_a = c2_ck_y;
  c2_uj_b = c2_q1_dot;
  c2_dk_y = c2_mg_a * c2_uj_b;
  c2_ng_a = c2_dk_y;
  c2_vj_b = c2_q3_dot;
  c2_ek_y = c2_ng_a * c2_vj_b;
  c2_og_a = c2_ek_y;
  c2_wj_b = c2_q0;
  c2_fk_y = c2_og_a * c2_wj_b;
  c2_delta6 = ((((((((c2_upphi_q2 - c2_bj_y) - c2_fj_y) - c2_jj_y) + c2_mj_y) +
                  c2_pj_y) + c2_sj_y) + c2_wj_y) + c2_bk_y) + c2_fk_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 95);
  c2_xj_b = c2_Jxx;
  c2_gk_y = 8.0 * c2_xj_b;
  c2_pg_a = c2_gk_y;
  c2_yj_b = c2_q2_dot;
  c2_hk_y = c2_pg_a * c2_yj_b;
  c2_qg_a = c2_hk_y;
  c2_ak_b = c2_q3_dot;
  c2_ik_y = c2_qg_a * c2_ak_b;
  c2_rg_a = c2_ik_y;
  c2_bk_b = c2_q2;
  c2_jk_y = c2_rg_a * c2_bk_b;
  c2_ck_b = c2_Jyy;
  c2_kk_y = 8.0 * c2_ck_b;
  c2_sg_a = c2_kk_y;
  c2_dk_b = c2_q1_dot;
  c2_lk_y = c2_sg_a * c2_dk_b;
  c2_tg_a = c2_lk_y;
  c2_ek_b = c2_q3_dot;
  c2_mk_y = c2_tg_a * c2_ek_b;
  c2_ug_a = c2_mk_y;
  c2_fk_b = c2_q1;
  c2_nk_y = c2_ug_a * c2_fk_b;
  c2_gk_b = c2_Jzz;
  c2_ok_y = 8.0 * c2_gk_b;
  c2_vg_a = c2_ok_y;
  c2_hk_b = c2_q0_dot;
  c2_pk_y = c2_vg_a * c2_hk_b;
  c2_wg_a = c2_pk_y;
  c2_ik_b = c2_q3_dot;
  c2_qk_y = c2_wg_a * c2_ik_b;
  c2_xg_a = c2_qk_y;
  c2_jk_b = c2_q0;
  c2_rk_y = c2_xg_a * c2_jk_b;
  c2_kk_b = c2_Jxx;
  c2_sk_y = 8.0 * c2_kk_b;
  c2_yg_a = c2_sk_y;
  c2_lk_b = c2_mpower(chartInstance, c2_q2_dot);
  c2_tk_y = c2_yg_a * c2_lk_b;
  c2_ah_a = c2_tk_y;
  c2_mk_b = c2_q3;
  c2_uk_y = c2_ah_a * c2_mk_b;
  c2_nk_b = c2_Jyy;
  c2_vk_y = 8.0 * c2_nk_b;
  c2_bh_a = c2_vk_y;
  c2_ok_b = c2_mpower(chartInstance, c2_q1_dot);
  c2_wk_y = c2_bh_a * c2_ok_b;
  c2_ch_a = c2_wk_y;
  c2_pk_b = c2_q3;
  c2_xk_y = c2_ch_a * c2_pk_b;
  c2_qk_b = c2_Jzz;
  c2_yk_y = 8.0 * c2_qk_b;
  c2_dh_a = c2_yk_y;
  c2_rk_b = c2_mpower(chartInstance, c2_q0_dot);
  c2_al_y = c2_dh_a * c2_rk_b;
  c2_eh_a = c2_al_y;
  c2_sk_b = c2_q3;
  c2_bl_y = c2_eh_a * c2_sk_b;
  c2_tk_b = c2_Jzz;
  c2_cl_y = 8.0 * c2_tk_b;
  c2_uk_b = c2_Jxx;
  c2_dl_y = 8.0 * c2_uk_b;
  c2_fh_a = c2_cl_y - c2_dl_y;
  c2_vk_b = c2_q0_dot;
  c2_el_y = c2_fh_a * c2_vk_b;
  c2_gh_a = c2_el_y;
  c2_wk_b = c2_q2_dot;
  c2_fl_y = c2_gh_a * c2_wk_b;
  c2_hh_a = c2_fl_y;
  c2_xk_b = c2_q1;
  c2_gl_y = c2_hh_a * c2_xk_b;
  c2_yk_b = c2_Jxx;
  c2_hl_y = 8.0 * c2_yk_b;
  c2_al_b = c2_Jyy;
  c2_il_y = 8.0 * c2_al_b;
  c2_ih_a = c2_hl_y - c2_il_y;
  c2_bl_b = c2_q1_dot;
  c2_jl_y = c2_ih_a * c2_bl_b;
  c2_jh_a = c2_jl_y;
  c2_cl_b = c2_q2_dot;
  c2_kl_y = c2_jh_a * c2_cl_b;
  c2_kh_a = c2_kl_y;
  c2_dl_b = c2_q0;
  c2_ll_y = c2_kh_a * c2_dl_b;
  c2_el_b = c2_Jyy;
  c2_ml_y = 8.0 * c2_el_b;
  c2_fl_b = c2_Jzz;
  c2_nl_y = 8.0 * c2_fl_b;
  c2_lh_a = c2_ml_y - c2_nl_y;
  c2_gl_b = c2_q0_dot;
  c2_ol_y = c2_lh_a * c2_gl_b;
  c2_mh_a = c2_ol_y;
  c2_hl_b = c2_q1_dot;
  c2_pl_y = c2_mh_a * c2_hl_b;
  c2_nh_a = c2_pl_y;
  c2_il_b = c2_q2;
  c2_ql_y = c2_nh_a * c2_il_b;
  c2_delta7 = ((((((((c2_upphi_q3 - c2_jk_y) - c2_nk_y) - c2_rk_y) + c2_uk_y) +
                  c2_xk_y) + c2_bl_y) + c2_gl_y) + c2_ll_y) + c2_ql_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 101);
  c2_d0 = c2_mpower(chartInstance, c2_q0_dot);
  c2_d1 = c2_mpower(chartInstance, c2_q1_dot);
  c2_d2 = c2_mpower(chartInstance, c2_q2_dot);
  c2_d3 = c2_mpower(chartInstance, c2_q3_dot);
  c2_quaternion_deltas[0] = c2_delta4;
  c2_quaternion_deltas[1] = c2_delta5;
  c2_quaternion_deltas[2] = c2_delta6;
  c2_quaternion_deltas[3] = c2_delta7;
  c2_quaternion_deltas[4] = ((-c2_d0 - c2_d1) - c2_d2) - c2_d3;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 103);
  c2_jl_b = c2_Jxx;
  c2_rl_y = 4.0 * c2_jl_b;
  c2_oh_a = c2_rl_y;
  c2_kl_b = c2_mpower(chartInstance, c2_q1);
  c2_sl_y = c2_oh_a * c2_kl_b;
  c2_ll_b = c2_Jyy;
  c2_tl_y = 4.0 * c2_ll_b;
  c2_ph_a = c2_tl_y;
  c2_ml_b = c2_mpower(chartInstance, c2_q2);
  c2_ul_y = c2_ph_a * c2_ml_b;
  c2_nl_b = c2_Jzz;
  c2_vl_y = 4.0 * c2_nl_b;
  c2_qh_a = c2_vl_y;
  c2_ol_b = c2_mpower(chartInstance, c2_q3);
  c2_wl_y = c2_qh_a * c2_ol_b;
  c2_pl_b = c2_Jzz;
  c2_xl_y = -4.0 * c2_pl_b;
  c2_rh_a = c2_xl_y;
  c2_ql_b = c2_q2;
  c2_yl_y = c2_rh_a * c2_ql_b;
  c2_sh_a = c2_yl_y;
  c2_rl_b = c2_q3;
  c2_am_y = c2_sh_a * c2_rl_b;
  c2_sl_b = c2_Jyy;
  c2_bm_y = 4.0 * c2_sl_b;
  c2_th_a = c2_bm_y;
  c2_tl_b = c2_q2;
  c2_cm_y = c2_th_a * c2_tl_b;
  c2_uh_a = c2_cm_y;
  c2_ul_b = c2_q3;
  c2_dm_y = c2_uh_a * c2_ul_b;
  c2_vl_b = c2_Jxx;
  c2_em_y = 4.0 * c2_vl_b;
  c2_vh_a = c2_em_y;
  c2_wl_b = c2_q0;
  c2_fm_y = c2_vh_a * c2_wl_b;
  c2_wh_a = c2_fm_y;
  c2_xl_b = c2_q1;
  c2_gm_y = c2_wh_a * c2_xl_b;
  c2_yl_b = c2_Jxx;
  c2_hm_y = -4.0 * c2_yl_b;
  c2_xh_a = c2_hm_y;
  c2_am_b = c2_q1;
  c2_im_y = c2_xh_a * c2_am_b;
  c2_yh_a = c2_im_y;
  c2_bm_b = c2_q3;
  c2_jm_y = c2_yh_a * c2_bm_b;
  c2_cm_b = c2_Jzz;
  c2_km_y = 4.0 * c2_cm_b;
  c2_ai_a = c2_km_y;
  c2_dm_b = c2_q1;
  c2_lm_y = c2_ai_a * c2_dm_b;
  c2_bi_a = c2_lm_y;
  c2_em_b = c2_q3;
  c2_mm_y = c2_bi_a * c2_em_b;
  c2_fm_b = c2_Jyy;
  c2_nm_y = 4.0 * c2_fm_b;
  c2_ci_a = c2_nm_y;
  c2_gm_b = c2_q0;
  c2_om_y = c2_ci_a * c2_gm_b;
  c2_di_a = c2_om_y;
  c2_hm_b = c2_q2;
  c2_pm_y = c2_di_a * c2_hm_b;
  c2_im_b = c2_Jyy;
  c2_qm_y = -4.0 * c2_im_b;
  c2_ei_a = c2_qm_y;
  c2_jm_b = c2_q1;
  c2_rm_y = c2_ei_a * c2_jm_b;
  c2_fi_a = c2_rm_y;
  c2_km_b = c2_q2;
  c2_sm_y = c2_fi_a * c2_km_b;
  c2_lm_b = c2_Jxx;
  c2_tm_y = 4.0 * c2_lm_b;
  c2_gi_a = c2_tm_y;
  c2_mm_b = c2_q1;
  c2_um_y = c2_gi_a * c2_mm_b;
  c2_hi_a = c2_um_y;
  c2_nm_b = c2_q2;
  c2_vm_y = c2_hi_a * c2_nm_b;
  c2_om_b = c2_Jzz;
  c2_wm_y = 4.0 * c2_om_b;
  c2_ii_a = c2_wm_y;
  c2_pm_b = c2_q0;
  c2_xm_y = c2_ii_a * c2_pm_b;
  c2_ji_a = c2_xm_y;
  c2_qm_b = c2_q3;
  c2_ym_y = c2_ji_a * c2_qm_b;
  c2_rm_b = c2_Jzz;
  c2_an_y = -4.0 * c2_rm_b;
  c2_ki_a = c2_an_y;
  c2_sm_b = c2_q2;
  c2_bn_y = c2_ki_a * c2_sm_b;
  c2_li_a = c2_bn_y;
  c2_tm_b = c2_q3;
  c2_cn_y = c2_li_a * c2_tm_b;
  c2_um_b = c2_Jyy;
  c2_dn_y = 4.0 * c2_um_b;
  c2_mi_a = c2_dn_y;
  c2_vm_b = c2_q2;
  c2_en_y = c2_mi_a * c2_vm_b;
  c2_ni_a = c2_en_y;
  c2_wm_b = c2_q3;
  c2_fn_y = c2_ni_a * c2_wm_b;
  c2_xm_b = c2_Jxx;
  c2_gn_y = 4.0 * c2_xm_b;
  c2_oi_a = c2_gn_y;
  c2_ym_b = c2_q0;
  c2_hn_y = c2_oi_a * c2_ym_b;
  c2_pi_a = c2_hn_y;
  c2_an_b = c2_q1;
  c2_in_y = c2_pi_a * c2_an_b;
  c2_bn_b = c2_Jxx;
  c2_jn_y = 4.0 * c2_bn_b;
  c2_qi_a = c2_jn_y;
  c2_cn_b = c2_mpower(chartInstance, c2_q0);
  c2_kn_y = c2_qi_a * c2_cn_b;
  c2_dn_b = c2_Jyy;
  c2_ln_y = 4.0 * c2_dn_b;
  c2_ri_a = c2_ln_y;
  c2_en_b = c2_mpower(chartInstance, c2_q3);
  c2_mn_y = c2_ri_a * c2_en_b;
  c2_fn_b = c2_Jzz;
  c2_nn_y = 4.0 * c2_fn_b;
  c2_si_a = c2_nn_y;
  c2_gn_b = c2_mpower(chartInstance, c2_q2);
  c2_on_y = c2_si_a * c2_gn_b;
  c2_hn_b = c2_Jyy;
  c2_pn_y = -4.0 * c2_hn_b;
  c2_ti_a = c2_pn_y;
  c2_in_b = c2_q0;
  c2_qn_y = c2_ti_a * c2_in_b;
  c2_ui_a = c2_qn_y;
  c2_jn_b = c2_q3;
  c2_rn_y = c2_ui_a * c2_jn_b;
  c2_kn_b = c2_Jxx;
  c2_sn_y = 4.0 * c2_kn_b;
  c2_vi_a = c2_sn_y;
  c2_ln_b = c2_q0;
  c2_tn_y = c2_vi_a * c2_ln_b;
  c2_wi_a = c2_tn_y;
  c2_mn_b = c2_q3;
  c2_un_y = c2_wi_a * c2_mn_b;
  c2_nn_b = c2_Jzz;
  c2_vn_y = 4.0 * c2_nn_b;
  c2_xi_a = c2_vn_y;
  c2_on_b = c2_q1;
  c2_wn_y = c2_xi_a * c2_on_b;
  c2_yi_a = c2_wn_y;
  c2_pn_b = c2_q2;
  c2_xn_y = c2_yi_a * c2_pn_b;
  c2_qn_b = c2_Jxx;
  c2_yn_y = -4.0 * c2_qn_b;
  c2_aj_a = c2_yn_y;
  c2_rn_b = c2_q0;
  c2_ao_y = c2_aj_a * c2_rn_b;
  c2_bj_a = c2_ao_y;
  c2_sn_b = c2_q2;
  c2_bo_y = c2_bj_a * c2_sn_b;
  c2_tn_b = c2_Jzz;
  c2_co_y = 4.0 * c2_tn_b;
  c2_cj_a = c2_co_y;
  c2_un_b = c2_q0;
  c2_do_y = c2_cj_a * c2_un_b;
  c2_dj_a = c2_do_y;
  c2_vn_b = c2_q2;
  c2_eo_y = c2_dj_a * c2_vn_b;
  c2_wn_b = c2_Jyy;
  c2_fo_y = 4.0 * c2_wn_b;
  c2_ej_a = c2_fo_y;
  c2_xn_b = c2_q1;
  c2_go_y = c2_ej_a * c2_xn_b;
  c2_fj_a = c2_go_y;
  c2_yn_b = c2_q3;
  c2_ho_y = c2_fj_a * c2_yn_b;
  c2_ao_b = c2_Jxx;
  c2_io_y = -4.0 * c2_ao_b;
  c2_gj_a = c2_io_y;
  c2_bo_b = c2_q1;
  c2_jo_y = c2_gj_a * c2_bo_b;
  c2_hj_a = c2_jo_y;
  c2_co_b = c2_q3;
  c2_ko_y = c2_hj_a * c2_co_b;
  c2_do_b = c2_Jzz;
  c2_lo_y = 4.0 * c2_do_b;
  c2_ij_a = c2_lo_y;
  c2_eo_b = c2_q1;
  c2_mo_y = c2_ij_a * c2_eo_b;
  c2_jj_a = c2_mo_y;
  c2_fo_b = c2_q3;
  c2_no_y = c2_jj_a * c2_fo_b;
  c2_go_b = c2_Jyy;
  c2_oo_y = 4.0 * c2_go_b;
  c2_kj_a = c2_oo_y;
  c2_ho_b = c2_q0;
  c2_po_y = c2_kj_a * c2_ho_b;
  c2_lj_a = c2_po_y;
  c2_io_b = c2_q2;
  c2_qo_y = c2_lj_a * c2_io_b;
  c2_jo_b = c2_Jyy;
  c2_ro_y = -4.0 * c2_jo_b;
  c2_mj_a = c2_ro_y;
  c2_ko_b = c2_q0;
  c2_so_y = c2_mj_a * c2_ko_b;
  c2_nj_a = c2_so_y;
  c2_lo_b = c2_q3;
  c2_to_y = c2_nj_a * c2_lo_b;
  c2_mo_b = c2_Jxx;
  c2_uo_y = 4.0 * c2_mo_b;
  c2_oj_a = c2_uo_y;
  c2_no_b = c2_q0;
  c2_vo_y = c2_oj_a * c2_no_b;
  c2_pj_a = c2_vo_y;
  c2_oo_b = c2_q3;
  c2_wo_y = c2_pj_a * c2_oo_b;
  c2_po_b = c2_Jzz;
  c2_xo_y = 4.0 * c2_po_b;
  c2_qj_a = c2_xo_y;
  c2_qo_b = c2_q1;
  c2_yo_y = c2_qj_a * c2_qo_b;
  c2_rj_a = c2_yo_y;
  c2_ro_b = c2_q2;
  c2_ap_y = c2_rj_a * c2_ro_b;
  c2_so_b = c2_Jxx;
  c2_bp_y = 4.0 * c2_so_b;
  c2_sj_a = c2_bp_y;
  c2_to_b = c2_mpower(chartInstance, c2_q3);
  c2_cp_y = c2_sj_a * c2_to_b;
  c2_uo_b = c2_Jyy;
  c2_dp_y = 4.0 * c2_uo_b;
  c2_tj_a = c2_dp_y;
  c2_vo_b = c2_mpower(chartInstance, c2_q0);
  c2_ep_y = c2_tj_a * c2_vo_b;
  c2_wo_b = c2_Jzz;
  c2_fp_y = 4.0 * c2_wo_b;
  c2_uj_a = c2_fp_y;
  c2_xo_b = c2_mpower(chartInstance, c2_q1);
  c2_gp_y = c2_uj_a * c2_xo_b;
  c2_yo_b = c2_Jzz;
  c2_hp_y = -4.0 * c2_yo_b;
  c2_vj_a = c2_hp_y;
  c2_ap_b = c2_q0;
  c2_ip_y = c2_vj_a * c2_ap_b;
  c2_wj_a = c2_ip_y;
  c2_bp_b = c2_q1;
  c2_jp_y = c2_wj_a * c2_bp_b;
  c2_cp_b = c2_Jyy;
  c2_kp_y = 4.0 * c2_cp_b;
  c2_xj_a = c2_kp_y;
  c2_dp_b = c2_q0;
  c2_lp_y = c2_xj_a * c2_dp_b;
  c2_yj_a = c2_lp_y;
  c2_ep_b = c2_q1;
  c2_mp_y = c2_yj_a * c2_ep_b;
  c2_fp_b = c2_Jxx;
  c2_np_y = 4.0 * c2_fp_b;
  c2_ak_a = c2_np_y;
  c2_gp_b = c2_q2;
  c2_op_y = c2_ak_a * c2_gp_b;
  c2_bk_a = c2_op_y;
  c2_hp_b = c2_q3;
  c2_pp_y = c2_bk_a * c2_hp_b;
  c2_ip_b = c2_Jyy;
  c2_qp_y = -4.0 * c2_ip_b;
  c2_ck_a = c2_qp_y;
  c2_jp_b = c2_q1;
  c2_rp_y = c2_ck_a * c2_jp_b;
  c2_dk_a = c2_rp_y;
  c2_kp_b = c2_q2;
  c2_sp_y = c2_dk_a * c2_kp_b;
  c2_lp_b = c2_Jxx;
  c2_tp_y = 4.0 * c2_lp_b;
  c2_ek_a = c2_tp_y;
  c2_mp_b = c2_q1;
  c2_up_y = c2_ek_a * c2_mp_b;
  c2_fk_a = c2_up_y;
  c2_np_b = c2_q2;
  c2_vp_y = c2_fk_a * c2_np_b;
  c2_op_b = c2_Jzz;
  c2_wp_y = 4.0 * c2_op_b;
  c2_gk_a = c2_wp_y;
  c2_pp_b = c2_q0;
  c2_xp_y = c2_gk_a * c2_pp_b;
  c2_hk_a = c2_xp_y;
  c2_qp_b = c2_q3;
  c2_yp_y = c2_hk_a * c2_qp_b;
  c2_rp_b = c2_Jxx;
  c2_aq_y = -4.0 * c2_rp_b;
  c2_ik_a = c2_aq_y;
  c2_sp_b = c2_q0;
  c2_bq_y = c2_ik_a * c2_sp_b;
  c2_jk_a = c2_bq_y;
  c2_tp_b = c2_q2;
  c2_cq_y = c2_jk_a * c2_tp_b;
  c2_up_b = c2_Jzz;
  c2_dq_y = 4.0 * c2_up_b;
  c2_kk_a = c2_dq_y;
  c2_vp_b = c2_q0;
  c2_eq_y = c2_kk_a * c2_vp_b;
  c2_lk_a = c2_eq_y;
  c2_wp_b = c2_q2;
  c2_fq_y = c2_lk_a * c2_wp_b;
  c2_xp_b = c2_Jyy;
  c2_gq_y = 4.0 * c2_xp_b;
  c2_mk_a = c2_gq_y;
  c2_yp_b = c2_q1;
  c2_hq_y = c2_mk_a * c2_yp_b;
  c2_nk_a = c2_hq_y;
  c2_aq_b = c2_q3;
  c2_iq_y = c2_nk_a * c2_aq_b;
  c2_bq_b = c2_Jzz;
  c2_jq_y = -4.0 * c2_bq_b;
  c2_ok_a = c2_jq_y;
  c2_cq_b = c2_q0;
  c2_kq_y = c2_ok_a * c2_cq_b;
  c2_pk_a = c2_kq_y;
  c2_dq_b = c2_q1;
  c2_lq_y = c2_pk_a * c2_dq_b;
  c2_eq_b = c2_Jyy;
  c2_mq_y = 4.0 * c2_eq_b;
  c2_qk_a = c2_mq_y;
  c2_fq_b = c2_q0;
  c2_nq_y = c2_qk_a * c2_fq_b;
  c2_rk_a = c2_nq_y;
  c2_gq_b = c2_q1;
  c2_oq_y = c2_rk_a * c2_gq_b;
  c2_hq_b = c2_Jxx;
  c2_pq_y = 4.0 * c2_hq_b;
  c2_sk_a = c2_pq_y;
  c2_iq_b = c2_q2;
  c2_qq_y = c2_sk_a * c2_iq_b;
  c2_tk_a = c2_qq_y;
  c2_jq_b = c2_q3;
  c2_rq_y = c2_tk_a * c2_jq_b;
  c2_kq_b = c2_Jxx;
  c2_sq_y = 4.0 * c2_kq_b;
  c2_uk_a = c2_sq_y;
  c2_lq_b = c2_mpower(chartInstance, c2_q2);
  c2_tq_y = c2_uk_a * c2_lq_b;
  c2_mq_b = c2_Jyy;
  c2_uq_y = 4.0 * c2_mq_b;
  c2_vk_a = c2_uq_y;
  c2_nq_b = c2_mpower(chartInstance, c2_q1);
  c2_vq_y = c2_vk_a * c2_nq_b;
  c2_oq_b = c2_Jzz;
  c2_wq_y = 4.0 * c2_oq_b;
  c2_wk_a = c2_wq_y;
  c2_pq_b = c2_mpower(chartInstance, c2_q0);
  c2_xq_y = c2_wk_a * c2_pq_b;
  c2_quaternion_multipliers[0] = (c2_sl_y + c2_ul_y) + c2_wl_y;
  c2_quaternion_multipliers[5] = (c2_am_y + c2_dm_y) - c2_gm_y;
  c2_quaternion_multipliers[10] = (c2_jm_y + c2_mm_y) - c2_pm_y;
  c2_quaternion_multipliers[15] = (c2_sm_y + c2_vm_y) - c2_ym_y;
  c2_quaternion_multipliers[1] = (c2_cn_y + c2_fn_y) - c2_in_y;
  c2_quaternion_multipliers[6] = (c2_kn_y + c2_mn_y) + c2_on_y;
  c2_quaternion_multipliers[11] = (c2_rn_y + c2_un_y) - c2_xn_y;
  c2_quaternion_multipliers[16] = (c2_bo_y + c2_eo_y) - c2_ho_y;
  c2_quaternion_multipliers[2] = (c2_ko_y + c2_no_y) - c2_qo_y;
  c2_quaternion_multipliers[7] = (c2_to_y + c2_wo_y) - c2_ap_y;
  c2_quaternion_multipliers[12] = (c2_cp_y + c2_ep_y) + c2_gp_y;
  c2_quaternion_multipliers[17] = (c2_jp_y + c2_mp_y) - c2_pp_y;
  c2_quaternion_multipliers[3] = (c2_sp_y + c2_vp_y) - c2_yp_y;
  c2_quaternion_multipliers[8] = (c2_cq_y + c2_fq_y) - c2_iq_y;
  c2_quaternion_multipliers[13] = (c2_lq_y + c2_oq_y) - c2_rq_y;
  c2_quaternion_multipliers[18] = (c2_tq_y + c2_vq_y) + c2_xq_y;
  c2_quaternion_multipliers[4] = c2_q0;
  c2_quaternion_multipliers[9] = c2_q1;
  c2_quaternion_multipliers[14] = c2_q2;
  c2_quaternion_multipliers[19] = c2_q3;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 110);
  for (c2_i58 = 0; c2_i58 < 20; c2_i58++) {
    c2_b_quaternion_multipliers[c2_i58] = c2_quaternion_multipliers[c2_i58];
  }

  for (c2_i59 = 0; c2_i59 < 5; c2_i59++) {
    c2_b_quaternion_deltas[c2_i59] = c2_quaternion_deltas[c2_i59];
  }

  c2_mldivide(chartInstance, c2_b_quaternion_multipliers, c2_b_quaternion_deltas,
              c2_dv2);
  for (c2_i60 = 0; c2_i60 < 4; c2_i60++) {
    c2_q_ddot[c2_i60] = c2_dv2[c2_i60];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 115);
  c2_i_A = c2_delta1;
  c2_B = c2_m;
  c2_r_x = c2_i_A;
  c2_yq_y = c2_B;
  c2_s_x = c2_r_x;
  c2_ar_y = c2_yq_y;
  c2_x_ddot = c2_s_x / c2_ar_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 116);
  c2_j_A = c2_delta2;
  c2_b_B = c2_m;
  c2_t_x = c2_j_A;
  c2_br_y = c2_b_B;
  c2_u_x = c2_t_x;
  c2_cr_y = c2_br_y;
  c2_y_ddot = c2_u_x / c2_cr_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 117);
  c2_k_A = c2_delta3;
  c2_c_B = c2_m;
  c2_v_x = c2_k_A;
  c2_dr_y = c2_c_B;
  c2_w_x = c2_v_x;
  c2_er_y = c2_dr_y;
  c2_fr_y = c2_w_x / c2_er_y;
  c2_z_ddot = c2_fr_y - c2_g;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 118);
  c2_q0_ddot = c2_q_ddot[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 119);
  c2_q1_ddot = c2_q_ddot[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 120);
  c2_q2_ddot = c2_q_ddot[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 121);
  c2_q3_ddot = c2_q_ddot[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -121);
  _SFD_SYMBOL_SCOPE_POP();
  *c2_b_x_ddot = c2_x_ddot;
  *c2_b_y_ddot = c2_y_ddot;
  *c2_b_z_ddot = c2_z_ddot;
  *c2_b_q0_ddot = c2_q0_ddot;
  *c2_b_q1_ddot = c2_q1_ddot;
  *c2_b_q2_ddot = c2_q2_ddot;
  *c2_b_q3_ddot = c2_q3_ddot;
  for (c2_i61 = 0; c2_i61 < 20; c2_i61++) {
    (*c2_c_quaternion_multipliers)[c2_i61] = c2_quaternion_multipliers[c2_i61];
  }

  for (c2_i62 = 0; c2_i62 < 5; c2_i62++) {
    (*c2_c_quaternion_deltas)[c2_i62] = c2_quaternion_deltas[c2_i62];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void registerMessagesc2_quad_control_sim_q
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i63;
  real_T c2_b_inData[5];
  int32_T c2_i64;
  real_T c2_u[5];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i63 = 0; c2_i63 < 5; c2_i63++) {
    c2_b_inData[c2_i63] = (*(real_T (*)[5])c2_inData)[c2_i63];
  }

  for (c2_i64 = 0; c2_i64 < 5; c2_i64++) {
    c2_u[c2_i64] = c2_b_inData[c2_i64];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 5), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_quaternion_deltas, const char_T
  *c2_identifier, real_T c2_y[5])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_quaternion_deltas),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_quaternion_deltas);
}

static void c2_b_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[5])
{
  real_T c2_dv3[5];
  int32_T c2_i65;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 1, 5);
  for (c2_i65 = 0; c2_i65 < 5; c2_i65++) {
    c2_y[c2_i65] = c2_dv3[c2_i65];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_quaternion_deltas;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[5];
  int32_T c2_i66;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_quaternion_deltas = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_quaternion_deltas),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_quaternion_deltas);
  for (c2_i66 = 0; c2_i66 < 5; c2_i66++) {
    (*(real_T (*)[5])c2_outData)[c2_i66] = c2_y[c2_i66];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i67;
  int32_T c2_i68;
  int32_T c2_i69;
  real_T c2_b_inData[20];
  int32_T c2_i70;
  int32_T c2_i71;
  int32_T c2_i72;
  real_T c2_u[20];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i67 = 0;
  for (c2_i68 = 0; c2_i68 < 4; c2_i68++) {
    for (c2_i69 = 0; c2_i69 < 5; c2_i69++) {
      c2_b_inData[c2_i69 + c2_i67] = (*(real_T (*)[20])c2_inData)[c2_i69 +
        c2_i67];
    }

    c2_i67 += 5;
  }

  c2_i70 = 0;
  for (c2_i71 = 0; c2_i71 < 4; c2_i71++) {
    for (c2_i72 = 0; c2_i72 < 5; c2_i72++) {
      c2_u[c2_i72 + c2_i70] = c2_b_inData[c2_i72 + c2_i70];
    }

    c2_i70 += 5;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 5, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_quaternion_multipliers, const char_T
  *c2_identifier, real_T c2_y[20])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_quaternion_multipliers),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_quaternion_multipliers);
}

static void c2_d_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[20])
{
  real_T c2_dv4[20];
  int32_T c2_i73;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv4, 1, 0, 0U, 1, 0U, 2, 5, 4);
  for (c2_i73 = 0; c2_i73 < 20; c2_i73++) {
    c2_y[c2_i73] = c2_dv4[c2_i73];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_quaternion_multipliers;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[20];
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_quaternion_multipliers = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_quaternion_multipliers),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_quaternion_multipliers);
  c2_i74 = 0;
  for (c2_i75 = 0; c2_i75 < 4; c2_i75++) {
    for (c2_i76 = 0; c2_i76 < 5; c2_i76++) {
      (*(real_T (*)[20])c2_outData)[c2_i76 + c2_i74] = c2_y[c2_i76 + c2_i74];
    }

    c2_i74 += 5;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_e_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_q3_ddot, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_q3_ddot), &c2_thisId);
  sf_mex_destroy(&c2_q3_ddot);
  return c2_y;
}

static real_T c2_f_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d4;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d4, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d4;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_q3_ddot;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_q3_ddot = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_q3_ddot), &c2_thisId);
  sf_mex_destroy(&c2_q3_ddot);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i77;
  real_T c2_b_inData[4];
  int32_T c2_i78;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i77 = 0; c2_i77 < 4; c2_i77++) {
    c2_b_inData[c2_i77] = (*(real_T (*)[4])c2_inData)[c2_i77];
  }

  for (c2_i78 = 0; c2_i78 < 4; c2_i78++) {
    c2_u[c2_i78] = c2_b_inData[c2_i78];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i79;
  real_T c2_b_inData[15];
  int32_T c2_i80;
  real_T c2_u[15];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i79 = 0; c2_i79 < 15; c2_i79++) {
    c2_b_inData[c2_i79] = (*(real_T (*)[15])c2_inData)[c2_i79];
  }

  for (c2_i80 = 0; c2_i80 < 15; c2_i80++) {
    c2_u[c2_i80] = c2_b_inData[c2_i80];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 15), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i81;
  real_T c2_b_inData[14];
  int32_T c2_i82;
  real_T c2_u[14];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i81 = 0; c2_i81 < 14; c2_i81++) {
    c2_b_inData[c2_i81] = (*(real_T (*)[14])c2_inData)[c2_i81];
  }

  for (c2_i82 = 0; c2_i82 < 14; c2_i82++) {
    c2_u[c2_i82] = c2_b_inData[c2_i82];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 14), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_g_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv5[4];
  int32_T c2_i83;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv5, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i83 = 0; c2_i83 < 4; c2_i83++) {
    c2_y[c2_i83] = c2_dv5[c2_i83];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_q_ddot;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i84;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_q_ddot = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_q_ddot), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_q_ddot);
  for (c2_i84 = 0; c2_i84 < 4; c2_i84++) {
    (*(real_T (*)[4])c2_outData)[c2_i84] = c2_y[c2_i84];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i85;
  real_T c2_b_inData[3];
  int32_T c2_i86;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i85 = 0; c2_i85 < 3; c2_i85++) {
    c2_b_inData[c2_i85] = (*(real_T (*)[3])c2_inData)[c2_i85];
  }

  for (c2_i86 = 0; c2_i86 < 3; c2_i86++) {
    c2_u[c2_i86] = c2_b_inData[c2_i86];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_h_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv6[3];
  int32_T c2_i87;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv6, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
    c2_y[c2_i87] = c2_dv6[c2_i87];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_dx2_dq3;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i88;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_dx2_dq3 = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_dx2_dq3), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_dx2_dq3);
  for (c2_i88 = 0; c2_i88 < 3; c2_i88++) {
    (*(real_T (*)[3])c2_outData)[c2_i88] = c2_y[c2_i88];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i89;
  int32_T c2_i90;
  int32_T c2_i91;
  real_T c2_b_inData[9];
  int32_T c2_i92;
  int32_T c2_i93;
  int32_T c2_i94;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i89 = 0;
  for (c2_i90 = 0; c2_i90 < 3; c2_i90++) {
    for (c2_i91 = 0; c2_i91 < 3; c2_i91++) {
      c2_b_inData[c2_i91 + c2_i89] = (*(real_T (*)[9])c2_inData)[c2_i91 + c2_i89];
    }

    c2_i89 += 3;
  }

  c2_i92 = 0;
  for (c2_i93 = 0; c2_i93 < 3; c2_i93++) {
    for (c2_i94 = 0; c2_i94 < 3; c2_i94++) {
      c2_u[c2_i94 + c2_i92] = c2_b_inData[c2_i94 + c2_i92];
    }

    c2_i92 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv7[9];
  int32_T c2_i95;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv7, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i95 = 0; c2_i95 < 9; c2_i95++) {
    c2_y[c2_i95] = c2_dv7[c2_i95];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_T_e2E;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i96;
  int32_T c2_i97;
  int32_T c2_i98;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_T_e2E = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_T_e2E), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_T_e2E);
  c2_i96 = 0;
  for (c2_i97 = 0; c2_i97 < 3; c2_i97++) {
    for (c2_i98 = 0; c2_i98 < 3; c2_i98++) {
      (*(real_T (*)[9])c2_outData)[c2_i98 + c2_i96] = c2_y[c2_i98 + c2_i96];
    }

    c2_i96 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_quad_control_sim_q_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo;
  c2_ResolvedFunctionInfo c2_info[209];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i99;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  c2_c_info_helper(c2_info);
  c2_d_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 209), FALSE);
  for (c2_i99 = 0; c2_i99 < 209; c2_i99++) {
    c2_r0 = &c2_info[c2_i99];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context", "nameCaptureInfo",
                    c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name", "nameCaptureInfo", c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved", "nameCaptureInfo",
                    c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c2_i99);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c2_i99);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[209])
{
  c2_info[0].context = "";
  c2_info[0].name = "mpower";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[0].fileTimeLo = 1286851242U;
  c2_info[0].fileTimeHi = 0U;
  c2_info[0].mFileTimeLo = 0U;
  c2_info[0].mFileTimeHi = 0U;
  c2_info[1].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[1].name = "power";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[1].fileTimeLo = 1348224330U;
  c2_info[1].fileTimeHi = 0U;
  c2_info[1].mFileTimeLo = 0U;
  c2_info[1].mFileTimeHi = 0U;
  c2_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[2].name = "eml_scalar_eg";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[2].fileTimeLo = 1286851196U;
  c2_info[2].fileTimeHi = 0U;
  c2_info[2].mFileTimeLo = 0U;
  c2_info[2].mFileTimeHi = 0U;
  c2_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[3].name = "eml_scalexp_alloc";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[3].fileTimeLo = 1352457260U;
  c2_info[3].fileTimeHi = 0U;
  c2_info[3].mFileTimeLo = 0U;
  c2_info[3].mFileTimeHi = 0U;
  c2_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower";
  c2_info[4].name = "floor";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[4].fileTimeLo = 1343862780U;
  c2_info[4].fileTimeHi = 0U;
  c2_info[4].mFileTimeLo = 0U;
  c2_info[4].mFileTimeHi = 0U;
  c2_info[5].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[5].name = "eml_scalar_floor";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[5].fileTimeLo = 1286851126U;
  c2_info[5].fileTimeHi = 0U;
  c2_info[5].mFileTimeLo = 0U;
  c2_info[5].mFileTimeHi = 0U;
  c2_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[6].name = "eml_scalar_eg";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[6].fileTimeLo = 1286851196U;
  c2_info[6].fileTimeHi = 0U;
  c2_info[6].mFileTimeLo = 0U;
  c2_info[6].mFileTimeHi = 0U;
  c2_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power";
  c2_info[7].name = "mtimes";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[7].fileTimeLo = 1289552092U;
  c2_info[7].fileTimeHi = 0U;
  c2_info[7].mFileTimeLo = 0U;
  c2_info[7].mFileTimeHi = 0U;
  c2_info[8].context = "";
  c2_info[8].name = "mtimes";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[8].fileTimeLo = 1289552092U;
  c2_info[8].fileTimeHi = 0U;
  c2_info[8].mFileTimeLo = 0U;
  c2_info[8].mFileTimeHi = 0U;
  c2_info[9].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[9].name = "eml_index_class";
  c2_info[9].dominantType = "";
  c2_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[9].fileTimeLo = 1323202978U;
  c2_info[9].fileTimeHi = 0U;
  c2_info[9].mFileTimeLo = 0U;
  c2_info[9].mFileTimeHi = 0U;
  c2_info[10].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[10].name = "eml_scalar_eg";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[10].fileTimeLo = 1286851196U;
  c2_info[10].fileTimeHi = 0U;
  c2_info[10].mFileTimeLo = 0U;
  c2_info[10].mFileTimeHi = 0U;
  c2_info[11].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[11].name = "eml_xgemm";
  c2_info[11].dominantType = "char";
  c2_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[11].fileTimeLo = 1299109172U;
  c2_info[11].fileTimeHi = 0U;
  c2_info[11].mFileTimeLo = 0U;
  c2_info[11].mFileTimeHi = 0U;
  c2_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c2_info[12].name = "eml_blas_inline";
  c2_info[12].dominantType = "";
  c2_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[12].fileTimeLo = 1299109168U;
  c2_info[12].fileTimeHi = 0U;
  c2_info[12].mFileTimeLo = 0U;
  c2_info[12].mFileTimeHi = 0U;
  c2_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c2_info[13].name = "mtimes";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[13].fileTimeLo = 1289552092U;
  c2_info[13].fileTimeHi = 0U;
  c2_info[13].mFileTimeLo = 0U;
  c2_info[13].mFileTimeHi = 0U;
  c2_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[14].name = "eml_index_class";
  c2_info[14].dominantType = "";
  c2_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[14].fileTimeLo = 1323202978U;
  c2_info[14].fileTimeHi = 0U;
  c2_info[14].mFileTimeLo = 0U;
  c2_info[14].mFileTimeHi = 0U;
  c2_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[15].name = "eml_scalar_eg";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[15].fileTimeLo = 1286851196U;
  c2_info[15].fileTimeHi = 0U;
  c2_info[15].mFileTimeLo = 0U;
  c2_info[15].mFileTimeHi = 0U;
  c2_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c2_info[16].name = "eml_refblas_xgemm";
  c2_info[16].dominantType = "char";
  c2_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c2_info[16].fileTimeLo = 1299109174U;
  c2_info[16].fileTimeHi = 0U;
  c2_info[16].mFileTimeLo = 0U;
  c2_info[16].mFileTimeHi = 0U;
  c2_info[17].context = "";
  c2_info[17].name = "sqrt";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[17].fileTimeLo = 1343862786U;
  c2_info[17].fileTimeHi = 0U;
  c2_info[17].mFileTimeLo = 0U;
  c2_info[17].mFileTimeHi = 0U;
  c2_info[18].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[18].name = "eml_error";
  c2_info[18].dominantType = "char";
  c2_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[18].fileTimeLo = 1343862758U;
  c2_info[18].fileTimeHi = 0U;
  c2_info[18].mFileTimeLo = 0U;
  c2_info[18].mFileTimeHi = 0U;
  c2_info[19].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[19].name = "eml_scalar_sqrt";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c2_info[19].fileTimeLo = 1286851138U;
  c2_info[19].fileTimeHi = 0U;
  c2_info[19].mFileTimeLo = 0U;
  c2_info[19].mFileTimeHi = 0U;
  c2_info[20].context = "";
  c2_info[20].name = "mrdivide";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[20].fileTimeLo = 1357983948U;
  c2_info[20].fileTimeHi = 0U;
  c2_info[20].mFileTimeLo = 1319762366U;
  c2_info[20].mFileTimeHi = 0U;
  c2_info[21].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c2_info[21].name = "rdivide";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[21].fileTimeLo = 1346542788U;
  c2_info[21].fileTimeHi = 0U;
  c2_info[21].mFileTimeLo = 0U;
  c2_info[21].mFileTimeHi = 0U;
  c2_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[22].name = "eml_scalexp_compatible";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[22].fileTimeLo = 1286851196U;
  c2_info[22].fileTimeHi = 0U;
  c2_info[22].mFileTimeLo = 0U;
  c2_info[22].mFileTimeHi = 0U;
  c2_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[23].name = "eml_div";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[23].fileTimeLo = 1313380210U;
  c2_info[23].fileTimeHi = 0U;
  c2_info[23].mFileTimeLo = 0U;
  c2_info[23].mFileTimeHi = 0U;
  c2_info[24].context = "";
  c2_info[24].name = "dot";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/dot.m";
  c2_info[24].fileTimeLo = 1313380216U;
  c2_info[24].fileTimeHi = 0U;
  c2_info[24].mFileTimeLo = 0U;
  c2_info[24].mFileTimeHi = 0U;
  c2_info[25].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/dot.m";
  c2_info[25].name = "isequal";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[25].fileTimeLo = 1286851158U;
  c2_info[25].fileTimeHi = 0U;
  c2_info[25].mFileTimeLo = 0U;
  c2_info[25].mFileTimeHi = 0U;
  c2_info[26].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[26].name = "eml_isequal_core";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c2_info[26].fileTimeLo = 1286851186U;
  c2_info[26].fileTimeHi = 0U;
  c2_info[26].mFileTimeLo = 0U;
  c2_info[26].mFileTimeHi = 0U;
  c2_info[27].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m!isequal_scalar";
  c2_info[27].name = "isnan";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[27].fileTimeLo = 1286851160U;
  c2_info[27].fileTimeHi = 0U;
  c2_info[27].mFileTimeLo = 0U;
  c2_info[27].mFileTimeHi = 0U;
  c2_info[28].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/dot.m";
  c2_info[28].name = "eml_index_class";
  c2_info[28].dominantType = "";
  c2_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[28].fileTimeLo = 1323202978U;
  c2_info[28].fileTimeHi = 0U;
  c2_info[28].mFileTimeLo = 0U;
  c2_info[28].mFileTimeHi = 0U;
  c2_info[29].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/dot.m";
  c2_info[29].name = "eml_scalar_eg";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[29].fileTimeLo = 1286851196U;
  c2_info[29].fileTimeHi = 0U;
  c2_info[29].mFileTimeLo = 0U;
  c2_info[29].mFileTimeHi = 0U;
  c2_info[30].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/specfun/dot.m!vdot";
  c2_info[30].name = "eml_xdotc";
  c2_info[30].dominantType = "double";
  c2_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[30].fileTimeLo = 1299109172U;
  c2_info[30].fileTimeHi = 0U;
  c2_info[30].mFileTimeLo = 0U;
  c2_info[30].mFileTimeHi = 0U;
  c2_info[31].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[31].name = "eml_blas_inline";
  c2_info[31].dominantType = "";
  c2_info[31].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[31].fileTimeLo = 1299109168U;
  c2_info[31].fileTimeHi = 0U;
  c2_info[31].mFileTimeLo = 0U;
  c2_info[31].mFileTimeHi = 0U;
  c2_info[32].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c2_info[32].name = "eml_xdot";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[32].fileTimeLo = 1299109172U;
  c2_info[32].fileTimeHi = 0U;
  c2_info[32].mFileTimeLo = 0U;
  c2_info[32].mFileTimeHi = 0U;
  c2_info[33].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c2_info[33].name = "eml_blas_inline";
  c2_info[33].dominantType = "";
  c2_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[33].fileTimeLo = 1299109168U;
  c2_info[33].fileTimeHi = 0U;
  c2_info[33].mFileTimeLo = 0U;
  c2_info[33].mFileTimeHi = 0U;
  c2_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c2_info[34].name = "length";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[34].fileTimeLo = 1303178606U;
  c2_info[34].fileTimeHi = 0U;
  c2_info[34].mFileTimeLo = 0U;
  c2_info[34].mFileTimeHi = 0U;
  c2_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[35].name = "eml_index_class";
  c2_info[35].dominantType = "";
  c2_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[35].fileTimeLo = 1323202978U;
  c2_info[35].fileTimeHi = 0U;
  c2_info[35].mFileTimeLo = 0U;
  c2_info[35].mFileTimeHi = 0U;
  c2_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c2_info[36].name = "eml_refblas_xdot";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[36].fileTimeLo = 1299109172U;
  c2_info[36].fileTimeHi = 0U;
  c2_info[36].mFileTimeLo = 0U;
  c2_info[36].mFileTimeHi = 0U;
  c2_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c2_info[37].name = "eml_refblas_xdotx";
  c2_info[37].dominantType = "char";
  c2_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[37].fileTimeLo = 1299109174U;
  c2_info[37].fileTimeHi = 0U;
  c2_info[37].mFileTimeLo = 0U;
  c2_info[37].mFileTimeHi = 0U;
  c2_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[38].name = "eml_scalar_eg";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[38].fileTimeLo = 1286851196U;
  c2_info[38].fileTimeHi = 0U;
  c2_info[38].mFileTimeLo = 0U;
  c2_info[38].mFileTimeHi = 0U;
  c2_info[39].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[39].name = "eml_index_class";
  c2_info[39].dominantType = "";
  c2_info[39].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[39].fileTimeLo = 1323202978U;
  c2_info[39].fileTimeHi = 0U;
  c2_info[39].mFileTimeLo = 0U;
  c2_info[39].mFileTimeHi = 0U;
  c2_info[40].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[40].name = "eml_int_forloop_overflow_check";
  c2_info[40].dominantType = "";
  c2_info[40].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[40].fileTimeLo = 1346542740U;
  c2_info[40].fileTimeHi = 0U;
  c2_info[40].mFileTimeLo = 0U;
  c2_info[40].mFileTimeHi = 0U;
  c2_info[41].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c2_info[41].name = "intmax";
  c2_info[41].dominantType = "char";
  c2_info[41].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[41].fileTimeLo = 1311287716U;
  c2_info[41].fileTimeHi = 0U;
  c2_info[41].mFileTimeLo = 0U;
  c2_info[41].mFileTimeHi = 0U;
  c2_info[42].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[42].name = "eml_index_minus";
  c2_info[42].dominantType = "double";
  c2_info[42].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[42].fileTimeLo = 1286851178U;
  c2_info[42].fileTimeHi = 0U;
  c2_info[42].mFileTimeLo = 0U;
  c2_info[42].mFileTimeHi = 0U;
  c2_info[43].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[43].name = "eml_index_class";
  c2_info[43].dominantType = "";
  c2_info[43].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[43].fileTimeLo = 1323202978U;
  c2_info[43].fileTimeHi = 0U;
  c2_info[43].mFileTimeLo = 0U;
  c2_info[43].mFileTimeHi = 0U;
  c2_info[44].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[44].name = "eml_index_times";
  c2_info[44].dominantType = "coder.internal.indexInt";
  c2_info[44].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[44].fileTimeLo = 1286851180U;
  c2_info[44].fileTimeHi = 0U;
  c2_info[44].mFileTimeLo = 0U;
  c2_info[44].mFileTimeHi = 0U;
  c2_info[45].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[45].name = "eml_index_class";
  c2_info[45].dominantType = "";
  c2_info[45].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[45].fileTimeLo = 1323202978U;
  c2_info[45].fileTimeHi = 0U;
  c2_info[45].mFileTimeLo = 0U;
  c2_info[45].mFileTimeHi = 0U;
  c2_info[46].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c2_info[46].name = "eml_index_plus";
  c2_info[46].dominantType = "coder.internal.indexInt";
  c2_info[46].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[46].fileTimeLo = 1286851178U;
  c2_info[46].fileTimeHi = 0U;
  c2_info[46].mFileTimeLo = 0U;
  c2_info[46].mFileTimeHi = 0U;
  c2_info[47].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[47].name = "eml_index_class";
  c2_info[47].dominantType = "";
  c2_info[47].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[47].fileTimeLo = 1323202978U;
  c2_info[47].fileTimeHi = 0U;
  c2_info[47].mFileTimeLo = 0U;
  c2_info[47].mFileTimeHi = 0U;
  c2_info[48].context = "";
  c2_info[48].name = "mldivide";
  c2_info[48].dominantType = "double";
  c2_info[48].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[48].fileTimeLo = 1357983948U;
  c2_info[48].fileTimeHi = 0U;
  c2_info[48].mFileTimeLo = 1319762366U;
  c2_info[48].mFileTimeHi = 0U;
  c2_info[49].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mldivide.p";
  c2_info[49].name = "eml_qrsolve";
  c2_info[49].dominantType = "double";
  c2_info[49].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[49].fileTimeLo = 1307683644U;
  c2_info[49].fileTimeHi = 0U;
  c2_info[49].mFileTimeLo = 0U;
  c2_info[49].mFileTimeHi = 0U;
  c2_info[50].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[50].name = "min";
  c2_info[50].dominantType = "double";
  c2_info[50].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[50].fileTimeLo = 1311287718U;
  c2_info[50].fileTimeHi = 0U;
  c2_info[50].mFileTimeLo = 0U;
  c2_info[50].mFileTimeHi = 0U;
  c2_info[51].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[51].name = "eml_min_or_max";
  c2_info[51].dominantType = "char";
  c2_info[51].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[51].fileTimeLo = 1334103890U;
  c2_info[51].fileTimeHi = 0U;
  c2_info[51].mFileTimeLo = 0U;
  c2_info[51].mFileTimeHi = 0U;
  c2_info[52].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[52].name = "eml_scalar_eg";
  c2_info[52].dominantType = "double";
  c2_info[52].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[52].fileTimeLo = 1286851196U;
  c2_info[52].fileTimeHi = 0U;
  c2_info[52].mFileTimeLo = 0U;
  c2_info[52].mFileTimeHi = 0U;
  c2_info[53].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[53].name = "eml_scalexp_alloc";
  c2_info[53].dominantType = "double";
  c2_info[53].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[53].fileTimeLo = 1352457260U;
  c2_info[53].fileTimeHi = 0U;
  c2_info[53].mFileTimeLo = 0U;
  c2_info[53].mFileTimeHi = 0U;
  c2_info[54].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[54].name = "eml_index_class";
  c2_info[54].dominantType = "";
  c2_info[54].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[54].fileTimeLo = 1323202978U;
  c2_info[54].fileTimeHi = 0U;
  c2_info[54].mFileTimeLo = 0U;
  c2_info[54].mFileTimeHi = 0U;
  c2_info[55].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[55].name = "eml_scalar_eg";
  c2_info[55].dominantType = "double";
  c2_info[55].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[55].fileTimeLo = 1286851196U;
  c2_info[55].fileTimeHi = 0U;
  c2_info[55].mFileTimeLo = 0U;
  c2_info[55].mFileTimeHi = 0U;
  c2_info[56].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[56].name = "eml_xgeqp3";
  c2_info[56].dominantType = "double";
  c2_info[56].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgeqp3.m";
  c2_info[56].fileTimeLo = 1286851204U;
  c2_info[56].fileTimeHi = 0U;
  c2_info[56].mFileTimeLo = 0U;
  c2_info[56].mFileTimeHi = 0U;
  c2_info[57].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgeqp3.m";
  c2_info[57].name = "eml_lapack_xgeqp3";
  c2_info[57].dominantType = "double";
  c2_info[57].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgeqp3.m";
  c2_info[57].fileTimeLo = 1286851208U;
  c2_info[57].fileTimeHi = 0U;
  c2_info[57].mFileTimeLo = 0U;
  c2_info[57].mFileTimeHi = 0U;
  c2_info[58].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgeqp3.m";
  c2_info[58].name = "eml_matlab_zgeqp3";
  c2_info[58].dominantType = "double";
  c2_info[58].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[58].fileTimeLo = 1290031766U;
  c2_info[58].fileTimeHi = 0U;
  c2_info[58].mFileTimeLo = 0U;
  c2_info[58].mFileTimeHi = 0U;
  c2_info[59].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[59].name = "eml_index_class";
  c2_info[59].dominantType = "";
  c2_info[59].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[59].fileTimeLo = 1323202978U;
  c2_info[59].fileTimeHi = 0U;
  c2_info[59].mFileTimeLo = 0U;
  c2_info[59].mFileTimeHi = 0U;
  c2_info[60].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[60].name = "min";
  c2_info[60].dominantType = "coder.internal.indexInt";
  c2_info[60].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[60].fileTimeLo = 1311287718U;
  c2_info[60].fileTimeHi = 0U;
  c2_info[60].mFileTimeLo = 0U;
  c2_info[60].mFileTimeHi = 0U;
  c2_info[61].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[61].name = "eml_scalar_eg";
  c2_info[61].dominantType = "coder.internal.indexInt";
  c2_info[61].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[61].fileTimeLo = 1286851196U;
  c2_info[61].fileTimeHi = 0U;
  c2_info[61].mFileTimeLo = 0U;
  c2_info[61].mFileTimeHi = 0U;
  c2_info[62].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c2_info[62].name = "eml_scalexp_alloc";
  c2_info[62].dominantType = "coder.internal.indexInt";
  c2_info[62].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[62].fileTimeLo = 1352457260U;
  c2_info[62].fileTimeHi = 0U;
  c2_info[62].mFileTimeLo = 0U;
  c2_info[62].mFileTimeHi = 0U;
  c2_info[63].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c2_info[63].name = "eml_scalar_eg";
  c2_info[63].dominantType = "coder.internal.indexInt";
  c2_info[63].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[63].fileTimeLo = 1286851196U;
  c2_info[63].fileTimeHi = 0U;
  c2_info[63].mFileTimeLo = 0U;
  c2_info[63].mFileTimeHi = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[209])
{
  c2_info[64].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[64].name = "eml_scalar_eg";
  c2_info[64].dominantType = "double";
  c2_info[64].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[64].fileTimeLo = 1286851196U;
  c2_info[64].fileTimeHi = 0U;
  c2_info[64].mFileTimeLo = 0U;
  c2_info[64].mFileTimeHi = 0U;
  c2_info[65].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[65].name = "colon";
  c2_info[65].dominantType = "coder.internal.indexInt";
  c2_info[65].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[65].fileTimeLo = 1348224328U;
  c2_info[65].fileTimeHi = 0U;
  c2_info[65].mFileTimeLo = 0U;
  c2_info[65].mFileTimeHi = 0U;
  c2_info[66].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[66].name = "colon";
  c2_info[66].dominantType = "double";
  c2_info[66].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[66].fileTimeLo = 1348224328U;
  c2_info[66].fileTimeHi = 0U;
  c2_info[66].mFileTimeLo = 0U;
  c2_info[66].mFileTimeHi = 0U;
  c2_info[67].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[67].name = "floor";
  c2_info[67].dominantType = "double";
  c2_info[67].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[67].fileTimeLo = 1343862780U;
  c2_info[67].fileTimeHi = 0U;
  c2_info[67].mFileTimeLo = 0U;
  c2_info[67].mFileTimeHi = 0U;
  c2_info[68].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[68].name = "intmin";
  c2_info[68].dominantType = "char";
  c2_info[68].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[68].fileTimeLo = 1311287718U;
  c2_info[68].fileTimeHi = 0U;
  c2_info[68].mFileTimeLo = 0U;
  c2_info[68].mFileTimeHi = 0U;
  c2_info[69].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c2_info[69].name = "intmax";
  c2_info[69].dominantType = "char";
  c2_info[69].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[69].fileTimeLo = 1311287716U;
  c2_info[69].fileTimeHi = 0U;
  c2_info[69].mFileTimeLo = 0U;
  c2_info[69].mFileTimeHi = 0U;
  c2_info[70].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[70].name = "intmin";
  c2_info[70].dominantType = "char";
  c2_info[70].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[70].fileTimeLo = 1311287718U;
  c2_info[70].fileTimeHi = 0U;
  c2_info[70].mFileTimeLo = 0U;
  c2_info[70].mFileTimeHi = 0U;
  c2_info[71].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[71].name = "intmax";
  c2_info[71].dominantType = "char";
  c2_info[71].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[71].fileTimeLo = 1311287716U;
  c2_info[71].fileTimeHi = 0U;
  c2_info[71].mFileTimeLo = 0U;
  c2_info[71].mFileTimeHi = 0U;
  c2_info[72].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c2_info[72].name = "eml_isa_uint";
  c2_info[72].dominantType = "coder.internal.indexInt";
  c2_info[72].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[72].fileTimeLo = 1286851184U;
  c2_info[72].fileTimeHi = 0U;
  c2_info[72].mFileTimeLo = 0U;
  c2_info[72].mFileTimeHi = 0U;
  c2_info[73].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[73].name = "eml_unsigned_class";
  c2_info[73].dominantType = "char";
  c2_info[73].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[73].fileTimeLo = 1323202980U;
  c2_info[73].fileTimeHi = 0U;
  c2_info[73].mFileTimeLo = 0U;
  c2_info[73].mFileTimeHi = 0U;
  c2_info[74].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c2_info[74].name = "eml_index_class";
  c2_info[74].dominantType = "";
  c2_info[74].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[74].fileTimeLo = 1323202978U;
  c2_info[74].fileTimeHi = 0U;
  c2_info[74].mFileTimeLo = 0U;
  c2_info[74].mFileTimeHi = 0U;
  c2_info[75].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[75].name = "eml_index_class";
  c2_info[75].dominantType = "";
  c2_info[75].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[75].fileTimeLo = 1323202978U;
  c2_info[75].fileTimeHi = 0U;
  c2_info[75].mFileTimeLo = 0U;
  c2_info[75].mFileTimeHi = 0U;
  c2_info[76].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[76].name = "intmax";
  c2_info[76].dominantType = "char";
  c2_info[76].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[76].fileTimeLo = 1311287716U;
  c2_info[76].fileTimeHi = 0U;
  c2_info[76].mFileTimeLo = 0U;
  c2_info[76].mFileTimeHi = 0U;
  c2_info[77].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[77].name = "eml_isa_uint";
  c2_info[77].dominantType = "coder.internal.indexInt";
  c2_info[77].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[77].fileTimeLo = 1286851184U;
  c2_info[77].fileTimeHi = 0U;
  c2_info[77].mFileTimeLo = 0U;
  c2_info[77].mFileTimeHi = 0U;
  c2_info[78].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c2_info[78].name = "eml_index_plus";
  c2_info[78].dominantType = "double";
  c2_info[78].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[78].fileTimeLo = 1286851178U;
  c2_info[78].fileTimeHi = 0U;
  c2_info[78].mFileTimeLo = 0U;
  c2_info[78].mFileTimeHi = 0U;
  c2_info[79].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c2_info[79].name = "eml_int_forloop_overflow_check";
  c2_info[79].dominantType = "";
  c2_info[79].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[79].fileTimeLo = 1346542740U;
  c2_info[79].fileTimeHi = 0U;
  c2_info[79].mFileTimeLo = 0U;
  c2_info[79].mFileTimeHi = 0U;
  c2_info[80].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[80].name = "eps";
  c2_info[80].dominantType = "char";
  c2_info[80].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[80].fileTimeLo = 1326760396U;
  c2_info[80].fileTimeHi = 0U;
  c2_info[80].mFileTimeLo = 0U;
  c2_info[80].mFileTimeHi = 0U;
  c2_info[81].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[81].name = "eml_is_float_class";
  c2_info[81].dominantType = "char";
  c2_info[81].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[81].fileTimeLo = 1286851182U;
  c2_info[81].fileTimeHi = 0U;
  c2_info[81].mFileTimeLo = 0U;
  c2_info[81].mFileTimeHi = 0U;
  c2_info[82].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[82].name = "eml_eps";
  c2_info[82].dominantType = "char";
  c2_info[82].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[82].fileTimeLo = 1326760396U;
  c2_info[82].fileTimeHi = 0U;
  c2_info[82].mFileTimeLo = 0U;
  c2_info[82].mFileTimeHi = 0U;
  c2_info[83].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c2_info[83].name = "eml_float_model";
  c2_info[83].dominantType = "char";
  c2_info[83].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[83].fileTimeLo = 1326760396U;
  c2_info[83].fileTimeHi = 0U;
  c2_info[83].mFileTimeLo = 0U;
  c2_info[83].mFileTimeHi = 0U;
  c2_info[84].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[84].name = "sqrt";
  c2_info[84].dominantType = "double";
  c2_info[84].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c2_info[84].fileTimeLo = 1343862786U;
  c2_info[84].fileTimeHi = 0U;
  c2_info[84].mFileTimeLo = 0U;
  c2_info[84].mFileTimeHi = 0U;
  c2_info[85].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[85].name = "eml_int_forloop_overflow_check";
  c2_info[85].dominantType = "";
  c2_info[85].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[85].fileTimeLo = 1346542740U;
  c2_info[85].fileTimeHi = 0U;
  c2_info[85].mFileTimeLo = 0U;
  c2_info[85].mFileTimeHi = 0U;
  c2_info[86].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[86].name = "eml_xnrm2";
  c2_info[86].dominantType = "double";
  c2_info[86].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[86].fileTimeLo = 1299109176U;
  c2_info[86].fileTimeHi = 0U;
  c2_info[86].mFileTimeLo = 0U;
  c2_info[86].mFileTimeHi = 0U;
  c2_info[87].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[87].name = "eml_blas_inline";
  c2_info[87].dominantType = "";
  c2_info[87].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[87].fileTimeLo = 1299109168U;
  c2_info[87].fileTimeHi = 0U;
  c2_info[87].mFileTimeLo = 0U;
  c2_info[87].mFileTimeHi = 0U;
  c2_info[88].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[88].name = "eml_index_class";
  c2_info[88].dominantType = "";
  c2_info[88].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[88].fileTimeLo = 1323202978U;
  c2_info[88].fileTimeHi = 0U;
  c2_info[88].mFileTimeLo = 0U;
  c2_info[88].mFileTimeHi = 0U;
  c2_info[89].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c2_info[89].name = "eml_refblas_xnrm2";
  c2_info[89].dominantType = "double";
  c2_info[89].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[89].fileTimeLo = 1299109184U;
  c2_info[89].fileTimeHi = 0U;
  c2_info[89].mFileTimeLo = 0U;
  c2_info[89].mFileTimeHi = 0U;
  c2_info[90].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[90].name = "realmin";
  c2_info[90].dominantType = "char";
  c2_info[90].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[90].fileTimeLo = 1307683642U;
  c2_info[90].fileTimeHi = 0U;
  c2_info[90].mFileTimeLo = 0U;
  c2_info[90].mFileTimeHi = 0U;
  c2_info[91].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[91].name = "eml_realmin";
  c2_info[91].dominantType = "char";
  c2_info[91].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[91].fileTimeLo = 1307683644U;
  c2_info[91].fileTimeHi = 0U;
  c2_info[91].mFileTimeLo = 0U;
  c2_info[91].mFileTimeHi = 0U;
  c2_info[92].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c2_info[92].name = "eml_float_model";
  c2_info[92].dominantType = "char";
  c2_info[92].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[92].fileTimeLo = 1326760396U;
  c2_info[92].fileTimeHi = 0U;
  c2_info[92].mFileTimeLo = 0U;
  c2_info[92].mFileTimeHi = 0U;
  c2_info[93].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[93].name = "eml_index_class";
  c2_info[93].dominantType = "";
  c2_info[93].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[93].fileTimeLo = 1323202978U;
  c2_info[93].fileTimeHi = 0U;
  c2_info[93].mFileTimeLo = 0U;
  c2_info[93].mFileTimeHi = 0U;
  c2_info[94].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[94].name = "eml_index_minus";
  c2_info[94].dominantType = "double";
  c2_info[94].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[94].fileTimeLo = 1286851178U;
  c2_info[94].fileTimeHi = 0U;
  c2_info[94].mFileTimeLo = 0U;
  c2_info[94].mFileTimeHi = 0U;
  c2_info[95].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[95].name = "eml_index_times";
  c2_info[95].dominantType = "coder.internal.indexInt";
  c2_info[95].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[95].fileTimeLo = 1286851180U;
  c2_info[95].fileTimeHi = 0U;
  c2_info[95].mFileTimeLo = 0U;
  c2_info[95].mFileTimeHi = 0U;
  c2_info[96].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[96].name = "eml_index_plus";
  c2_info[96].dominantType = "coder.internal.indexInt";
  c2_info[96].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[96].fileTimeLo = 1286851178U;
  c2_info[96].fileTimeHi = 0U;
  c2_info[96].mFileTimeLo = 0U;
  c2_info[96].mFileTimeHi = 0U;
  c2_info[97].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[97].name = "eml_int_forloop_overflow_check";
  c2_info[97].dominantType = "";
  c2_info[97].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[97].fileTimeLo = 1346542740U;
  c2_info[97].fileTimeHi = 0U;
  c2_info[97].mFileTimeLo = 0U;
  c2_info[97].mFileTimeHi = 0U;
  c2_info[98].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c2_info[98].name = "abs";
  c2_info[98].dominantType = "double";
  c2_info[98].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[98].fileTimeLo = 1343862766U;
  c2_info[98].fileTimeHi = 0U;
  c2_info[98].mFileTimeLo = 0U;
  c2_info[98].mFileTimeHi = 0U;
  c2_info[99].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[99].name = "eml_scalar_abs";
  c2_info[99].dominantType = "double";
  c2_info[99].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[99].fileTimeLo = 1286851112U;
  c2_info[99].fileTimeHi = 0U;
  c2_info[99].mFileTimeLo = 0U;
  c2_info[99].mFileTimeHi = 0U;
  c2_info[100].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[100].name = "eml_index_plus";
  c2_info[100].dominantType = "coder.internal.indexInt";
  c2_info[100].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[100].fileTimeLo = 1286851178U;
  c2_info[100].fileTimeHi = 0U;
  c2_info[100].mFileTimeLo = 0U;
  c2_info[100].mFileTimeHi = 0U;
  c2_info[101].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[101].name = "eml_index_minus";
  c2_info[101].dominantType = "double";
  c2_info[101].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[101].fileTimeLo = 1286851178U;
  c2_info[101].fileTimeHi = 0U;
  c2_info[101].mFileTimeLo = 0U;
  c2_info[101].mFileTimeHi = 0U;
  c2_info[102].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[102].name = "eml_index_plus";
  c2_info[102].dominantType = "double";
  c2_info[102].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[102].fileTimeLo = 1286851178U;
  c2_info[102].fileTimeHi = 0U;
  c2_info[102].mFileTimeLo = 0U;
  c2_info[102].mFileTimeHi = 0U;
  c2_info[103].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[103].name = "eml_index_times";
  c2_info[103].dominantType = "coder.internal.indexInt";
  c2_info[103].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[103].fileTimeLo = 1286851180U;
  c2_info[103].fileTimeHi = 0U;
  c2_info[103].mFileTimeLo = 0U;
  c2_info[103].mFileTimeHi = 0U;
  c2_info[104].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[104].name = "eml_index_minus";
  c2_info[104].dominantType = "coder.internal.indexInt";
  c2_info[104].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[104].fileTimeLo = 1286851178U;
  c2_info[104].fileTimeHi = 0U;
  c2_info[104].mFileTimeLo = 0U;
  c2_info[104].mFileTimeHi = 0U;
  c2_info[105].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[105].name = "eml_ixamax";
  c2_info[105].dominantType = "double";
  c2_info[105].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[105].fileTimeLo = 1299109170U;
  c2_info[105].fileTimeHi = 0U;
  c2_info[105].mFileTimeLo = 0U;
  c2_info[105].mFileTimeHi = 0U;
  c2_info[106].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c2_info[106].name = "eml_blas_inline";
  c2_info[106].dominantType = "";
  c2_info[106].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[106].fileTimeLo = 1299109168U;
  c2_info[106].fileTimeHi = 0U;
  c2_info[106].mFileTimeLo = 0U;
  c2_info[106].mFileTimeHi = 0U;
  c2_info[107].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c2_info[107].name = "length";
  c2_info[107].dominantType = "double";
  c2_info[107].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[107].fileTimeLo = 1303178606U;
  c2_info[107].fileTimeHi = 0U;
  c2_info[107].mFileTimeLo = 0U;
  c2_info[107].mFileTimeHi = 0U;
  c2_info[108].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[108].name = "eml_index_class";
  c2_info[108].dominantType = "";
  c2_info[108].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[108].fileTimeLo = 1323202978U;
  c2_info[108].fileTimeHi = 0U;
  c2_info[108].mFileTimeLo = 0U;
  c2_info[108].mFileTimeHi = 0U;
  c2_info[109].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c2_info[109].name = "eml_refblas_ixamax";
  c2_info[109].dominantType = "double";
  c2_info[109].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[109].fileTimeLo = 1299109170U;
  c2_info[109].fileTimeHi = 0U;
  c2_info[109].mFileTimeLo = 0U;
  c2_info[109].mFileTimeHi = 0U;
  c2_info[110].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[110].name = "eml_index_class";
  c2_info[110].dominantType = "";
  c2_info[110].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[110].fileTimeLo = 1323202978U;
  c2_info[110].fileTimeHi = 0U;
  c2_info[110].mFileTimeLo = 0U;
  c2_info[110].mFileTimeHi = 0U;
  c2_info[111].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[111].name = "eml_xcabs1";
  c2_info[111].dominantType = "double";
  c2_info[111].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[111].fileTimeLo = 1286851106U;
  c2_info[111].fileTimeHi = 0U;
  c2_info[111].mFileTimeLo = 0U;
  c2_info[111].mFileTimeHi = 0U;
  c2_info[112].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[112].name = "abs";
  c2_info[112].dominantType = "double";
  c2_info[112].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[112].fileTimeLo = 1343862766U;
  c2_info[112].fileTimeHi = 0U;
  c2_info[112].mFileTimeLo = 0U;
  c2_info[112].mFileTimeHi = 0U;
  c2_info[113].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[113].name = "eml_int_forloop_overflow_check";
  c2_info[113].dominantType = "";
  c2_info[113].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[113].fileTimeLo = 1346542740U;
  c2_info[113].fileTimeHi = 0U;
  c2_info[113].mFileTimeLo = 0U;
  c2_info[113].mFileTimeHi = 0U;
  c2_info[114].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c2_info[114].name = "eml_index_plus";
  c2_info[114].dominantType = "coder.internal.indexInt";
  c2_info[114].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[114].fileTimeLo = 1286851178U;
  c2_info[114].fileTimeHi = 0U;
  c2_info[114].mFileTimeLo = 0U;
  c2_info[114].mFileTimeHi = 0U;
  c2_info[115].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[115].name = "eml_xswap";
  c2_info[115].dominantType = "double";
  c2_info[115].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[115].fileTimeLo = 1299109178U;
  c2_info[115].fileTimeHi = 0U;
  c2_info[115].mFileTimeLo = 0U;
  c2_info[115].mFileTimeHi = 0U;
  c2_info[116].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c2_info[116].name = "eml_blas_inline";
  c2_info[116].dominantType = "";
  c2_info[116].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[116].fileTimeLo = 1299109168U;
  c2_info[116].fileTimeHi = 0U;
  c2_info[116].mFileTimeLo = 0U;
  c2_info[116].mFileTimeHi = 0U;
  c2_info[117].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[117].name = "eml_index_class";
  c2_info[117].dominantType = "";
  c2_info[117].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[117].fileTimeLo = 1323202978U;
  c2_info[117].fileTimeHi = 0U;
  c2_info[117].mFileTimeLo = 0U;
  c2_info[117].mFileTimeHi = 0U;
  c2_info[118].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c2_info[118].name = "eml_refblas_xswap";
  c2_info[118].dominantType = "double";
  c2_info[118].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[118].fileTimeLo = 1299109186U;
  c2_info[118].fileTimeHi = 0U;
  c2_info[118].mFileTimeLo = 0U;
  c2_info[118].mFileTimeHi = 0U;
  c2_info[119].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[119].name = "eml_index_class";
  c2_info[119].dominantType = "";
  c2_info[119].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[119].fileTimeLo = 1323202978U;
  c2_info[119].fileTimeHi = 0U;
  c2_info[119].mFileTimeLo = 0U;
  c2_info[119].mFileTimeHi = 0U;
  c2_info[120].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[120].name = "abs";
  c2_info[120].dominantType = "coder.internal.indexInt";
  c2_info[120].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[120].fileTimeLo = 1343862766U;
  c2_info[120].fileTimeHi = 0U;
  c2_info[120].mFileTimeLo = 0U;
  c2_info[120].mFileTimeHi = 0U;
  c2_info[121].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[121].name = "eml_scalar_abs";
  c2_info[121].dominantType = "coder.internal.indexInt";
  c2_info[121].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[121].fileTimeLo = 1286851112U;
  c2_info[121].fileTimeHi = 0U;
  c2_info[121].mFileTimeLo = 0U;
  c2_info[121].mFileTimeHi = 0U;
  c2_info[122].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[122].name = "eml_int_forloop_overflow_check";
  c2_info[122].dominantType = "";
  c2_info[122].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[122].fileTimeLo = 1346542740U;
  c2_info[122].fileTimeHi = 0U;
  c2_info[122].mFileTimeLo = 0U;
  c2_info[122].mFileTimeHi = 0U;
  c2_info[123].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c2_info[123].name = "eml_index_plus";
  c2_info[123].dominantType = "coder.internal.indexInt";
  c2_info[123].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[123].fileTimeLo = 1286851178U;
  c2_info[123].fileTimeHi = 0U;
  c2_info[123].mFileTimeLo = 0U;
  c2_info[123].mFileTimeHi = 0U;
  c2_info[124].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[124].name = "eml_matlab_zlarfg";
  c2_info[124].dominantType = "double";
  c2_info[124].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[124].fileTimeLo = 1286851222U;
  c2_info[124].fileTimeHi = 0U;
  c2_info[124].mFileTimeLo = 0U;
  c2_info[124].mFileTimeHi = 0U;
  c2_info[125].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[125].name = "eml_scalar_eg";
  c2_info[125].dominantType = "double";
  c2_info[125].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[125].fileTimeLo = 1286851196U;
  c2_info[125].fileTimeHi = 0U;
  c2_info[125].mFileTimeLo = 0U;
  c2_info[125].mFileTimeHi = 0U;
  c2_info[126].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[126].name = "eml_xnrm2";
  c2_info[126].dominantType = "double";
  c2_info[126].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c2_info[126].fileTimeLo = 1299109176U;
  c2_info[126].fileTimeHi = 0U;
  c2_info[126].mFileTimeLo = 0U;
  c2_info[126].mFileTimeHi = 0U;
  c2_info[127].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m!below_threshold";
  c2_info[127].name = "length";
  c2_info[127].dominantType = "double";
  c2_info[127].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[127].fileTimeLo = 1303178606U;
  c2_info[127].fileTimeHi = 0U;
  c2_info[127].mFileTimeLo = 0U;
  c2_info[127].mFileTimeHi = 0U;
}

static void c2_c_info_helper(c2_ResolvedFunctionInfo c2_info[209])
{
  c2_info[128].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c2_info[128].name = "eml_index_class";
  c2_info[128].dominantType = "";
  c2_info[128].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[128].fileTimeLo = 1323202978U;
  c2_info[128].fileTimeHi = 0U;
  c2_info[128].mFileTimeLo = 0U;
  c2_info[128].mFileTimeHi = 0U;
  c2_info[129].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[129].name = "eml_dlapy2";
  c2_info[129].dominantType = "double";
  c2_info[129].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_dlapy2.m";
  c2_info[129].fileTimeLo = 1350443054U;
  c2_info[129].fileTimeHi = 0U;
  c2_info[129].mFileTimeLo = 0U;
  c2_info[129].mFileTimeHi = 0U;
  c2_info[130].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[130].name = "realmin";
  c2_info[130].dominantType = "char";
  c2_info[130].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c2_info[130].fileTimeLo = 1307683642U;
  c2_info[130].fileTimeHi = 0U;
  c2_info[130].mFileTimeLo = 0U;
  c2_info[130].mFileTimeHi = 0U;
  c2_info[131].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[131].name = "eps";
  c2_info[131].dominantType = "char";
  c2_info[131].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[131].fileTimeLo = 1326760396U;
  c2_info[131].fileTimeHi = 0U;
  c2_info[131].mFileTimeLo = 0U;
  c2_info[131].mFileTimeHi = 0U;
  c2_info[132].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[132].name = "abs";
  c2_info[132].dominantType = "double";
  c2_info[132].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[132].fileTimeLo = 1343862766U;
  c2_info[132].fileTimeHi = 0U;
  c2_info[132].mFileTimeLo = 0U;
  c2_info[132].mFileTimeHi = 0U;
  c2_info[133].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[133].name = "eml_index_class";
  c2_info[133].dominantType = "";
  c2_info[133].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[133].fileTimeLo = 1323202978U;
  c2_info[133].fileTimeHi = 0U;
  c2_info[133].mFileTimeLo = 0U;
  c2_info[133].mFileTimeHi = 0U;
  c2_info[134].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[134].name = "eml_index_plus";
  c2_info[134].dominantType = "double";
  c2_info[134].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[134].fileTimeLo = 1286851178U;
  c2_info[134].fileTimeHi = 0U;
  c2_info[134].mFileTimeLo = 0U;
  c2_info[134].mFileTimeHi = 0U;
  c2_info[135].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[135].name = "eml_xscal";
  c2_info[135].dominantType = "double";
  c2_info[135].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[135].fileTimeLo = 1299109176U;
  c2_info[135].fileTimeHi = 0U;
  c2_info[135].mFileTimeLo = 0U;
  c2_info[135].mFileTimeHi = 0U;
  c2_info[136].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c2_info[136].name = "eml_blas_inline";
  c2_info[136].dominantType = "";
  c2_info[136].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[136].fileTimeLo = 1299109168U;
  c2_info[136].fileTimeHi = 0U;
  c2_info[136].mFileTimeLo = 0U;
  c2_info[136].mFileTimeHi = 0U;
  c2_info[137].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c2_info[137].name = "length";
  c2_info[137].dominantType = "double";
  c2_info[137].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[137].fileTimeLo = 1303178606U;
  c2_info[137].fileTimeHi = 0U;
  c2_info[137].mFileTimeLo = 0U;
  c2_info[137].mFileTimeHi = 0U;
  c2_info[138].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[138].name = "eml_index_class";
  c2_info[138].dominantType = "";
  c2_info[138].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[138].fileTimeLo = 1323202978U;
  c2_info[138].fileTimeHi = 0U;
  c2_info[138].mFileTimeLo = 0U;
  c2_info[138].mFileTimeHi = 0U;
  c2_info[139].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[139].name = "eml_scalar_eg";
  c2_info[139].dominantType = "double";
  c2_info[139].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[139].fileTimeLo = 1286851196U;
  c2_info[139].fileTimeHi = 0U;
  c2_info[139].mFileTimeLo = 0U;
  c2_info[139].mFileTimeHi = 0U;
  c2_info[140].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c2_info[140].name = "eml_refblas_xscal";
  c2_info[140].dominantType = "double";
  c2_info[140].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[140].fileTimeLo = 1299109184U;
  c2_info[140].fileTimeHi = 0U;
  c2_info[140].mFileTimeLo = 0U;
  c2_info[140].mFileTimeHi = 0U;
  c2_info[141].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[141].name = "eml_index_class";
  c2_info[141].dominantType = "";
  c2_info[141].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[141].fileTimeLo = 1323202978U;
  c2_info[141].fileTimeHi = 0U;
  c2_info[141].mFileTimeLo = 0U;
  c2_info[141].mFileTimeHi = 0U;
  c2_info[142].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[142].name = "eml_index_minus";
  c2_info[142].dominantType = "double";
  c2_info[142].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[142].fileTimeLo = 1286851178U;
  c2_info[142].fileTimeHi = 0U;
  c2_info[142].mFileTimeLo = 0U;
  c2_info[142].mFileTimeHi = 0U;
  c2_info[143].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[143].name = "eml_index_times";
  c2_info[143].dominantType = "coder.internal.indexInt";
  c2_info[143].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[143].fileTimeLo = 1286851180U;
  c2_info[143].fileTimeHi = 0U;
  c2_info[143].mFileTimeLo = 0U;
  c2_info[143].mFileTimeHi = 0U;
  c2_info[144].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[144].name = "eml_index_plus";
  c2_info[144].dominantType = "coder.internal.indexInt";
  c2_info[144].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[144].fileTimeLo = 1286851178U;
  c2_info[144].fileTimeHi = 0U;
  c2_info[144].mFileTimeLo = 0U;
  c2_info[144].mFileTimeHi = 0U;
  c2_info[145].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c2_info[145].name = "eml_int_forloop_overflow_check";
  c2_info[145].dominantType = "";
  c2_info[145].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[145].fileTimeLo = 1346542740U;
  c2_info[145].fileTimeHi = 0U;
  c2_info[145].mFileTimeLo = 0U;
  c2_info[145].mFileTimeHi = 0U;
  c2_info[146].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[146].name = "mtimes";
  c2_info[146].dominantType = "double";
  c2_info[146].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[146].fileTimeLo = 1289552092U;
  c2_info[146].fileTimeHi = 0U;
  c2_info[146].mFileTimeLo = 0U;
  c2_info[146].mFileTimeHi = 0U;
  c2_info[147].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[147].name = "eml_div";
  c2_info[147].dominantType = "double";
  c2_info[147].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[147].fileTimeLo = 1313380210U;
  c2_info[147].fileTimeHi = 0U;
  c2_info[147].mFileTimeLo = 0U;
  c2_info[147].mFileTimeHi = 0U;
  c2_info[148].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m";
  c2_info[148].name = "eml_int_forloop_overflow_check";
  c2_info[148].dominantType = "";
  c2_info[148].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[148].fileTimeLo = 1346542740U;
  c2_info[148].fileTimeHi = 0U;
  c2_info[148].mFileTimeLo = 0U;
  c2_info[148].mFileTimeHi = 0U;
  c2_info[149].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[149].name = "eml_matlab_zlarf";
  c2_info[149].dominantType = "char";
  c2_info[149].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[149].fileTimeLo = 1286851222U;
  c2_info[149].fileTimeHi = 0U;
  c2_info[149].mFileTimeLo = 0U;
  c2_info[149].mFileTimeHi = 0U;
  c2_info[150].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[150].name = "eml_scalar_eg";
  c2_info[150].dominantType = "double";
  c2_info[150].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[150].fileTimeLo = 1286851196U;
  c2_info[150].fileTimeHi = 0U;
  c2_info[150].mFileTimeLo = 0U;
  c2_info[150].mFileTimeHi = 0U;
  c2_info[151].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[151].name = "eml_index_class";
  c2_info[151].dominantType = "";
  c2_info[151].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[151].fileTimeLo = 1323202978U;
  c2_info[151].fileTimeHi = 0U;
  c2_info[151].mFileTimeLo = 0U;
  c2_info[151].mFileTimeHi = 0U;
  c2_info[152].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[152].name = "isequal";
  c2_info[152].dominantType = "double";
  c2_info[152].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[152].fileTimeLo = 1286851158U;
  c2_info[152].fileTimeHi = 0U;
  c2_info[152].mFileTimeLo = 0U;
  c2_info[152].mFileTimeHi = 0U;
  c2_info[153].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[153].name = "coder.internal.indexIntRelop";
  c2_info[153].dominantType = "";
  c2_info[153].resolved =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m";
  c2_info[153].fileTimeLo = 1326760722U;
  c2_info[153].fileTimeHi = 0U;
  c2_info[153].mFileTimeLo = 0U;
  c2_info[153].mFileTimeHi = 0U;
  c2_info[154].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass";
  c2_info[154].name = "eml_float_model";
  c2_info[154].dominantType = "char";
  c2_info[154].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c2_info[154].fileTimeLo = 1326760396U;
  c2_info[154].fileTimeHi = 0U;
  c2_info[154].mFileTimeLo = 0U;
  c2_info[154].mFileTimeHi = 0U;
  c2_info[155].context =
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass";
  c2_info[155].name = "intmin";
  c2_info[155].dominantType = "char";
  c2_info[155].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[155].fileTimeLo = 1311287718U;
  c2_info[155].fileTimeHi = 0U;
  c2_info[155].mFileTimeLo = 0U;
  c2_info[155].mFileTimeHi = 0U;
  c2_info[156].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[156].name = "eml_index_minus";
  c2_info[156].dominantType = "double";
  c2_info[156].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[156].fileTimeLo = 1286851178U;
  c2_info[156].fileTimeHi = 0U;
  c2_info[156].mFileTimeLo = 0U;
  c2_info[156].mFileTimeHi = 0U;
  c2_info[157].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[157].name = "eml_index_times";
  c2_info[157].dominantType = "double";
  c2_info[157].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[157].fileTimeLo = 1286851180U;
  c2_info[157].fileTimeHi = 0U;
  c2_info[157].mFileTimeLo = 0U;
  c2_info[157].mFileTimeHi = 0U;
  c2_info[158].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[158].name = "eml_index_plus";
  c2_info[158].dominantType = "coder.internal.indexInt";
  c2_info[158].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[158].fileTimeLo = 1286851178U;
  c2_info[158].fileTimeHi = 0U;
  c2_info[158].mFileTimeLo = 0U;
  c2_info[158].mFileTimeHi = 0U;
  c2_info[159].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc";
  c2_info[159].name = "eml_index_class";
  c2_info[159].dominantType = "";
  c2_info[159].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[159].fileTimeLo = 1323202978U;
  c2_info[159].fileTimeHi = 0U;
  c2_info[159].mFileTimeLo = 0U;
  c2_info[159].mFileTimeHi = 0U;
  c2_info[160].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc";
  c2_info[160].name = "eml_index_minus";
  c2_info[160].dominantType = "double";
  c2_info[160].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[160].fileTimeLo = 1286851178U;
  c2_info[160].fileTimeHi = 0U;
  c2_info[160].mFileTimeLo = 0U;
  c2_info[160].mFileTimeHi = 0U;
  c2_info[161].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc";
  c2_info[161].name = "eml_index_times";
  c2_info[161].dominantType = "coder.internal.indexInt";
  c2_info[161].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[161].fileTimeLo = 1286851180U;
  c2_info[161].fileTimeHi = 0U;
  c2_info[161].mFileTimeLo = 0U;
  c2_info[161].mFileTimeHi = 0U;
  c2_info[162].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc";
  c2_info[162].name = "eml_index_plus";
  c2_info[162].dominantType = "coder.internal.indexInt";
  c2_info[162].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[162].fileTimeLo = 1286851178U;
  c2_info[162].fileTimeHi = 0U;
  c2_info[162].mFileTimeLo = 0U;
  c2_info[162].mFileTimeHi = 0U;
  c2_info[163].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc";
  c2_info[163].name = "eml_int_forloop_overflow_check";
  c2_info[163].dominantType = "";
  c2_info[163].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[163].fileTimeLo = 1346542740U;
  c2_info[163].fileTimeHi = 0U;
  c2_info[163].mFileTimeLo = 0U;
  c2_info[163].mFileTimeHi = 0U;
  c2_info[164].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[164].name = "eml_xgemv";
  c2_info[164].dominantType = "char";
  c2_info[164].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m";
  c2_info[164].fileTimeLo = 1299109174U;
  c2_info[164].fileTimeHi = 0U;
  c2_info[164].mFileTimeLo = 0U;
  c2_info[164].mFileTimeHi = 0U;
  c2_info[165].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m";
  c2_info[165].name = "eml_blas_inline";
  c2_info[165].dominantType = "";
  c2_info[165].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[165].fileTimeLo = 1299109168U;
  c2_info[165].fileTimeHi = 0U;
  c2_info[165].mFileTimeLo = 0U;
  c2_info[165].mFileTimeHi = 0U;
  c2_info[166].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m!below_threshold";
  c2_info[166].name = "length";
  c2_info[166].dominantType = "double";
  c2_info[166].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[166].fileTimeLo = 1303178606U;
  c2_info[166].fileTimeHi = 0U;
  c2_info[166].mFileTimeLo = 0U;
  c2_info[166].mFileTimeHi = 0U;
  c2_info[167].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m!below_threshold";
  c2_info[167].name = "min";
  c2_info[167].dominantType = "double";
  c2_info[167].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[167].fileTimeLo = 1311287718U;
  c2_info[167].fileTimeHi = 0U;
  c2_info[167].mFileTimeLo = 0U;
  c2_info[167].mFileTimeHi = 0U;
  c2_info[168].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m!below_threshold";
  c2_info[168].name = "mtimes";
  c2_info[168].dominantType = "double";
  c2_info[168].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[168].fileTimeLo = 1289552092U;
  c2_info[168].fileTimeHi = 0U;
  c2_info[168].mFileTimeLo = 0U;
  c2_info[168].mFileTimeHi = 0U;
  c2_info[169].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m";
  c2_info[169].name = "eml_index_class";
  c2_info[169].dominantType = "";
  c2_info[169].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[169].fileTimeLo = 1323202978U;
  c2_info[169].fileTimeHi = 0U;
  c2_info[169].mFileTimeLo = 0U;
  c2_info[169].mFileTimeHi = 0U;
  c2_info[170].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m";
  c2_info[170].name = "eml_scalar_eg";
  c2_info[170].dominantType = "double";
  c2_info[170].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[170].fileTimeLo = 1286851196U;
  c2_info[170].fileTimeHi = 0U;
  c2_info[170].mFileTimeLo = 0U;
  c2_info[170].mFileTimeHi = 0U;
  c2_info[171].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemv.m";
  c2_info[171].name = "eml_refblas_xgemv";
  c2_info[171].dominantType = "char";
  c2_info[171].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[171].fileTimeLo = 1299109176U;
  c2_info[171].fileTimeHi = 0U;
  c2_info[171].mFileTimeLo = 0U;
  c2_info[171].mFileTimeHi = 0U;
  c2_info[172].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[172].name = "eml_index_minus";
  c2_info[172].dominantType = "double";
  c2_info[172].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[172].fileTimeLo = 1286851178U;
  c2_info[172].fileTimeHi = 0U;
  c2_info[172].mFileTimeLo = 0U;
  c2_info[172].mFileTimeHi = 0U;
  c2_info[173].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[173].name = "eml_index_class";
  c2_info[173].dominantType = "";
  c2_info[173].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[173].fileTimeLo = 1323202978U;
  c2_info[173].fileTimeHi = 0U;
  c2_info[173].mFileTimeLo = 0U;
  c2_info[173].mFileTimeHi = 0U;
  c2_info[174].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[174].name = "eml_index_times";
  c2_info[174].dominantType = "coder.internal.indexInt";
  c2_info[174].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[174].fileTimeLo = 1286851180U;
  c2_info[174].fileTimeHi = 0U;
  c2_info[174].mFileTimeLo = 0U;
  c2_info[174].mFileTimeHi = 0U;
  c2_info[175].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[175].name = "eml_index_plus";
  c2_info[175].dominantType = "coder.internal.indexInt";
  c2_info[175].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[175].fileTimeLo = 1286851178U;
  c2_info[175].fileTimeHi = 0U;
  c2_info[175].mFileTimeLo = 0U;
  c2_info[175].mFileTimeHi = 0U;
  c2_info[176].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[176].name = "eml_int_forloop_overflow_check";
  c2_info[176].dominantType = "";
  c2_info[176].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[176].fileTimeLo = 1346542740U;
  c2_info[176].fileTimeHi = 0U;
  c2_info[176].mFileTimeLo = 0U;
  c2_info[176].mFileTimeHi = 0U;
  c2_info[177].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[177].name = "eml_scalar_eg";
  c2_info[177].dominantType = "double";
  c2_info[177].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[177].fileTimeLo = 1286851196U;
  c2_info[177].fileTimeHi = 0U;
  c2_info[177].mFileTimeLo = 0U;
  c2_info[177].mFileTimeHi = 0U;
  c2_info[178].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemv.m";
  c2_info[178].name = "eml_conjtimes";
  c2_info[178].dominantType = "double";
  c2_info[178].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_conjtimes.m";
  c2_info[178].fileTimeLo = 1286851096U;
  c2_info[178].fileTimeHi = 0U;
  c2_info[178].mFileTimeLo = 0U;
  c2_info[178].mFileTimeHi = 0U;
  c2_info[179].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m";
  c2_info[179].name = "eml_xgerc";
  c2_info[179].dominantType = "double";
  c2_info[179].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m";
  c2_info[179].fileTimeLo = 1299109174U;
  c2_info[179].fileTimeHi = 0U;
  c2_info[179].mFileTimeLo = 0U;
  c2_info[179].mFileTimeHi = 0U;
  c2_info[180].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m";
  c2_info[180].name = "eml_blas_inline";
  c2_info[180].dominantType = "";
  c2_info[180].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[180].fileTimeLo = 1299109168U;
  c2_info[180].fileTimeHi = 0U;
  c2_info[180].mFileTimeLo = 0U;
  c2_info[180].mFileTimeHi = 0U;
  c2_info[181].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m";
  c2_info[181].name = "eml_xger";
  c2_info[181].dominantType = "double";
  c2_info[181].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[181].fileTimeLo = 1299109174U;
  c2_info[181].fileTimeHi = 0U;
  c2_info[181].mFileTimeLo = 0U;
  c2_info[181].mFileTimeHi = 0U;
  c2_info[182].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c2_info[182].name = "eml_blas_inline";
  c2_info[182].dominantType = "";
  c2_info[182].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c2_info[182].fileTimeLo = 1299109168U;
  c2_info[182].fileTimeHi = 0U;
  c2_info[182].mFileTimeLo = 0U;
  c2_info[182].mFileTimeHi = 0U;
  c2_info[183].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[183].name = "intmax";
  c2_info[183].dominantType = "char";
  c2_info[183].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[183].fileTimeLo = 1311287716U;
  c2_info[183].fileTimeHi = 0U;
  c2_info[183].mFileTimeLo = 0U;
  c2_info[183].mFileTimeHi = 0U;
  c2_info[184].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[184].name = "min";
  c2_info[184].dominantType = "double";
  c2_info[184].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m";
  c2_info[184].fileTimeLo = 1311287718U;
  c2_info[184].fileTimeHi = 0U;
  c2_info[184].mFileTimeLo = 0U;
  c2_info[184].mFileTimeHi = 0U;
  c2_info[185].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[185].name = "length";
  c2_info[185].dominantType = "double";
  c2_info[185].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[185].fileTimeLo = 1303178606U;
  c2_info[185].fileTimeHi = 0U;
  c2_info[185].mFileTimeLo = 0U;
  c2_info[185].mFileTimeHi = 0U;
  c2_info[186].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c2_info[186].name = "mtimes";
  c2_info[186].dominantType = "double";
  c2_info[186].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[186].fileTimeLo = 1289552092U;
  c2_info[186].fileTimeHi = 0U;
  c2_info[186].mFileTimeLo = 0U;
  c2_info[186].mFileTimeHi = 0U;
  c2_info[187].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[187].name = "eml_index_class";
  c2_info[187].dominantType = "";
  c2_info[187].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[187].fileTimeLo = 1323202978U;
  c2_info[187].fileTimeHi = 0U;
  c2_info[187].mFileTimeLo = 0U;
  c2_info[187].mFileTimeHi = 0U;
  c2_info[188].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c2_info[188].name = "eml_refblas_xger";
  c2_info[188].dominantType = "double";
  c2_info[188].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[188].fileTimeLo = 1299109176U;
  c2_info[188].fileTimeHi = 0U;
  c2_info[188].mFileTimeLo = 0U;
  c2_info[188].mFileTimeHi = 0U;
  c2_info[189].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c2_info[189].name = "eml_refblas_xgerx";
  c2_info[189].dominantType = "char";
  c2_info[189].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[189].fileTimeLo = 1299109178U;
  c2_info[189].fileTimeHi = 0U;
  c2_info[189].mFileTimeLo = 0U;
  c2_info[189].mFileTimeHi = 0U;
  c2_info[190].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[190].name = "eml_index_class";
  c2_info[190].dominantType = "";
  c2_info[190].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[190].fileTimeLo = 1323202978U;
  c2_info[190].fileTimeHi = 0U;
  c2_info[190].mFileTimeLo = 0U;
  c2_info[190].mFileTimeHi = 0U;
  c2_info[191].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[191].name = "abs";
  c2_info[191].dominantType = "coder.internal.indexInt";
  c2_info[191].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[191].fileTimeLo = 1343862766U;
  c2_info[191].fileTimeHi = 0U;
  c2_info[191].mFileTimeLo = 0U;
  c2_info[191].mFileTimeHi = 0U;
}

static void c2_d_info_helper(c2_ResolvedFunctionInfo c2_info[209])
{
  c2_info[192].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[192].name = "eml_index_minus";
  c2_info[192].dominantType = "double";
  c2_info[192].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[192].fileTimeLo = 1286851178U;
  c2_info[192].fileTimeHi = 0U;
  c2_info[192].mFileTimeLo = 0U;
  c2_info[192].mFileTimeHi = 0U;
  c2_info[193].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[193].name = "eml_int_forloop_overflow_check";
  c2_info[193].dominantType = "";
  c2_info[193].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c2_info[193].fileTimeLo = 1346542740U;
  c2_info[193].fileTimeHi = 0U;
  c2_info[193].mFileTimeLo = 0U;
  c2_info[193].mFileTimeHi = 0U;
  c2_info[194].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[194].name = "eml_index_plus";
  c2_info[194].dominantType = "double";
  c2_info[194].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[194].fileTimeLo = 1286851178U;
  c2_info[194].fileTimeHi = 0U;
  c2_info[194].mFileTimeLo = 0U;
  c2_info[194].mFileTimeHi = 0U;
  c2_info[195].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c2_info[195].name = "eml_index_plus";
  c2_info[195].dominantType = "coder.internal.indexInt";
  c2_info[195].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[195].fileTimeLo = 1286851178U;
  c2_info[195].fileTimeHi = 0U;
  c2_info[195].mFileTimeLo = 0U;
  c2_info[195].mFileTimeHi = 0U;
  c2_info[196].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[196].name = "abs";
  c2_info[196].dominantType = "double";
  c2_info[196].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[196].fileTimeLo = 1343862766U;
  c2_info[196].fileTimeHi = 0U;
  c2_info[196].mFileTimeLo = 0U;
  c2_info[196].mFileTimeHi = 0U;
  c2_info[197].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m";
  c2_info[197].name = "mtimes";
  c2_info[197].dominantType = "double";
  c2_info[197].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[197].fileTimeLo = 1289552092U;
  c2_info[197].fileTimeHi = 0U;
  c2_info[197].mFileTimeLo = 0U;
  c2_info[197].mFileTimeHi = 0U;
  c2_info[198].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[198].name = "max";
  c2_info[198].dominantType = "double";
  c2_info[198].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[198].fileTimeLo = 1311287716U;
  c2_info[198].fileTimeHi = 0U;
  c2_info[198].mFileTimeLo = 0U;
  c2_info[198].mFileTimeHi = 0U;
  c2_info[199].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c2_info[199].name = "eml_min_or_max";
  c2_info[199].dominantType = "char";
  c2_info[199].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c2_info[199].fileTimeLo = 1334103890U;
  c2_info[199].fileTimeHi = 0U;
  c2_info[199].mFileTimeLo = 0U;
  c2_info[199].mFileTimeHi = 0U;
  c2_info[200].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[200].name = "eml_xcabs1";
  c2_info[200].dominantType = "double";
  c2_info[200].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c2_info[200].fileTimeLo = 1286851106U;
  c2_info[200].fileTimeHi = 0U;
  c2_info[200].mFileTimeLo = 0U;
  c2_info[200].mFileTimeHi = 0U;
  c2_info[201].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[201].name = "mtimes";
  c2_info[201].dominantType = "double";
  c2_info[201].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[201].fileTimeLo = 1289552092U;
  c2_info[201].fileTimeHi = 0U;
  c2_info[201].mFileTimeLo = 0U;
  c2_info[201].mFileTimeHi = 0U;
  c2_info[202].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[202].name = "eps";
  c2_info[202].dominantType = "char";
  c2_info[202].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[202].fileTimeLo = 1326760396U;
  c2_info[202].fileTimeHi = 0U;
  c2_info[202].mFileTimeLo = 0U;
  c2_info[202].mFileTimeHi = 0U;
  c2_info[203].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[203].name = "eml_flt2str";
  c2_info[203].dominantType = "double";
  c2_info[203].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[203].fileTimeLo = 1309483596U;
  c2_info[203].fileTimeHi = 0U;
  c2_info[203].mFileTimeLo = 0U;
  c2_info[203].mFileTimeHi = 0U;
  c2_info[204].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c2_info[204].name = "char";
  c2_info[204].dominantType = "double";
  c2_info[204].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/strfun/char.m";
  c2_info[204].fileTimeLo = 1319762368U;
  c2_info[204].fileTimeHi = 0U;
  c2_info[204].mFileTimeLo = 0U;
  c2_info[204].mFileTimeHi = 0U;
  c2_info[205].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[205].name = "eml_warning";
  c2_info[205].dominantType = "char";
  c2_info[205].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[205].fileTimeLo = 1286851202U;
  c2_info[205].fileTimeHi = 0U;
  c2_info[205].mFileTimeLo = 0U;
  c2_info[205].mFileTimeHi = 0U;
  c2_info[206].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[206].name = "eml_scalar_eg";
  c2_info[206].dominantType = "double";
  c2_info[206].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[206].fileTimeLo = 1286851196U;
  c2_info[206].fileTimeHi = 0U;
  c2_info[206].mFileTimeLo = 0U;
  c2_info[206].mFileTimeHi = 0U;
  c2_info[207].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[207].name = "eml_conjtimes";
  c2_info[207].dominantType = "double";
  c2_info[207].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_conjtimes.m";
  c2_info[207].fileTimeLo = 1286851096U;
  c2_info[207].fileTimeHi = 0U;
  c2_info[207].mFileTimeLo = 0U;
  c2_info[207].mFileTimeHi = 0U;
  c2_info[208].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m";
  c2_info[208].name = "eml_div";
  c2_info[208].dominantType = "double";
  c2_info[208].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[208].fileTimeLo = 1313380210U;
  c2_info[208].fileTimeHi = 0U;
  c2_info[208].mFileTimeLo = 0U;
  c2_info[208].mFileTimeHi = 0U;
}

static real_T c2_mpower(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_e_a;
  real_T c2_b;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_eml_scalar_eg(chartInstance);
  c2_e_a = c2_d_a;
  c2_b = c2_d_a;
  return c2_e_a * c2_b;
}

static void c2_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static real_T c2_sqrt(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_eml_error(SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
  int32_T c2_i100;
  static char_T c2_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  int32_T c2_i101;
  static char_T c2_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  for (c2_i100 = 0; c2_i100 < 30; c2_i100++) {
    c2_u[c2_i100] = c2_cv0[c2_i100];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c2_i101 = 0; c2_i101 < 4; c2_i101++) {
    c2_b_u[c2_i101] = c2_cv1[c2_i101];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c2_y, 14, c2_b_y));
}

static real_T c2_dot(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
                     real_T c2_a[3], real_T c2_b[3])
{
  real_T c2_c;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  c2_c = 0.0;
  for (c2_k = 1; c2_k < 4; c2_k++) {
    c2_b_k = c2_k;
    c2_c += c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 3, 1, 0) - 1] * c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 3, 1, 0) - 1];
  }

  return c2_c;
}

static void c2_c_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static void c2_mldivide(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_A[20], real_T c2_B[5], real_T c2_Y[4])
{
  int32_T c2_i102;
  real_T c2_b_A[20];
  int32_T c2_i103;
  real_T c2_b_B[5];
  int32_T c2_i104;
  int32_T c2_jpvt[4];
  int32_T c2_i105;
  real_T c2_work[4];
  real_T c2_TOL3Z;
  int32_T c2_k;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_i106;
  real_T c2_c_A[20];
  real_T c2_vn1[4];
  real_T c2_vn2[4];
  int32_T c2_a;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_b_a;
  int32_T c2_im1;
  int32_T c2_c_a;
  int32_T c2_ip1;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_e_a;
  int32_T c2_b;
  int32_T c2_i_i;
  int32_T c2_b_b;
  int32_T c2_nmi;
  int32_T c2_c_b;
  int32_T c2_mmi;
  int32_T c2_d_b;
  int32_T c2_mmip1;
  int32_T c2_e_b;
  int32_T c2_nmip1;
  int32_T c2_f_a;
  int32_T c2_i107;
  real_T c2_b_vn1[4];
  int32_T c2_f_b;
  int32_T c2_pvt;
  int32_T c2_g_a;
  int32_T c2_b_c;
  int32_T c2_g_b;
  int32_T c2_c_c;
  int32_T c2_h_b;
  int32_T c2_pvtcol;
  int32_T c2_i_b;
  int32_T c2_d_c;
  int32_T c2_j_b;
  int32_T c2_mcol;
  int32_T c2_itemp;
  real_T c2_atmp;
  int32_T c2_h_a;
  int32_T c2_e_c;
  real_T c2_b_atmp;
  real_T c2_d5;
  real_T c2_tau[4];
  real_T c2_c_atmp;
  real_T c2_d6;
  real_T c2_d7;
  int32_T c2_i_a;
  int32_T c2_f_c;
  int32_T c2_j_a;
  int32_T c2_k_b;
  int32_T c2_i_ip1;
  int32_T c2_b_ip1;
  boolean_T c2_overflow;
  int32_T c2_c_j;
  int32_T c2_k_a;
  int32_T c2_g_c;
  int32_T c2_l_b;
  int32_T c2_h_c;
  int32_T c2_l_a;
  int32_T c2_m_b;
  int32_T c2_i_j;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_y;
  real_T c2_temp1;
  real_T c2_m_a;
  real_T c2_n_b;
  real_T c2_b_y;
  real_T c2_temp2;
  real_T c2_n_a;
  real_T c2_o_b;
  real_T c2_c_y;
  real_T c2_o_a;
  real_T c2_p_b;
  int32_T c2_p_a;
  int32_T c2_i_c;
  int32_T c2_n;
  int32_T c2_ix0;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  real_T c2_d_y;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_q_a;
  int32_T c2_j_c;
  int32_T c2_r_a;
  int32_T c2_k_c;
  int32_T c2_s_a;
  int32_T c2_q_b;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_t_a;
  int32_T c2_r_b;
  int32_T c2_u_a;
  int32_T c2_s_b;
  boolean_T c2_b_overflow;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_absxk;
  real_T c2_t;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_v_a;
  real_T c2_t_b;
  real_T c2_e_y;
  real_T c2_rankR;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_f_y;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_g_y;
  real_T c2_d;
  real_T c2_u_b;
  real_T c2_h_y;
  real_T c2_w_a;
  real_T c2_tol;
  int32_T c2_d_k;
  real_T c2_e_k;
  real_T c2_n_x;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_i_y;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_j_y;
  real_T c2_b_d;
  real_T c2_s_x;
  int32_T c2_i108;
  static char_T c2_cv2[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c2_u[8];
  const mxArray *c2_k_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_l_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_m_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_n_y = NULL;
  char_T c2_str[14];
  int32_T c2_i109;
  char_T c2_b_str[14];
  int32_T c2_i110;
  int32_T c2_d_j;
  real_T c2_e_j;
  real_T c2_tauj;
  real_T c2_wj;
  real_T c2_d8;
  int32_T c2_i111;
  int32_T c2_c_i;
  real_T c2_d_i;
  real_T c2_x_a;
  real_T c2_v_b;
  real_T c2_z;
  real_T c2_y_a;
  real_T c2_w_b;
  real_T c2_d9;
  int32_T c2_i112;
  int32_T c2_e_i;
  real_T c2_ab_a;
  real_T c2_x_b;
  real_T c2_o_y;
  real_T c2_rr;
  real_T c2_b_rr;
  int32_T c2_i113;
  int32_T c2_f_i;
  real_T c2_c_rr;
  int32_T c2_i114;
  int32_T c2_f_j;
  int32_T c2_pj;
  real_T c2_t_x;
  real_T c2_p_y;
  real_T c2_b_z;
  real_T c2_d10;
  int32_T c2_i115;
  int32_T c2_g_i;
  real_T c2_bb_a;
  real_T c2_y_b;
  real_T c2_q_y;
  boolean_T exitg1;
  for (c2_i102 = 0; c2_i102 < 20; c2_i102++) {
    c2_b_A[c2_i102] = c2_A[c2_i102];
  }

  for (c2_i103 = 0; c2_i103 < 5; c2_i103++) {
    c2_b_B[c2_i103] = c2_B[c2_i103];
  }

  c2_d_eml_scalar_eg(chartInstance);
  for (c2_i104 = 0; c2_i104 < 4; c2_i104++) {
    c2_jpvt[c2_i104] = 1 + c2_i104;
  }

  c2_d_eml_scalar_eg(chartInstance);
  for (c2_i105 = 0; c2_i105 < 4; c2_i105++) {
    c2_work[c2_i105] = 0.0;
  }

  c2_eps(chartInstance);
  c2_TOL3Z = 2.2204460492503131E-16;
  c2_b_sqrt(chartInstance, &c2_TOL3Z);
  c2_d_eml_scalar_eg(chartInstance);
  c2_k = 1;
  for (c2_j = 1; c2_j < 5; c2_j++) {
    c2_b_j = c2_j;
    for (c2_i106 = 0; c2_i106 < 20; c2_i106++) {
      c2_c_A[c2_i106] = c2_b_A[c2_i106];
    }

    c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_eml_xnrm2(chartInstance, c2_c_A,
      c2_k);
    c2_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 1, 0) - 1];
    c2_a = c2_k + 5;
    c2_k = c2_a;
  }

  for (c2_i = 1; c2_i < 5; c2_i++) {
    c2_b_i = c2_i;
    c2_b_a = c2_b_i - 1;
    c2_im1 = c2_b_a;
    c2_c_a = c2_b_i;
    c2_ip1 = c2_c_a;
    c2_d_a = c2_im1;
    c2_c = c2_d_a * 5;
    c2_e_a = c2_b_i;
    c2_b = c2_c;
    c2_i_i = c2_e_a + c2_b;
    c2_b_b = c2_b_i;
    c2_nmi = 4 - c2_b_b;
    c2_c_b = c2_b_i;
    c2_mmi = 5 - c2_c_b;
    c2_d_b = c2_mmi + 1;
    c2_mmip1 = c2_d_b;
    c2_e_b = c2_nmi;
    c2_nmip1 = c2_e_b;
    c2_f_a = c2_im1;
    for (c2_i107 = 0; c2_i107 < 4; c2_i107++) {
      c2_b_vn1[c2_i107] = c2_vn1[c2_i107];
    }

    c2_f_b = c2_eml_ixamax(chartInstance, c2_nmip1 + 1, c2_b_vn1, c2_b_i);
    c2_pvt = c2_f_a + c2_f_b;
    if (c2_pvt != c2_b_i) {
      c2_g_a = c2_pvt;
      c2_b_c = c2_g_a;
      c2_g_b = c2_b_c - 1;
      c2_c_c = 5 * c2_g_b;
      c2_h_b = c2_c_c;
      c2_pvtcol = c2_h_b;
      c2_i_b = c2_im1;
      c2_d_c = 5 * c2_i_b;
      c2_j_b = c2_d_c;
      c2_mcol = c2_j_b;
      c2_b_eml_xswap(chartInstance, c2_b_A, c2_pvtcol + 1, c2_mcol + 1);
      c2_itemp = c2_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_pvt), 1, 4, 1, 0) - 1];
      c2_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_pvt), 1, 4, 1, 0) - 1] = c2_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
      c2_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_i), 1, 4, 1, 0) - 1] = c2_itemp;
      c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_pvt), 1, 4, 1, 0) - 1] = c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
      c2_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_pvt), 1, 4, 1, 0) - 1] = c2_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1];
    }

    c2_atmp = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", (real_T)c2_i_i), 1, 20, 1, 0) - 1];
    if (c2_b_i < 5) {
      c2_h_a = c2_i_i;
      c2_e_c = c2_h_a;
      c2_b_atmp = c2_atmp;
      c2_d5 = c2_c_eml_matlab_zlarfg(chartInstance, c2_mmip1, &c2_b_atmp, c2_b_A,
        c2_e_c + 1);
      c2_atmp = c2_b_atmp;
      c2_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_i), 1, 4, 1, 0) - 1] = c2_d5;
    } else {
      c2_c_atmp = c2_atmp;
      c2_d6 = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c2_i_i), 1, 20, 1, 0) - 1];
      c2_d7 = c2_d_eml_matlab_zlarfg(chartInstance, &c2_c_atmp, &c2_d6);
      c2_atmp = c2_c_atmp;
      c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_i_i), 1, 20, 1, 0) - 1] = c2_d6;
      c2_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_i), 1, 4, 1, 0) - 1] = c2_d7;
    }

    c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_i_i), 1, 20, 1, 0) - 1] = c2_atmp;
    if (c2_b_i < 4) {
      c2_atmp = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_i_i), 1, 20, 1, 0) - 1];
      c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_i_i), 1, 20, 1, 0) - 1] = 1.0;
      c2_i_a = c2_b_i;
      c2_f_c = c2_i_a * 5;
      c2_j_a = c2_b_i;
      c2_k_b = c2_f_c;
      c2_i_ip1 = c2_j_a + c2_k_b;
      c2_b_eml_matlab_zlarf(chartInstance, c2_mmip1, c2_nmi, c2_i_i,
                            c2_tau[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 4, 1, 0) - 1], c2_b_A,
                            c2_i_ip1, c2_work);
      c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_i_i), 1, 20, 1, 0) - 1] = c2_atmp;
    }

    c2_b_ip1 = c2_ip1 + 1;
    c2_overflow = FALSE;
    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_c_j = c2_b_ip1; c2_c_j < 5; c2_c_j++) {
      c2_b_j = c2_c_j;
      c2_k_a = c2_b_j;
      c2_g_c = c2_k_a;
      c2_l_b = c2_g_c - 1;
      c2_h_c = 5 * c2_l_b;
      c2_l_a = c2_b_i;
      c2_m_b = c2_h_c;
      c2_i_j = c2_l_a + c2_m_b;
      if (c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j), 1, 4, 1, 0) - 1] != 0.0) {
        c2_x = c2_b_A[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_i), 1, 5, 1, 0) + 5 *
                       (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 2, 0) - 1)) - 1];
        c2_b_x = c2_x;
        c2_y = muDoubleScalarAbs(c2_b_x);
        c2_temp1 = c2_y / c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 1, 0) - 1];
        c2_m_a = c2_temp1;
        c2_n_b = c2_temp1;
        c2_b_y = c2_m_a * c2_n_b;
        c2_temp1 = 1.0 - c2_b_y;
        if (c2_temp1 < 0.0) {
          c2_temp1 = 0.0;
        }

        c2_temp2 = c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 1, 0) - 1] /
          c2_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_j), 1, 4, 1, 0) - 1];
        c2_n_a = c2_temp2;
        c2_o_b = c2_temp2;
        c2_c_y = c2_n_a * c2_o_b;
        c2_o_a = c2_temp1;
        c2_p_b = c2_c_y;
        c2_temp2 = c2_o_a * c2_p_b;
        if (c2_temp2 <= c2_TOL3Z) {
          if (c2_b_i < 5) {
            c2_p_a = c2_i_j;
            c2_i_c = c2_p_a;
            c2_n = c2_mmi;
            c2_ix0 = c2_i_c + 1;
            c2_b_n = c2_n;
            c2_b_ix0 = c2_ix0;
            c2_below_threshold(chartInstance);
            c2_c_n = c2_b_n;
            c2_c_ix0 = c2_b_ix0;
            c2_d_y = 0.0;
            if (c2_c_n < 1) {
            } else if (c2_c_n == 1) {
              c2_c_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_c_ix0), 1, 20, 1, 0) - 1];
              c2_d_x = c2_c_x;
              c2_d_y = muDoubleScalarAbs(c2_d_x);
            } else {
              c2_realmin(chartInstance);
              c2_scale = 2.2250738585072014E-308;
              c2_kstart = c2_c_ix0;
              c2_q_a = c2_c_n;
              c2_j_c = c2_q_a;
              c2_r_a = c2_j_c - 1;
              c2_k_c = c2_r_a;
              c2_s_a = c2_kstart;
              c2_q_b = c2_k_c;
              c2_kend = c2_s_a + c2_q_b;
              c2_b_kstart = c2_kstart;
              c2_b_kend = c2_kend;
              c2_t_a = c2_b_kstart;
              c2_r_b = c2_b_kend;
              c2_u_a = c2_t_a;
              c2_s_b = c2_r_b;
              if (c2_u_a > c2_s_b) {
                c2_b_overflow = FALSE;
              } else {
                c2_b_overflow = (c2_s_b > 2147483646);
              }

              if (c2_b_overflow) {
                c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
              }

              for (c2_b_k = c2_b_kstart; c2_b_k <= c2_b_kend; c2_b_k++) {
                c2_c_k = c2_b_k;
                c2_e_x = c2_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                  _SFD_INTEGER_CHECK("", (real_T)c2_c_k), 1, 20, 1, 0) - 1];
                c2_f_x = c2_e_x;
                c2_absxk = muDoubleScalarAbs(c2_f_x);
                if (c2_absxk > c2_scale) {
                  c2_t = c2_scale / c2_absxk;
                  c2_d_y = 1.0 + c2_d_y * c2_t * c2_t;
                  c2_scale = c2_absxk;
                } else {
                  c2_t = c2_absxk / c2_scale;
                  c2_d_y += c2_t * c2_t;
                }
              }

              c2_d_y = c2_scale * muDoubleScalarSqrt(c2_d_y);
            }

            c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_d_y;
            c2_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_b_j), 1, 4, 1, 0) - 1] =
              c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
              ("", (real_T)c2_b_j), 1, 4, 1, 0) - 1];
          } else {
            c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_b_j), 1, 4, 1, 0) - 1] = 0.0;
            c2_vn2[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c2_b_j), 1, 4, 1, 0) - 1] = 0.0;
          }
        } else {
          c2_g_x = c2_temp1;
          c2_h_x = c2_g_x;
          if (c2_h_x < 0.0) {
            c2_eml_error(chartInstance);
          }

          c2_h_x = muDoubleScalarSqrt(c2_h_x);
          c2_v_a = c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c2_b_j), 1, 4, 1, 0) - 1];
          c2_t_b = c2_h_x;
          c2_e_y = c2_v_a * c2_t_b;
          c2_vn1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_j), 1, 4, 1, 0) - 1] = c2_e_y;
        }
      }
    }
  }

  c2_rankR = 0.0;
  c2_eps(chartInstance);
  c2_i_x = c2_b_A[0];
  c2_j_x = c2_i_x;
  c2_k_x = c2_j_x;
  c2_f_y = muDoubleScalarAbs(c2_k_x);
  c2_l_x = 0.0;
  c2_m_x = c2_l_x;
  c2_g_y = muDoubleScalarAbs(c2_m_x);
  c2_d = c2_f_y + c2_g_y;
  c2_u_b = c2_d;
  c2_h_y = 5.0 * c2_u_b;
  c2_w_a = c2_h_y;
  c2_tol = c2_w_a * 2.2204460492503131E-16;
  c2_d_k = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_d_k < 4)) {
    c2_e_k = 1.0 + (real_T)c2_d_k;
    c2_n_x = c2_b_A[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c2_e_k), 1, 5, 1, 0) + 5 * ((int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_e_k), 1,
      4, 2, 0) - 1)) - 1];
    c2_o_x = c2_n_x;
    c2_p_x = c2_o_x;
    c2_i_y = muDoubleScalarAbs(c2_p_x);
    c2_q_x = 0.0;
    c2_r_x = c2_q_x;
    c2_j_y = muDoubleScalarAbs(c2_r_x);
    c2_b_d = c2_i_y + c2_j_y;
    if (c2_b_d <= c2_tol) {
      c2_s_x = c2_tol;
      for (c2_i108 = 0; c2_i108 < 8; c2_i108++) {
        c2_u[c2_i108] = c2_cv2[c2_i108];
      }

      c2_k_y = NULL;
      sf_mex_assign(&c2_k_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 8),
                    FALSE);
      c2_b_u = 14.0;
      c2_l_y = NULL;
      sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_c_u = 6.0;
      c2_m_y = NULL;
      sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_d_u = c2_s_x;
      c2_n_y = NULL;
      sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0),
                    FALSE);
      c2_j_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U,
        14, sf_mex_call_debug("sprintf", 1U, 3U, 14, c2_k_y, 14, c2_l_y, 14,
        c2_m_y), 14, c2_n_y), "sprintf", c2_str);
      for (c2_i109 = 0; c2_i109 < 14; c2_i109++) {
        c2_b_str[c2_i109] = c2_str[c2_i109];
      }

      c2_eml_warning(chartInstance, c2_rankR, c2_b_str);
      exitg1 = TRUE;
    } else {
      c2_rankR++;
      c2_d_k++;
    }
  }

  for (c2_i110 = 0; c2_i110 < 4; c2_i110++) {
    c2_Y[c2_i110] = 0.0;
  }

  for (c2_d_j = 0; c2_d_j < 4; c2_d_j++) {
    c2_e_j = 1.0 + (real_T)c2_d_j;
    c2_tauj = c2_tau[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c2_e_j), 1, 4, 1, 0) - 1];
    if (c2_tauj != 0.0) {
      c2_wj = c2_b_B[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c2_e_j), 1, 5, 1, 0) - 1];
      c2_d8 = c2_e_j + 1.0;
      c2_i111 = (int32_T)(5.0 + (1.0 - c2_d8));
      _SFD_FOR_LOOP_VECTOR_CHECK(c2_d8, 1.0, 5.0, mxDOUBLE_CLASS, c2_i111);
      for (c2_c_i = 0; c2_c_i < c2_i111; c2_c_i++) {
        c2_d_i = c2_d8 + (real_T)c2_c_i;
        c2_x_a = c2_b_A[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", c2_d_i), 1, 5, 1, 0) + 5 * ((int32_T)
          (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          c2_e_j), 1, 4, 2, 0) - 1)) - 1];
        c2_v_b = c2_b_B[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", c2_d_i), 1, 5, 1, 0) - 1];
        c2_z = c2_x_a * c2_v_b;
        c2_wj += c2_z;
      }

      c2_y_a = c2_tauj;
      c2_w_b = c2_wj;
      c2_wj = c2_y_a * c2_w_b;
      if (c2_wj != 0.0) {
        c2_b_B[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", c2_e_j), 1, 5, 1, 0) - 1] = c2_b_B[(int32_T)
          (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          c2_e_j), 1, 5, 1, 0) - 1] - c2_wj;
        c2_d9 = c2_e_j + 1.0;
        c2_i112 = (int32_T)(5.0 + (1.0 - c2_d9));
        _SFD_FOR_LOOP_VECTOR_CHECK(c2_d9, 1.0, 5.0, mxDOUBLE_CLASS, c2_i112);
        for (c2_e_i = 0; c2_e_i < c2_i112; c2_e_i++) {
          c2_d_i = c2_d9 + (real_T)c2_e_i;
          c2_ab_a = c2_b_A[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", c2_d_i), 1, 5, 1, 0) + 5 * ((int32_T)
                             (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c2_e_j), 1, 4, 2, 0) - 1)) - 1];
          c2_x_b = c2_wj;
          c2_o_y = c2_ab_a * c2_x_b;
          c2_b_B[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c2_d_i), 1, 5, 1, 0) - 1] = c2_b_B[(int32_T)
            (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", c2_d_i), 1, 5, 1, 0) - 1] - c2_o_y;
        }
      }
    }
  }

  c2_rr = c2_rankR;
  c2_b_rr = c2_rr;
  c2_i113 = (int32_T)c2_b_rr;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_rr, mxDOUBLE_CLASS, c2_i113);
  for (c2_f_i = 0; c2_f_i < c2_i113; c2_f_i++) {
    c2_d_i = 1.0 + (real_T)c2_f_i;
    c2_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_jpvt[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c2_d_i), 1, 4, 1, 0) - 1]), 1, 4, 1, 0) - 1] =
      c2_b_B[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c2_d_i), 1, 5, 1, 0) - 1];
  }

  c2_c_rr = c2_rr;
  c2_i114 = (int32_T)-(1.0 + (-1.0 - c2_c_rr));
  _SFD_FOR_LOOP_VECTOR_CHECK(c2_c_rr, -1.0, 1.0, mxDOUBLE_CLASS, c2_i114);
  for (c2_f_j = 0; c2_f_j < c2_i114; c2_f_j++) {
    c2_e_j = c2_c_rr + -(real_T)c2_f_j;
    c2_pj = c2_jpvt[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c2_e_j), 1, 4, 1, 0) - 1];
    c2_t_x = c2_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_pj), 1, 4, 1, 0) - 1];
    c2_p_y = c2_b_A[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", c2_e_j), 1, 5, 1, 0) + 5 * ((int32_T)(real_T)
      _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_e_j), 1,
      4, 2, 0) - 1)) - 1];
    c2_b_z = c2_t_x / c2_p_y;
    c2_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_pj), 1, 4, 1, 0) - 1] = c2_b_z;
    c2_d10 = c2_e_j - 1.0;
    c2_i115 = (int32_T)c2_d10;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d10, mxDOUBLE_CLASS, c2_i115);
    for (c2_g_i = 0; c2_g_i < c2_i115; c2_g_i++) {
      c2_d_i = 1.0 + (real_T)c2_g_i;
      c2_bb_a = c2_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c2_pj), 1, 4, 1, 0) - 1];
      c2_y_b = c2_b_A[((int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c2_d_i), 1, 5, 1, 0) + 5 * ((int32_T)(real_T)
        _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c2_e_j),
        1, 4, 2, 0) - 1)) - 1];
      c2_q_y = c2_bb_a * c2_y_b;
      c2_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_jpvt[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", c2_d_i), 1, 4, 1, 0) - 1]), 1, 4, 1, 0)
        - 1] = c2_Y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jpvt[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", c2_d_i), 1, 4, 1, 0) - 1]), 1, 4, 1, 0)
        - 1] - c2_q_y;
    }
  }
}

static void c2_d_eml_scalar_eg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static void c2_eps(SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static real_T c2_eml_xnrm2(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_x[20], int32_T c2_ix0)
{
  real_T c2_y;
  int32_T c2_b_ix0;
  int32_T c2_c_ix0;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_a;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_b_a;
  int32_T c2_b;
  int32_T c2_c_a;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_b_ix0 = c2_ix0;
  c2_c_ix0 = c2_b_ix0;
  c2_y = 0.0;
  c2_realmin(chartInstance);
  c2_scale = 2.2250738585072014E-308;
  c2_kstart = c2_c_ix0;
  c2_a = c2_kstart;
  c2_kend = c2_a;
  c2_b_kstart = c2_kstart;
  c2_b_kend = c2_kend + 4;
  c2_b_a = c2_b_kstart;
  c2_b = c2_b_kend;
  c2_c_a = c2_b_a;
  c2_b_b = c2_b;
  if (c2_c_a > c2_b_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_b_b > 2147483646);
  }

  if (c2_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_overflow);
  }

  for (c2_k = c2_b_kstart; c2_k <= c2_b_kend; c2_k++) {
    c2_b_k = c2_k;
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 20, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_absxk = muDoubleScalarAbs(c2_c_x);
    if (c2_absxk > c2_scale) {
      c2_t = c2_scale / c2_absxk;
      c2_y = 1.0 + c2_y * c2_t * c2_t;
      c2_scale = c2_absxk;
    } else {
      c2_t = c2_absxk / c2_scale;
      c2_y += c2_t * c2_t;
    }
  }

  return c2_scale * muDoubleScalarSqrt(c2_y);
}

static void c2_realmin(SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void c2_check_forloop_overflow_error
  (SFc2_quad_control_sim_qInstanceStruct *chartInstance, boolean_T c2_overflow)
{
  int32_T c2_i116;
  static char_T c2_cv3[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c2_u[34];
  const mxArray *c2_y = NULL;
  int32_T c2_i117;
  static char_T c2_cv4[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c2_b_u[23];
  const mxArray *c2_b_y = NULL;
  if (!c2_overflow) {
  } else {
    for (c2_i116 = 0; c2_i116 < 34; c2_i116++) {
      c2_u[c2_i116] = c2_cv3[c2_i116];
    }

    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c2_i117 = 0; c2_i117 < 23; c2_i117++) {
      c2_b_u[c2_i117] = c2_cv4[c2_i117];
    }

    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c2_y, 14, c2_b_y));
  }
}

static int32_T c2_eml_ixamax(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T c2_x[4], int32_T c2_ix0)
{
  int32_T c2_idxmax;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  int32_T c2_ix;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_y;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_y;
  real_T c2_smax;
  int32_T c2_d_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_a;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_c_y;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_d_y;
  real_T c2_s;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  if (c2_c_n < 1) {
    c2_idxmax = 0;
  } else {
    c2_idxmax = 1;
    if (c2_c_n > 1) {
      c2_ix = c2_c_ix0;
      c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_ix), 1, 4, 1, 0) - 1];
      c2_c_x = c2_b_x;
      c2_d_x = c2_c_x;
      c2_y = muDoubleScalarAbs(c2_d_x);
      c2_e_x = 0.0;
      c2_f_x = c2_e_x;
      c2_b_y = muDoubleScalarAbs(c2_f_x);
      c2_smax = c2_y + c2_b_y;
      c2_d_n = c2_c_n;
      c2_b = c2_d_n;
      c2_b_b = c2_b;
      if (2 > c2_b_b) {
        c2_overflow = FALSE;
      } else {
        c2_overflow = (c2_b_b > 2147483646);
      }

      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_k = 2; c2_k <= c2_d_n; c2_k++) {
        c2_b_k = c2_k;
        c2_a = c2_ix + 1;
        c2_ix = c2_a;
        c2_g_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 4, 1, 0) - 1];
        c2_h_x = c2_g_x;
        c2_i_x = c2_h_x;
        c2_c_y = muDoubleScalarAbs(c2_i_x);
        c2_j_x = 0.0;
        c2_k_x = c2_j_x;
        c2_d_y = muDoubleScalarAbs(c2_k_x);
        c2_s = c2_c_y + c2_d_y;
        if (c2_s > c2_smax) {
          c2_idxmax = c2_b_k;
          c2_smax = c2_s;
        }
      }
    }
  }

  return c2_idxmax;
}

static void c2_eml_xswap(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_x[20], int32_T c2_ix0, int32_T c2_iy0, real_T c2_b_x[20])
{
  int32_T c2_i118;
  for (c2_i118 = 0; c2_i118 < 20; c2_i118++) {
    c2_b_x[c2_i118] = c2_x[c2_i118];
  }

  c2_b_eml_xswap(chartInstance, c2_b_x, c2_ix0, c2_iy0);
}

static void c2_below_threshold(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static void c2_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T c2_alpha1, real_T c2_x[20], int32_T
  c2_ix0, real_T *c2_b_alpha1, real_T c2_b_x[20], real_T *c2_tau)
{
  int32_T c2_i119;
  *c2_b_alpha1 = c2_alpha1;
  for (c2_i119 = 0; c2_i119 < 20; c2_i119++) {
    c2_b_x[c2_i119] = c2_x[c2_i119];
  }

  *c2_tau = c2_c_eml_matlab_zlarfg(chartInstance, c2_n, c2_b_alpha1, c2_b_x,
    c2_ix0);
}

static real_T c2_b_eml_xnrm2(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T c2_x[20], int32_T c2_ix0)
{
  real_T c2_y;
  int32_T c2_b_n;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  int32_T c2_c_ix0;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_scale;
  int32_T c2_kstart;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b_a;
  int32_T c2_b_c;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_kend;
  int32_T c2_b_kstart;
  int32_T c2_b_kend;
  int32_T c2_d_a;
  int32_T c2_b_b;
  int32_T c2_e_a;
  int32_T c2_c_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_absxk;
  real_T c2_t;
  c2_b_n = c2_n;
  c2_b_ix0 = c2_ix0;
  c2_below_threshold(chartInstance);
  c2_c_n = c2_b_n;
  c2_c_ix0 = c2_b_ix0;
  c2_y = 0.0;
  if (c2_c_n < 1) {
  } else if (c2_c_n == 1) {
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_c_ix0), 1, 20, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_y = muDoubleScalarAbs(c2_c_x);
  } else {
    c2_realmin(chartInstance);
    c2_scale = 2.2250738585072014E-308;
    c2_kstart = c2_c_ix0;
    c2_a = c2_c_n;
    c2_c = c2_a;
    c2_b_a = c2_c - 1;
    c2_b_c = c2_b_a;
    c2_c_a = c2_kstart;
    c2_b = c2_b_c;
    c2_kend = c2_c_a + c2_b;
    c2_b_kstart = c2_kstart;
    c2_b_kend = c2_kend;
    c2_d_a = c2_b_kstart;
    c2_b_b = c2_b_kend;
    c2_e_a = c2_d_a;
    c2_c_b = c2_b_b;
    if (c2_e_a > c2_c_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_c_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_k = c2_b_kstart; c2_k <= c2_b_kend; c2_k++) {
      c2_b_k = c2_k;
      c2_d_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_b_k), 1, 20, 1, 0) - 1];
      c2_e_x = c2_d_x;
      c2_absxk = muDoubleScalarAbs(c2_e_x);
      if (c2_absxk > c2_scale) {
        c2_t = c2_scale / c2_absxk;
        c2_y = 1.0 + c2_y * c2_t * c2_t;
        c2_scale = c2_absxk;
      } else {
        c2_t = c2_absxk / c2_scale;
        c2_y += c2_t * c2_t;
      }
    }

    c2_y = c2_scale * muDoubleScalarSqrt(c2_y);
  }

  return c2_y;
}

static void c2_eml_xscal(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_a, real_T c2_x[20], int32_T c2_ix0, real_T c2_b_x[20])
{
  int32_T c2_i120;
  for (c2_i120 = 0; c2_i120 < 20; c2_i120++) {
    c2_b_x[c2_i120] = c2_x[c2_i120];
  }

  c2_c_eml_xscal(chartInstance, c2_n, c2_a, c2_b_x, c2_ix0);
}

static void c2_b_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, real_T c2_alpha1, real_T c2_x, real_T *c2_b_alpha1, real_T
  *c2_b_x, real_T *c2_tau)
{
  *c2_b_alpha1 = c2_alpha1;
  *c2_b_x = c2_x;
  *c2_tau = c2_d_eml_matlab_zlarfg(chartInstance, c2_b_alpha1, c2_b_x);
}

static void c2_c_eml_xnrm2(SFc2_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static real_T c2_b_eml_xscal(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_d_eml_xscal(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_eml_matlab_zlarf(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_iv0, real_T c2_tau,
  real_T c2_C[20], int32_T c2_ic0, real_T c2_work[4], real_T c2_b_C[20], real_T
  c2_b_work[4])
{
  int32_T c2_i121;
  int32_T c2_i122;
  for (c2_i121 = 0; c2_i121 < 20; c2_i121++) {
    c2_b_C[c2_i121] = c2_C[c2_i121];
  }

  for (c2_i122 = 0; c2_i122 < 4; c2_i122++) {
    c2_b_work[c2_i122] = c2_work[c2_i122];
  }

  c2_b_eml_matlab_zlarf(chartInstance, c2_m, c2_n, c2_iv0, c2_tau, c2_b_C,
                        c2_ic0, c2_b_work);
}

static void c2_eml_xgemv(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_A[20], int32_T c2_ia0, real_T c2_x[20],
  int32_T c2_ix0, real_T c2_y[4], real_T c2_b_y[4])
{
  int32_T c2_i123;
  int32_T c2_i124;
  real_T c2_b_A[20];
  int32_T c2_i125;
  real_T c2_b_x[20];
  for (c2_i123 = 0; c2_i123 < 4; c2_i123++) {
    c2_b_y[c2_i123] = c2_y[c2_i123];
  }

  for (c2_i124 = 0; c2_i124 < 20; c2_i124++) {
    c2_b_A[c2_i124] = c2_A[c2_i124];
  }

  for (c2_i125 = 0; c2_i125 < 20; c2_i125++) {
    c2_b_x[c2_i125] = c2_x[c2_i125];
  }

  c2_b_eml_xgemv(chartInstance, c2_m, c2_n, c2_b_A, c2_ia0, c2_b_x, c2_ix0,
                 c2_b_y);
}

static void c2_eml_xger(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, real_T c2_y[4],
  real_T c2_A[20], int32_T c2_ia0, real_T c2_b_A[20])
{
  int32_T c2_i126;
  int32_T c2_i127;
  real_T c2_b_y[4];
  for (c2_i126 = 0; c2_i126 < 20; c2_i126++) {
    c2_b_A[c2_i126] = c2_A[c2_i126];
  }

  for (c2_i127 = 0; c2_i127 < 4; c2_i127++) {
    c2_b_y[c2_i127] = c2_y[c2_i127];
  }

  c2_b_eml_xger(chartInstance, c2_m, c2_n, c2_alpha1, c2_ix0, c2_b_y, c2_b_A,
                c2_ia0);
}

static void c2_eml_warning(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_varargin_2, char_T c2_varargin_3[14])
{
  int32_T c2_i128;
  static char_T c2_varargin_1[32] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'r', 'a', 'n', 'k', 'D', 'e', 'f', 'i', 'c', 'i',
    'e', 'n', 't', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c2_u[32];
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  int32_T c2_i129;
  char_T c2_c_u[14];
  const mxArray *c2_c_y = NULL;
  for (c2_i128 = 0; c2_i128 < 32; c2_i128++) {
    c2_u[c2_i128] = c2_varargin_1[c2_i128];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 32), FALSE);
  c2_b_u = c2_varargin_2;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  for (c2_i129 = 0; c2_i129 < 14; c2_i129++) {
    c2_c_u[c2_i129] = c2_varargin_3[c2_i129];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_c_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 3U,
    14, c2_y, 14, c2_b_y, 14, c2_c_y));
}

static void c2_j_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_sprintf, const char_T *c2_identifier, char_T
  c2_y[14])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_sprintf), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_sprintf);
}

static void c2_k_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  char_T c2_y[14])
{
  char_T c2_cv5[14];
  int32_T c2_i130;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_cv5, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c2_i130 = 0; c2_i130 < 14; c2_i130++) {
    c2_y[c2_i130] = c2_cv5[c2_i130];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_l_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i131;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i131, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i131;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_m_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_quad_control_sim_q, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_quad_control_sim_q), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_quad_control_sim_q);
  return c2_y;
}

static uint8_T c2_n_emlrt_marshallIn(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sqrt(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T *c2_x)
{
  if (*c2_x < 0.0) {
    c2_eml_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void c2_b_eml_xswap(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T c2_x[20], int32_T c2_ix0, int32_T c2_iy0)
{
  int32_T c2_b_ix0;
  int32_T c2_b_iy0;
  int32_T c2_c_ix0;
  int32_T c2_c_iy0;
  int32_T c2_ix;
  int32_T c2_iy;
  int32_T c2_k;
  real_T c2_temp;
  int32_T c2_a;
  int32_T c2_b_a;
  c2_b_ix0 = c2_ix0;
  c2_b_iy0 = c2_iy0;
  c2_c_ix0 = c2_b_ix0;
  c2_c_iy0 = c2_b_iy0;
  c2_ix = c2_c_ix0;
  c2_iy = c2_c_iy0;
  for (c2_k = 1; c2_k < 6; c2_k++) {
    c2_temp = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c2_ix), 1, 20, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_ix), 1, 20, 1, 0) - 1] = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iy), 1, 20, 1, 0) - 1];
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_iy), 1, 20, 1, 0) - 1] = c2_temp;
    c2_a = c2_ix + 1;
    c2_ix = c2_a;
    c2_b_a = c2_iy + 1;
    c2_iy = c2_b_a;
  }
}

static real_T c2_c_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_n, real_T *c2_alpha1, real_T c2_x[20], int32_T
  c2_ix0)
{
  real_T c2_tau;
  int32_T c2_nm1;
  int32_T c2_i132;
  int32_T c2_i133;
  int32_T c2_i134;
  real_T c2_b_x[20];
  real_T c2_xnorm;
  real_T c2_x1;
  real_T c2_x2;
  real_T c2_a;
  real_T c2_b;
  real_T c2_beta1;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_y;
  int32_T c2_knt;
  int32_T c2_b_a;
  real_T c2_d11;
  real_T c2_c_a;
  real_T c2_d_a;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_b_y;
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_i137;
  real_T c2_g_x[20];
  real_T c2_b_x1;
  real_T c2_b_x2;
  real_T c2_e_a;
  real_T c2_b_b;
  real_T c2_h_x;
  real_T c2_c_y;
  real_T c2_d_y;
  int32_T c2_b_knt;
  int32_T c2_c_b;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  real_T c2_f_a;
  real_T c2_i_x;
  real_T c2_e_y;
  real_T c2_f_y;
  c2_tau = 0.0;
  if (c2_n <= 0) {
  } else {
    c2_nm1 = c2_n - 1;
    c2_i132 = 0;
    for (c2_i133 = 0; c2_i133 < 4; c2_i133++) {
      for (c2_i134 = 0; c2_i134 < 5; c2_i134++) {
        c2_b_x[c2_i134 + c2_i132] = c2_x[c2_i134 + c2_i132];
      }

      c2_i132 += 5;
    }

    c2_xnorm = c2_b_eml_xnrm2(chartInstance, c2_nm1, c2_b_x, c2_ix0);
    if (c2_xnorm != 0.0) {
      c2_x1 = *c2_alpha1;
      c2_x2 = c2_xnorm;
      c2_a = c2_x1;
      c2_b = c2_x2;
      c2_beta1 = muDoubleScalarHypot(c2_a, c2_b);
      if (*c2_alpha1 >= 0.0) {
        c2_beta1 = -c2_beta1;
      }

      c2_realmin(chartInstance);
      c2_eps(chartInstance);
      c2_c_x = c2_beta1;
      c2_d_x = c2_c_x;
      c2_y = muDoubleScalarAbs(c2_d_x);
      if (c2_y < 1.0020841800044864E-292) {
        c2_knt = 0;
        do {
          c2_b_a = c2_knt + 1;
          c2_knt = c2_b_a;
          c2_d11 = 9.9792015476736E+291;
          c2_c_eml_xscal(chartInstance, c2_nm1, c2_d11, c2_x, c2_ix0);
          c2_c_a = c2_beta1;
          c2_beta1 = c2_c_a * 9.9792015476736E+291;
          c2_d_a = *c2_alpha1;
          *c2_alpha1 = c2_d_a * 9.9792015476736E+291;
          c2_e_x = c2_beta1;
          c2_f_x = c2_e_x;
          c2_b_y = muDoubleScalarAbs(c2_f_x);
        } while (!(c2_b_y >= 1.0020841800044864E-292));

        c2_i135 = 0;
        for (c2_i136 = 0; c2_i136 < 4; c2_i136++) {
          for (c2_i137 = 0; c2_i137 < 5; c2_i137++) {
            c2_g_x[c2_i137 + c2_i135] = c2_x[c2_i137 + c2_i135];
          }

          c2_i135 += 5;
        }

        c2_xnorm = c2_b_eml_xnrm2(chartInstance, c2_nm1, c2_g_x, c2_ix0);
        c2_b_x1 = *c2_alpha1;
        c2_b_x2 = c2_xnorm;
        c2_e_a = c2_b_x1;
        c2_b_b = c2_b_x2;
        c2_beta1 = muDoubleScalarHypot(c2_e_a, c2_b_b);
        if (*c2_alpha1 >= 0.0) {
          c2_beta1 = -c2_beta1;
        }

        c2_h_x = c2_beta1 - *c2_alpha1;
        c2_c_y = c2_beta1;
        c2_tau = c2_h_x / c2_c_y;
        c2_d_y = *c2_alpha1 - c2_beta1;
        *c2_alpha1 = 1.0 / c2_d_y;
        c2_c_eml_xscal(chartInstance, c2_nm1, *c2_alpha1, c2_x, c2_ix0);
        c2_b_knt = c2_knt;
        c2_c_b = c2_b_knt;
        c2_d_b = c2_c_b;
        if (1 > c2_d_b) {
          c2_overflow = FALSE;
        } else {
          c2_overflow = (c2_d_b > 2147483646);
        }

        if (c2_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_overflow);
        }

        for (c2_k = 1; c2_k <= c2_b_knt; c2_k++) {
          c2_f_a = c2_beta1;
          c2_beta1 = c2_f_a * 1.0020841800044864E-292;
        }

        *c2_alpha1 = c2_beta1;
      } else {
        c2_i_x = c2_beta1 - *c2_alpha1;
        c2_e_y = c2_beta1;
        c2_tau = c2_i_x / c2_e_y;
        c2_f_y = *c2_alpha1 - c2_beta1;
        *c2_alpha1 = 1.0 / c2_f_y;
        c2_c_eml_xscal(chartInstance, c2_nm1, *c2_alpha1, c2_x, c2_ix0);
        *c2_alpha1 = c2_beta1;
      }
    }
  }

  return c2_tau;
}

static void c2_c_eml_xscal(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_n, real_T c2_a, real_T c2_x[20], int32_T c2_ix0)
{
  int32_T c2_b_n;
  real_T c2_b_a;
  int32_T c2_b_ix0;
  int32_T c2_c_n;
  real_T c2_c_a;
  int32_T c2_c_ix0;
  int32_T c2_d_ix0;
  int32_T c2_d_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_e_a;
  int32_T c2_b_b;
  int32_T c2_i138;
  int32_T c2_f_a;
  int32_T c2_c_b;
  int32_T c2_g_a;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_b_n = c2_n;
  c2_b_a = c2_a;
  c2_b_ix0 = c2_ix0;
  c2_c_n = c2_b_n;
  c2_c_a = c2_b_a;
  c2_c_ix0 = c2_b_ix0;
  c2_d_ix0 = c2_c_ix0;
  c2_d_a = c2_c_n;
  c2_c = c2_d_a;
  c2_b = c2_c - 1;
  c2_b_c = c2_b;
  c2_e_a = c2_c_ix0;
  c2_b_b = c2_b_c;
  c2_i138 = c2_e_a + c2_b_b;
  c2_f_a = c2_d_ix0;
  c2_c_b = c2_i138;
  c2_g_a = c2_f_a;
  c2_d_b = c2_c_b;
  if (c2_g_a > c2_d_b) {
    c2_overflow = FALSE;
  } else {
    c2_overflow = (c2_d_b > 2147483646);
  }

  if (c2_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_overflow);
  }

  for (c2_k = c2_d_ix0; c2_k <= c2_i138; c2_k++) {
    c2_b_k = c2_k;
    c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c2_b_k), 1, 20, 1, 0) - 1] = c2_c_a * c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 20, 1, 0) - 1];
  }
}

static real_T c2_d_eml_matlab_zlarfg(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, real_T *c2_alpha1, real_T *c2_x)
{
  real_T c2_tau;
  c2_tau = 0.0;
  c2_c_eml_xnrm2(chartInstance);
  return c2_tau;
}

static void c2_d_eml_xscal(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  real_T *c2_x)
{
}

static void c2_b_eml_matlab_zlarf(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance, int32_T c2_m, int32_T c2_n, int32_T c2_iv0, real_T c2_tau,
  real_T c2_C[20], int32_T c2_ic0, real_T c2_work[4])
{
  int32_T c2_lastv;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_b;
  int32_T c2_b_c;
  int32_T c2_b_a;
  int32_T c2_b_b;
  int32_T c2_i;
  int32_T c2_c_a;
  int32_T c2_d_a;
  int32_T c2_b_m;
  int32_T c2_b_n;
  int32_T c2_ia0;
  int32_T c2_lastc;
  int32_T c2_e_a;
  int32_T c2_c_c;
  int32_T c2_f_a;
  int32_T c2_d_c;
  int32_T c2_g_a;
  int32_T c2_c_b;
  int32_T c2_coltop;
  int32_T c2_h_a;
  int32_T c2_e_c;
  int32_T c2_i_a;
  int32_T c2_d_b;
  int32_T c2_colbottom;
  int32_T c2_b_coltop;
  int32_T c2_b_colbottom;
  int32_T c2_j_a;
  int32_T c2_e_b;
  int32_T c2_k_a;
  int32_T c2_f_b;
  boolean_T c2_overflow;
  int32_T c2_ia;
  int32_T c2_b_ia;
  int32_T c2_l_a;
  int32_T c2_i139;
  int32_T c2_i140;
  int32_T c2_i141;
  real_T c2_b_C[20];
  int32_T c2_i142;
  int32_T c2_i143;
  int32_T c2_i144;
  real_T c2_c_C[20];
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_alpha1;
  int32_T c2_ix0;
  int32_T c2_b_ia0;
  int32_T c2_i145;
  real_T c2_b_work[4];
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  if (c2_tau != 0.0) {
    c2_lastv = c2_m;
    c2_a = c2_lastv;
    c2_c = c2_a;
    c2_b = c2_c - 1;
    c2_b_c = c2_b;
    c2_b_a = c2_iv0;
    c2_b_b = c2_b_c;
    c2_i = c2_b_a + c2_b_b;
    exitg3 = FALSE;
    while ((exitg3 == FALSE) && (c2_lastv > 0)) {
      if (c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_i), 1, 20, 1, 0) - 1] == 0.0) {
        c2_c_a = c2_lastv - 1;
        c2_lastv = c2_c_a;
        c2_d_a = c2_i - 1;
        c2_i = c2_d_a;
      } else {
        exitg3 = TRUE;
      }
    }

    c2_b_m = c2_lastv;
    c2_b_n = c2_n;
    c2_ia0 = c2_ic0;
    c2_lastc = c2_b_n;
    exitg2 = FALSE;
    while ((exitg2 == FALSE) && (c2_lastc > 0)) {
      c2_e_a = c2_lastc;
      c2_c_c = c2_e_a;
      c2_f_a = c2_c_c - 1;
      c2_d_c = c2_f_a * 5;
      c2_g_a = c2_ia0;
      c2_c_b = c2_d_c;
      c2_coltop = c2_g_a + c2_c_b;
      c2_h_a = c2_b_m;
      c2_e_c = c2_h_a;
      c2_i_a = c2_coltop;
      c2_d_b = c2_e_c - 1;
      c2_colbottom = c2_i_a + c2_d_b;
      c2_b_coltop = c2_coltop;
      c2_b_colbottom = c2_colbottom;
      c2_j_a = c2_b_coltop;
      c2_e_b = c2_b_colbottom;
      c2_k_a = c2_j_a;
      c2_f_b = c2_e_b;
      if (c2_k_a > c2_f_b) {
        c2_overflow = FALSE;
      } else {
        c2_overflow = (c2_f_b > 2147483646);
      }

      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      c2_ia = c2_b_coltop;
      do {
        exitg1 = 0;
        if (c2_ia <= c2_b_colbottom) {
          c2_b_ia = c2_ia;
          if (c2_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c2_b_ia), 1, 20, 1, 0) - 1] != 0.0) {
            exitg1 = 1;
          } else {
            c2_ia++;
          }
        } else {
          c2_l_a = c2_lastc - 1;
          c2_lastc = c2_l_a;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = TRUE;
      }
    }
  } else {
    c2_lastv = 0;
    c2_lastc = 0;
  }

  if (c2_lastv > 0) {
    c2_i139 = 0;
    for (c2_i140 = 0; c2_i140 < 4; c2_i140++) {
      for (c2_i141 = 0; c2_i141 < 5; c2_i141++) {
        c2_b_C[c2_i141 + c2_i139] = c2_C[c2_i141 + c2_i139];
      }

      c2_i139 += 5;
    }

    c2_i142 = 0;
    for (c2_i143 = 0; c2_i143 < 4; c2_i143++) {
      for (c2_i144 = 0; c2_i144 < 5; c2_i144++) {
        c2_c_C[c2_i144 + c2_i142] = c2_C[c2_i144 + c2_i142];
      }

      c2_i142 += 5;
    }

    c2_b_eml_xgemv(chartInstance, c2_lastv, c2_lastc, c2_b_C, c2_ic0, c2_c_C,
                   c2_iv0, c2_work);
    c2_c_m = c2_lastv;
    c2_c_n = c2_lastc;
    c2_alpha1 = -c2_tau;
    c2_ix0 = c2_iv0;
    c2_b_ia0 = c2_ic0;
    for (c2_i145 = 0; c2_i145 < 4; c2_i145++) {
      c2_b_work[c2_i145] = c2_work[c2_i145];
    }

    c2_b_eml_xger(chartInstance, c2_c_m, c2_c_n, c2_alpha1, c2_ix0, c2_b_work,
                  c2_C, c2_b_ia0);
  }
}

static void c2_b_eml_xgemv(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_A[20], int32_T c2_ia0, real_T c2_x[20],
  int32_T c2_ix0, real_T c2_y[4])
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  int32_T c2_b_ia0;
  int32_T c2_b_ix0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  int32_T c2_c_ia0;
  int32_T c2_c_ix0;
  int32_T c2_a;
  int32_T c2_mm1;
  int32_T c2_b_a;
  int32_T c2_nm1;
  int32_T c2_b;
  int32_T c2_c;
  int32_T c2_b_b;
  int32_T c2_iyend;
  int32_T c2_b_iyend;
  int32_T c2_c_b;
  int32_T c2_d_b;
  boolean_T c2_overflow;
  int32_T c2_iy;
  int32_T c2_b_iy;
  int32_T c2_d_ia0;
  int32_T c2_e_b;
  int32_T c2_b_c;
  int32_T c2_c_a;
  int32_T c2_f_b;
  int32_T c2_i146;
  int32_T c2_d_a;
  int32_T c2_g_b;
  int32_T c2_e_a;
  int32_T c2_h_b;
  boolean_T c2_b_overflow;
  int32_T c2_iac;
  int32_T c2_b_iac;
  int32_T c2_ix;
  real_T c2_c_c;
  int32_T c2_c_iac;
  int32_T c2_f_a;
  int32_T c2_i_b;
  int32_T c2_i147;
  int32_T c2_g_a;
  int32_T c2_j_b;
  int32_T c2_h_a;
  int32_T c2_k_b;
  boolean_T c2_c_overflow;
  int32_T c2_ia;
  int32_T c2_b_ia;
  real_T c2_i_a;
  real_T c2_l_b;
  real_T c2_z;
  int32_T c2_j_a;
  int32_T c2_k_a;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_ia0 = c2_ia0;
  c2_b_ix0 = c2_ix0;
  c2_c_m = c2_b_m;
  c2_c_n = c2_b_n;
  c2_c_ia0 = c2_b_ia0;
  c2_c_ix0 = c2_b_ix0;
  if (c2_c_m == 0) {
  } else if (c2_c_n == 0) {
  } else {
    c2_a = c2_c_m;
    c2_mm1 = c2_a;
    c2_b_a = c2_c_n - 1;
    c2_nm1 = c2_b_a;
    c2_b = c2_nm1;
    c2_c = c2_b;
    c2_b_b = c2_c;
    c2_iyend = c2_b_b;
    c2_b_iyend = c2_iyend + 1;
    c2_c_b = c2_b_iyend;
    c2_d_b = c2_c_b;
    if (1 > c2_d_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_d_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_iy = 1; c2_iy <= c2_b_iyend; c2_iy++) {
      c2_b_iy = c2_iy;
      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_iy), 1, 4, 1, 0) - 1] = 0.0;
    }

    c2_b_iy = 1;
    c2_d_ia0 = c2_c_ia0;
    c2_e_b = c2_nm1;
    c2_b_c = 5 * c2_e_b;
    c2_c_a = c2_c_ia0;
    c2_f_b = c2_b_c;
    c2_i146 = c2_c_a + c2_f_b;
    c2_d_a = c2_d_ia0;
    c2_g_b = c2_i146;
    c2_e_a = c2_d_a;
    c2_h_b = c2_g_b;
    if (c2_e_a > c2_h_b) {
      c2_b_overflow = FALSE;
    } else {
      c2_b_overflow = (c2_h_b > 2147483642);
    }

    if (c2_b_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
    }

    for (c2_iac = c2_d_ia0; c2_iac <= c2_i146; c2_iac += 5) {
      c2_b_iac = c2_iac;
      c2_ix = c2_c_ix0;
      c2_c_c = 0.0;
      c2_c_iac = c2_b_iac;
      c2_f_a = c2_b_iac;
      c2_i_b = c2_mm1 - 1;
      c2_i147 = c2_f_a + c2_i_b;
      c2_g_a = c2_c_iac;
      c2_j_b = c2_i147;
      c2_h_a = c2_g_a;
      c2_k_b = c2_j_b;
      if (c2_h_a > c2_k_b) {
        c2_c_overflow = FALSE;
      } else {
        c2_c_overflow = (c2_k_b > 2147483646);
      }

      if (c2_c_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
      }

      for (c2_ia = c2_c_iac; c2_ia <= c2_i147; c2_ia++) {
        c2_b_ia = c2_ia;
        c2_i_a = c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_ia), 1, 20, 1, 0) - 1];
        c2_l_b = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_ix), 1, 20, 1, 0) - 1];
        c2_z = c2_i_a * c2_l_b;
        c2_c_c += c2_z;
        c2_j_a = c2_ix + 1;
        c2_ix = c2_j_a;
      }

      c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_iy), 1, 4, 1, 0) - 1] = c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_b_iy), 1, 4, 1, 0) - 1] +
        c2_c_c;
      c2_k_a = c2_b_iy + 1;
      c2_b_iy = c2_k_a;
    }
  }
}

static void c2_b_eml_xger(SFc2_quad_control_sim_qInstanceStruct *chartInstance,
  int32_T c2_m, int32_T c2_n, real_T c2_alpha1, int32_T c2_ix0, real_T c2_y[4],
  real_T c2_A[20], int32_T c2_ia0)
{
  int32_T c2_b_m;
  int32_T c2_b_n;
  real_T c2_b_alpha1;
  int32_T c2_b_ix0;
  int32_T c2_b_ia0;
  int32_T c2_c_m;
  int32_T c2_c_n;
  real_T c2_c_alpha1;
  int32_T c2_c_ix0;
  int32_T c2_c_ia0;
  int32_T c2_d_m;
  int32_T c2_d_n;
  real_T c2_d_alpha1;
  int32_T c2_d_ix0;
  int32_T c2_d_ia0;
  int32_T c2_ixstart;
  int32_T c2_a;
  int32_T c2_jA;
  int32_T c2_jy;
  int32_T c2_e_n;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_j;
  real_T c2_yjy;
  real_T c2_temp;
  int32_T c2_ix;
  int32_T c2_c_b;
  int32_T c2_i148;
  int32_T c2_b_a;
  int32_T c2_d_b;
  int32_T c2_i149;
  int32_T c2_c_a;
  int32_T c2_e_b;
  int32_T c2_d_a;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_ijA;
  int32_T c2_b_ijA;
  int32_T c2_e_a;
  int32_T c2_f_a;
  int32_T c2_g_a;
  c2_b_m = c2_m;
  c2_b_n = c2_n;
  c2_b_alpha1 = c2_alpha1;
  c2_b_ix0 = c2_ix0;
  c2_b_ia0 = c2_ia0;
  c2_c_m = c2_b_m;
  c2_c_n = c2_b_n;
  c2_c_alpha1 = c2_b_alpha1;
  c2_c_ix0 = c2_b_ix0;
  c2_c_ia0 = c2_b_ia0;
  c2_d_m = c2_c_m;
  c2_d_n = c2_c_n;
  c2_d_alpha1 = c2_c_alpha1;
  c2_d_ix0 = c2_c_ix0;
  c2_d_ia0 = c2_c_ia0;
  if (c2_d_alpha1 == 0.0) {
  } else {
    c2_ixstart = c2_d_ix0;
    c2_a = c2_d_ia0 - 1;
    c2_jA = c2_a;
    c2_jy = 1;
    c2_e_n = c2_d_n;
    c2_b = c2_e_n;
    c2_b_b = c2_b;
    if (1 > c2_b_b) {
      c2_overflow = FALSE;
    } else {
      c2_overflow = (c2_b_b > 2147483646);
    }

    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_e_n; c2_j++) {
      c2_yjy = c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c2_jy), 1, 4, 1, 0) - 1];
      if (c2_yjy != 0.0) {
        c2_temp = c2_yjy * c2_d_alpha1;
        c2_ix = c2_ixstart;
        c2_c_b = c2_jA + 1;
        c2_i148 = c2_c_b;
        c2_b_a = c2_d_m;
        c2_d_b = c2_jA;
        c2_i149 = c2_b_a + c2_d_b;
        c2_c_a = c2_i148;
        c2_e_b = c2_i149;
        c2_d_a = c2_c_a;
        c2_f_b = c2_e_b;
        if (c2_d_a > c2_f_b) {
          c2_b_overflow = FALSE;
        } else {
          c2_b_overflow = (c2_f_b > 2147483646);
        }

        if (c2_b_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
        }

        for (c2_ijA = c2_i148; c2_ijA <= c2_i149; c2_ijA++) {
          c2_b_ijA = c2_ijA;
          c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 20, 1, 0) - 1] =
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_ijA), 1, 20, 1, 0) - 1] +
            c2_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_ix), 1, 20, 1, 0) - 1] * c2_temp;
          c2_e_a = c2_ix + 1;
          c2_ix = c2_e_a;
        }
      }

      c2_f_a = c2_jy + 1;
      c2_jy = c2_f_a;
      c2_g_a = c2_jA + 5;
      c2_jA = c2_g_a;
    }
  }
}

static void init_dsm_address_info(SFc2_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_quad_control_sim_q_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2883089381U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(158583298U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3946022785U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1169483398U);
}

mxArray *sf_c2_quad_control_sim_q_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("SaxOABWOfz0HAxc02VCScG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(14);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(15);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,9,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(4);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_quad_control_sim_q_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_quad_control_sim_q(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[8],T\"q0_ddot\",},{M[1],M[9],T\"q1_ddot\",},{M[1],M[10],T\"q2_ddot\",},{M[1],M[11],T\"q3_ddot\",},{M[1],M[18],T\"quaternion_deltas\",},{M[1],M[14],T\"quaternion_multipliers\",},{M[1],M[5],T\"x_ddot\",},{M[1],M[6],T\"y_ddot\",},{M[1],M[7],T\"z_ddot\",},{M[8],M[0],T\"is_active_c2_quad_control_sim_q\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 10, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_quad_control_sim_q_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_quad_control_sim_qInstanceStruct *chartInstance;
    chartInstance = (SFc2_quad_control_sim_qInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _quad_control_sim_qMachineNumber_,
           2,
           1,
           1,
           12,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_quad_control_sim_qMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_quad_control_sim_qMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _quad_control_sim_qMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"states");
          _SFD_SET_DATA_PROPS(1,2,0,1,"x_ddot");
          _SFD_SET_DATA_PROPS(2,2,0,1,"y_ddot");
          _SFD_SET_DATA_PROPS(3,2,0,1,"z_ddot");
          _SFD_SET_DATA_PROPS(4,2,0,1,"q0_ddot");
          _SFD_SET_DATA_PROPS(5,2,0,1,"q1_ddot");
          _SFD_SET_DATA_PROPS(6,2,0,1,"q2_ddot");
          _SFD_SET_DATA_PROPS(7,2,0,1,"q3_ddot");
          _SFD_SET_DATA_PROPS(8,1,1,0,"params");
          _SFD_SET_DATA_PROPS(9,1,1,0,"w");
          _SFD_SET_DATA_PROPS(10,2,0,1,"quaternion_multipliers");
          _SFD_SET_DATA_PROPS(11,2,0,1,"quaternion_deltas");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,5054);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 14;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 5;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)
            c2_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          real_T *c2_x_ddot;
          real_T *c2_y_ddot;
          real_T *c2_z_ddot;
          real_T *c2_q0_ddot;
          real_T *c2_q1_ddot;
          real_T *c2_q2_ddot;
          real_T *c2_q3_ddot;
          real_T (*c2_states)[14];
          real_T (*c2_params)[15];
          real_T (*c2_w)[4];
          real_T (*c2_quaternion_multipliers)[20];
          real_T (*c2_quaternion_deltas)[5];
          c2_quaternion_deltas = (real_T (*)[5])ssGetOutputPortSignal
            (chartInstance->S, 9);
          c2_quaternion_multipliers = (real_T (*)[20])ssGetOutputPortSignal
            (chartInstance->S, 8);
          c2_w = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 2);
          c2_params = (real_T (*)[15])ssGetInputPortSignal(chartInstance->S, 1);
          c2_q3_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
          c2_q2_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c2_q1_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c2_q0_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c2_z_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c2_y_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_x_ddot = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_states = (real_T (*)[14])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_states);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_x_ddot);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_y_ddot);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_z_ddot);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_q0_ddot);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_q1_ddot);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_q2_ddot);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_q3_ddot);
          _SFD_SET_DATA_VALUE_PTR(8U, *c2_params);
          _SFD_SET_DATA_VALUE_PTR(9U, *c2_w);
          _SFD_SET_DATA_VALUE_PTR(10U, *c2_quaternion_multipliers);
          _SFD_SET_DATA_VALUE_PTR(11U, *c2_quaternion_deltas);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _quad_control_sim_qMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "jL7jwbAPQUk9O32VKIEg";
}

static void sf_opaque_initialize_c2_quad_control_sim_q(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
  initialize_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_quad_control_sim_q(void *chartInstanceVar)
{
  enable_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_quad_control_sim_q(void *chartInstanceVar)
{
  disable_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_quad_control_sim_q(void *chartInstanceVar)
{
  sf_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_quad_control_sim_q(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_quad_control_sim_q
    ((SFc2_quad_control_sim_qInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_quad_control_sim_q();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_quad_control_sim_q(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_quad_control_sim_q();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_quad_control_sim_q(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_quad_control_sim_q(S);
}

static void sf_opaque_set_sim_state_c2_quad_control_sim_q(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_quad_control_sim_q(S, st);
}

static void sf_opaque_terminate_c2_quad_control_sim_q(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_quad_control_sim_qInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_quad_control_sim_q_optimization_info();
    }

    finalize_c2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_quad_control_sim_q((SFc2_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_quad_control_sim_q(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_quad_control_sim_q
      ((SFc2_quad_control_sim_qInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_quad_control_sim_q(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_quad_control_sim_q_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,9);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=9; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(640644172U));
  ssSetChecksum1(S,(820898020U));
  ssSetChecksum2(S,(2016240405U));
  ssSetChecksum3(S,(4120237861U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_quad_control_sim_q(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_quad_control_sim_q(SimStruct *S)
{
  SFc2_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc2_quad_control_sim_qInstanceStruct *)utMalloc(sizeof
    (SFc2_quad_control_sim_qInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_quad_control_sim_qInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_quad_control_sim_q;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_quad_control_sim_q;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_quad_control_sim_q;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_quad_control_sim_q;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_quad_control_sim_q;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_quad_control_sim_q;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_quad_control_sim_q;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_quad_control_sim_q;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_quad_control_sim_q;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_quad_control_sim_q;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_quad_control_sim_q;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_quad_control_sim_q_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_quad_control_sim_q(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_quad_control_sim_q(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_quad_control_sim_q(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_quad_control_sim_q_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
