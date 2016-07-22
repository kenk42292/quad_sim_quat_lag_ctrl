/* Include files */

#include <stddef.h>
#include "blas.h"
#include "quad_control_sim_q_sfun.h"
#include "c3_quad_control_sim_q.h"
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
static const char * c3_debug_family_names[63] = { "m", "g", "l", "Jxx", "Jyy",
  "Jzz", "k1", "k2", "k3", "k4", "b1", "b2", "b3", "b4", "d", "q0", "q1", "q2",
  "q3", "q0_dot", "q1_dot", "q2_dot", "q3_dot", "q0_r", "q1_r", "q2_r", "q3_r",
  "a_u", "q0_err", "q1_err", "q2_err", "q3_err", "n", "wx", "wy", "wz", "tau_x",
  "tau_y", "tau_z", "gamma1", "gamma2", "gamma3", "gamma4", "w3_sq3", "w1_sq3",
  "w2_sq3", "w4_sq3", "w2_sq", "w1_sq", "w3_sq", "w4_sq", "w1", "w2", "w3", "w4",
  "nargin", "nargout", "inputs", "states", "pq", "pw", "params", "w" };

/* Function Declarations */
static void initialize_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void initialize_params_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void enable_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance);
static void disable_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct *
  chartInstance);
static void c3_update_debugger_state_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void set_sim_state_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance, const mxArray *c3_st);
static void finalize_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance);
static void sf_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance);
static void c3_chartstep_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void initSimStructsc3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void registerMessagesc3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData);
static void c3_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_w, const char_T *c3_identifier, real_T c3_y
  [4]);
static void c3_b_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4]);
static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static real_T c3_c_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[39]);
static real_T c3_norm(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c3_x[4]);
static real_T c3_sqrt(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c3_x);
static void c3_eml_error(SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static real_T c3_acos(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c3_x);
static void c3_b_eml_error(SFc3_quad_control_sim_qInstanceStruct *chartInstance);
static void c3_eml_scalar_eg(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance);
static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData);
static int32_T c3_d_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData);
static uint8_T c3_e_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_quad_control_sim_q, const
  char_T *c3_identifier);
static uint8_T c3_f_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_b_sqrt(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T *c3_x);
static void c3_b_acos(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T *c3_x);
static void init_dsm_address_info(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
  chartInstance->c3_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c3_is_active_c3_quad_control_sim_q = 0U;
}

static void initialize_params_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void enable_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct *
  chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c3_update_debugger_state_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  int32_T c3_i0;
  real_T c3_u[4];
  const mxArray *c3_b_y = NULL;
  uint8_T c3_hoistedGlobal;
  uint8_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T (*c3_w)[4];
  c3_w = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_st = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(2), FALSE);
  for (c3_i0 = 0; c3_i0 < 4; c3_i0++) {
    c3_u[c3_i0] = (*c3_w)[c3_i0];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_hoistedGlobal = chartInstance->c3_is_active_c3_quad_control_sim_q;
  c3_b_u = c3_hoistedGlobal;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  sf_mex_assign(&c3_st, c3_y, FALSE);
  return c3_st;
}

static void set_sim_state_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  real_T c3_dv0[4];
  int32_T c3_i1;
  real_T (*c3_w)[4];
  c3_w = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c3_doneDoubleBufferReInit = TRUE;
  c3_u = sf_mex_dup(c3_st);
  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 0)), "w",
                      c3_dv0);
  for (c3_i1 = 0; c3_i1 < 4; c3_i1++) {
    (*c3_w)[c3_i1] = c3_dv0[c3_i1];
  }

  chartInstance->c3_is_active_c3_quad_control_sim_q = c3_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
     "is_active_c3_quad_control_sim_q");
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_quad_control_sim_q(chartInstance);
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static void sf_c3_quad_control_sim_q(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance)
{
  int32_T c3_i2;
  int32_T c3_i3;
  int32_T c3_i4;
  int32_T c3_i5;
  real_T *c3_pq;
  real_T *c3_pw;
  real_T (*c3_params)[15];
  real_T (*c3_w)[4];
  real_T (*c3_states)[14];
  real_T (*c3_inputs)[5];
  c3_params = (real_T (*)[15])ssGetInputPortSignal(chartInstance->S, 4);
  c3_pw = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c3_pq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_w = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_states = (real_T (*)[14])ssGetInputPortSignal(chartInstance->S, 1);
  c3_inputs = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  for (c3_i2 = 0; c3_i2 < 5; c3_i2++) {
    _SFD_DATA_RANGE_CHECK((*c3_inputs)[c3_i2], 0U);
  }

  for (c3_i3 = 0; c3_i3 < 14; c3_i3++) {
    _SFD_DATA_RANGE_CHECK((*c3_states)[c3_i3], 1U);
  }

  for (c3_i4 = 0; c3_i4 < 4; c3_i4++) {
    _SFD_DATA_RANGE_CHECK((*c3_w)[c3_i4], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c3_pq, 3U);
  _SFD_DATA_RANGE_CHECK(*c3_pw, 4U);
  for (c3_i5 = 0; c3_i5 < 15; c3_i5++) {
    _SFD_DATA_RANGE_CHECK((*c3_params)[c3_i5], 5U);
  }

  chartInstance->c3_sfEvent = CALL_EVENT;
  c3_chartstep_c3_quad_control_sim_q(chartInstance);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_quad_control_sim_qMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c3_chartstep_c3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
  real_T c3_hoistedGlobal;
  real_T c3_b_hoistedGlobal;
  int32_T c3_i6;
  real_T c3_inputs[5];
  int32_T c3_i7;
  real_T c3_states[14];
  real_T c3_pq;
  real_T c3_pw;
  int32_T c3_i8;
  real_T c3_params[15];
  uint32_T c3_debug_family_var_map[63];
  real_T c3_m;
  real_T c3_g;
  real_T c3_l;
  real_T c3_Jxx;
  real_T c3_Jyy;
  real_T c3_Jzz;
  real_T c3_k1;
  real_T c3_k2;
  real_T c3_k3;
  real_T c3_k4;
  real_T c3_b1;
  real_T c3_b2;
  real_T c3_b3;
  real_T c3_b4;
  real_T c3_d;
  real_T c3_q0;
  real_T c3_q1;
  real_T c3_q2;
  real_T c3_q3;
  real_T c3_q0_dot;
  real_T c3_q1_dot;
  real_T c3_q2_dot;
  real_T c3_q3_dot;
  real_T c3_q0_r;
  real_T c3_q1_r;
  real_T c3_q2_r;
  real_T c3_q3_r;
  real_T c3_a_u;
  real_T c3_q0_err;
  real_T c3_q1_err;
  real_T c3_q2_err;
  real_T c3_q3_err;
  real_T c3_n;
  real_T c3_wx;
  real_T c3_wy;
  real_T c3_wz;
  real_T c3_tau_x;
  real_T c3_tau_y;
  real_T c3_tau_z;
  real_T c3_gamma1;
  real_T c3_gamma2;
  real_T c3_gamma3;
  real_T c3_gamma4;
  real_T c3_w3_sq3;
  real_T c3_w1_sq3;
  real_T c3_w2_sq3;
  real_T c3_w4_sq3;
  real_T c3_w2_sq;
  real_T c3_w1_sq;
  real_T c3_w3_sq;
  real_T c3_w4_sq;
  real_T c3_w1;
  real_T c3_w2;
  real_T c3_w3;
  real_T c3_w4;
  real_T c3_nargin = 5.0;
  real_T c3_nargout = 1.0;
  real_T c3_w[4];
  real_T c3_a;
  real_T c3_b;
  real_T c3_y;
  real_T c3_b_a;
  real_T c3_b_b;
  real_T c3_b_y;
  real_T c3_c_a;
  real_T c3_c_b;
  real_T c3_c_y;
  real_T c3_d_a;
  real_T c3_d_b;
  real_T c3_d_y;
  real_T c3_e_a;
  real_T c3_e_b;
  real_T c3_e_y;
  real_T c3_f_a;
  real_T c3_f_b;
  real_T c3_f_y;
  real_T c3_g_a;
  real_T c3_g_b;
  real_T c3_g_y;
  real_T c3_h_a;
  real_T c3_h_b;
  real_T c3_h_y;
  real_T c3_i_a;
  real_T c3_i_b;
  real_T c3_i_y;
  real_T c3_j_a;
  real_T c3_j_b;
  real_T c3_j_y;
  real_T c3_k_a;
  real_T c3_k_b;
  real_T c3_k_y;
  real_T c3_l_a;
  real_T c3_l_b;
  real_T c3_l_y;
  real_T c3_m_a;
  real_T c3_m_b;
  real_T c3_m_y;
  real_T c3_n_a;
  real_T c3_n_b;
  real_T c3_n_y;
  real_T c3_o_a;
  real_T c3_o_b;
  real_T c3_o_y;
  real_T c3_p_a;
  real_T c3_p_b;
  real_T c3_p_y;
  real_T c3_b_q0_err[4];
  real_T c3_A;
  real_T c3_B;
  real_T c3_x;
  real_T c3_q_y;
  real_T c3_b_x;
  real_T c3_r_y;
  real_T c3_b_A;
  real_T c3_b_B;
  real_T c3_c_x;
  real_T c3_s_y;
  real_T c3_d_x;
  real_T c3_t_y;
  real_T c3_c_A;
  real_T c3_c_B;
  real_T c3_e_x;
  real_T c3_u_y;
  real_T c3_f_x;
  real_T c3_v_y;
  real_T c3_d_A;
  real_T c3_d_B;
  real_T c3_g_x;
  real_T c3_w_y;
  real_T c3_h_x;
  real_T c3_x_y;
  real_T c3_q_a;
  real_T c3_q_b;
  real_T c3_y_y;
  real_T c3_r_a;
  real_T c3_r_b;
  real_T c3_ab_y;
  real_T c3_s_a;
  real_T c3_s_b;
  real_T c3_bb_y;
  real_T c3_t_a;
  real_T c3_t_b;
  real_T c3_cb_y;
  real_T c3_u_b;
  real_T c3_u_a;
  real_T c3_v_b;
  real_T c3_db_y;
  real_T c3_v_a;
  real_T c3_w_b;
  real_T c3_eb_y;
  real_T c3_w_a;
  real_T c3_x_b;
  real_T c3_fb_y;
  real_T c3_x_a;
  real_T c3_y_b;
  real_T c3_gb_y;
  real_T c3_ab_b;
  real_T c3_y_a;
  real_T c3_bb_b;
  real_T c3_hb_y;
  real_T c3_ab_a;
  real_T c3_cb_b;
  real_T c3_ib_y;
  real_T c3_bb_a;
  real_T c3_db_b;
  real_T c3_jb_y;
  real_T c3_cb_a;
  real_T c3_eb_b;
  real_T c3_kb_y;
  real_T c3_fb_b;
  real_T c3_db_a;
  real_T c3_gb_b;
  real_T c3_lb_y;
  real_T c3_eb_a;
  real_T c3_hb_b;
  real_T c3_mb_y;
  real_T c3_e_A;
  real_T c3_i_x;
  real_T c3_j_x;
  real_T c3_nb_y;
  real_T c3_fb_a;
  real_T c3_ib_b;
  real_T c3_ob_y;
  real_T c3_gb_a;
  real_T c3_jb_b;
  real_T c3_pb_y;
  real_T c3_hb_a;
  real_T c3_kb_b;
  real_T c3_qb_y;
  real_T c3_ib_a;
  real_T c3_lb_b;
  real_T c3_rb_y;
  real_T c3_f_A;
  real_T c3_k_x;
  real_T c3_l_x;
  real_T c3_sb_y;
  real_T c3_jb_a;
  real_T c3_mb_b;
  real_T c3_tb_y;
  real_T c3_kb_a;
  real_T c3_nb_b;
  real_T c3_ub_y;
  real_T c3_lb_a;
  real_T c3_ob_b;
  real_T c3_vb_y;
  real_T c3_mb_a;
  real_T c3_pb_b;
  real_T c3_wb_y;
  real_T c3_nb_a;
  real_T c3_qb_b;
  real_T c3_xb_y;
  real_T c3_ob_a;
  real_T c3_rb_b;
  real_T c3_yb_y;
  real_T c3_e_B;
  real_T c3_ac_y;
  real_T c3_bc_y;
  real_T c3_cc_y;
  real_T c3_pb_a;
  real_T c3_sb_b;
  real_T c3_f_B;
  real_T c3_dc_y;
  real_T c3_ec_y;
  real_T c3_fc_y;
  real_T c3_qb_a;
  real_T c3_tb_b;
  real_T c3_rb_a;
  real_T c3_ub_b;
  real_T c3_gc_y;
  real_T c3_vb_b;
  real_T c3_hc_y;
  real_T c3_m_x;
  real_T c3_n_x;
  real_T c3_g_A;
  real_T c3_g_B;
  real_T c3_o_x;
  real_T c3_ic_y;
  real_T c3_p_x;
  real_T c3_jc_y;
  real_T c3_h_B;
  real_T c3_kc_y;
  real_T c3_lc_y;
  real_T c3_mc_y;
  real_T c3_sb_a;
  real_T c3_wb_b;
  real_T c3_h_A;
  real_T c3_i_B;
  real_T c3_q_x;
  real_T c3_nc_y;
  real_T c3_r_x;
  real_T c3_oc_y;
  real_T c3_pc_y;
  real_T c3_i_A;
  real_T c3_s_x;
  real_T c3_t_x;
  real_T c3_qc_y;
  real_T c3_tb_a;
  real_T c3_xb_b;
  real_T c3_rc_y;
  real_T c3_j_A;
  real_T c3_j_B;
  real_T c3_u_x;
  real_T c3_sc_y;
  real_T c3_v_x;
  real_T c3_tc_y;
  real_T c3_uc_y;
  real_T c3_k_A;
  real_T c3_w_x;
  real_T c3_x_x;
  real_T c3_vc_y;
  real_T c3_ub_a;
  real_T c3_yb_b;
  real_T c3_wc_y;
  real_T c3_l_A;
  real_T c3_k_B;
  real_T c3_y_x;
  real_T c3_xc_y;
  real_T c3_ab_x;
  real_T c3_yc_y;
  real_T c3_ad_y;
  real_T c3_m_A;
  real_T c3_bb_x;
  real_T c3_cb_x;
  real_T c3_bd_y;
  real_T c3_vb_a;
  real_T c3_ac_b;
  real_T c3_cd_y;
  real_T c3_n_A;
  real_T c3_l_B;
  real_T c3_db_x;
  real_T c3_dd_y;
  real_T c3_eb_x;
  real_T c3_ed_y;
  real_T c3_fd_y;
  real_T c3_wb_a;
  real_T c3_bc_b;
  real_T c3_gd_y;
  real_T c3_o_A;
  real_T c3_m_B;
  real_T c3_fb_x;
  real_T c3_hd_y;
  real_T c3_gb_x;
  real_T c3_id_y;
  real_T c3_jd_y;
  real_T c3_xb_a;
  real_T c3_cc_b;
  real_T c3_kd_y;
  real_T c3_p_A;
  real_T c3_n_B;
  real_T c3_hb_x;
  real_T c3_ld_y;
  real_T c3_ib_x;
  real_T c3_md_y;
  real_T c3_nd_y;
  real_T c3_yb_a;
  real_T c3_dc_b;
  real_T c3_od_y;
  real_T c3_q_A;
  real_T c3_o_B;
  real_T c3_jb_x;
  real_T c3_pd_y;
  real_T c3_kb_x;
  real_T c3_qd_y;
  real_T c3_r_A;
  real_T c3_lb_x;
  real_T c3_mb_x;
  real_T c3_rd_y;
  real_T c3_ac_a;
  real_T c3_ec_b;
  real_T c3_sd_y;
  real_T c3_s_A;
  real_T c3_p_B;
  real_T c3_nb_x;
  real_T c3_td_y;
  real_T c3_ob_x;
  real_T c3_ud_y;
  real_T c3_t_A;
  real_T c3_pb_x;
  real_T c3_qb_x;
  real_T c3_vd_y;
  real_T c3_bc_a;
  real_T c3_fc_b;
  real_T c3_wd_y;
  real_T c3_u_A;
  real_T c3_q_B;
  real_T c3_rb_x;
  real_T c3_xd_y;
  real_T c3_sb_x;
  real_T c3_yd_y;
  real_T c3_v_A;
  real_T c3_tb_x;
  real_T c3_ub_x;
  real_T c3_ae_y;
  real_T c3_cc_a;
  real_T c3_gc_b;
  real_T c3_be_y;
  real_T c3_w_A;
  real_T c3_r_B;
  real_T c3_vb_x;
  real_T c3_ce_y;
  real_T c3_wb_x;
  real_T c3_de_y;
  real_T c3_x_A;
  real_T c3_s_B;
  real_T c3_xb_x;
  real_T c3_ee_y;
  real_T c3_yb_x;
  real_T c3_fe_y;
  real_T c3_ge_y;
  real_T c3_y_A;
  real_T c3_ac_x;
  real_T c3_bc_x;
  real_T c3_he_y;
  real_T c3_dc_a;
  real_T c3_hc_b;
  real_T c3_ie_y;
  real_T c3_ab_A;
  real_T c3_t_B;
  real_T c3_cc_x;
  real_T c3_je_y;
  real_T c3_dc_x;
  real_T c3_ke_y;
  real_T c3_le_y;
  real_T c3_bb_A;
  real_T c3_ec_x;
  real_T c3_fc_x;
  real_T c3_me_y;
  real_T c3_ec_a;
  real_T c3_ic_b;
  real_T c3_ne_y;
  real_T c3_cb_A;
  real_T c3_u_B;
  real_T c3_gc_x;
  real_T c3_oe_y;
  real_T c3_hc_x;
  real_T c3_pe_y;
  real_T c3_qe_y;
  real_T c3_db_A;
  real_T c3_ic_x;
  real_T c3_jc_x;
  real_T c3_re_y;
  real_T c3_fc_a;
  real_T c3_jc_b;
  real_T c3_se_y;
  real_T c3_eb_A;
  real_T c3_v_B;
  real_T c3_kc_x;
  real_T c3_te_y;
  real_T c3_lc_x;
  real_T c3_ue_y;
  real_T c3_ve_y;
  real_T c3_gc_a;
  real_T c3_kc_b;
  real_T c3_we_y;
  real_T c3_fb_A;
  real_T c3_w_B;
  real_T c3_mc_x;
  real_T c3_xe_y;
  real_T c3_nc_x;
  real_T c3_ye_y;
  real_T c3_af_y;
  real_T c3_hc_a;
  real_T c3_lc_b;
  real_T c3_bf_y;
  real_T c3_gb_A;
  real_T c3_x_B;
  real_T c3_oc_x;
  real_T c3_cf_y;
  real_T c3_pc_x;
  real_T c3_df_y;
  real_T c3_ef_y;
  real_T c3_ic_a;
  real_T c3_mc_b;
  real_T c3_ff_y;
  real_T c3_hb_A;
  real_T c3_y_B;
  real_T c3_qc_x;
  real_T c3_gf_y;
  real_T c3_rc_x;
  real_T c3_hf_y;
  real_T c3_ib_A;
  real_T c3_sc_x;
  real_T c3_tc_x;
  real_T c3_if_y;
  real_T c3_jc_a;
  real_T c3_nc_b;
  real_T c3_jf_y;
  real_T c3_jb_A;
  real_T c3_ab_B;
  real_T c3_uc_x;
  real_T c3_kf_y;
  real_T c3_vc_x;
  real_T c3_lf_y;
  real_T c3_kb_A;
  real_T c3_wc_x;
  real_T c3_xc_x;
  real_T c3_mf_y;
  real_T c3_kc_a;
  real_T c3_oc_b;
  real_T c3_nf_y;
  real_T c3_lb_A;
  real_T c3_bb_B;
  real_T c3_yc_x;
  real_T c3_of_y;
  real_T c3_ad_x;
  real_T c3_pf_y;
  real_T c3_lc_a;
  real_T c3_pc_b;
  real_T c3_qf_y;
  real_T c3_mb_A;
  real_T c3_bd_x;
  real_T c3_cd_x;
  real_T c3_rf_y;
  real_T c3_nb_A;
  real_T c3_cb_B;
  real_T c3_dd_x;
  real_T c3_sf_y;
  real_T c3_ed_x;
  real_T c3_tf_y;
  real_T c3_varargin_2;
  real_T c3_varargin_3;
  real_T c3_uf_y;
  real_T c3_vf_y;
  real_T c3_yk;
  real_T c3_wf_y;
  real_T c3_maxval;
  real_T c3_b_varargin_2;
  real_T c3_b_varargin_3;
  real_T c3_xf_y;
  real_T c3_yf_y;
  real_T c3_b_yk;
  real_T c3_ag_y;
  real_T c3_b_maxval;
  real_T c3_fd_x;
  real_T c3_gd_x;
  real_T c3_c_varargin_2;
  real_T c3_c_varargin_3;
  real_T c3_bg_y;
  real_T c3_cg_y;
  real_T c3_c_yk;
  real_T c3_dg_y;
  real_T c3_c_maxval;
  real_T c3_hd_x;
  real_T c3_id_x;
  real_T c3_d_varargin_2;
  real_T c3_d_varargin_3;
  real_T c3_eg_y;
  real_T c3_fg_y;
  real_T c3_d_yk;
  real_T c3_gg_y;
  real_T c3_d_maxval;
  real_T c3_jd_x;
  real_T c3_kd_x;
  int32_T c3_i9;
  real_T *c3_b_pw;
  real_T *c3_b_pq;
  real_T (*c3_b_w)[4];
  real_T (*c3_b_params)[15];
  real_T (*c3_b_states)[14];
  real_T (*c3_b_inputs)[5];
  c3_b_params = (real_T (*)[15])ssGetInputPortSignal(chartInstance->S, 4);
  c3_b_pw = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c3_b_pq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c3_b_w = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c3_b_states = (real_T (*)[14])ssGetInputPortSignal(chartInstance->S, 1);
  c3_b_inputs = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
  c3_hoistedGlobal = *c3_b_pq;
  c3_b_hoistedGlobal = *c3_b_pw;
  for (c3_i6 = 0; c3_i6 < 5; c3_i6++) {
    c3_inputs[c3_i6] = (*c3_b_inputs)[c3_i6];
  }

  for (c3_i7 = 0; c3_i7 < 14; c3_i7++) {
    c3_states[c3_i7] = (*c3_b_states)[c3_i7];
  }

  c3_pq = c3_hoistedGlobal;
  c3_pw = c3_b_hoistedGlobal;
  for (c3_i8 = 0; c3_i8 < 15; c3_i8++) {
    c3_params[c3_i8] = (*c3_b_params)[c3_i8];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 63U, 63U, c3_debug_family_names,
    c3_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_m, 0U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_g, 1U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_l, 2U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Jxx, 3U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Jyy, 4U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_Jzz, 5U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_k1, 6U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_k2, 7U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_k3, 8U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_k4, 9U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b1, 10U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b2, 11U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b3, 12U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_b4, 13U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_d, 14U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q0, 15U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q1, 16U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q2, 17U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q3, 18U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q0_dot, 19U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q1_dot, 20U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q2_dot, 21U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q3_dot, 22U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q0_r, 23U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q1_r, 24U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q2_r, 25U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q3_r, 26U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_a_u, 27U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q0_err, 28U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q1_err, 29U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q2_err, 30U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_q3_err, 31U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_n, 32U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_wx, 33U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_wy, 34U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_wz, 35U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tau_x, 36U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tau_y, 37U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_tau_z, 38U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_gamma1, 39U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_gamma2, 40U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_gamma3, 41U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_gamma4, 42U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w3_sq3, 43U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w1_sq3, 44U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w2_sq3, 45U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w4_sq3, 46U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w2_sq, 47U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w1_sq, 48U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w3_sq, 49U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w4_sq, 50U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w1, 51U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w2, 52U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w3, 53U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_w4, 54U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargin, 55U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c3_nargout, 56U, c3_c_sf_marshallOut,
    c3_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_inputs, 57U, c3_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_states, 58U, c3_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_pq, 59U, c3_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c3_pw, 60U, c3_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c3_params, 61U, c3_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c3_w, 62U, c3_sf_marshallOut,
    c3_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 3);
  c3_m = c3_params[0];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 4);
  c3_g = c3_params[1];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 5);
  c3_l = c3_params[2];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 7);
  c3_Jxx = c3_params[3];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 8);
  c3_Jyy = c3_params[4];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 9);
  c3_Jzz = c3_params[5];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 11);
  c3_k1 = c3_params[6];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 12);
  c3_k2 = c3_params[7];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 13);
  c3_k3 = c3_params[8];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 14);
  c3_k4 = c3_params[9];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 16);
  c3_b1 = c3_params[10];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 17);
  c3_b2 = c3_params[11];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 18);
  c3_b3 = c3_params[12];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 19);
  c3_b4 = c3_params[13];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 21);
  c3_d = c3_params[14];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
  c3_q0 = c3_states[6];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
  c3_q1 = c3_states[8];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
  c3_q2 = c3_states[10];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 23);
  c3_q3 = c3_states[12];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 24);
  c3_q0_dot = c3_states[7];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 24);
  c3_q1_dot = c3_states[9];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 24);
  c3_q2_dot = c3_states[11];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 24);
  c3_q3_dot = c3_states[13];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
  c3_q0_r = c3_inputs[0];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
  c3_q1_r = c3_inputs[1];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
  c3_q2_r = c3_inputs[2];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
  c3_q3_r = c3_inputs[3];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 26);
  c3_a_u = c3_inputs[4];
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 28);
  c3_a = c3_q0_r;
  c3_b = c3_q0;
  c3_y = c3_a * c3_b;
  c3_b_a = c3_q1_r;
  c3_b_b = c3_q1;
  c3_b_y = c3_b_a * c3_b_b;
  c3_c_a = c3_q2_r;
  c3_c_b = c3_q2;
  c3_c_y = c3_c_a * c3_c_b;
  c3_d_a = c3_q3_r;
  c3_d_b = c3_q3;
  c3_d_y = c3_d_a * c3_d_b;
  c3_q0_err = ((c3_y + c3_b_y) + c3_c_y) + c3_d_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 29);
  c3_e_a = c3_q1_r;
  c3_e_b = c3_q0;
  c3_e_y = c3_e_a * c3_e_b;
  c3_f_a = c3_q3_r;
  c3_f_b = c3_q2;
  c3_f_y = c3_f_a * c3_f_b;
  c3_g_a = c3_q0_r;
  c3_g_b = c3_q1;
  c3_g_y = c3_g_a * c3_g_b;
  c3_h_a = c3_q3_r;
  c3_h_b = c3_q3;
  c3_h_y = c3_h_a * c3_h_b;
  c3_q1_err = ((c3_e_y + c3_f_y) - c3_g_y) - c3_h_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 30);
  c3_i_a = c3_q2_r;
  c3_i_b = c3_q0;
  c3_i_y = c3_i_a * c3_i_b;
  c3_j_a = c3_q1_r;
  c3_j_b = c3_q3;
  c3_j_y = c3_j_a * c3_j_b;
  c3_k_a = c3_q3_r;
  c3_k_b = c3_q1;
  c3_k_y = c3_k_a * c3_k_b;
  c3_l_a = c3_q0_r;
  c3_l_b = c3_q2;
  c3_l_y = c3_l_a * c3_l_b;
  c3_q2_err = ((c3_i_y + c3_j_y) - c3_k_y) - c3_l_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 31);
  c3_m_a = c3_q3_r;
  c3_m_b = c3_q0;
  c3_m_y = c3_m_a * c3_m_b;
  c3_n_a = c3_q2_r;
  c3_n_b = c3_q1;
  c3_n_y = c3_n_a * c3_n_b;
  c3_o_a = c3_q1_r;
  c3_o_b = c3_q2;
  c3_o_y = c3_o_a * c3_o_b;
  c3_p_a = c3_q0_r;
  c3_p_b = c3_q3;
  c3_p_y = c3_p_a * c3_p_b;
  c3_q3_err = ((c3_m_y + c3_n_y) - c3_o_y) - c3_p_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 33);
  if (CV_EML_IF(0, 1, 0, c3_q0_err < 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 34);
    c3_q1_err = -c3_q1_err;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 35);
    c3_q2_err = -c3_q2_err;
    _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 36);
    c3_q3_err = -c3_q3_err;
  }

  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 40);
  c3_b_q0_err[0] = c3_q0_err;
  c3_b_q0_err[1] = c3_q1_err;
  c3_b_q0_err[2] = c3_q2_err;
  c3_b_q0_err[3] = c3_q3_err;
  c3_n = c3_norm(chartInstance, c3_b_q0_err);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 41);
  c3_A = c3_q0_err;
  c3_B = c3_n;
  c3_x = c3_A;
  c3_q_y = c3_B;
  c3_b_x = c3_x;
  c3_r_y = c3_q_y;
  c3_q0_err = c3_b_x / c3_r_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 41);
  c3_b_A = c3_q1_err;
  c3_b_B = c3_n;
  c3_c_x = c3_b_A;
  c3_s_y = c3_b_B;
  c3_d_x = c3_c_x;
  c3_t_y = c3_s_y;
  c3_q1_err = c3_d_x / c3_t_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 41);
  c3_c_A = c3_q2_err;
  c3_c_B = c3_n;
  c3_e_x = c3_c_A;
  c3_u_y = c3_c_B;
  c3_f_x = c3_e_x;
  c3_v_y = c3_u_y;
  c3_q2_err = c3_f_x / c3_v_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 41);
  c3_d_A = c3_q3_err;
  c3_d_B = c3_n;
  c3_g_x = c3_d_A;
  c3_w_y = c3_d_B;
  c3_h_x = c3_g_x;
  c3_x_y = c3_w_y;
  c3_q3_err = c3_h_x / c3_x_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 43);
  c3_q_a = c3_q1_dot;
  c3_q_b = c3_q0;
  c3_y_y = c3_q_a * c3_q_b;
  c3_r_a = c3_q2_dot;
  c3_r_b = c3_q3;
  c3_ab_y = c3_r_a * c3_r_b;
  c3_s_a = c3_q3_dot;
  c3_s_b = c3_q2;
  c3_bb_y = c3_s_a * c3_s_b;
  c3_t_a = c3_q0_dot;
  c3_t_b = c3_q1;
  c3_cb_y = c3_t_a * c3_t_b;
  c3_u_b = ((c3_y_y + c3_ab_y) - c3_bb_y) - c3_cb_y;
  c3_wx = 2.0 * c3_u_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 44);
  c3_u_a = c3_q3_dot;
  c3_v_b = c3_q1;
  c3_db_y = c3_u_a * c3_v_b;
  c3_v_a = c3_q2_dot;
  c3_w_b = c3_q0;
  c3_eb_y = c3_v_a * c3_w_b;
  c3_w_a = c3_q1_dot;
  c3_x_b = c3_q3;
  c3_fb_y = c3_w_a * c3_x_b;
  c3_x_a = c3_q0_dot;
  c3_y_b = c3_q2;
  c3_gb_y = c3_x_a * c3_y_b;
  c3_ab_b = ((c3_db_y + c3_eb_y) - c3_fb_y) - c3_gb_y;
  c3_wy = 2.0 * c3_ab_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 45);
  c3_y_a = c3_q3_dot;
  c3_bb_b = c3_q0;
  c3_hb_y = c3_y_a * c3_bb_b;
  c3_ab_a = c3_q1_dot;
  c3_cb_b = c3_q2;
  c3_ib_y = c3_ab_a * c3_cb_b;
  c3_bb_a = c3_q2_dot;
  c3_db_b = c3_q1;
  c3_jb_y = c3_bb_a * c3_db_b;
  c3_cb_a = c3_q0_dot;
  c3_eb_b = c3_q3;
  c3_kb_y = c3_cb_a * c3_eb_b;
  c3_fb_b = ((c3_hb_y + c3_ib_y) - c3_jb_y) - c3_kb_y;
  c3_wz = 2.0 * c3_fb_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 50);
  c3_db_a = c3_pq;
  c3_gb_b = c3_q1_err;
  c3_lb_y = c3_db_a * c3_gb_b;
  c3_eb_a = c3_pw;
  c3_hb_b = c3_Jxx;
  c3_mb_y = c3_eb_a * c3_hb_b;
  c3_e_A = c3_l;
  c3_i_x = c3_e_A;
  c3_j_x = c3_i_x;
  c3_nb_y = c3_j_x / 1.4142135623730951;
  c3_fb_a = c3_mb_y;
  c3_ib_b = c3_nb_y;
  c3_ob_y = c3_fb_a * c3_ib_b;
  c3_gb_a = c3_ob_y;
  c3_jb_b = c3_wx;
  c3_pb_y = c3_gb_a * c3_jb_b;
  c3_tau_x = c3_lb_y - c3_pb_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 51);
  c3_hb_a = c3_pq;
  c3_kb_b = c3_q2_err;
  c3_qb_y = c3_hb_a * c3_kb_b;
  c3_ib_a = c3_pw;
  c3_lb_b = c3_Jyy;
  c3_rb_y = c3_ib_a * c3_lb_b;
  c3_f_A = c3_l;
  c3_k_x = c3_f_A;
  c3_l_x = c3_k_x;
  c3_sb_y = c3_l_x / 1.4142135623730951;
  c3_jb_a = c3_rb_y;
  c3_mb_b = c3_sb_y;
  c3_tb_y = c3_jb_a * c3_mb_b;
  c3_kb_a = c3_tb_y;
  c3_nb_b = c3_wy;
  c3_ub_y = c3_kb_a * c3_nb_b;
  c3_tau_y = c3_qb_y - c3_ub_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 52);
  c3_lb_a = c3_pq;
  c3_ob_b = c3_q3_err;
  c3_vb_y = c3_lb_a * c3_ob_b;
  c3_mb_a = c3_pw;
  c3_pb_b = c3_Jzz;
  c3_wb_y = c3_mb_a * c3_pb_b;
  c3_nb_a = c3_wb_y;
  c3_qb_b = c3_l;
  c3_xb_y = c3_nb_a * c3_qb_b;
  c3_ob_a = c3_xb_y;
  c3_rb_b = c3_wz;
  c3_yb_y = c3_ob_a * c3_rb_b;
  c3_tau_z = c3_vb_y - c3_yb_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 54);
  c3_e_B = c3_l;
  c3_ac_y = c3_e_B;
  c3_bc_y = c3_ac_y;
  c3_cc_y = 1.4142135623730951 / c3_bc_y;
  c3_pb_a = c3_cc_y;
  c3_sb_b = c3_tau_x;
  c3_gamma1 = c3_pb_a * c3_sb_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 55);
  c3_f_B = c3_l;
  c3_dc_y = c3_f_B;
  c3_ec_y = c3_dc_y;
  c3_fc_y = 1.4142135623730951 / c3_ec_y;
  c3_qb_a = c3_fc_y;
  c3_tb_b = c3_tau_y;
  c3_gamma2 = c3_qb_a * c3_tb_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 56);
  c3_rb_a = c3_m;
  c3_ub_b = c3_a_u;
  c3_gc_y = c3_rb_a * c3_ub_b;
  c3_vb_b = c3_q0;
  c3_b_acos(chartInstance, &c3_vb_b);
  c3_hc_y = 2.0 * c3_vb_b;
  c3_m_x = c3_hc_y;
  c3_n_x = c3_m_x;
  c3_n_x = muDoubleScalarCos(c3_n_x);
  c3_g_A = c3_gc_y;
  c3_g_B = c3_n_x;
  c3_o_x = c3_g_A;
  c3_ic_y = c3_g_B;
  c3_p_x = c3_o_x;
  c3_jc_y = c3_ic_y;
  c3_gamma3 = c3_p_x / c3_jc_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 57);
  c3_h_B = c3_l;
  c3_kc_y = c3_h_B;
  c3_lc_y = c3_kc_y;
  c3_mc_y = 1.0 / c3_lc_y;
  c3_sb_a = c3_mc_y;
  c3_wb_b = c3_tau_z;
  c3_gamma4 = c3_sb_a * c3_wb_b;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 60);
  c3_h_A = c3_b1;
  c3_i_B = c3_k1;
  c3_q_x = c3_h_A;
  c3_nc_y = c3_i_B;
  c3_r_x = c3_q_x;
  c3_oc_y = c3_nc_y;
  c3_pc_y = c3_r_x / c3_oc_y;
  c3_i_A = c3_gamma2 - c3_gamma1;
  c3_s_x = c3_i_A;
  c3_t_x = c3_s_x;
  c3_qc_y = c3_t_x / 2.0;
  c3_tb_a = c3_pc_y;
  c3_xb_b = c3_qc_y;
  c3_rc_y = c3_tb_a * c3_xb_b;
  c3_j_A = c3_b2;
  c3_j_B = c3_k2;
  c3_u_x = c3_j_A;
  c3_sc_y = c3_j_B;
  c3_v_x = c3_u_x;
  c3_tc_y = c3_sc_y;
  c3_uc_y = c3_v_x / c3_tc_y;
  c3_k_A = c3_gamma2 + c3_gamma3;
  c3_w_x = c3_k_A;
  c3_x_x = c3_w_x;
  c3_vc_y = c3_x_x / 2.0;
  c3_ub_a = c3_uc_y;
  c3_yb_b = c3_vc_y;
  c3_wc_y = c3_ub_a * c3_yb_b;
  c3_l_A = c3_b4;
  c3_k_B = c3_k4;
  c3_y_x = c3_l_A;
  c3_xc_y = c3_k_B;
  c3_ab_x = c3_y_x;
  c3_yc_y = c3_xc_y;
  c3_ad_y = c3_ab_x / c3_yc_y;
  c3_m_A = c3_gamma3 - c3_gamma1;
  c3_bb_x = c3_m_A;
  c3_cb_x = c3_bb_x;
  c3_bd_y = c3_cb_x / 2.0;
  c3_vb_a = c3_ad_y;
  c3_ac_b = c3_bd_y;
  c3_cd_y = c3_vb_a * c3_ac_b;
  c3_n_A = c3_b1;
  c3_l_B = c3_k1;
  c3_db_x = c3_n_A;
  c3_dd_y = c3_l_B;
  c3_eb_x = c3_db_x;
  c3_ed_y = c3_dd_y;
  c3_fd_y = c3_eb_x / c3_ed_y;
  c3_wb_a = c3_fd_y;
  c3_bc_b = c3_k3;
  c3_gd_y = c3_wb_a * c3_bc_b;
  c3_o_A = c3_b2;
  c3_m_B = c3_k2;
  c3_fb_x = c3_o_A;
  c3_hd_y = c3_m_B;
  c3_gb_x = c3_fb_x;
  c3_id_y = c3_hd_y;
  c3_jd_y = c3_gb_x / c3_id_y;
  c3_xb_a = c3_jd_y;
  c3_cc_b = c3_k3;
  c3_kd_y = c3_xb_a * c3_cc_b;
  c3_p_A = c3_b4;
  c3_n_B = c3_k4;
  c3_hb_x = c3_p_A;
  c3_ld_y = c3_n_B;
  c3_ib_x = c3_hb_x;
  c3_md_y = c3_ld_y;
  c3_nd_y = c3_ib_x / c3_md_y;
  c3_yb_a = c3_nd_y;
  c3_dc_b = c3_k3;
  c3_od_y = c3_yb_a * c3_dc_b;
  c3_q_A = ((c3_gamma4 + c3_rc_y) + c3_wc_y) + c3_cd_y;
  c3_o_B = ((c3_gd_y + c3_kd_y) + c3_b3) + c3_od_y;
  c3_jb_x = c3_q_A;
  c3_pd_y = c3_o_B;
  c3_kb_x = c3_jb_x;
  c3_qd_y = c3_pd_y;
  c3_w3_sq3 = c3_kb_x / c3_qd_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 61);
  c3_r_A = c3_gamma1 - c3_gamma2;
  c3_lb_x = c3_r_A;
  c3_mb_x = c3_lb_x;
  c3_rd_y = c3_mb_x / 2.0;
  c3_ac_a = c3_k3;
  c3_ec_b = c3_w3_sq3;
  c3_sd_y = c3_ac_a * c3_ec_b;
  c3_s_A = c3_rd_y + c3_sd_y;
  c3_p_B = c3_k1;
  c3_nb_x = c3_s_A;
  c3_td_y = c3_p_B;
  c3_ob_x = c3_nb_x;
  c3_ud_y = c3_td_y;
  c3_w1_sq3 = c3_ob_x / c3_ud_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 62);
  c3_t_A = c3_gamma2 + c3_gamma3;
  c3_pb_x = c3_t_A;
  c3_qb_x = c3_pb_x;
  c3_vd_y = c3_qb_x / 2.0;
  c3_bc_a = c3_k3;
  c3_fc_b = c3_w3_sq3;
  c3_wd_y = c3_bc_a * c3_fc_b;
  c3_u_A = c3_vd_y - c3_wd_y;
  c3_q_B = c3_k2;
  c3_rb_x = c3_u_A;
  c3_xd_y = c3_q_B;
  c3_sb_x = c3_rb_x;
  c3_yd_y = c3_xd_y;
  c3_w2_sq3 = c3_sb_x / c3_yd_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 63);
  c3_v_A = c3_gamma3 - c3_gamma1;
  c3_tb_x = c3_v_A;
  c3_ub_x = c3_tb_x;
  c3_ae_y = c3_ub_x / 2.0;
  c3_cc_a = c3_k3;
  c3_gc_b = c3_w3_sq3;
  c3_be_y = c3_cc_a * c3_gc_b;
  c3_w_A = c3_ae_y - c3_be_y;
  c3_r_B = c3_k4;
  c3_vb_x = c3_w_A;
  c3_ce_y = c3_r_B;
  c3_wb_x = c3_vb_x;
  c3_de_y = c3_ce_y;
  c3_w4_sq3 = c3_wb_x / c3_de_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 65);
  c3_x_A = c3_b1;
  c3_s_B = c3_k1;
  c3_xb_x = c3_x_A;
  c3_ee_y = c3_s_B;
  c3_yb_x = c3_xb_x;
  c3_fe_y = c3_ee_y;
  c3_ge_y = c3_yb_x / c3_fe_y;
  c3_y_A = c3_gamma1 + c3_gamma3;
  c3_ac_x = c3_y_A;
  c3_bc_x = c3_ac_x;
  c3_he_y = c3_bc_x / 2.0;
  c3_dc_a = c3_ge_y;
  c3_hc_b = c3_he_y;
  c3_ie_y = c3_dc_a * c3_hc_b;
  c3_ab_A = c3_b3;
  c3_t_B = c3_k3;
  c3_cc_x = c3_ab_A;
  c3_je_y = c3_t_B;
  c3_dc_x = c3_cc_x;
  c3_ke_y = c3_je_y;
  c3_le_y = c3_dc_x / c3_ke_y;
  c3_bb_A = c3_gamma2 + c3_gamma3;
  c3_ec_x = c3_bb_A;
  c3_fc_x = c3_ec_x;
  c3_me_y = c3_fc_x / 2.0;
  c3_ec_a = c3_le_y;
  c3_ic_b = c3_me_y;
  c3_ne_y = c3_ec_a * c3_ic_b;
  c3_cb_A = c3_b4;
  c3_u_B = c3_k4;
  c3_gc_x = c3_cb_A;
  c3_oe_y = c3_u_B;
  c3_hc_x = c3_gc_x;
  c3_pe_y = c3_oe_y;
  c3_qe_y = c3_hc_x / c3_pe_y;
  c3_db_A = c3_gamma1 + c3_gamma2;
  c3_ic_x = c3_db_A;
  c3_jc_x = c3_ic_x;
  c3_re_y = c3_jc_x / 2.0;
  c3_fc_a = c3_qe_y;
  c3_jc_b = c3_re_y;
  c3_se_y = c3_fc_a * c3_jc_b;
  c3_eb_A = c3_b1;
  c3_v_B = c3_k1;
  c3_kc_x = c3_eb_A;
  c3_te_y = c3_v_B;
  c3_lc_x = c3_kc_x;
  c3_ue_y = c3_te_y;
  c3_ve_y = c3_lc_x / c3_ue_y;
  c3_gc_a = c3_ve_y;
  c3_kc_b = c3_k2;
  c3_we_y = c3_gc_a * c3_kc_b;
  c3_fb_A = c3_b3;
  c3_w_B = c3_k3;
  c3_mc_x = c3_fb_A;
  c3_xe_y = c3_w_B;
  c3_nc_x = c3_mc_x;
  c3_ye_y = c3_xe_y;
  c3_af_y = c3_nc_x / c3_ye_y;
  c3_hc_a = c3_af_y;
  c3_lc_b = c3_k2;
  c3_bf_y = c3_hc_a * c3_lc_b;
  c3_gb_A = c3_b4;
  c3_x_B = c3_k4;
  c3_oc_x = c3_gb_A;
  c3_cf_y = c3_x_B;
  c3_pc_x = c3_oc_x;
  c3_df_y = c3_cf_y;
  c3_ef_y = c3_pc_x / c3_df_y;
  c3_ic_a = c3_ef_y;
  c3_mc_b = c3_k2;
  c3_ff_y = c3_ic_a * c3_mc_b;
  c3_hb_A = ((c3_ie_y + c3_ne_y) + c3_se_y) - c3_gamma4;
  c3_y_B = ((c3_we_y + c3_b2) + c3_bf_y) + c3_ff_y;
  c3_qc_x = c3_hb_A;
  c3_gf_y = c3_y_B;
  c3_rc_x = c3_qc_x;
  c3_hf_y = c3_gf_y;
  c3_w2_sq = c3_rc_x / c3_hf_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 66);
  c3_ib_A = c3_gamma1 + c3_gamma3;
  c3_sc_x = c3_ib_A;
  c3_tc_x = c3_sc_x;
  c3_if_y = c3_tc_x / 2.0;
  c3_jc_a = c3_k2;
  c3_nc_b = c3_w2_sq;
  c3_jf_y = c3_jc_a * c3_nc_b;
  c3_jb_A = c3_if_y - c3_jf_y;
  c3_ab_B = c3_k1;
  c3_uc_x = c3_jb_A;
  c3_kf_y = c3_ab_B;
  c3_vc_x = c3_uc_x;
  c3_lf_y = c3_kf_y;
  c3_w1_sq = c3_vc_x / c3_lf_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 67);
  c3_kb_A = c3_gamma2 + c3_gamma3;
  c3_wc_x = c3_kb_A;
  c3_xc_x = c3_wc_x;
  c3_mf_y = c3_xc_x / 2.0;
  c3_kc_a = c3_k2;
  c3_oc_b = c3_w2_sq;
  c3_nf_y = c3_kc_a * c3_oc_b;
  c3_lb_A = c3_mf_y - c3_nf_y;
  c3_bb_B = c3_k3;
  c3_yc_x = c3_lb_A;
  c3_of_y = c3_bb_B;
  c3_ad_x = c3_yc_x;
  c3_pf_y = c3_of_y;
  c3_w3_sq = c3_ad_x / c3_pf_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 68);
  c3_lc_a = c3_k2;
  c3_pc_b = c3_w2_sq;
  c3_qf_y = c3_lc_a * c3_pc_b;
  c3_mb_A = c3_gamma1 + c3_gamma2;
  c3_bd_x = c3_mb_A;
  c3_cd_x = c3_bd_x;
  c3_rf_y = c3_cd_x / 2.0;
  c3_nb_A = c3_qf_y - c3_rf_y;
  c3_cb_B = c3_k4;
  c3_dd_x = c3_nb_A;
  c3_sf_y = c3_cb_B;
  c3_ed_x = c3_dd_x;
  c3_tf_y = c3_sf_y;
  c3_w4_sq = c3_ed_x / c3_tf_y;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 70);
  c3_varargin_2 = c3_w1_sq;
  c3_varargin_3 = c3_varargin_2;
  c3_uf_y = c3_varargin_3;
  c3_vf_y = c3_uf_y;
  c3_eml_scalar_eg(chartInstance);
  c3_yk = c3_vf_y;
  c3_wf_y = c3_yk;
  c3_eml_scalar_eg(chartInstance);
  c3_maxval = muDoubleScalarMax(0.0, c3_wf_y);
  c3_w1 = c3_maxval;
  c3_b_sqrt(chartInstance, &c3_w1);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 71);
  c3_b_varargin_2 = c3_w2_sq;
  c3_b_varargin_3 = c3_b_varargin_2;
  c3_xf_y = c3_b_varargin_3;
  c3_yf_y = c3_xf_y;
  c3_eml_scalar_eg(chartInstance);
  c3_b_yk = c3_yf_y;
  c3_ag_y = c3_b_yk;
  c3_eml_scalar_eg(chartInstance);
  c3_b_maxval = muDoubleScalarMax(0.0, c3_ag_y);
  c3_fd_x = c3_b_maxval;
  c3_w2 = c3_fd_x;
  if (c3_w2 < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_gd_x = c3_w2;
  c3_w2 = c3_gd_x;
  c3_w2 = muDoubleScalarSqrt(c3_w2);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 72);
  c3_c_varargin_2 = c3_w3_sq;
  c3_c_varargin_3 = c3_c_varargin_2;
  c3_bg_y = c3_c_varargin_3;
  c3_cg_y = c3_bg_y;
  c3_eml_scalar_eg(chartInstance);
  c3_c_yk = c3_cg_y;
  c3_dg_y = c3_c_yk;
  c3_eml_scalar_eg(chartInstance);
  c3_c_maxval = muDoubleScalarMax(0.0, c3_dg_y);
  c3_hd_x = c3_c_maxval;
  c3_w3 = c3_hd_x;
  if (c3_w3 < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_id_x = c3_w3;
  c3_w3 = c3_id_x;
  c3_w3 = muDoubleScalarSqrt(c3_w3);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 73);
  c3_d_varargin_2 = c3_w4_sq;
  c3_d_varargin_3 = c3_d_varargin_2;
  c3_eg_y = c3_d_varargin_3;
  c3_fg_y = c3_eg_y;
  c3_eml_scalar_eg(chartInstance);
  c3_d_yk = c3_fg_y;
  c3_gg_y = c3_d_yk;
  c3_eml_scalar_eg(chartInstance);
  c3_d_maxval = muDoubleScalarMax(0.0, c3_gg_y);
  c3_jd_x = c3_d_maxval;
  c3_w4 = c3_jd_x;
  if (c3_w4 < 0.0) {
    c3_eml_error(chartInstance);
  }

  c3_kd_x = c3_w4;
  c3_w4 = c3_kd_x;
  c3_w4 = muDoubleScalarSqrt(c3_w4);
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, 75);
  c3_w[0] = c3_w1;
  c3_w[1] = c3_w2;
  c3_w[2] = c3_w3;
  c3_w[3] = c3_w4;
  _SFD_EML_CALL(0U, chartInstance->c3_sfEvent, -75);
  _SFD_SYMBOL_SCOPE_POP();
  for (c3_i9 = 0; c3_i9 < 4; c3_i9++) {
    (*c3_b_w)[c3_i9] = c3_w[c3_i9];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c3_sfEvent);
}

static void initSimStructsc3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void registerMessagesc3_quad_control_sim_q
  (SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshallOut(void *chartInstanceVoid, void *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i10;
  real_T c3_b_inData[4];
  int32_T c3_i11;
  real_T c3_u[4];
  const mxArray *c3_y = NULL;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i10 = 0; c3_i10 < 4; c3_i10++) {
    c3_b_inData[c3_i10] = (*(real_T (*)[4])c3_inData)[c3_i10];
  }

  for (c3_i11 = 0; c3_i11 < 4; c3_i11++) {
    c3_u[c3_i11] = c3_b_inData[c3_i11];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static void c3_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_w, const char_T *c3_identifier, real_T c3_y
  [4])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_w), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_w);
}

static void c3_b_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real_T c3_y[4])
{
  real_T c3_dv1[4];
  int32_T c3_i12;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv1, 1, 0, 0U, 1, 0U, 1, 4);
  for (c3_i12 = 0; c3_i12 < 4; c3_i12++) {
    c3_y[c3_i12] = c3_dv1[c3_i12];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_w;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y[4];
  int32_T c3_i13;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_w = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_w), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_w);
  for (c3_i13 = 0; c3_i13 < 4; c3_i13++) {
    (*(real_T (*)[4])c3_outData)[c3_i13] = c3_y[c3_i13];
  }

  sf_mex_destroy(&c3_mxArrayInData);
}

static const mxArray *c3_b_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i14;
  real_T c3_b_inData[15];
  int32_T c3_i15;
  real_T c3_u[15];
  const mxArray *c3_y = NULL;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i14 = 0; c3_i14 < 15; c3_i14++) {
    c3_b_inData[c3_i14] = (*(real_T (*)[15])c3_inData)[c3_i14];
  }

  for (c3_i15 = 0; c3_i15 < 15; c3_i15++) {
    c3_u[c3_i15] = c3_b_inData[c3_i15];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 15), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static const mxArray *c3_c_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  real_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(real_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static const mxArray *c3_d_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i16;
  real_T c3_b_inData[14];
  int32_T c3_i17;
  real_T c3_u[14];
  const mxArray *c3_y = NULL;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i16 = 0; c3_i16 < 14; c3_i16++) {
    c3_b_inData[c3_i16] = (*(real_T (*)[14])c3_inData)[c3_i16];
  }

  for (c3_i17 = 0; c3_i17 < 14; c3_i17++) {
    c3_u[c3_i17] = c3_b_inData[c3_i17];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 14), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static const mxArray *c3_e_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_i18;
  real_T c3_b_inData[5];
  int32_T c3_i19;
  real_T c3_u[5];
  const mxArray *c3_y = NULL;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  for (c3_i18 = 0; c3_i18 < 5; c3_i18++) {
    c3_b_inData[c3_i18] = (*(real_T (*)[5])c3_inData)[c3_i18];
  }

  for (c3_i19 = 0; c3_i19 < 5; c3_i19++) {
    c3_u[c3_i19] = c3_b_inData[c3_i19];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 0, 0U, 1U, 0U, 1, 5), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static real_T c3_c_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_y;
  real_T c3_d0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_d0, 1, 0, 0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_nargout;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  real_T c3_y;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_nargout = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_nargout), &c3_thisId);
  sf_mex_destroy(&c3_nargout);
  *(real_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

const mxArray *sf_c3_quad_control_sim_q_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo;
  c3_ResolvedFunctionInfo c3_info[39];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i20;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  c3_info_helper(c3_info);
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 39), FALSE);
  for (c3_i20 = 0; c3_i20 < 39; c3_i20++) {
    c3_r0 = &c3_info[c3_i20];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context", "nameCaptureInfo",
                    c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name", "nameCaptureInfo", c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved", "nameCaptureInfo",
                    c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c3_i20);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c3_i20);
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[39])
{
  c3_info[0].context = "";
  c3_info[0].name = "mtimes";
  c3_info[0].dominantType = "double";
  c3_info[0].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[0].fileTimeLo = 1289552092U;
  c3_info[0].fileTimeHi = 0U;
  c3_info[0].mFileTimeLo = 0U;
  c3_info[0].mFileTimeHi = 0U;
  c3_info[1].context = "";
  c3_info[1].name = "norm";
  c3_info[1].dominantType = "double";
  c3_info[1].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m";
  c3_info[1].fileTimeLo = 1336554494U;
  c3_info[1].fileTimeHi = 0U;
  c3_info[1].mFileTimeLo = 0U;
  c3_info[1].mFileTimeHi = 0U;
  c3_info[2].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c3_info[2].name = "eml_index_class";
  c3_info[2].dominantType = "";
  c3_info[2].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[2].fileTimeLo = 1323202978U;
  c3_info[2].fileTimeHi = 0U;
  c3_info[2].mFileTimeLo = 0U;
  c3_info[2].mFileTimeHi = 0U;
  c3_info[3].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c3_info[3].name = "eml_xnrm2";
  c3_info[3].dominantType = "double";
  c3_info[3].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c3_info[3].fileTimeLo = 1299109176U;
  c3_info[3].fileTimeHi = 0U;
  c3_info[3].mFileTimeLo = 0U;
  c3_info[3].mFileTimeHi = 0U;
  c3_info[4].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c3_info[4].name = "eml_blas_inline";
  c3_info[4].dominantType = "";
  c3_info[4].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c3_info[4].fileTimeLo = 1299109168U;
  c3_info[4].fileTimeHi = 0U;
  c3_info[4].mFileTimeLo = 0U;
  c3_info[4].mFileTimeHi = 0U;
  c3_info[5].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c3_info[5].name = "eml_index_class";
  c3_info[5].dominantType = "";
  c3_info[5].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[5].fileTimeLo = 1323202978U;
  c3_info[5].fileTimeHi = 0U;
  c3_info[5].mFileTimeLo = 0U;
  c3_info[5].mFileTimeHi = 0U;
  c3_info[6].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c3_info[6].name = "eml_refblas_xnrm2";
  c3_info[6].dominantType = "double";
  c3_info[6].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[6].fileTimeLo = 1299109184U;
  c3_info[6].fileTimeHi = 0U;
  c3_info[6].mFileTimeLo = 0U;
  c3_info[6].mFileTimeHi = 0U;
  c3_info[7].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[7].name = "realmin";
  c3_info[7].dominantType = "char";
  c3_info[7].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c3_info[7].fileTimeLo = 1307683642U;
  c3_info[7].fileTimeHi = 0U;
  c3_info[7].mFileTimeLo = 0U;
  c3_info[7].mFileTimeHi = 0U;
  c3_info[8].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m";
  c3_info[8].name = "eml_realmin";
  c3_info[8].dominantType = "char";
  c3_info[8].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c3_info[8].fileTimeLo = 1307683644U;
  c3_info[8].fileTimeHi = 0U;
  c3_info[8].mFileTimeLo = 0U;
  c3_info[8].mFileTimeHi = 0U;
  c3_info[9].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c3_info[9].name = "eml_float_model";
  c3_info[9].dominantType = "char";
  c3_info[9].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c3_info[9].fileTimeLo = 1326760396U;
  c3_info[9].fileTimeHi = 0U;
  c3_info[9].mFileTimeLo = 0U;
  c3_info[9].mFileTimeHi = 0U;
  c3_info[10].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[10].name = "eml_index_class";
  c3_info[10].dominantType = "";
  c3_info[10].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[10].fileTimeLo = 1323202978U;
  c3_info[10].fileTimeHi = 0U;
  c3_info[10].mFileTimeLo = 0U;
  c3_info[10].mFileTimeHi = 0U;
  c3_info[11].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[11].name = "eml_index_minus";
  c3_info[11].dominantType = "double";
  c3_info[11].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c3_info[11].fileTimeLo = 1286851178U;
  c3_info[11].fileTimeHi = 0U;
  c3_info[11].mFileTimeLo = 0U;
  c3_info[11].mFileTimeHi = 0U;
  c3_info[12].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c3_info[12].name = "eml_index_class";
  c3_info[12].dominantType = "";
  c3_info[12].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[12].fileTimeLo = 1323202978U;
  c3_info[12].fileTimeHi = 0U;
  c3_info[12].mFileTimeLo = 0U;
  c3_info[12].mFileTimeHi = 0U;
  c3_info[13].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[13].name = "eml_index_times";
  c3_info[13].dominantType = "coder.internal.indexInt";
  c3_info[13].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c3_info[13].fileTimeLo = 1286851180U;
  c3_info[13].fileTimeHi = 0U;
  c3_info[13].mFileTimeLo = 0U;
  c3_info[13].mFileTimeHi = 0U;
  c3_info[14].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c3_info[14].name = "eml_index_class";
  c3_info[14].dominantType = "";
  c3_info[14].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[14].fileTimeLo = 1323202978U;
  c3_info[14].fileTimeHi = 0U;
  c3_info[14].mFileTimeLo = 0U;
  c3_info[14].mFileTimeHi = 0U;
  c3_info[15].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[15].name = "eml_index_plus";
  c3_info[15].dominantType = "coder.internal.indexInt";
  c3_info[15].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c3_info[15].fileTimeLo = 1286851178U;
  c3_info[15].fileTimeHi = 0U;
  c3_info[15].mFileTimeLo = 0U;
  c3_info[15].mFileTimeHi = 0U;
  c3_info[16].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c3_info[16].name = "eml_index_class";
  c3_info[16].dominantType = "";
  c3_info[16].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[16].fileTimeLo = 1323202978U;
  c3_info[16].fileTimeHi = 0U;
  c3_info[16].mFileTimeLo = 0U;
  c3_info[16].mFileTimeHi = 0U;
  c3_info[17].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[17].name = "eml_int_forloop_overflow_check";
  c3_info[17].dominantType = "";
  c3_info[17].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c3_info[17].fileTimeLo = 1346542740U;
  c3_info[17].fileTimeHi = 0U;
  c3_info[17].mFileTimeLo = 0U;
  c3_info[17].mFileTimeHi = 0U;
  c3_info[18].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c3_info[18].name = "intmax";
  c3_info[18].dominantType = "char";
  c3_info[18].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c3_info[18].fileTimeLo = 1311287716U;
  c3_info[18].fileTimeHi = 0U;
  c3_info[18].mFileTimeLo = 0U;
  c3_info[18].mFileTimeHi = 0U;
  c3_info[19].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c3_info[19].name = "abs";
  c3_info[19].dominantType = "double";
  c3_info[19].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c3_info[19].fileTimeLo = 1343862766U;
  c3_info[19].fileTimeHi = 0U;
  c3_info[19].mFileTimeLo = 0U;
  c3_info[19].mFileTimeHi = 0U;
  c3_info[20].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c3_info[20].name = "eml_scalar_abs";
  c3_info[20].dominantType = "double";
  c3_info[20].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c3_info[20].fileTimeLo = 1286851112U;
  c3_info[20].fileTimeHi = 0U;
  c3_info[20].mFileTimeLo = 0U;
  c3_info[20].mFileTimeHi = 0U;
  c3_info[21].context = "";
  c3_info[21].name = "mrdivide";
  c3_info[21].dominantType = "double";
  c3_info[21].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[21].fileTimeLo = 1357983948U;
  c3_info[21].fileTimeHi = 0U;
  c3_info[21].mFileTimeLo = 1319762366U;
  c3_info[21].mFileTimeHi = 0U;
  c3_info[22].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c3_info[22].name = "rdivide";
  c3_info[22].dominantType = "double";
  c3_info[22].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[22].fileTimeLo = 1346542788U;
  c3_info[22].fileTimeHi = 0U;
  c3_info[22].mFileTimeLo = 0U;
  c3_info[22].mFileTimeHi = 0U;
  c3_info[23].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[23].name = "eml_scalexp_compatible";
  c3_info[23].dominantType = "double";
  c3_info[23].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c3_info[23].fileTimeLo = 1286851196U;
  c3_info[23].fileTimeHi = 0U;
  c3_info[23].mFileTimeLo = 0U;
  c3_info[23].mFileTimeHi = 0U;
  c3_info[24].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[24].name = "eml_div";
  c3_info[24].dominantType = "double";
  c3_info[24].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[24].fileTimeLo = 1313380210U;
  c3_info[24].fileTimeHi = 0U;
  c3_info[24].mFileTimeLo = 0U;
  c3_info[24].mFileTimeHi = 0U;
  c3_info[25].context = "";
  c3_info[25].name = "sqrt";
  c3_info[25].dominantType = "double";
  c3_info[25].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[25].fileTimeLo = 1343862786U;
  c3_info[25].fileTimeHi = 0U;
  c3_info[25].mFileTimeLo = 0U;
  c3_info[25].mFileTimeHi = 0U;
  c3_info[26].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[26].name = "eml_error";
  c3_info[26].dominantType = "char";
  c3_info[26].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[26].fileTimeLo = 1343862758U;
  c3_info[26].fileTimeHi = 0U;
  c3_info[26].mFileTimeLo = 0U;
  c3_info[26].mFileTimeHi = 0U;
  c3_info[27].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c3_info[27].name = "eml_scalar_sqrt";
  c3_info[27].dominantType = "double";
  c3_info[27].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c3_info[27].fileTimeLo = 1286851138U;
  c3_info[27].fileTimeHi = 0U;
  c3_info[27].mFileTimeLo = 0U;
  c3_info[27].mFileTimeHi = 0U;
  c3_info[28].context = "";
  c3_info[28].name = "acos";
  c3_info[28].dominantType = "double";
  c3_info[28].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/acos.m";
  c3_info[28].fileTimeLo = 1343862766U;
  c3_info[28].fileTimeHi = 0U;
  c3_info[28].mFileTimeLo = 0U;
  c3_info[28].mFileTimeHi = 0U;
  c3_info[29].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/acos.m";
  c3_info[29].name = "eml_error";
  c3_info[29].dominantType = "char";
  c3_info[29].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c3_info[29].fileTimeLo = 1343862758U;
  c3_info[29].fileTimeHi = 0U;
  c3_info[29].mFileTimeLo = 0U;
  c3_info[29].mFileTimeHi = 0U;
  c3_info[30].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/acos.m";
  c3_info[30].name = "eml_scalar_acos";
  c3_info[30].dominantType = "double";
  c3_info[30].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_acos.m";
  c3_info[30].fileTimeLo = 1343862776U;
  c3_info[30].fileTimeHi = 0U;
  c3_info[30].mFileTimeLo = 0U;
  c3_info[30].mFileTimeHi = 0U;
  c3_info[31].context = "";
  c3_info[31].name = "cos";
  c3_info[31].dominantType = "double";
  c3_info[31].resolved = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c3_info[31].fileTimeLo = 1343862772U;
  c3_info[31].fileTimeHi = 0U;
  c3_info[31].mFileTimeLo = 0U;
  c3_info[31].mFileTimeHi = 0U;
  c3_info[32].context = "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m";
  c3_info[32].name = "eml_scalar_cos";
  c3_info[32].dominantType = "double";
  c3_info[32].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c3_info[32].fileTimeLo = 1286851122U;
  c3_info[32].fileTimeHi = 0U;
  c3_info[32].mFileTimeLo = 0U;
  c3_info[32].mFileTimeHi = 0U;
  c3_info[33].context = "";
  c3_info[33].name = "max";
  c3_info[33].dominantType = "double";
  c3_info[33].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c3_info[33].fileTimeLo = 1311287716U;
  c3_info[33].fileTimeHi = 0U;
  c3_info[33].mFileTimeLo = 0U;
  c3_info[33].mFileTimeHi = 0U;
  c3_info[34].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m";
  c3_info[34].name = "eml_min_or_max";
  c3_info[34].dominantType = "char";
  c3_info[34].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c3_info[34].fileTimeLo = 1334103890U;
  c3_info[34].fileTimeHi = 0U;
  c3_info[34].mFileTimeLo = 0U;
  c3_info[34].mFileTimeHi = 0U;
  c3_info[35].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c3_info[35].name = "eml_scalar_eg";
  c3_info[35].dominantType = "double";
  c3_info[35].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[35].fileTimeLo = 1286851196U;
  c3_info[35].fileTimeHi = 0U;
  c3_info[35].mFileTimeLo = 0U;
  c3_info[35].mFileTimeHi = 0U;
  c3_info[36].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c3_info[36].name = "eml_scalexp_alloc";
  c3_info[36].dominantType = "double";
  c3_info[36].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c3_info[36].fileTimeLo = 1352457260U;
  c3_info[36].fileTimeHi = 0U;
  c3_info[36].mFileTimeLo = 0U;
  c3_info[36].mFileTimeHi = 0U;
  c3_info[37].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c3_info[37].name = "eml_index_class";
  c3_info[37].dominantType = "";
  c3_info[37].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c3_info[37].fileTimeLo = 1323202978U;
  c3_info[37].fileTimeHi = 0U;
  c3_info[37].mFileTimeLo = 0U;
  c3_info[37].mFileTimeHi = 0U;
  c3_info[38].context =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c3_info[38].name = "eml_scalar_eg";
  c3_info[38].dominantType = "double";
  c3_info[38].resolved =
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c3_info[38].fileTimeLo = 1286851196U;
  c3_info[38].fileTimeHi = 0U;
  c3_info[38].mFileTimeLo = 0U;
  c3_info[38].mFileTimeHi = 0U;
}

static real_T c3_norm(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c3_x[4])
{
  real_T c3_y;
  real_T c3_scale;
  int32_T c3_k;
  int32_T c3_b_k;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_absxk;
  real_T c3_t;
  c3_y = 0.0;
  c3_scale = 2.2250738585072014E-308;
  for (c3_k = 1; c3_k < 5; c3_k++) {
    c3_b_k = c3_k;
    c3_b_x = c3_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c3_b_k), 1, 4, 1, 0) - 1];
    c3_c_x = c3_b_x;
    c3_absxk = muDoubleScalarAbs(c3_c_x);
    if (c3_absxk > c3_scale) {
      c3_t = c3_scale / c3_absxk;
      c3_y = 1.0 + c3_y * c3_t * c3_t;
      c3_scale = c3_absxk;
    } else {
      c3_t = c3_absxk / c3_scale;
      c3_y += c3_t * c3_t;
    }
  }

  return c3_scale * muDoubleScalarSqrt(c3_y);
}

static real_T c3_sqrt(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_b_sqrt(chartInstance, &c3_b_x);
  return c3_b_x;
}

static void c3_eml_error(SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
  int32_T c3_i21;
  static char_T c3_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[30];
  const mxArray *c3_y = NULL;
  int32_T c3_i22;
  static char_T c3_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c3_b_u[4];
  const mxArray *c3_b_y = NULL;
  for (c3_i21 = 0; c3_i21 < 30; c3_i21++) {
    c3_u[c3_i21] = c3_cv0[c3_i21];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c3_i22 = 0; c3_i22 < 4; c3_i22++) {
    c3_b_u[c3_i22] = c3_cv1[c3_i22];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c3_y, 14, c3_b_y));
}

static real_T c3_acos(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T c3_x)
{
  real_T c3_b_x;
  c3_b_x = c3_x;
  c3_b_acos(chartInstance, &c3_b_x);
  return c3_b_x;
}

static void c3_b_eml_error(SFc3_quad_control_sim_qInstanceStruct *chartInstance)
{
  int32_T c3_i23;
  static char_T c3_cv2[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c3_u[30];
  const mxArray *c3_y = NULL;
  int32_T c3_i24;
  static char_T c3_cv3[4] = { 'a', 'c', 'o', 's' };

  char_T c3_b_u[4];
  const mxArray *c3_b_y = NULL;
  for (c3_i23 = 0; c3_i23 < 30; c3_i23++) {
    c3_u[c3_i23] = c3_cv2[c3_i23];
  }

  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", c3_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c3_i24 = 0; c3_i24 < 4; c3_i24++) {
    c3_b_u[c3_i24] = c3_cv3[c3_i24];
  }

  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", c3_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c3_y, 14, c3_b_y));
}

static void c3_eml_scalar_eg(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance)
{
}

static const mxArray *c3_f_sf_marshallOut(void *chartInstanceVoid, void
  *c3_inData)
{
  const mxArray *c3_mxArrayOutData = NULL;
  int32_T c3_u;
  const mxArray *c3_y = NULL;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_mxArrayOutData = NULL;
  c3_u = *(int32_T *)c3_inData;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_create("y", &c3_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c3_mxArrayOutData, c3_y, FALSE);
  return c3_mxArrayOutData;
}

static int32_T c3_d_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  int32_T c3_y;
  int32_T c3_i25;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_i25, 1, 6, 0U, 0, 0U, 0);
  c3_y = c3_i25;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c3_mxArrayInData, const char_T *c3_varName, void *c3_outData)
{
  const mxArray *c3_b_sfEvent;
  const char_T *c3_identifier;
  emlrtMsgIdentifier c3_thisId;
  int32_T c3_y;
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)chartInstanceVoid;
  c3_b_sfEvent = sf_mex_dup(c3_mxArrayInData);
  c3_identifier = c3_varName;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_sfEvent),
    &c3_thisId);
  sf_mex_destroy(&c3_b_sfEvent);
  *(int32_T *)c3_outData = c3_y;
  sf_mex_destroy(&c3_mxArrayInData);
}

static uint8_T c3_e_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_quad_control_sim_q, const
  char_T *c3_identifier)
{
  uint8_T c3_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = c3_identifier;
  c3_thisId.fParent = NULL;
  c3_y = c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_quad_control_sim_q), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_quad_control_sim_q);
  return c3_y;
}

static uint8_T c3_f_emlrt_marshallIn(SFc3_quad_control_sim_qInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_y;
  uint8_T c3_u0;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_u0;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_b_sqrt(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T *c3_x)
{
  if (*c3_x < 0.0) {
    c3_eml_error(chartInstance);
  }

  *c3_x = muDoubleScalarSqrt(*c3_x);
}

static void c3_b_acos(SFc3_quad_control_sim_qInstanceStruct *chartInstance,
                      real_T *c3_x)
{
  boolean_T guard1 = FALSE;
  guard1 = FALSE;
  if (*c3_x < -1.0) {
    guard1 = TRUE;
  } else {
    if (1.0 < *c3_x) {
      guard1 = TRUE;
    }
  }

  if (guard1 == TRUE) {
    c3_b_eml_error(chartInstance);
  }

  *c3_x = muDoubleScalarAcos(*c3_x);
}

static void init_dsm_address_info(SFc3_quad_control_sim_qInstanceStruct
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

void sf_c3_quad_control_sim_q_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3491549459U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2260737818U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(733783632U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2445726044U);
}

mxArray *sf_c3_quad_control_sim_q_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("X68igPGpTaBmnvcXJ1msgG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
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
      pr[0] = (double)(14);
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
      pr[0] = (double)(15);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(4);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_quad_control_sim_q_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c3_quad_control_sim_q(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"w\",},{M[8],M[0],T\"is_active_c3_quad_control_sim_q\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_quad_control_sim_q_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc3_quad_control_sim_qInstanceStruct *chartInstance;
    chartInstance = (SFc3_quad_control_sim_qInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _quad_control_sim_qMachineNumber_,
           3,
           1,
           1,
           6,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"inputs");
          _SFD_SET_DATA_PROPS(1,1,1,0,"states");
          _SFD_SET_DATA_PROPS(2,2,0,1,"w");
          _SFD_SET_DATA_PROPS(3,1,1,0,"pq");
          _SFD_SET_DATA_PROPS(4,1,1,0,"pw");
          _SFD_SET_DATA_PROPS(5,1,1,0,"params");
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
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2263);
        _SFD_CV_INIT_EML_IF(0,1,0,789,802,-1,888);
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
          dimVector[0]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 14;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_sf_marshallOut,(MexInFcnForType)
            c3_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c3_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 15;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c3_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c3_pq;
          real_T *c3_pw;
          real_T (*c3_inputs)[5];
          real_T (*c3_states)[14];
          real_T (*c3_w)[4];
          real_T (*c3_params)[15];
          c3_params = (real_T (*)[15])ssGetInputPortSignal(chartInstance->S, 4);
          c3_pw = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c3_pq = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c3_w = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
          c3_states = (real_T (*)[14])ssGetInputPortSignal(chartInstance->S, 1);
          c3_inputs = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c3_inputs);
          _SFD_SET_DATA_VALUE_PTR(1U, *c3_states);
          _SFD_SET_DATA_VALUE_PTR(2U, *c3_w);
          _SFD_SET_DATA_VALUE_PTR(3U, c3_pq);
          _SFD_SET_DATA_VALUE_PTR(4U, c3_pw);
          _SFD_SET_DATA_VALUE_PTR(5U, *c3_params);
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
  return "kQRy8wuWuOiMgjzVU6a65G";
}

static void sf_opaque_initialize_c3_quad_control_sim_q(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
  initialize_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_quad_control_sim_q(void *chartInstanceVar)
{
  enable_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c3_quad_control_sim_q(void *chartInstanceVar)
{
  disable_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c3_quad_control_sim_q(void *chartInstanceVar)
{
  sf_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c3_quad_control_sim_q(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c3_quad_control_sim_q
    ((SFc3_quad_control_sim_qInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_quad_control_sim_q();/* state var info */
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

extern void sf_internal_set_sim_state_c3_quad_control_sim_q(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c3_quad_control_sim_q();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c3_quad_control_sim_q(SimStruct* S)
{
  return sf_internal_get_sim_state_c3_quad_control_sim_q(S);
}

static void sf_opaque_set_sim_state_c3_quad_control_sim_q(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c3_quad_control_sim_q(S, st);
}

static void sf_opaque_terminate_c3_quad_control_sim_q(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_quad_control_sim_qInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_quad_control_sim_q_optimization_info();
    }

    finalize_c3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_quad_control_sim_q((SFc3_quad_control_sim_qInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_quad_control_sim_q(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_quad_control_sim_q
      ((SFc3_quad_control_sim_qInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c3_quad_control_sim_q(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_quad_control_sim_q_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,3,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1928334644U));
  ssSetChecksum1(S,(4070190481U));
  ssSetChecksum2(S,(1875267965U));
  ssSetChecksum3(S,(1788068038U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_quad_control_sim_q(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_quad_control_sim_q(SimStruct *S)
{
  SFc3_quad_control_sim_qInstanceStruct *chartInstance;
  chartInstance = (SFc3_quad_control_sim_qInstanceStruct *)utMalloc(sizeof
    (SFc3_quad_control_sim_qInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc3_quad_control_sim_qInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c3_quad_control_sim_q;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_quad_control_sim_q;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c3_quad_control_sim_q;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_quad_control_sim_q;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c3_quad_control_sim_q;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_quad_control_sim_q;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_quad_control_sim_q;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_quad_control_sim_q;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_quad_control_sim_q;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_quad_control_sim_q;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c3_quad_control_sim_q;
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

void c3_quad_control_sim_q_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_quad_control_sim_q(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_quad_control_sim_q(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_quad_control_sim_q(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_quad_control_sim_q_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
