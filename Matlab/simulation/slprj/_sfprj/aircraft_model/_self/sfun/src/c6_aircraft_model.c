/* Include files */

#include <stddef.h>
#include "blas.h"
#include "aircraft_model_sfun.h"
#include "c6_aircraft_model.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "aircraft_model_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c6_debug_family_names[11] = { "wyptsi", "size_waypoint_list",
  "wypts0", "nargin", "nargout", "change_current_waypoint", "current_waypoint",
  "wypt_send_1", "wypt_send_2", "wypt_send_3", "current_waypoint1" };

/* Function Declarations */
static void initialize_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void initialize_params_c6_aircraft_model
  (SFc6_aircraft_modelInstanceStruct *chartInstance);
static void enable_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void disable_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void c6_update_debugger_state_c6_aircraft_model
  (SFc6_aircraft_modelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_aircraft_model
  (SFc6_aircraft_modelInstanceStruct *chartInstance);
static void set_sim_state_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_st);
static void finalize_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void sf_gateway_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void mdl_start_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void initSimStructsc6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static real_T c6_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_b_current_waypoint1, const char_T
  *c6_identifier);
static real_T c6_b_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_emlrt_marshallOut(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const c6_wypt_send_3 c6_u);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static c6_wypt_send_3 c6_c_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_c_wypt_send_3, const char_T *c6_identifier);
static c6_wypt_send_3 c6_d_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_e_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_emlrt_marshallOut(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const c6_wypt_send_2 c6_u);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static c6_wypt_send_2 c6_f_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_c_wypt_send_2, const char_T *c6_identifier);
static c6_wypt_send_2 c6_g_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_emlrt_marshallOut(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const c6_wypt_send_1 c6_u);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static c6_wypt_send_1 c6_h_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_c_wypt_send_1, const char_T *c6_identifier);
static c6_wypt_send_1 c6_i_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_j_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_k_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_wypt_send_1 c6_y[5]);
static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_wypt_send_1_bus_io(void *chartInstanceVoid, void
  *c6_pData);
static const mxArray *c6_wypt_send_2_bus_io(void *chartInstanceVoid, void
  *c6_pData);
static const mxArray *c6_wypt_send_3_bus_io(void *chartInstanceVoid, void
  *c6_pData);
static uint8_T c6_l_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_aircraft_model, const char_T *
  c6_identifier);
static uint8_T c6_m_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void init_dsm_address_info(SFc6_aircraft_modelInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc6_aircraft_modelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c6_is_active_c6_aircraft_model = 0U;
}

static void initialize_params_c6_aircraft_model
  (SFc6_aircraft_modelInstanceStruct *chartInstance)
{
  const mxArray *c6_m0 = NULL;
  int32_T c6_i0;
  c6_wypt_send_1 c6_rv0[5];
  c6_wypt_send_1 *c6_r0;
  const mxArray *c6_mxField;
  int32_T c6_i1;
  c6_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c6_i0 = 0;
  while (c6_i0 < 5) {
    c6_r0 = &c6_rv0[_SFD_EML_ARRAY_BOUNDS_CHECK(0U, c6_i0, 0, 4, 1, 0)];
    c6_mxField = sf_mex_getfield(c6_m0, "active", "wypts0", c6_i0);
    sf_mex_import_named("wypts0", sf_mex_dup(c6_mxField), &c6_r0->active, 1, 0,
                        0U, 0, 0U, 0);
    c6_mxField = sf_mex_getfield(c6_m0, "wtype", "wypts0", c6_i0);
    sf_mex_import_named("wypts0", sf_mex_dup(c6_mxField), &c6_r0->wtype, 1, 0,
                        0U, 0, 0U, 0);
    c6_mxField = sf_mex_getfield(c6_m0, "p", "wypts0", c6_i0);
    sf_mex_import_named("wypts0", sf_mex_dup(c6_mxField), c6_r0->p, 1, 0, 0U, 1,
                        0U, 1, 2);
    c6_mxField = sf_mex_getfield(c6_m0, "param1", "wypts0", c6_i0);
    sf_mex_import_named("wypts0", sf_mex_dup(c6_mxField), &c6_r0->param1, 1, 0,
                        0U, 0, 0U, 0);
    c6_mxField = sf_mex_getfield(c6_m0, "param2", "wypts0", c6_i0);
    sf_mex_import_named("wypts0", sf_mex_dup(c6_mxField), &c6_r0->param2, 1, 0,
                        0U, 0, 0U, 0);
    c6_i0++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  sf_mex_destroy(&c6_m0);
  for (c6_i1 = 0; c6_i1 < 5; c6_i1++) {
    chartInstance->c6_wypts0[c6_i1] = c6_rv0[c6_i1];
  }
}

static void enable_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c6_update_debugger_state_c6_aircraft_model
  (SFc6_aircraft_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c6_aircraft_model
  (SFc6_aircraft_modelInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  real_T c6_hoistedGlobal;
  real_T c6_u;
  const mxArray *c6_b_y = NULL;
  c6_wypt_send_1 c6_r1;
  int32_T c6_i2;
  c6_wypt_send_2 c6_r2;
  int32_T c6_i3;
  c6_wypt_send_3 c6_r3;
  int32_T c6_i4;
  uint8_T c6_b_hoistedGlobal;
  uint8_T c6_b_u;
  const mxArray *c6_c_y = NULL;
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellmatrix(5, 1), false);
  c6_hoistedGlobal = *chartInstance->c6_current_waypoint1;
  c6_u = c6_hoistedGlobal;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  c6_r1.active = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[0];
  c6_r1.wtype = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[8];
  for (c6_i2 = 0; c6_i2 < 2; c6_i2++) {
    c6_r1.p[c6_i2] = ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[16])
      [c6_i2];
  }

  c6_r1.param1 = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[32];
  c6_r1.param2 = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[40];
  sf_mex_setcell(c6_y, 1, c6_c_emlrt_marshallOut(chartInstance, c6_r1));
  c6_r2.active = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[0];
  c6_r2.wtype = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[8];
  for (c6_i3 = 0; c6_i3 < 2; c6_i3++) {
    c6_r2.p[c6_i3] = ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[16])
      [c6_i3];
  }

  c6_r2.param1 = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[32];
  c6_r2.param2 = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[40];
  sf_mex_setcell(c6_y, 2, c6_b_emlrt_marshallOut(chartInstance, c6_r2));
  c6_r3.active = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[0];
  c6_r3.wtype = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[8];
  for (c6_i4 = 0; c6_i4 < 2; c6_i4++) {
    c6_r3.p[c6_i4] = ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[16])
      [c6_i4];
  }

  c6_r3.param1 = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[32];
  c6_r3.param2 = *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[40];
  sf_mex_setcell(c6_y, 3, c6_emlrt_marshallOut(chartInstance, c6_r3));
  c6_b_hoistedGlobal = chartInstance->c6_is_active_c6_aircraft_model;
  c6_b_u = c6_b_hoistedGlobal;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c6_y, 4, c6_c_y);
  sf_mex_assign(&c6_st, c6_y, false);
  return c6_st;
}

static void set_sim_state_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  c6_wypt_send_1 c6_r4;
  int32_T c6_i5;
  c6_wypt_send_2 c6_r5;
  int32_T c6_i6;
  c6_wypt_send_3 c6_r6;
  int32_T c6_i7;
  chartInstance->c6_doneDoubleBufferReInit = true;
  c6_u = sf_mex_dup(c6_st);
  *chartInstance->c6_current_waypoint1 = c6_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c6_u, 0)), "current_waypoint1");
  c6_r4 = c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)),
    "wypt_send_1");
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[0] = c6_r4.active;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[8] = c6_r4.wtype;
  for (c6_i5 = 0; c6_i5 < 2; c6_i5++) {
    ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[16])[c6_i5] =
      c6_r4.p[c6_i5];
  }

  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[32] = c6_r4.param1;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[40] = c6_r4.param2;
  c6_r5 = c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 2)),
    "wypt_send_2");
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[0] = c6_r5.active;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[8] = c6_r5.wtype;
  for (c6_i6 = 0; c6_i6 < 2; c6_i6++) {
    ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[16])[c6_i6] =
      c6_r5.p[c6_i6];
  }

  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[32] = c6_r5.param1;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[40] = c6_r5.param2;
  c6_r6 = c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 3)),
    "wypt_send_3");
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[0] = c6_r6.active;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[8] = c6_r6.wtype;
  for (c6_i7 = 0; c6_i7 < 2; c6_i7++) {
    ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[16])[c6_i7] =
      c6_r6.p[c6_i7];
  }

  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[32] = c6_r6.param1;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[40] = c6_r6.param2;
  chartInstance->c6_is_active_c6_aircraft_model = c6_l_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 4)),
     "is_active_c6_aircraft_model");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_aircraft_model(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  real_T c6_hoistedGlobal;
  real_T c6_b_hoistedGlobal;
  real_T c6_b_change_current_waypoint;
  real_T c6_b_current_waypoint;
  uint32_T c6_debug_family_var_map[11];
  c6_wypt_send_1 c6_wyptsi;
  real_T c6_size_waypoint_list;
  c6_wypt_send_1 c6_c_wypts0[5];
  real_T c6_nargin = 3.0;
  real_T c6_nargout = 4.0;
  c6_wypt_send_1 c6_c_wypt_send_1;
  c6_wypt_send_2 c6_c_wypt_send_2;
  c6_wypt_send_3 c6_c_wypt_send_3;
  real_T c6_b_current_waypoint1;
  int32_T c6_i8;
  static c6_wypt_send_1 c6_d_wypts0[5] = { { 1.0, 0.0, { -40.0, 40.0 }, 30.0,
      0.0 }, { 1.0, 0.0, { 20.0, 100.0 }, 30.0, 0.0 }, { 1.0, 0.0, { 50.0, 200.0
      }, 30.0, 0.0 }, { 1.0, 0.0, { -50.0, 200.0 }, 30.0, 0.0 }, { 1.0, 0.0, { -
        80.0, 0.0 }, 30.0, 0.0 } };

  static c6_wypt_send_1 c6_r7 = { 0.0, 0.0, { 0.0, 0.0 }, 30.0, 0.0 };

  static c6_wypt_send_1 c6_r8 = { 1.0, 0.0, { -50.0, 200.0 }, 30.0, 0.0 };

  int32_T c6_i9;
  int32_T c6_i10;
  static c6_wypt_send_1 c6_r9 = { 1.0, 0.0, { -80.0, 0.0 }, 30.0, 0.0 };

  c6_wypt_send_1 c6_r10;
  int32_T c6_i11;
  int32_T c6_i12;
  int32_T c6_i13;
  int32_T c6_i14;
  int32_T c6_i15;
  int32_T c6_i16;
  int32_T c6_i17;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c6_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c6_change_current_waypoint, 1U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *chartInstance->c6_change_current_waypoint;
  c6_b_hoistedGlobal = *chartInstance->c6_current_waypoint;
  c6_b_change_current_waypoint = c6_hoistedGlobal;
  c6_b_current_waypoint = c6_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_wyptsi, 0U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_size_waypoint_list, 1U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_c_wypts0, 2U, c6_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 3U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 4U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_b_change_current_waypoint, 5U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_b_current_waypoint, 6U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_c_wypt_send_1, 7U,
    c6_d_sf_marshallOut, c6_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_c_wypt_send_2, 8U,
    c6_c_sf_marshallOut, c6_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_c_wypt_send_3, 9U,
    c6_b_sf_marshallOut, c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_b_current_waypoint1, 10U,
    c6_sf_marshallOut, c6_sf_marshallIn);
  for (c6_i8 = 0; c6_i8 < 5; c6_i8++) {
    c6_c_wypts0[c6_i8] = c6_d_wypts0[c6_i8];
  }

  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 15);
  c6_wyptsi = c6_r7;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 22);
  c6_size_waypoint_list = 5.0;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 29);
  if (CV_EML_IF(0, 1, 0, c6_b_change_current_waypoint != 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 32);
    if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c6_b_current_waypoint,
          c6_size_waypoint_list, -1, 0U, c6_b_current_waypoint ==
          c6_size_waypoint_list))) {
      CV_EML_MCDC(0, 1, 0, true);
      CV_EML_IF(0, 1, 1, true);
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 34);
      c6_b_current_waypoint = 1.0;
    } else {
      CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c6_size_waypoint_list,
        1.0, -1, 0U, c6_size_waypoint_list == 1.0));
      CV_EML_MCDC(0, 1, 0, false);
      CV_EML_IF(0, 1, 1, false);
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 38);
      c6_b_current_waypoint++;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 48);
  CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c6_size_waypoint_list, 1.0,
             -1, 0U, c6_size_waypoint_list == 1.0));
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 54);
  CV_EML_IF(0, 1, 3, CV_RELATIONAL_EVAL(4U, 0U, 3, c6_size_waypoint_list, 2.0,
             -1, 0U, c6_size_waypoint_list == 2.0));
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 72);
  if (CV_EML_IF(0, 1, 5, CV_RELATIONAL_EVAL(4U, 0U, 5, c6_b_current_waypoint,
        c6_size_waypoint_list, -1, 0U, c6_b_current_waypoint ==
        c6_size_waypoint_list))) {
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 74);
    c6_c_wypt_send_1 = c6_r8;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 75);
    c6_c_wypt_send_2.active = 1.0;
    c6_c_wypt_send_2.wtype = 0.0;
    for (c6_i9 = 0; c6_i9 < 2; c6_i9++) {
      c6_c_wypt_send_2.p[c6_i9] = -80.0 + 80.0 * (real_T)c6_i9;
    }

    c6_c_wypt_send_2.param1 = 30.0;
    c6_c_wypt_send_2.param2 = 0.0;
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 76);
    c6_c_wypt_send_3.active = 1.0;
    c6_c_wypt_send_3.wtype = 0.0;
    for (c6_i10 = 0; c6_i10 < 2; c6_i10++) {
      c6_c_wypt_send_3.p[c6_i10] = -40.0 + 80.0 * (real_T)c6_i10;
    }

    c6_c_wypt_send_3.param1 = 30.0;
    c6_c_wypt_send_3.param2 = 0.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 78);
    if (CV_EML_IF(0, 1, 6, CV_RELATIONAL_EVAL(4U, 0U, 6, c6_b_current_waypoint,
          1.0, -1, 0U, c6_b_current_waypoint == 1.0))) {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 80);
      c6_c_wypt_send_1 = c6_r9;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 81);
      c6_r10 = c6_d_wypts0[_SFD_EML_ARRAY_BOUNDS_CHECK("wypts0", (int32_T)
        _SFD_INTEGER_CHECK("current_waypoint", c6_b_current_waypoint), 1, 5, 1,
        0) - 1];
      c6_c_wypt_send_2.active = c6_r10.active;
      c6_c_wypt_send_2.wtype = c6_r10.wtype;
      for (c6_i11 = 0; c6_i11 < 2; c6_i11++) {
        c6_c_wypt_send_2.p[c6_i11] = c6_r10.p[c6_i11];
      }

      c6_c_wypt_send_2.param1 = c6_r10.param1;
      c6_c_wypt_send_2.param2 = c6_r10.param2;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 82);
      c6_r10 = c6_d_wypts0[_SFD_EML_ARRAY_BOUNDS_CHECK("wypts0", (int32_T)
        _SFD_INTEGER_CHECK("current_waypoint+1", c6_b_current_waypoint + 1.0), 1,
        5, 1, 0) - 1];
      c6_c_wypt_send_3.active = c6_r10.active;
      c6_c_wypt_send_3.wtype = c6_r10.wtype;
      for (c6_i12 = 0; c6_i12 < 2; c6_i12++) {
        c6_c_wypt_send_3.p[c6_i12] = c6_r10.p[c6_i12];
      }

      c6_c_wypt_send_3.param1 = c6_r10.param1;
      c6_c_wypt_send_3.param2 = c6_r10.param2;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 86);
      c6_c_wypt_send_1 = c6_d_wypts0[_SFD_EML_ARRAY_BOUNDS_CHECK("wypts0",
        (int32_T)_SFD_INTEGER_CHECK("current_waypoint-1", c6_b_current_waypoint
        - 1.0), 1, 5, 1, 0) - 1];
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 87);
      c6_r10 = c6_d_wypts0[_SFD_EML_ARRAY_BOUNDS_CHECK("wypts0", (int32_T)
        _SFD_INTEGER_CHECK("current_waypoint", c6_b_current_waypoint), 1, 5, 1,
        0) - 1];
      c6_c_wypt_send_2.active = c6_r10.active;
      c6_c_wypt_send_2.wtype = c6_r10.wtype;
      for (c6_i13 = 0; c6_i13 < 2; c6_i13++) {
        c6_c_wypt_send_2.p[c6_i13] = c6_r10.p[c6_i13];
      }

      c6_c_wypt_send_2.param1 = c6_r10.param1;
      c6_c_wypt_send_2.param2 = c6_r10.param2;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 88);
      c6_r10 = c6_d_wypts0[_SFD_EML_ARRAY_BOUNDS_CHECK("wypts0", (int32_T)
        _SFD_INTEGER_CHECK("current_waypoint+1", c6_b_current_waypoint + 1.0), 1,
        5, 1, 0) - 1];
      c6_c_wypt_send_3.active = c6_r10.active;
      c6_c_wypt_send_3.wtype = c6_r10.wtype;
      for (c6_i14 = 0; c6_i14 < 2; c6_i14++) {
        c6_c_wypt_send_3.p[c6_i14] = c6_r10.p[c6_i14];
      }

      c6_c_wypt_send_3.param1 = c6_r10.param1;
      c6_c_wypt_send_3.param2 = c6_r10.param2;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 94);
  c6_b_current_waypoint1 = c6_b_current_waypoint;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -94);
  _SFD_SYMBOL_SCOPE_POP();
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[0] =
    c6_c_wypt_send_1.active;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[8] =
    c6_c_wypt_send_1.wtype;
  for (c6_i15 = 0; c6_i15 < 2; c6_i15++) {
    ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[16])[c6_i15] =
      c6_c_wypt_send_1.p[c6_i15];
  }

  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[32] =
    c6_c_wypt_send_1.param1;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_1)[40] =
    c6_c_wypt_send_1.param2;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[0] =
    c6_c_wypt_send_2.active;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[8] =
    c6_c_wypt_send_2.wtype;
  for (c6_i16 = 0; c6_i16 < 2; c6_i16++) {
    ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[16])[c6_i16] =
      c6_c_wypt_send_2.p[c6_i16];
  }

  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[32] =
    c6_c_wypt_send_2.param1;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_2)[40] =
    c6_c_wypt_send_2.param2;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[0] =
    c6_c_wypt_send_3.active;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[8] =
    c6_c_wypt_send_3.wtype;
  for (c6_i17 = 0; c6_i17 < 2; c6_i17++) {
    ((real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[16])[c6_i17] =
      c6_c_wypt_send_3.p[c6_i17];
  }

  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[32] =
    c6_c_wypt_send_3.param1;
  *(real_T *)&((char_T *)chartInstance->c6_b_wypt_send_3)[40] =
    c6_c_wypt_send_3.param2;
  *chartInstance->c6_current_waypoint1 = c6_b_current_waypoint1;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c6_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_aircraft_modelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c6_current_waypoint, 5U);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c6_current_waypoint1, 6U);
}

static void mdl_start_c6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc6_aircraft_model(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber)
{
  (void)c6_machineNumber;
  (void)c6_chartNumber;
  (void)c6_instanceNumber;
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static real_T c6_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_b_current_waypoint1, const char_T
  *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_current_waypoint1),
    &c6_thisId);
  sf_mex_destroy(&c6_b_current_waypoint1);
  return c6_y;
}

static real_T c6_b_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_current_waypoint1;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_b_current_waypoint1 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_current_waypoint1),
    &c6_thisId);
  sf_mex_destroy(&c6_b_current_waypoint1);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_emlrt_marshallOut(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const c6_wypt_send_3 c6_u)
{
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  int32_T c6_i18;
  real_T c6_d_u[2];
  const mxArray *c6_d_y = NULL;
  real_T c6_e_u;
  const mxArray *c6_e_y = NULL;
  real_T c6_f_u;
  const mxArray *c6_f_y = NULL;
  (void)chartInstance;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c6_b_u = c6_u.active;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_b_y, "active", "active", 0);
  c6_c_u = c6_u.wtype;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_c_y, "wtype", "wtype", 0);
  for (c6_i18 = 0; c6_i18 < 2; c6_i18++) {
    c6_d_u[c6_i18] = c6_u.p[c6_i18];
  }

  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_addfield(c6_y, c6_d_y, "p", "p", 0);
  c6_e_u = c6_u.param1;
  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_e_y, "param1", "param1", 0);
  c6_f_u = c6_u.param2;
  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_f_y, "param2", "param2", 0);
  return c6_y;
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  sf_mex_assign(&c6_mxArrayOutData, c6_emlrt_marshallOut(chartInstance,
    *(c6_wypt_send_3 *)c6_inData), false);
  return c6_mxArrayOutData;
}

static c6_wypt_send_3 c6_c_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_c_wypt_send_3, const char_T *c6_identifier)
{
  c6_wypt_send_3 c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypt_send_3),
    &c6_thisId);
  sf_mex_destroy(&c6_c_wypt_send_3);
  return c6_y;
}

static c6_wypt_send_3 c6_d_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  c6_wypt_send_3 c6_y;
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[5] = { "active", "wtype", "p", "param1",
    "param2" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 5, c6_fieldNames, 0U, NULL);
  c6_thisId.fIdentifier = "active";
  c6_y.active = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "active", "active", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "wtype";
  c6_y.wtype = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "wtype", "wtype", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "p";
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "p", "p",
    0)), &c6_thisId, c6_y.p);
  c6_thisId.fIdentifier = "param1";
  c6_y.param1 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "param1", "param1", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "param2";
  c6_y.param2 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "param2", "param2", 0)), &c6_thisId);
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_e_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2])
{
  real_T c6_dv0[2];
  int32_T c6_i19;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv0, 1, 0, 0U, 1, 0U, 1, 2);
  for (c6_i19 = 0; c6_i19 < 2; c6_i19++) {
    c6_y[c6_i19] = c6_dv0[c6_i19];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_c_wypt_send_3;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  c6_wypt_send_3 c6_y;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_c_wypt_send_3 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypt_send_3),
    &c6_thisId);
  sf_mex_destroy(&c6_c_wypt_send_3);
  *(c6_wypt_send_3 *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_emlrt_marshallOut(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const c6_wypt_send_2 c6_u)
{
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  int32_T c6_i20;
  real_T c6_d_u[2];
  const mxArray *c6_d_y = NULL;
  real_T c6_e_u;
  const mxArray *c6_e_y = NULL;
  real_T c6_f_u;
  const mxArray *c6_f_y = NULL;
  (void)chartInstance;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c6_b_u = c6_u.active;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_b_y, "active", "active", 0);
  c6_c_u = c6_u.wtype;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_c_y, "wtype", "wtype", 0);
  for (c6_i20 = 0; c6_i20 < 2; c6_i20++) {
    c6_d_u[c6_i20] = c6_u.p[c6_i20];
  }

  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_addfield(c6_y, c6_d_y, "p", "p", 0);
  c6_e_u = c6_u.param1;
  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_e_y, "param1", "param1", 0);
  c6_f_u = c6_u.param2;
  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_f_y, "param2", "param2", 0);
  return c6_y;
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  sf_mex_assign(&c6_mxArrayOutData, c6_b_emlrt_marshallOut(chartInstance,
    *(c6_wypt_send_2 *)c6_inData), false);
  return c6_mxArrayOutData;
}

static c6_wypt_send_2 c6_f_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_c_wypt_send_2, const char_T *c6_identifier)
{
  c6_wypt_send_2 c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypt_send_2),
    &c6_thisId);
  sf_mex_destroy(&c6_c_wypt_send_2);
  return c6_y;
}

static c6_wypt_send_2 c6_g_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  c6_wypt_send_2 c6_y;
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[5] = { "active", "wtype", "p", "param1",
    "param2" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 5, c6_fieldNames, 0U, NULL);
  c6_thisId.fIdentifier = "active";
  c6_y.active = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "active", "active", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "wtype";
  c6_y.wtype = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "wtype", "wtype", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "p";
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "p", "p",
    0)), &c6_thisId, c6_y.p);
  c6_thisId.fIdentifier = "param1";
  c6_y.param1 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "param1", "param1", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "param2";
  c6_y.param2 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "param2", "param2", 0)), &c6_thisId);
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_c_wypt_send_2;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  c6_wypt_send_2 c6_y;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_c_wypt_send_2 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypt_send_2),
    &c6_thisId);
  sf_mex_destroy(&c6_c_wypt_send_2);
  *(c6_wypt_send_2 *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_emlrt_marshallOut(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const c6_wypt_send_1 c6_u)
{
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  int32_T c6_i21;
  real_T c6_d_u[2];
  const mxArray *c6_d_y = NULL;
  real_T c6_e_u;
  const mxArray *c6_e_y = NULL;
  real_T c6_f_u;
  const mxArray *c6_f_y = NULL;
  (void)chartInstance;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c6_b_u = c6_u.active;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_b_y, "active", "active", 0);
  c6_c_u = c6_u.wtype;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_c_y, "wtype", "wtype", 0);
  for (c6_i21 = 0; c6_i21 < 2; c6_i21++) {
    c6_d_u[c6_i21] = c6_u.p[c6_i21];
  }

  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_addfield(c6_y, c6_d_y, "p", "p", 0);
  c6_e_u = c6_u.param1;
  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_e_y, "param1", "param1", 0);
  c6_f_u = c6_u.param2;
  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c6_y, c6_f_y, "param2", "param2", 0);
  return c6_y;
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  sf_mex_assign(&c6_mxArrayOutData, c6_c_emlrt_marshallOut(chartInstance,
    *(c6_wypt_send_1 *)c6_inData), false);
  return c6_mxArrayOutData;
}

static c6_wypt_send_1 c6_h_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_c_wypt_send_1, const char_T *c6_identifier)
{
  c6_wypt_send_1 c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypt_send_1),
    &c6_thisId);
  sf_mex_destroy(&c6_c_wypt_send_1);
  return c6_y;
}

static c6_wypt_send_1 c6_i_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  c6_wypt_send_1 c6_y;
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[5] = { "active", "wtype", "p", "param1",
    "param2" };

  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 5, c6_fieldNames, 0U, NULL);
  c6_thisId.fIdentifier = "active";
  c6_y.active = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "active", "active", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "wtype";
  c6_y.wtype = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "wtype", "wtype", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "p";
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "p", "p",
    0)), &c6_thisId, c6_y.p);
  c6_thisId.fIdentifier = "param1";
  c6_y.param1 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "param1", "param1", 0)), &c6_thisId);
  c6_thisId.fIdentifier = "param2";
  c6_y.param2 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield
    (c6_u, "param2", "param2", 0)), &c6_thisId);
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_c_wypt_send_1;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  c6_wypt_send_1 c6_y;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_c_wypt_send_1 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypt_send_1),
    &c6_thisId);
  sf_mex_destroy(&c6_c_wypt_send_1);
  *(c6_wypt_send_1 *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData;
  int32_T c6_i22;
  c6_wypt_send_1 c6_b_inData[5];
  int32_T c6_i23;
  c6_wypt_send_1 c6_u[5];
  const mxArray *c6_y = NULL;
  static int32_T c6_iv0[1] = { 5 };

  int32_T c6_iv1[1];
  int32_T c6_i24;
  const c6_wypt_send_1 *c6_r11;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  int32_T c6_i25;
  real_T c6_dv1[2];
  int32_T c6_i26;
  real_T c6_d_u[2];
  const mxArray *c6_d_y = NULL;
  real_T c6_e_u;
  const mxArray *c6_e_y = NULL;
  real_T c6_f_u;
  const mxArray *c6_f_y = NULL;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_mxArrayOutData = NULL;
  for (c6_i22 = 0; c6_i22 < 5; c6_i22++) {
    c6_b_inData[c6_i22] = (*(c6_wypt_send_1 (*)[5])c6_inData)[c6_i22];
  }

  for (c6_i23 = 0; c6_i23 < 5; c6_i23++) {
    c6_u[c6_i23] = c6_b_inData[c6_i23];
  }

  c6_y = NULL;
  c6_iv1[0] = c6_iv0[0];
  sf_mex_assign(&c6_y, sf_mex_createstructarray("structure", 1, c6_iv1), false);
  for (c6_i24 = 0; c6_i24 < 5; c6_i24++) {
    c6_r11 = &c6_u[c6_i24];
    c6_b_u = c6_r11->active;
    c6_b_y = NULL;
    sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), false);
    sf_mex_addfield(c6_y, c6_b_y, "active", "active", c6_i24);
    c6_c_u = c6_r11->wtype;
    c6_c_y = NULL;
    sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), false);
    sf_mex_addfield(c6_y, c6_c_y, "wtype", "wtype", c6_i24);
    for (c6_i25 = 0; c6_i25 < 2; c6_i25++) {
      c6_dv1[c6_i25] = c6_r11->p[c6_i25];
    }

    for (c6_i26 = 0; c6_i26 < 2; c6_i26++) {
      c6_d_u[c6_i26] = c6_dv1[c6_i26];
    }

    c6_d_y = NULL;
    sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 1, 2),
                  false);
    sf_mex_addfield(c6_y, c6_d_y, "p", "p", c6_i24);
    c6_e_u = c6_r11->param1;
    c6_e_y = NULL;
    sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_e_u, 0, 0U, 0U, 0U, 0), false);
    sf_mex_addfield(c6_y, c6_e_y, "param1", "param1", c6_i24);
    c6_f_u = c6_r11->param2;
    c6_f_y = NULL;
    sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_f_u, 0, 0U, 0U, 0U, 0), false);
    sf_mex_addfield(c6_y, c6_f_y, "param2", "param2", c6_i24);
  }

  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

const mxArray *sf_c6_aircraft_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  sf_mex_assign(&c6_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c6_nameCaptureInfo;
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static int32_T c6_j_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i27;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i27, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i27;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_k_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  c6_wypt_send_1 c6_y[5])
{
  static uint32_T c6_uv0[1] = { 5U };

  uint32_T c6_uv1[1];
  emlrtMsgIdentifier c6_thisId;
  static const char * c6_fieldNames[5] = { "active", "wtype", "p", "param1",
    "param2" };

  c6_wypt_send_1 (*c6_r12)[5];
  int32_T c6_i28;
  c6_uv1[0] = c6_uv0[0];
  c6_thisId.fParent = c6_parentId;
  sf_mex_check_struct(c6_parentId, c6_u, 5, c6_fieldNames, 1U, c6_uv1);
  c6_r12 = (c6_wypt_send_1 (*)[5])c6_y;
  for (c6_i28 = 0; c6_i28 < 5; c6_i28++) {
    c6_thisId.fIdentifier = "active";
    (*c6_r12)[c6_i28].active = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
      (sf_mex_getfield(c6_u, "active", "active", c6_i28)), &c6_thisId);
    c6_thisId.fIdentifier = "wtype";
    (*c6_r12)[c6_i28].wtype = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
      (sf_mex_getfield(c6_u, "wtype", "wtype", c6_i28)), &c6_thisId);
    c6_thisId.fIdentifier = "p";
    c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c6_u, "p",
      "p", c6_i28)), &c6_thisId, (*c6_r12)[c6_i28].p);
    c6_thisId.fIdentifier = "param1";
    (*c6_r12)[c6_i28].param1 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
      (sf_mex_getfield(c6_u, "param1", "param1", c6_i28)), &c6_thisId);
    c6_thisId.fIdentifier = "param2";
    (*c6_r12)[c6_i28].param2 = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup
      (sf_mex_getfield(c6_u, "param2", "param2", c6_i28)), &c6_thisId);
  }

  sf_mex_destroy(&c6_u);
}

static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_c_wypts0;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  c6_wypt_send_1 c6_y[5];
  int32_T c6_i29;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_c_wypts0 = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_c_wypts0), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_c_wypts0);
  for (c6_i29 = 0; c6_i29 < 5; c6_i29++) {
    (*(c6_wypt_send_1 (*)[5])c6_outData)[c6_i29] = c6_y[c6_i29];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_wypt_send_1_bus_io(void *chartInstanceVoid, void
  *c6_pData)
{
  const mxArray *c6_mxVal = NULL;
  c6_wypt_send_1 c6_tmp;
  int32_T c6_i30;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxVal = NULL;
  c6_tmp.active = *(real_T *)&((char_T *)(c6_wypt_send_1 *)c6_pData)[0];
  c6_tmp.wtype = *(real_T *)&((char_T *)(c6_wypt_send_1 *)c6_pData)[8];
  for (c6_i30 = 0; c6_i30 < 2; c6_i30++) {
    c6_tmp.p[c6_i30] = ((real_T *)&((char_T *)(c6_wypt_send_1 *)c6_pData)[16])
      [c6_i30];
  }

  c6_tmp.param1 = *(real_T *)&((char_T *)(c6_wypt_send_1 *)c6_pData)[32];
  c6_tmp.param2 = *(real_T *)&((char_T *)(c6_wypt_send_1 *)c6_pData)[40];
  sf_mex_assign(&c6_mxVal, c6_d_sf_marshallOut(chartInstance, &c6_tmp), false);
  return c6_mxVal;
}

static const mxArray *c6_wypt_send_2_bus_io(void *chartInstanceVoid, void
  *c6_pData)
{
  const mxArray *c6_mxVal = NULL;
  c6_wypt_send_2 c6_tmp;
  int32_T c6_i31;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxVal = NULL;
  c6_tmp.active = *(real_T *)&((char_T *)(c6_wypt_send_2 *)c6_pData)[0];
  c6_tmp.wtype = *(real_T *)&((char_T *)(c6_wypt_send_2 *)c6_pData)[8];
  for (c6_i31 = 0; c6_i31 < 2; c6_i31++) {
    c6_tmp.p[c6_i31] = ((real_T *)&((char_T *)(c6_wypt_send_2 *)c6_pData)[16])
      [c6_i31];
  }

  c6_tmp.param1 = *(real_T *)&((char_T *)(c6_wypt_send_2 *)c6_pData)[32];
  c6_tmp.param2 = *(real_T *)&((char_T *)(c6_wypt_send_2 *)c6_pData)[40];
  sf_mex_assign(&c6_mxVal, c6_c_sf_marshallOut(chartInstance, &c6_tmp), false);
  return c6_mxVal;
}

static const mxArray *c6_wypt_send_3_bus_io(void *chartInstanceVoid, void
  *c6_pData)
{
  const mxArray *c6_mxVal = NULL;
  c6_wypt_send_3 c6_tmp;
  int32_T c6_i32;
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)chartInstanceVoid;
  c6_mxVal = NULL;
  c6_tmp.active = *(real_T *)&((char_T *)(c6_wypt_send_3 *)c6_pData)[0];
  c6_tmp.wtype = *(real_T *)&((char_T *)(c6_wypt_send_3 *)c6_pData)[8];
  for (c6_i32 = 0; c6_i32 < 2; c6_i32++) {
    c6_tmp.p[c6_i32] = ((real_T *)&((char_T *)(c6_wypt_send_3 *)c6_pData)[16])
      [c6_i32];
  }

  c6_tmp.param1 = *(real_T *)&((char_T *)(c6_wypt_send_3 *)c6_pData)[32];
  c6_tmp.param2 = *(real_T *)&((char_T *)(c6_wypt_send_3 *)c6_pData)[40];
  sf_mex_assign(&c6_mxVal, c6_b_sf_marshallOut(chartInstance, &c6_tmp), false);
  return c6_mxVal;
}

static uint8_T c6_l_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_aircraft_model, const char_T *
  c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_m_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_aircraft_model), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_aircraft_model);
  return c6_y;
}

static uint8_T c6_m_emlrt_marshallIn(SFc6_aircraft_modelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void init_dsm_address_info(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc6_aircraft_modelInstanceStruct
  *chartInstance)
{
  chartInstance->c6_change_current_waypoint = (real_T *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c6_b_wypt_send_1 = (c6_wypt_send_1 *)
    ssGetOutputPortSignal_wrapper(chartInstance->S, 1);
  chartInstance->c6_b_wypt_send_2 = (c6_wypt_send_2 *)
    ssGetOutputPortSignal_wrapper(chartInstance->S, 2);
  chartInstance->c6_b_wypt_send_3 = (c6_wypt_send_3 *)
    ssGetOutputPortSignal_wrapper(chartInstance->S, 3);
  chartInstance->c6_current_waypoint = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c6_current_waypoint1 = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
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

void sf_c6_aircraft_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3963432541U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2464857174U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1111696758U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(892706264U);
}

mxArray* sf_c6_aircraft_model_get_post_codegen_info(void);
mxArray *sf_c6_aircraft_model_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("A1Rol9hWKHrFxy1lSAqVE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c6_aircraft_model_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_aircraft_model_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c6_aircraft_model_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "incompatibleSymbol", };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 3, infoFields);
  mxArray *fallbackReason = mxCreateString("feature_off");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxArray *fallbackType = mxCreateString("early");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c6_aircraft_model_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c6_aircraft_model_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c6_aircraft_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[9],T\"current_waypoint1\",},{M[1],M[5],T\"wypt_send_1\",},{M[1],M[10],T\"wypt_send_2\",},{M[1],M[11],T\"wypt_send_3\",},{M[8],M[0],T\"is_active_c6_aircraft_model\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_aircraft_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_aircraft_modelInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc6_aircraft_modelInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _aircraft_modelMachineNumber_,
           6,
           1,
           1,
           0,
           7,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_aircraft_modelMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_aircraft_modelMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _aircraft_modelMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,10,0,0,"wypts0");
          _SFD_SET_DATA_PROPS(1,1,1,0,"change_current_waypoint");
          _SFD_SET_DATA_PROPS(2,2,0,1,"wypt_send_1");
          _SFD_SET_DATA_PROPS(3,2,0,1,"wypt_send_2");
          _SFD_SET_DATA_PROPS(4,2,0,1,"wypt_send_3");
          _SFD_SET_DATA_PROPS(5,1,1,0,"current_waypoint");
          _SFD_SET_DATA_PROPS(6,2,0,1,"current_waypoint1");
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
        _SFD_CV_INIT_EML(0,1,1,7,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2200);
        _SFD_CV_INIT_EML_IF(0,1,0,776,802,-1,1058);
        _SFD_CV_INIT_EML_IF(0,1,1,849,917,970,1049);
        _SFD_CV_INIT_EML_IF(0,1,2,1122,1148,1237,2160);
        _SFD_CV_INIT_EML_IF(0,1,3,1237,1267,1565,2160);
        _SFD_CV_INIT_EML_IF(0,1,4,1277,1318,1433,1559);
        _SFD_CV_INIT_EML_IF(0,1,5,1579,1620,1778,2155);
        _SFD_CV_INIT_EML_IF(0,1,6,1778,1806,1977,2155);

        {
          static int condStart[] = { 852, 894 };

          static int condEnd[] = { 890, 917 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,852,917,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,852,890,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,894,917,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,1125,1148,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,1244,1267,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,5,1582,1620,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,6,1785,1806,-1,0);

        {
          unsigned int dimVector[1];
          dimVector[0]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)
            c6_f_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_wypt_send_1_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_wypt_send_2_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_wypt_send_3_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)c6_sf_marshallIn);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c6_wypts0);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c6_change_current_waypoint);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c6_b_wypt_send_1);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c6_b_wypt_send_2);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c6_b_wypt_send_3);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c6_current_waypoint);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c6_current_waypoint1);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _aircraft_modelMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "PxFZXuT6qO0uw59PtmjEZ";
}

static void sf_opaque_initialize_c6_aircraft_model(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_aircraft_modelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInstanceVar);
  initialize_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c6_aircraft_model(void *chartInstanceVar)
{
  enable_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c6_aircraft_model(void *chartInstanceVar)
{
  disable_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_aircraft_model(void *chartInstanceVar)
{
  sf_gateway_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c6_aircraft_model(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c6_aircraft_model(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c6_aircraft_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_aircraft_modelInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_aircraft_model_optimization_info();
    }

    finalize_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_aircraft_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c6_aircraft_model((SFc6_aircraft_modelInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_aircraft_model(SimStruct *S)
{
  /* Actual parameters from chart:
     wypts0
   */
  const char_T *rtParamNames[] = { "wypts0" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_aircraft_model_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,6,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3366099870U));
  ssSetChecksum1(S,(3348805547U));
  ssSetChecksum2(S,(22515469U));
  ssSetChecksum3(S,(3937622621U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_aircraft_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_aircraft_model(SimStruct *S)
{
  SFc6_aircraft_modelInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc6_aircraft_modelInstanceStruct *)utMalloc(sizeof
    (SFc6_aircraft_modelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_aircraft_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_aircraft_model;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_aircraft_model;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_aircraft_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c6_aircraft_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c6_aircraft_model;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_aircraft_model;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_aircraft_model;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_aircraft_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_aircraft_model;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_aircraft_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c6_aircraft_model;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c6_aircraft_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_aircraft_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_aircraft_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_aircraft_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_aircraft_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
