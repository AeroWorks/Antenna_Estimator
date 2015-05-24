#ifndef __c6_aircraft_model_h__
#define __c6_aircraft_model_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_wypt_send_1_tag
#define struct_wypt_send_1_tag

struct wypt_send_1_tag
{
  real_T active;
  real_T wtype;
  real_T p[2];
  real_T param1;
  real_T param2;
};

#endif                                 /*struct_wypt_send_1_tag*/

#ifndef typedef_c6_wypt_send_1
#define typedef_c6_wypt_send_1

typedef struct wypt_send_1_tag c6_wypt_send_1;

#endif                                 /*typedef_c6_wypt_send_1*/

#ifndef struct_wypt_send_2_tag
#define struct_wypt_send_2_tag

struct wypt_send_2_tag
{
  real_T active;
  real_T wtype;
  real_T p[2];
  real_T param1;
  real_T param2;
};

#endif                                 /*struct_wypt_send_2_tag*/

#ifndef typedef_c6_wypt_send_2
#define typedef_c6_wypt_send_2

typedef struct wypt_send_2_tag c6_wypt_send_2;

#endif                                 /*typedef_c6_wypt_send_2*/

#ifndef struct_wypt_send_3_tag
#define struct_wypt_send_3_tag

struct wypt_send_3_tag
{
  real_T active;
  real_T wtype;
  real_T p[2];
  real_T param1;
  real_T param2;
};

#endif                                 /*struct_wypt_send_3_tag*/

#ifndef typedef_c6_wypt_send_3
#define typedef_c6_wypt_send_3

typedef struct wypt_send_3_tag c6_wypt_send_3;

#endif                                 /*typedef_c6_wypt_send_3*/

#ifndef typedef_SFc6_aircraft_modelInstanceStruct
#define typedef_SFc6_aircraft_modelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c6_sfEvent;
  boolean_T c6_isStable;
  boolean_T c6_doneDoubleBufferReInit;
  uint8_T c6_is_active_c6_aircraft_model;
  c6_wypt_send_1 c6_wypts0[5];
  real_T *c6_change_current_waypoint;
  c6_wypt_send_1 *c6_b_wypt_send_1;
  c6_wypt_send_2 *c6_b_wypt_send_2;
  c6_wypt_send_3 *c6_b_wypt_send_3;
  real_T *c6_current_waypoint;
  real_T *c6_current_waypoint1;
} SFc6_aircraft_modelInstanceStruct;

#endif                                 /*typedef_SFc6_aircraft_modelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c6_aircraft_model_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c6_aircraft_model_get_check_sum(mxArray *plhs[]);
extern void c6_aircraft_model_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
