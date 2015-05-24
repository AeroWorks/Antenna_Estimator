/* Include files */

#include "aircraft_model_sfun.h"
#include "aircraft_model_sfun_debug_macros.h"
#include "c4_aircraft_model.h"
#include "c6_aircraft_model.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _aircraft_modelMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void aircraft_model_initializer(void)
{
}

void aircraft_model_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_aircraft_model_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==4) {
    c4_aircraft_model_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_aircraft_model_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

extern void sf_aircraft_model_uses_exported_functions(int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[])
{
  plhs[0] = mxCreateLogicalScalar(0);
}

unsigned int sf_aircraft_model_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1350439262U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(252398062U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3995883776U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(437883322U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(374083291U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1756289319U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1439819770U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3707039315U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 4:
        {
          extern void sf_c4_aircraft_model_get_check_sum(mxArray *plhs[]);
          sf_c4_aircraft_model_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_aircraft_model_get_check_sum(mxArray *plhs[]);
          sf_c6_aircraft_model_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2515920432U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3908508645U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2530489944U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2924353608U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2468151854U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3294632272U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1552814585U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(241000571U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_aircraft_model_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 4:
      {
        if (strcmp(aiChksum, "FVwnhuOek8PsbDQbqGQ91F") == 0) {
          extern mxArray *sf_c4_aircraft_model_get_autoinheritance_info(void);
          plhs[0] = sf_c4_aircraft_model_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "A1Rol9hWKHrFxy1lSAqVE") == 0) {
          extern mxArray *sf_c6_aircraft_model_get_autoinheritance_info(void);
          plhs[0] = sf_c6_aircraft_model_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_aircraft_model_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 4:
      {
        extern const mxArray
          *sf_c4_aircraft_model_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_aircraft_model_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_aircraft_model_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_aircraft_model_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_aircraft_model_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 4:
      {
        if (strcmp(tpChksum, "l2PajromeSqcCV41c7voeG") == 0) {
          extern mxArray *sf_c4_aircraft_model_third_party_uses_info(void);
          plhs[0] = sf_c4_aircraft_model_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "PxFZXuT6qO0uw59PtmjEZ") == 0) {
          extern mxArray *sf_c6_aircraft_model_third_party_uses_info(void);
          plhs[0] = sf_c6_aircraft_model_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_aircraft_model_jit_fallback_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the jit_fallback_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 4:
      {
        if (strcmp(tpChksum, "l2PajromeSqcCV41c7voeG") == 0) {
          extern mxArray *sf_c4_aircraft_model_jit_fallback_info(void);
          plhs[0] = sf_c4_aircraft_model_jit_fallback_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "PxFZXuT6qO0uw59PtmjEZ") == 0) {
          extern mxArray *sf_c6_aircraft_model_jit_fallback_info(void);
          plhs[0] = sf_c6_aircraft_model_jit_fallback_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_aircraft_model_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 4:
      {
        if (strcmp(tpChksum, "l2PajromeSqcCV41c7voeG") == 0) {
          extern mxArray *sf_c4_aircraft_model_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_aircraft_model_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "PxFZXuT6qO0uw59PtmjEZ") == 0) {
          extern mxArray *sf_c6_aircraft_model_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_aircraft_model_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void sf_aircraft_model_get_post_codegen_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{
  unsigned int chartFileNumber = (unsigned int) mxGetScalar(prhs[0]);
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  switch (chartFileNumber) {
   case 4:
    {
      if (strcmp(tpChksum, "l2PajromeSqcCV41c7voeG") == 0) {
        extern mxArray *sf_c4_aircraft_model_get_post_codegen_info(void);
        plhs[0] = sf_c4_aircraft_model_get_post_codegen_info();
        return;
      }
    }
    break;

   case 6:
    {
      if (strcmp(tpChksum, "PxFZXuT6qO0uw59PtmjEZ") == 0) {
        extern mxArray *sf_c6_aircraft_model_get_post_codegen_info(void);
        plhs[0] = sf_c6_aircraft_model_get_post_codegen_info();
        return;
      }
    }
    break;

   default:
    break;
  }

  plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
}

void aircraft_model_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _aircraft_modelMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "aircraft_model","sfun",0,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _aircraft_modelMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _aircraft_modelMachineNumber_,0);
}

void aircraft_model_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_aircraft_model_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("aircraft_model",
      "aircraft_model");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_aircraft_model_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
