/* Include files */

#include "quad_control_sim_q_sfun.h"
#include "quad_control_sim_q_sfun_debug_macros.h"
#include "c1_quad_control_sim_q.h"
#include "c2_quad_control_sim_q.h"
#include "c3_quad_control_sim_q.h"
#include "c4_quad_control_sim_q.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _quad_control_sim_qMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void quad_control_sim_q_initializer(void)
{
}

void quad_control_sim_q_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_quad_control_sim_q_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_quad_control_sim_q_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_quad_control_sim_q_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_quad_control_sim_q_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_quad_control_sim_q_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_quad_control_sim_q_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3229261259U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2134533808U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(524038451U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(762900962U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2373137021U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2921816043U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3242774204U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3137886240U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_quad_control_sim_q_get_check_sum(mxArray *plhs[]);
          sf_c1_quad_control_sim_q_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_quad_control_sim_q_get_check_sum(mxArray *plhs[]);
          sf_c2_quad_control_sim_q_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_quad_control_sim_q_get_check_sum(mxArray *plhs[]);
          sf_c3_quad_control_sim_q_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_quad_control_sim_q_get_check_sum(mxArray *plhs[]);
          sf_c4_quad_control_sim_q_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3564696471U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(678668628U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1090454852U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3896867807U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1800589300U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(926916886U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2235807544U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2257811116U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_quad_control_sim_q_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(aiChksum, "AwosKyoRHqSAsOxnZtwzCG") == 0) {
          extern mxArray *sf_c1_quad_control_sim_q_get_autoinheritance_info(void);
          plhs[0] = sf_c1_quad_control_sim_q_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "SaxOABWOfz0HAxc02VCScG") == 0) {
          extern mxArray *sf_c2_quad_control_sim_q_get_autoinheritance_info(void);
          plhs[0] = sf_c2_quad_control_sim_q_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "X68igPGpTaBmnvcXJ1msgG") == 0) {
          extern mxArray *sf_c3_quad_control_sim_q_get_autoinheritance_info(void);
          plhs[0] = sf_c3_quad_control_sim_q_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "9b0BWg82ISHsFAg0LvVt3F") == 0) {
          extern mxArray *sf_c4_quad_control_sim_q_get_autoinheritance_info(void);
          plhs[0] = sf_c4_quad_control_sim_q_get_autoinheritance_info();
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

unsigned int sf_quad_control_sim_q_get_eml_resolved_functions_info( int nlhs,
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
     case 1:
      {
        extern const mxArray
          *sf_c1_quad_control_sim_q_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_quad_control_sim_q_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_quad_control_sim_q_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_quad_control_sim_q_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_quad_control_sim_q_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_quad_control_sim_q_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_quad_control_sim_q_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_quad_control_sim_q_get_eml_resolved_functions_info();
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

unsigned int sf_quad_control_sim_q_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(tpChksum, "dMpKJIytilDmv7FphF2sM") == 0) {
          extern mxArray *sf_c1_quad_control_sim_q_third_party_uses_info(void);
          plhs[0] = sf_c1_quad_control_sim_q_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "jL7jwbAPQUk9O32VKIEg") == 0) {
          extern mxArray *sf_c2_quad_control_sim_q_third_party_uses_info(void);
          plhs[0] = sf_c2_quad_control_sim_q_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "kQRy8wuWuOiMgjzVU6a65G") == 0) {
          extern mxArray *sf_c3_quad_control_sim_q_third_party_uses_info(void);
          plhs[0] = sf_c3_quad_control_sim_q_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "2yG0QtdzMk0T8nsUQCx6UG") == 0) {
          extern mxArray *sf_c4_quad_control_sim_q_third_party_uses_info(void);
          plhs[0] = sf_c4_quad_control_sim_q_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void quad_control_sim_q_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _quad_control_sim_qMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "quad_control_sim_q","sfun",0,4,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _quad_control_sim_qMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _quad_control_sim_qMachineNumber_,0);
}

void quad_control_sim_q_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_quad_control_sim_q_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "quad_control_sim_q", "quad_control_sim_q");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_quad_control_sim_q_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
