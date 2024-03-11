/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 *  -------------------------------------------------------------------------
 *  | See matlabroot/simulink/src/sfuntmpl_doc.c for a more detailed template |
 *  -------------------------------------------------------------------------
 *
 * Copyright 1990-2002 The MathWorks, Inc.
 * $Revision: 1.27 $
 */
/*****************************************************************************
;Description: PWM Sfunction on Linux for Sbs104 on ADT652.
;             two parameters:    o Channel:   0   ---> 3
;                                o DutyCycle: 0   ---> 100
;                                o Frequency: 1   ---> 19500 		
*****************************************************************************/

/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  Node_1_test_RTT_delay
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*****************
 * Include files *
 *****************/

/*ANSI C headers*/
#include <stdio.h>
#include <string.h>

// define the parameters - sample time yy
#define SAMPLE_TIME_PARAM (ssGetSFcnParam(S,0))//the first paras
#define SAMPLE_TIME ((real_T) (mxGetPr(SAMPLE_TIME_PARAM)[0]))

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0: remote_port(27000) */ // 1 dim
#define U1(element) (*uPtrs1[element])  /* Pointer to Input Port1: date + num */ // 2 dims

static FILE *fp_send; // write receive data --data.txt
static unsigned int count_send=0; //0~65535 0~4294967295
static float RTT[65535]={0};
static unsigned int write_num=6000;
static char dir_txt[50] = {0}; // save txt 704_1

/*====================*
 * S-function methods *
 *====================*/

// yy
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
    static void mdlCheckParameters(SimStruct *S)
    {  

        static char_T errMsg[256];
        static boolean_T allParamsOK = 1;

        if (mxGetNumberOfElements(SAMPLE_TIME_PARAM) != 1) 
        {
           sprintf(errMsg, "sample time must be a positive scalar.\n");
           allParamsOK = 0;
           goto EXIT_POINT;
         }

         EXIT_POINT:
         if ( !allParamsOK ) 
         {
           ssSetErrorStatus(S, errMsg);
         }
    } /* end: mdlCheckParameters */
#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
    {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1); // yy u=[RTT]
    ssSetInputPortWidth(S, 1, 2); // yy u=[date num_index]
    
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    
    // ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, 0,1);
    ssSetInputPortDirectFeedThrough(S, 1,1);

    if (!ssSetNumOutputPorts(S, 0)) return; // yy

    ssSetNumSampleTimes(S, SAMPLE_TIME);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME);//INHERITED_SAMPLE_TIME
    ssSetOffsetTime(S, 0, 0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);      
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    InputRealPtrsType uPtrs1 = ssGetInputPortSignalPtrs(S,1);//ssGetInputPortSignalPtrs(S,1); // second input-2 dims
    
	// yy
    int txttmp[4] = {0}; // type conversion
    txttmp[0]= U1(0); // second input two dims
    txttmp[1]= U1(1);
    
    sprintf(dir_txt, "/home/pi/RTT_%d_%d.txt ", txttmp[0],txttmp[1]);
    // data_ras1_recvFrom_1_704.txt 
    printf("%s\n", dir_txt); 
	#ifdef RT
    fp_send = fopen(dir_txt, "wt+");
    fclose(fp_send);
    #endif
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{      
  	// const real_T *u  =ssGetInputPortRealSignal(S,0); // u=[stamp RTT]
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    // InputRealPtrsType uPtrs1 = ssGetInputPortSignalPtrs(S,1);//ssGetInputPortSignalPtrs(S,1);

    RTT[count_send] = U(0);
    // printf("input: %f\n", RTT[count_send]);
    count_send++;

    if(count_send>=65535){count_send=0;}
    
    #ifdef RT
	if(count_send >= write_num)
    {
        fp_send = fopen(dir_txt, "at+");
        if(fp_send == NULL)
            printf("fail to open the file! \n");
        else
        {
            printf("The file is open! \n");
        }
        
        for(int i=0;i<count_send;i++){
            fprintf(fp_send,"%f %d\n",RTT[i],i);
            // printf("RTTf: %f\n", RTT[i]);
            // 
        }
        //fprintf(fp_send,"%d %f %d\n",time_stamp_send[i],data_send[i],i); 
        // printf("-------write %s--------!\n", dir_txt);
        //fprintf(fp,"%f %d\n",time_stamp,data,count);
        printf("---write: %s!\n", dir_txt);
        fclose(fp_send);
        count_send =0;
    }
    #endif
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    
}

/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
