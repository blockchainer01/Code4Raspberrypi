/*  File    : UDP_Recive.c
 *  
 *  Copyright 1990-2002 The MathWorks, Inc.
 *  $Revision: 1.9 $
 */
/*****************************************************************************
;Description: UDP_Recive Sfunction on Raspberry.		
*****************************************************************************/

#define S_FUNCTION_NAME UDP_Send_2_new_2
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"
#ifdef RT
   #include<stdio.h>
   #include<unistd.h>
   #include<sys/types.h>
   #include<sys/socket.h>
   #include<arpa/inet.h>
   #include <memory.h>
#endif

// define the parameters - sample time yy
#define SAMPLE_TIME_PARAM (ssGetSFcnParam(S,0))//the first paras
#define SAMPLE_TIME ((real_T) (mxGetPr(SAMPLE_TIME_PARAM)[0]))

//#define SET_LOCAL_IP_PORT_PARAM (ssGetSFcnParam(S,0))
//#define SET_LOCAL_IP_PORT ((int16_T) (mxGetPr(SET_LOCAL_IP_PORT_PARAM)[0]))
// #define SET_DATA_SIZE_PARAM (ssGetSFcnParam(S,1))
// #define SET_DATA_SIZE ((int16_T) (mxGetPr(SET_DATA_SIZE_PARAM)[0]))
#define U(element) (*uPtrs[element])  /* Pointer to Input Port0: remote_port(27000) */ // 1 dim
#define U1(element) (*uPtrs1[element])  /* Pointer to Input Port1: 46 + 27 */ // 2 dims
#define U2(element) (*uPtrs2[element])  /* Pointer to Input Port2:  */ // 1 dim
#define U3(element) (*uPtrs3[element])  /* Pointer to Input Port3 */ // 1 dim
#ifdef RT
//创建socket对象
static int sockfd;
static struct sockaddr_in addr;
#endif
static unsigned int count=0;
static float data[65525]={0};
static FILE *fp;
static union U{
    float v[2];
    unsigned char c[8];
}data_union;
static union U_ip{
    int inti[4];
    unsigned char c[32];
}ip_union;
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
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }
    

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 4)) return;
    ssSetInputPortWidth(S, 0, 1); // 1 dim
    ssSetInputPortWidth(S, 1, 2); // 2 dims
    ssSetInputPortWidth(S, 2, 1); // 1 dim
    ssSetInputPortWidth(S, 3, 1); // 1 dim
    /*ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);*/
    
//     ssSetInputPortDataType(S, 1, SS_INT32);
    //ssSetInputPortDataType(S, 3, SS_SINGLE);

    if (!ssSetNumOutputPorts(S, 0)) return;//设置输出变量的个数


    ssSetNumSampleTimes(S, SAMPLE_TIME);
    // ssSetNumSampleTimes(S, -1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    //ssSetNumIWidth(S, 0, 1);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the operating point save/restore compliance to be same as a 
     * built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
	//ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetSampleTime(S, 0, SAMPLE_TIME);
    //ssSetSampleTime(S, 1, -1);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your      S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *
   * You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
      InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
      InputRealPtrsType uPtrs1 = ssGetInputPortSignalPtrs(S,1);
     #ifdef RT
    // sockfd=socket(AF_INET,SOCK_DGRAM,0);
    sockfd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
    if(sockfd==-1){
        printf("can not create socket\n");
	    close(sockfd);
    }
    int iptmp[4] = {0};
    iptmp[0]= U1(0); // second input two dims
    iptmp[1]= U1(1);

    char strip[20] = {0}; // save ip
    sprintf(strip, "192.168.%d.%d ", iptmp[0],iptmp[1]);
    printf("%s\n", strip);
    
	int remote_port = U(0);//25000;//SET_LOCAL_IP_PORT;
    //int remote_ip = U1(0);
    addr.sin_family =AF_INET;
    addr.sin_port =htons(remote_port);
    addr.sin_addr.s_addr=inet_addr(strip);
 
    int ret =connect(sockfd,(struct sockaddr*)&addr,sizeof(addr));
    if(0>ret)
    {
        printf("connect error\n");
        close(sockfd);
        //exit(1);
        //return -1;
    }
     
      #endif
   
        
       
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs(S,2);
    InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs(S,3);
    
    float data = U2(0);
    float time = U3(0);
    //float *input1 = ssGetInputPortSignal(S,2);
    #ifdef RT
	data_union.v[0]=data;
    data_union.v[1]=time;
    sendto(sockfd,&data_union,sizeof(data_union),0,(struct sockaddr*)&addr,sizeof(addr));
    #endif
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    #ifdef RT
	close(sockfd);
    #endif
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
