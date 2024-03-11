/*  File    : UDP_Recive.c
 *  
 *  Copyright 1990-2002 The MathWorks, Inc.
 *  $Revision: 1.9 $
 */
/*****************************************************************************
;Description: UDP Recive Sfunction on Raspberry.		
*****************************************************************************/

#define S_FUNCTION_NAME UDP_Recive_2_new_2
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
#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */
#define U1(element) (*uPtrs1[element])  /* Pointer to Input Port1: date + num */ // 2 dims
#define U2(element) (*uPtrs2[element])  /* Pointer to Input Port2:  */ // 1 dim

#ifdef RT
//创建socket对象
static int sockfd;
static struct sockaddr_in addr;
#endif
static unsigned int count=0;
static float data[65525]={0};
static float time[65525]={0};
static FILE *fp;
static union U{
    float v;
    unsigned char c[4];
}data_union;
static unsigned int write_num=6000;
static char dir_txt[50] = {0}; // save txt 704_1
static float RTT_udp[65525]={0};
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

    if (!ssSetNumInputPorts(S, 3)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortWidth(S, 1, 3); // yy Node_1_recvFrom_1_704
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 1); // yy stamp 
    ssSetOutputPortWidth(S, 1, 1); // yy duty cycle
    ssSetOutputPortWidth(S, 2, 1);
    
    ssSetNumSampleTimes(S, SAMPLE_TIME);
//     ssSetNumSampleTimes(S, 1);
//     ssSetNumSampleTimes(S, -1);
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
// 	ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
//     ssSetNumSampleTimes(S, -1);
    ssSetSampleTime(S, 0, SAMPLE_TIME);
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
      InputRealPtrsType uPtrs1 = ssGetInputPortSignalPtrs(S,1);// second input-2 dims
      
     #ifdef RT
    sockfd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
    if(sockfd==-1){
        printf("can not create socket\n");
	    close(sockfd);
        //exit(1);
	    //return -1;
    }
    
	int local_port = U(0);//22000;//SET_LOCAL_IP_PORT;
    
    addr.sin_family =AF_INET;
    addr.sin_port =htons(local_port);
    addr.sin_addr.s_addr=INADDR_ANY;
 

    int ret =bind(sockfd,(struct sockaddr*)&addr,sizeof(addr));
    if(0>ret)
    {
        printf("bind error\n");
        close(sockfd);
        //exit(1);
        //return -1;
    }
     
	/*fp = fopen("/home/pi/data_test_zhou.txt", "wt+");
    if(fp == NULL){
        printf("fail to open the file! \n");
		    //return -1;
        //exit(1);
	  }
    else
    {
        printf("The file is open! \n"); 
    }*/
      #endif
    
	int txttmp[4] = {0}; // type conversion
    txttmp[0]= U1(0); // second input two dims
    txttmp[1]= U1(1);
    txttmp[2]= U1(2);
    
    sprintf(dir_txt, "/home/pi/data_test_Node_%d_recvFrom_%d_%d.txt ", txttmp[0],txttmp[1], txttmp[2]);
    printf("%s\n", dir_txt);     
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
    // const real_T *u3  =ssGetInputPortRealSignal(S,2);
	real_T       *y1 = ssGetOutputPortSignal(S,0); // yy
    real_T       *y2 = ssGetOutputPortSignal(S,1); // yy
    
    #ifdef RT
	int buf[2] = {0};
    socklen_t len=sizeof(addr);
    recvfrom(sockfd,&buf,sizeof(buf),0,(struct sockaddr*)&addr,&len);
    for(int i=0;i<2;i++){
        data_union.c[3] = buf[i] >> 24;
        data_union.c[2] = buf[i] >> 16;
        data_union.c[1] = buf[i] >> 8;
        data_union.c[0] = buf[i];
        if(i==0) {data[count] = data_union.v;
        y2[0] = data_union.v; // yy data_send
        }
        else {time[count] = data_union.v;
        y1[0] = data_union.v; // yy stamp
        }
    }
    RTT_udp[count] = U2(0); //u3[0]; add an extra signal time_stamp_send
    // printf("recv num =%f\n",RTT_udp[count]); //U2(0));
	count++;
    if(count>=write_num) {
        fp = fopen(dir_txt, "wt+");
        if(fp == NULL){
            printf("fail to open the file! \n");
        }
        else
        {
            printf("The file is open! \n"); 
        }
        for(int i=0;i<count;i++){
            // fprintf(fp,"%f %f %d \n",data[i],time[i],i);
            fprintf(fp,"%f %f %d %f \n",data[i],time[i],i, RTT_udp[i]);
        }
        // printf("RTTf: %f\n", y2[0]);
        // printf("RTTd: %d\n", y2[0]);
        printf("---write: %s\n", dir_txt);
        fclose(fp);
        count=0;
    }
	
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
    fp = fopen("/home/pi/data_test_zhou_terminal_recv.txt", "wt+");
    if(fp == NULL){
        printf("fail to open the file! \n");
    }
    else
    {
        printf("The file is open! \n"); 
    }
	for(int i=0;i<count;i++){
        fprintf(fp,"%f %f %d \n",data[i],time[i],i);
    }
    fclose(fp);
    #endif
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
