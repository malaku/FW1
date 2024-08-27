/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FW1.c
 *
 * Code generated for Simulink model 'FW1'.
 *
 * Model version                  : 1.66
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Thu Nov 30 00:34:46 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "FW1.h"
#include "rtwtypes.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> // Add this line for write() and close()
#include <linux/can.h>
#include <linux/can/raw.h>
#include <arpa/inet.h>


#define CAN_INTERFACE "can1" // Adjust this interface name if needed

int openCANSocket() {
    int p;
    struct sockaddr_can addr;
    struct ifreq ifr;

    p = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (p == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    strcpy(ifr.ifr_name, CAN_INTERFACE);
    ioctl(p, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(p, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    return p;
}

void closeCANSocket(int socketDescriptor) {
    close(socketDescriptor);
}

void sendCANMessage(int socketDescriptor, uint32_t messageID, float value) {
    struct can_frame frame;

    frame.can_id = messageID;
    frame.can_id &= CAN_SFF_MASK; // Ensure it's a standard identifier
    frame.can_dlc = sizeof(float);

    // Serialize the float value into network byte order (big-endian)
    memcpy(frame.data, &value, sizeof(float));
    printf("Value sent: %f\n", value);

    if (write(socketDescriptor, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Error sending CAN message");
    }
}

void receiveCANMessage(int socketDescriptor, int targetID, float *tempvar) {
    struct can_frame frame;

    printf("Listening for CAN messages...\n");

    while (1) {
        ssize_t nbytes = read(socketDescriptor, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("read");
            break;
        }

        if ((frame.can_id & CAN_ERR_FLAG) == 0 && (frame.can_id & CAN_RTR_FLAG) == 0 &&
            (frame.can_id & CAN_SFF_MASK) == targetID && frame.can_dlc == sizeof(float)) {
            // Deserialize the float value from network byte order and copy it directly into *tempvar
            memcpy(tempvar, frame.data, sizeof(float));
            printf("Received CAN message with ID %x and float value: %f\n", frame.can_id, *tempvar);
            break;
        }
    }
}

void receiveFirstCANMessage(int socketDescriptor, int *receivedCANID) {
    struct can_frame frame;

    printf("Listening for the first CAN message...\n");

    while (1) {
        ssize_t nbytes = read(socketDescriptor, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("read");
            break;
        }

        if ((frame.can_id & CAN_ERR_FLAG) == 0 && (frame.can_id & CAN_RTR_FLAG) == 0) {
            // Store the received CAN ID
            *receivedCANID = frame.can_id;

            
            break;
        }
    }
}/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Continuous states */
X rtX;

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void FW1SteeringAngle_Init(DW_FW1SteeringAngle *localDW,
  P_FW1SteeringAngle *localP, X_FW1SteeringAngle *localX);
static void FW1SteeringAngle_Deriv(real_T rtu_desiredfw1, real_T *rty_Actualfw1,
  DW_FW1SteeringAngle *localDW, P_FW1SteeringAngle *localP, X_FW1SteeringAngle
  *localX, XDot_FW1SteeringAngle *localXdot);
static void FW1SteeringAngle_Update(RT_MODEL * const rtM, real_T *rty_Actualfw1,
  DW_FW1SteeringAngle *localDW);
static void FW1SteeringAngle(RT_MODEL * const rtM, real_T *rty_Actualfw1,
  DW_FW1SteeringAngle *localDW, P_FW1SteeringAngle *localP, X_FW1SteeringAngle
  *localX);

/* private model entry point functions */
extern void FW1_derivatives(void);

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 5;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  FW1_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  FW1_step();
  FW1_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  FW1_step();
  FW1_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  FW1_step();
  FW1_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* System initialize for atomic system: '<Root>/FW1 Steering Angle' */
static void FW1SteeringAngle_Init(DW_FW1SteeringAngle *localDW,
  P_FW1SteeringAngle *localP, X_FW1SteeringAngle *localX)
{
  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn1' */
  localX->TransferFcn1_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S37>/Integrator' */
  localX->Integrator_CSTATE =
    localP->PIDController1_InitialConditionForIntegrator;

  /* InitializeConditions for Integrator: '<S1>/Integrator' */
  localX->Integrator_CSTATE_g = localP->Integrator_IC;

  /* InitializeConditions for Memory: '<S4>/Memory4' */
  localDW->Memory4_PreviousInput = localP->Memory4_InitialCondition;

  /* InitializeConditions for Memory: '<S4>/Memory3' */
  localDW->Memory3_PreviousInput = localP->Memory3_InitialCondition;

  /* InitializeConditions for Integrator: '<S32>/Filter' */
  localX->Filter_CSTATE = localP->PIDController1_InitialConditionForFilter;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn' */
  localX->TransferFcn_CSTATE = 0.0;
}

/* Outputs for atomic system: '<Root>/FW1 Steering Angle' */
static void FW1SteeringAngle(RT_MODEL * const rtM, real_T *rty_Actualfw1,
  DW_FW1SteeringAngle *localDW, P_FW1SteeringAngle *localP, X_FW1SteeringAngle
  *localX)
{
  /* TransferFcn: '<S1>/Transfer Fcn1' */
  localDW->theta_D = localP->TransferFcn1_C * localX->TransferFcn1_CSTATE;

  /* Gain: '<S1>/Back EMF Constant' */
  localDW->BackEMFConstant = rtP.Ke * localDW->theta_D;

  /* Integrator: '<S37>/Integrator' */
  localDW->Integrator = localX->Integrator_CSTATE;
  if (rtmIsMajorTimeStep(rtM)) {
    /* Memory: '<S4>/Memory4' */
    localDW->Memory4 = localDW->Memory4_PreviousInput;

    /* Memory: '<S4>/Memory3' */
    localDW->Memory3 = localDW->Memory3_PreviousInput;
  }

  /* MATLAB Function: '<S4>/Switch' incorporates:
   *  Integrator: '<S1>/Integrator'
   */
  localDW->d = localX->Integrator_CSTATE_g;

  /* MATLAB Function: '<S4>/Ratchet' incorporates:
   *  Integrator: '<S1>/Integrator'
   *  MATLAB Function: '<S4>/Switch'
   */
  if (localX->Integrator_CSTATE_g > localDW->Memory3) {
    if (localX->Integrator_CSTATE_g > localDW->Memory4) {
      *rty_Actualfw1 = localX->Integrator_CSTATE_g;
    } else {
      *rty_Actualfw1 = localDW->Memory4;
    }
  } else if (localDW->Memory4 > localX->Integrator_CSTATE_g) {
    *rty_Actualfw1 = localX->Integrator_CSTATE_g;
  } else {
    *rty_Actualfw1 = localDW->Memory4;
  }

  /* End of MATLAB Function: '<S4>/Ratchet' */

  /* Integrator: '<S32>/Filter' */
  localDW->Filter = localX->Filter_CSTATE;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Constant: '<S1>/Constant4'
   *  Gain: '<S1>/Torque Constant'
   *  TransferFcn: '<S1>/Transfer Fcn'
   */
  localDW->Sum1 = localP->TransferFcn_C * localX->TransferFcn_CSTATE * rtP.Ka -
    localP->Constant4_Value;
}

/* Update for atomic system: '<Root>/FW1 Steering Angle' */
static void FW1SteeringAngle_Update(RT_MODEL * const rtM, real_T *rty_Actualfw1,
  DW_FW1SteeringAngle *localDW)
{
  if (rtmIsMajorTimeStep(rtM)) {
    /* Update for Memory: '<S4>/Memory4' */
    localDW->Memory4_PreviousInput = *rty_Actualfw1;

    /* Update for Memory: '<S4>/Memory3' */
    localDW->Memory3_PreviousInput = localDW->d;
  }
}

/* Derivatives for atomic system: '<Root>/FW1 Steering Angle' */
static void FW1SteeringAngle_Deriv(real_T rtu_desiredfw1, real_T *rty_Actualfw1,
  DW_FW1SteeringAngle *localDW, P_FW1SteeringAngle *localP, X_FW1SteeringAngle
  *localX, XDot_FW1SteeringAngle *localXdot)
{
  real_T FilterCoefficient;
  real_T Sum4;

  /* Sum: '<S1>/Sum4' */
  Sum4 = rtu_desiredfw1 - *rty_Actualfw1;

  /* Gain: '<S40>/Filter Coefficient' incorporates:
   *  Gain: '<S31>/Derivative Gain'
   *  Sum: '<S32>/SumD'
   */
  FilterCoefficient = (localP->PIDController1_D * Sum4 - localDW->Filter) *
    localP->PIDController1_N;

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn1' */
  localXdot->TransferFcn1_CSTATE = localP->TransferFcn1_A *
    localX->TransferFcn1_CSTATE;
  localXdot->TransferFcn1_CSTATE += localDW->Sum1;

  /* Derivatives for Integrator: '<S37>/Integrator' incorporates:
   *  Gain: '<S34>/Integral Gain'
   */
  localXdot->Integrator_CSTATE = localP->PIDController1_I * Sum4;

  /* Derivatives for Integrator: '<S1>/Integrator' */
  localXdot->Integrator_CSTATE_g = localDW->theta_D;

  /* Derivatives for Integrator: '<S32>/Filter' */
  localXdot->Filter_CSTATE = FilterCoefficient;

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn' incorporates:
   *  Gain: '<S1>/Power Amplifier'
   *  Gain: '<S42>/Proportional Gain'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S46>/Sum'
   */
  localXdot->TransferFcn_CSTATE = localP->TransferFcn_A *
    localX->TransferFcn_CSTATE;
  localXdot->TransferFcn_CSTATE += ((localP->PIDController1_P * Sum4 +
    localDW->Integrator) + FilterCoefficient) * localP->PowerAmplifier_Gain -
    localDW->BackEMFConstant;
}

/* Model step function */
void FW1_step(void)
{
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  /* Outputs for Atomic SubSystem: '<Root>/FW1 Steering Angle' */
  FW1SteeringAngle(rtM, &rtDW.y, &rtDW.FW1SteeringAngle_i,
                   &rtP.FW1SteeringAngle_i, &rtX.FW1SteeringAngle_i);

  /* End of Outputs for SubSystem: '<Root>/FW1 Steering Angle' */

  /* Outport: '<Root>/Actual fw1' */
  rtY.Actualfw1 = rtDW.y;
  if (rtmIsMajorTimeStep(rtM)) {
    /* Update for Atomic SubSystem: '<Root>/FW1 Steering Angle' */

    /* Update for Inport: '<Root>/desired fw1' incorporates:
     *  Inport: '<Root>/Fx'
     *  Inport: '<Root>/Fy'
     */
    FW1SteeringAngle_Update(rtM, &rtDW.y, &rtDW.FW1SteeringAngle_i);

    /* End of Update for SubSystem: '<Root>/FW1 Steering Angle' */
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
printf("Time: %f\n",rtM->Timing.clockTick1++);

  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void FW1_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Atomic SubSystem: '<Root>/FW1 Steering Angle' */

  /* Derivatives for Inport: '<Root>/desired fw1' incorporates:
   *  Inport: '<Root>/Fx'
   *  Inport: '<Root>/Fy'
   */
  FW1SteeringAngle_Deriv(rtU.desiredfw1, &rtDW.y, &rtDW.FW1SteeringAngle_i,
    &rtP.FW1SteeringAngle_i, &rtX.FW1SteeringAngle_i,
    &_rtXdot->FW1SteeringAngle_i);

  /* End of Derivatives for SubSystem: '<Root>/FW1 Steering Angle' */
}

/* Model initialize function */
void FW1_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->intgData.f[3] = rtM->odeF[3];
  rtM->contStates = ((X *) &rtX);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetSolverName(&rtM->solverInfo,"ode4");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.01;

  /* SystemInitialize for Atomic SubSystem: '<Root>/FW1 Steering Angle' */
  FW1SteeringAngle_Init(&rtDW.FW1SteeringAngle_i, &rtP.FW1SteeringAngle_i,
                        &rtX.FW1SteeringAngle_i);

  /* End of SystemInitialize for SubSystem: '<Root>/FW1 Steering Angle' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
