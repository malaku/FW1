/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FW1_data.c
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

/* Block parameters (default storage) */
P rtP = {
  /* Variable: Ka
   * Referenced by: '<S1>/Torque Constant'
   */
  0.05,

  /* Variable: Ke
   * Referenced by: '<S1>/Back EMF Constant'
   */
  0.168,

  /* Variable: Ws1
   * Referenced by: '<S1>/Constant'
   */
  4622.9115276693683,

  /* Variable: cs1
   * Referenced by:
   *   '<S1>/Constant2'
   *   '<S1>/Gain2'
   */
  4500.0,

  /* Variable: mu
   * Referenced by: '<S1>/Constant3'
   */
  0.8,

  /* Variable: tpo
   * Referenced by: '<S1>/Constant1'
   */
  0.033333333333333333,

  /* Start of '<Root>/FW1 Steering Angle' */
  {
    /* Mask Parameter: PIDController1_D
     * Referenced by: '<S31>/Derivative Gain'
     */
    -0.0158696838015561,

    /* Mask Parameter: PIDController1_I
     * Referenced by: '<S34>/Integral Gain'
     */
    7.00007905364812,

    /* Mask Parameter: PIDController1_InitialConditionForFilter
     * Referenced by: '<S32>/Filter'
     */
    0.0,

    /* Mask Parameter: PIDController1_InitialConditionForIntegrator
     * Referenced by: '<S37>/Integrator'
     */
    0.0,

    /* Mask Parameter: PIDController1_N
     * Referenced by: '<S40>/Filter Coefficient'
     */
    157.462242975086,

    /* Mask Parameter: PIDController1_P
     * Referenced by: '<S42>/Proportional Gain'
     */
    2.49887600669841,

    /* Expression: 1
     * Referenced by: '<S1>/Power Amplifier'
     */
    1.0,

    /* Computed Parameter: TransferFcn1_A
     * Referenced by: '<S1>/Transfer Fcn1'
     */
    -0.4,

    /* Computed Parameter: TransferFcn1_C
     * Referenced by: '<S1>/Transfer Fcn1'
     */
    4000.0,

    /* Expression: 0
     * Referenced by: '<S1>/Integrator'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S4>/Memory4'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S4>/Memory3'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S1>/Constant4'
     */
    0.0,

    /* Computed Parameter: TransferFcn_A
     * Referenced by: '<S1>/Transfer Fcn'
     */
    -333.33333333333331,

    /* Computed Parameter: TransferFcn_C
     * Referenced by: '<S1>/Transfer Fcn'
     */
    666.66666666666663
  }
  /* End of '<Root>/FW1 Steering Angle' */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
