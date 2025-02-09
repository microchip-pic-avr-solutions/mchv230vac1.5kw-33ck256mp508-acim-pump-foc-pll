// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_pll.h
 *
 * @brief This module implements Back EMF based PLL Estimator.
 *
 * Component: ESTIMATOR
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

#ifndef __ESTIM_PLL_H
#define __ESTIM_PLL_H

#ifdef __cplusplus
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "motor_control.h"
#include "measure.h"
#include "foc_control_types.h"
#include "motor_params.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS">

 /* Description:
    This structure will host parameters related to angle/speed estimator
    parameters. */
        
typedef struct
{
    /* Integration constant */
    int16_t qDeltaT;
    /* angle of estimation */
    int16_t qTheta;
    /* internal variable for angle */
    int32_t qThetaStateVar;
    /* synchronous speed estimation */
    int16_t qOmegaMr;
    /* difference Ialpha */
    int16_t qDIalpha;
    /* difference Ibeta */
    int16_t qDIbeta;
    /* Phase voltage valpha*/
    int16_t qValpha;
    /* Phase voltage Vbeta*/
    int16_t qVbeta;
    /* counter in Last DI tables */
    int16_t qDiCounter;
    /* dI*Ls/dt alpha */
    int16_t qVIndalpha;
    /* dI*Ls/dt beta */
    int16_t qVIndbeta;
    /* BEMF d filtered */
    int16_t qEsdf;
    /* state variable for BEMF d Filtered */
    int32_t qEsdStateVar;
    /* BEMF q filtered */
    int16_t qEsqf;
    /* state variable for BEMF q Filtered */
    int32_t qEsqStateVar;
    /* filter constant for d-q BEMF */
    int16_t qKfilterEsdq;
    /* Estimated filtered rotor speed */
    int16_t qOmegaRotorFil;
    /* Filter constant for Estimated rotor speed */
    int16_t qOmegaFiltConst;
    /* State Variable for Estimated rotor speed */
    int32_t qOmegaRotorStateVar;
    /* dIalphabeta/dt */
    int16_t qDIlimitLS;
    /* dIalphabeta/dt */
    int16_t qDIlimitHS;
    /*  last  value for Ialpha */
    int16_t qLastIalphaHS[8];
    /* last  value for Ibeta */
    int16_t qLastIbetaHS[8];
    /* estimator angle initial offset */
    int16_t qThetaOffset;
    /* Rotor D-axis flux*/
    int16_t qinvPsiRd;
     /* Rotor D-axis flux shift*/
    int16_t qinvPsiRdShift;
    /* Slip Speed Estimation*/
    int16_t qOmegaSlip;
    /* Magnetizing Component of Current Imr Calculation*/
    int16_t qImrEstim;
    /* Filter constant for Estimated Imr */
    int16_t qImrEstimFilterK;
    /* State Variable for Estimated Imr */
    int32_t qImrEstimStateVar;
    /* Estimated Rotor Speed*/
    int16_t qOmegaRotor;
    /* Id * Lm^2/Lr */
    int16_t qId_LmSqrbyLr;
    /* BEMF Filter Constant */
    int16_t qEmagFiltConst;    
    /* Inverse Rotor Time constant*/
    int16_t qInvTr;
    /* 1/(Lm^2/Lr) */
    int16_t qInvLmSqrbyLr;
    /* Threshold speed for derivative calculation */
    int16_t qThresholdSpeedDerivative;
    /* Threshold speed for omega computation */
    int16_t qThresholdSpeedBEMF;
    /* Back EMF voltage in alpha-beta */
    MC_ALPHABETA_T  BEMFAlphaBeta;    
    /* Back EMF voltage in DQ */
    MC_DQ_T         BEMFdq;            
    
    const MC_DQ_T    *pIdq;  
    const MCAPP_CONTROL_T *pCtrlParam;
    const MC_ALPHABETA_T *pIAlphaBeta;
    const MC_ALPHABETA_T *pVAlphaBeta;
    const MCAPP_MOTOR_T *pMotor;
    
} MCAPP_ESTIMATOR_PLL_T;        
        
        

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_EstimatorPLLInit (MCAPP_ESTIMATOR_PLL_T *);
void MCAPP_EstimatorPLL (MCAPP_ESTIMATOR_PLL_T *);

// </editor-fold>

#ifdef __cplusplus
    }
#endif

#endif /* end of __ESTIM_PLL_H */
