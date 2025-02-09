// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_pll.c
 *
 * @brief This module implements PLL Estimator.
 * This is a sensor-less speed observer based on motor back EMF.
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

/* _Q15abs and _Q15sqrt function use */
#include <libq.h>
#include "estim_pll.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">
#define INVPSIRD_DEFAULT 32766 /* Maximum value for Inverse of Rotor Flux linkage*/
// </editor-fold>

/**
* <B> Function: void MCAPP_EstimatorPLLInit(MCAPP_PLL_ESTIMATOR_T *)  </B>
*
* @brief Function to reset PLL Estimator Data Structure variables.
*
* @param    pointer to the data structure containing PLL Estimator parameters.
* @return   none.
* @example
* <CODE> MMCAPP_EstimatorPLLInit(&estimator); </CODE>
*
*/
void MCAPP_EstimatorPLLInit(MCAPP_ESTIMATOR_PLL_T *pEstim)
{
    pEstim->qDiCounter = 0;
    pEstim->qEsdStateVar = 0;
    pEstim->qEsqStateVar = 0;
    pEstim->qImrEstim = 0;
    pEstim->qImrEstimStateVar = 0;
    pEstim->qThetaOffset = 0;
            
    pEstim->qThetaStateVar = 0;
    pEstim->qTheta = 0;
    
    pEstim->qOmegaRotorFil = 0;
    pEstim->qOmegaRotorStateVar = 0;
}

/**
* <B> Function: void MCAPP_EstimatorPLL(MCAPP_PLL_ESTIMATOR_T *)  </B>
*
* @brief Observer to determine rotor speed and position based on
* motor parameters and feedbacks.
*
* @param    pointer to the data structure containing PLL Estimator parameters.
* @return   none.
* @example
* <CODE> MCAPP_EstimatorPLL(&estimator); </CODE>
*
*/
void MCAPP_EstimatorPLL(MCAPP_ESTIMATOR_PLL_T *pEstim)
{
    const MCAPP_MOTOR_T *pMotor = pEstim->pMotor;
    const MC_ALPHABETA_T *pIAlphaBeta = pEstim->pIAlphaBeta;
    const MC_DQ_T *pIdq = pEstim->pIdq;
    const MCAPP_CONTROL_T *pCtrlParam = pEstim->pCtrlParam;
 
    MC_SINCOS_T     estimSinCos;        /* Sine-cosine for estimator */  

    
    int16_t deltaEs;
    
    pEstim->qValpha = pEstim->pVAlphaBeta->alpha;
    pEstim->qVbeta  = pEstim->pVAlphaBeta->beta;
    
    uint16_t index = (pEstim->qDiCounter - 7)&0x0007;
    /* dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
       For lower speed the granularity of difference is higher - the
       difference is made between 2 sampled values @ 8 ADC ISR cycles */
    if (_Q15abs(pEstim->qOmegaRotorFil) < pEstim->qThresholdSpeedDerivative) 
    {
        pEstim->qDIalpha = (pIAlphaBeta->alpha - pEstim->qLastIalphaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR
           cycle .The following limitation assures a limitation per low speed -
           up to the nominal speed */
        if (pEstim->qDIalpha > pEstim->qDIlimitLS) 
        {
            pEstim->qDIalpha = pEstim->qDIlimitLS;
        }
        if (pEstim->qDIalpha < -pEstim->qDIlimitLS) 
        {
            pEstim->qDIalpha = -pEstim->qDIlimitLS;
        }
        pEstim->qVIndalpha = (int16_t) (__builtin_mulss(pMotor->qSigmaLsDt,
                pEstim->qDIalpha) >> (pMotor->qSigmaLsDtScale+3));

        pEstim->qDIbeta = (pIAlphaBeta->beta - pEstim->qLastIbetaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR cycle
           the following limitation assures a limitation per low speed - up to
           the nominal speed */
        if (pEstim->qDIbeta > pEstim->qDIlimitLS) 
        {
            pEstim->qDIbeta = pEstim->qDIlimitLS;
        }
        if (pEstim->qDIbeta < -pEstim->qDIlimitLS) 
        {
            pEstim->qDIbeta = -pEstim->qDIlimitLS;
        }
        pEstim->qVIndbeta = (int16_t) (__builtin_mulss(pMotor->qSigmaLsDt,
                pEstim->qDIbeta) >> (pMotor->qSigmaLsDtScale+3));
    }
    else
    {
        pEstim->qDIalpha = (pIAlphaBeta->alpha - pEstim->qLastIalphaHS[(pEstim->qDiCounter)]);
        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (pEstim->qDIalpha > pEstim->qDIlimitHS) 
        {
            pEstim->qDIalpha = pEstim->qDIlimitHS;
        }
        if (pEstim->qDIalpha < -pEstim->qDIlimitHS) 
        {
            pEstim->qDIalpha = -pEstim->qDIlimitHS;
        }
        pEstim->qVIndalpha = (int16_t) (__builtin_mulss(pMotor->qSigmaLsDt,
                        pEstim->qDIalpha) >> pMotor->qSigmaLsDtScale);

        pEstim->qDIbeta = (pIAlphaBeta->beta - pEstim->qLastIbetaHS[(pEstim->qDiCounter)]);

        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (pEstim->qDIbeta > pEstim->qDIlimitHS) 
        {
            pEstim->qDIbeta = pEstim->qDIlimitHS;
        }
        if (pEstim->qDIbeta < -pEstim->qDIlimitHS) 
        {
            pEstim->qDIbeta = -pEstim->qDIlimitHS;
        }
        pEstim->qVIndbeta = (int16_t) (__builtin_mulss(pMotor->qSigmaLsDt,
                        pEstim->qDIbeta) >> pMotor->qSigmaLsDtScale);
    }    
   
    /* Update the sample history of Ialpha and Ibeta */
    pEstim->qDiCounter = (pEstim->qDiCounter + 1) & 0x0007;
    pEstim->qLastIalphaHS[pEstim->qDiCounter] = pIAlphaBeta->alpha;
    pEstim->qLastIbetaHS[pEstim->qDiCounter] = pIAlphaBeta->beta;

   /* Calculate the BEMF voltage:
     *  Ealphabeta = Valphabeta - Rs*Ialphabeta - SigmaLs*(dIalphabeta/dt)  */
    
    pEstim->BEMFAlphaBeta.alpha =  (pEstim->qValpha 
        -(int16_t) (__builtin_mulss(pMotor->qRs, pIAlphaBeta->alpha) >> pMotor->qRsScale) 
        - pEstim->qVIndalpha);
    
    pEstim->BEMFAlphaBeta.beta =   (pEstim->qVbeta 
        -(int16_t) (__builtin_mulss(pMotor->qRs, pIAlphaBeta->beta) >> pMotor->qRsScale) 
        - pEstim->qVIndbeta);
    
    /* Calculate sine and cosine components of the rotor flux angle */
    MC_CalculateSineCosine_Assembly_Ram((pEstim->qTheta + pEstim->qThetaOffset),
                                                         &estimSinCos);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&pEstim->BEMFAlphaBeta, &estimSinCos, &pEstim->BEMFdq);

    
    const int16_t ddiff = (int16_t) (pEstim->BEMFdq.d - pEstim->qEsdf);
    pEstim->qEsdStateVar += __builtin_mulss(ddiff, pEstim->qKfilterEsdq);
    pEstim->qEsdf = (int16_t) (pEstim->qEsdStateVar >> 15);

    const int16_t qdiff = (int16_t) (pEstim->BEMFdq.q - pEstim->qEsqf);
    pEstim->qEsqStateVar += __builtin_mulss(qdiff, pEstim->qKfilterEsdq);
    pEstim->qEsqf = (int16_t) (pEstim->qEsqStateVar >> 15);
    
    /* To avoid Math Trap Error*/
    if(pCtrlParam->qIdRef > pMotor->qInvLmSqrbyLr)
    {    
        pEstim->qInvLmSqrbyLr = pMotor->qInvLmSqrbyLr;
        /* Calculating Inverse of Rotor Flux linkage*/
        pEstim->qinvPsiRd = __builtin_divf(pEstim->qInvLmSqrbyLr,pCtrlParam->qIdRef);
        pEstim->qinvPsiRdShift = 0;
    }
    else if(pCtrlParam->qIdRef > (pMotor->qInvLmSqrbyLr>>1))
    {   /* Calculating Inverse of Rotor Flux linkage*/
        /* Pre dividing qInvLmSqrbyLr by 2 to perform __builtin_divf operation without overflow  */
        pEstim->qInvLmSqrbyLr = pMotor->qInvLmSqrbyLr>>1; 
        pEstim->qinvPsiRd = __builtin_divf(pEstim->qInvLmSqrbyLr,pCtrlParam->qIdRef);
        pEstim->qinvPsiRdShift = 1;
    }
    else if(pCtrlParam->qIdRef > (pMotor->qInvLmSqrbyLr>>2))
    {   /* Calculating Inverse of Rotor Flux linkage*/
        /* Pre dividing qInvLmSqrbyLr by 4 to perform __builtin_divf operation without overflow  */
        pEstim->qInvLmSqrbyLr = pMotor->qInvLmSqrbyLr>>2;
        pEstim->qinvPsiRd = __builtin_divf(pEstim->qInvLmSqrbyLr,pCtrlParam->qIdRef);
        pEstim->qinvPsiRdShift = 2;
    }
    else{
        /* Default Inverse of Rotor Flux linkage*/
        pEstim->qinvPsiRd = INVPSIRD_DEFAULT;
        pEstim->qinvPsiRdShift = 3;
    }


     /*  For stability the condition for low speed */
    if (_Q15abs(pEstim->qOmegaRotorFil) > pEstim->qThresholdSpeedBEMF) 
    {
        /* At speed greater than decimation speed, calculate the estimated
         * velocity based on:
         * OmegaMr = Invpsi * (Eqfiltered - sgn(Eqfiltered) * Edfiltered)
         */
        if (pEstim->qEsqf > 0) 
        {
            deltaEs = (int16_t) (pEstim->qEsqf - pEstim->qEsdf);
            pEstim->qOmegaMr =  (int16_t) (__builtin_mulss(pEstim->qinvPsiRd,
                                    deltaEs) >> (15-pEstim->qinvPsiRdShift) );
        } 
        else 
        {
            deltaEs = (int16_t) (pEstim->qEsqf + pEstim->qEsdf);
            pEstim->qOmegaMr = (int16_t) (__builtin_mulss(pEstim->qinvPsiRd,
                                    deltaEs) >> (15-pEstim->qinvPsiRdShift));
        }
    }        
    /* if estimator speed<decimation speed => condition VelRef<>0 */
    else 
    {
        /* At speed lower than or equal to decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = (1/ke) * (Eqfiltered - sgn(omega) * Edfiltered)
         * to improve stability.
         */
        if (pEstim->qOmegaRotorFil > 0) 
        {
            deltaEs = (int16_t) (pEstim->qEsqf - pEstim->qEsdf);
            pEstim->qOmegaMr = (int16_t) (__builtin_mulss(pEstim->qinvPsiRd,
                                    deltaEs) >> (15-pEstim->qinvPsiRdShift));
        } 
        else 
        {
            deltaEs = (int16_t) (pEstim->qEsqf + pEstim->qEsdf);
            pEstim->qOmegaMr = (int16_t) (__builtin_mulss(pEstim->qinvPsiRd,
                                    deltaEs) >> (15-pEstim->qinvPsiRdShift));
        }
    }
 
    pEstim->qOmegaMr = pEstim->qOmegaMr << (15-pMotor->qInvLmSqrbyLrScale);
     

    /* Integrate the estimated rotor flux velocity to get estimated rotor angle */  
    pEstim->qThetaStateVar += __builtin_mulss(pEstim->qOmegaMr, pEstim->qDeltaT);
    pEstim->qTheta = (int16_t) (pEstim->qThetaStateVar >> 15);
    
    /* Estimate the Magnetizing Current Imr = (Id/(TrS + 1))*/
    const int16_t imrdiff = (int16_t) (pCtrlParam->qIdRef - pEstim->qImrEstim);
    pEstim->qImrEstimStateVar += __builtin_mulss(imrdiff,
                                                    pEstim->qImrEstimFilterK);
    pEstim->qImrEstim = (int16_t) (pEstim->qImrEstimStateVar >> 15);
    
    /* Estimate the slip value wslip = ((1/Tr) * (iq/imr))*/
    const int32_t iqTr = __builtin_mulss(pEstim->qInvTr,pIdq->q);

    if((pEstim->qImrEstim > 0))
    {    
        pEstim->qOmegaSlip =  __builtin_divsd(iqTr,pEstim->qImrEstim);
    }
    else{
        pEstim->qOmegaSlip = 0;
    }

    /* Estimate the rotor velocity by subtracting slip from synchronous Speed(rotor flux) */
    pEstim->qOmegaRotor = pEstim->qOmegaMr - pEstim->qOmegaSlip ;
            
    /* Filter the estimated  rotor velocity using a first order low-pass filter */
    const int16_t Omegadiff = (int16_t) (pEstim->qOmegaRotor - pEstim->qOmegaRotorFil);
    pEstim->qOmegaRotorStateVar += __builtin_mulss(Omegadiff, pEstim->qOmegaFiltConst);
    pEstim->qOmegaRotorFil = (int16_t) (pEstim->qOmegaRotorStateVar >> 15);  
}
