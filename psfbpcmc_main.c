//#############################################################################
//
// FILE: psfbpcmc_main.c
//
// TITLE: This is the main file for the solution, following is the
//         <solution>.c -> solution source file
//         <solution>.h -> solution header file
//         <solution>_settings.h -> powerSUITE generated settings
//         <solution>_hal.c -> solution hardware abstraction layer
//         <solution>_hal.h -> solution hardware abstraction layer header file
//         <solution>_clatask.cla -> cla task file
//
//#############################################################################
// $TI Release: PMP23126 v1.00.01.00 $
// $Release Date: Thu Jan 26 15:57:57 CST 2023 $
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

/*
 * BEGINNER'S GUIDE TO THIS FILE
 * =============================
 *
 * PURPOSE
 * -------
 * This program performs four jobs on a TMS320F280049C:
 *
 *   1. Generate the primary and secondary bridge PWM signals.
 *   2. Measure primary-coil current with an ACS37030 current sensor.
 *   3. Extract current frequency, fundamental, and third harmonic.
 *   4. Use measurements at two PWM frequencies to estimate:
 *        L    = common primary/secondary self-inductance,
 *        M    = mutual inductance,
 *        Z1   = receiver fundamental terminal ratio R1+j*X1,
 *        Z3   = receiver third-harmonic terminal ratio R3+j*X3, and
 *        k    = magnetic coupling coefficient.
 *
 * HIGH-LEVEL DATA FLOW
 * --------------------
 *
 *   PWM1 reference and bridge voltage command
 *                    |
 *                    v
 *   ACS37030 VOUT/VREF -> ADCC/ADCB -> 1024-sample current buffer
 *                    |
 *                    v
 *   mean-crossing lock check + PWM-referenced harmonic least squares
 *                    |
 *                    v
 *   I1 and I3 complex phasors at frequency f1
 *                    |
 *       automatic safe PWM transition from case 7 to case 5
 *                    |
 *                    v
 *   I1 and I3 complex phasors at frequency f2
 *                    |
 *                    v
 *   Z(f1), Z(3*f1), Z(f2), Z(3*f2) -> complex-load L/M/Z1/Z3 fit
 *
 * IMPORTANT UNITS
 * ---------------
 * Variable suffix             Unit
 *   MilliAmps / MA              mA
 *   MilliVolts / MV             mV
 *   MilliOhms / MOHM            mOhm
 *   NanoHenry / NH              nH
 *   PicoFarad / PF              pF
 *   FrequencyHz / HZ            Hz
 *   CentiDegrees                0.01 degree
 *   Permille                    parts per thousand (1000 = 1.000)
 *
 * CURRENT-SENSOR EQUATION
 * -----------------------
 * ADC_difference = ADC(VOUT) - ADC(VREF)
 *
 * For a 3.3 V, 12-bit ADC and 66 mV/A ACS37030 sensitivity:
 *
 *   current_mA = ADC_difference * (3.3 / 4096 / 0.066) * 1000
 *              = ADC_difference * 34375 / 2816.
 *
 * FREQUENCY EQUATION
 * ------------------
 * Rising mean-crossings are linearly interpolated. If p[k] is a crossing
 * location measured in samples, then:
 *
 *   period_samples[k] = p[k] - p[k-1]
 *   frequency_Hz      = sample_rate_Hz / average(period_samples).
 *
 * HARMONIC/PHASOR EQUATIONS
 * --------------------------
 * PWM1 COUNTER ZERO is the time reference, even when the actual bridge
 * voltage fundamental has a nonzero phase. Fit the five basis columns:
 *   B[n] = [1, cos(theta[n]), sin(theta[n]), cos(3*theta[n]), sin(3*theta[n])]
 *   a = solve(B^T B, B^T x).
 * Then I1 = a[1]-j*a[2] and I3 = a[3]-j*a[4], converted from ADC counts to mA.
 * Joint least squares retains cross terms for a non-integer-cycle window.
 *   |Ih| = sqrt(Ih_real^2+Ih_imag^2), phase = atan2(Ih_imag,Ih_real).
 * These are PEAK phasors. The known sampling/PWM clock ratio advances theta;
 * current zero crossings independently verify frequency lock.
 *
 * Reconstruct the primary switching-node voltage from AQ, RED/FED and the
 * current-dependent ideal body-diode clamps during both-gates-off intervals.
 * Integrate its complex fundamental and third. Ideal +/-4*Vs/(h*pi) only
 * applies to an exactly opposed bridge with instantaneous transitions.
 * Measured complex voltage overrides are supported for each operating point.
 *
 * INPUT-IMPEDANCE EQUATION
 * ------------------------
 * For complex voltage V = Vr + j*Vi and current I = Ir + j*Ii:
 *
 *   Z = V/I = Rin + j*Xin
 *   Rin = (Vr*Ir + Vi*Ii) / (Ir^2 + Ii^2)
 *   Xin = (Vi*Ir - Vr*Ii) / (Ir^2 + Ii^2).
 *
 * Vi need not be zero: dead time and leg phase can rotate both harmonics.
 * The program forms four complex impedance observations at two frequencies.
 *
 * COMPLEX-RECEIVER COUPLED-COIL MODEL
 * ----------------------------------
 * Based on parameterestimation.m from Parameter estimation.zip, extended to
 * keep ALL FOUR observations (two frequencies, fundamental and third).
 *
 *   Zrec,1 = Rrec1+j*Xrec1; Zrec,3 = Rrec3+j*Xrec3.
 *   Zp = Rp+j*(w*L-1/(w*Cp)); Zs = Rs+j*(w*L-1/(w*Cs)).
 *   Zin,h = Zp+(w*M)^2/(Zs+Zrec,h), w=h*2*pi*f.
 *
 * The six independent real unknowns are L, |M|, Rrec1, Xrec1, Rrec3, Xrec3.
 * All eight measured real/imaginary impedance components enter the fit.
 * Zrec,1 is NOT required to equal Zrec,3. A negative Rrec3 is permitted.
 * There is no passive-bridge harmonic-voltage prediction or DC conversion.
 *
 * REQUIRED ASSUMPTIONS:
 *   - Series-series compensation and equal self-inductances Lp=Ls=L.
 *   - Known Cp,Cs and effective series losses Rp,Rs.
 *   - L and M are stable during the two-frequency experiment.
 *   - For each harmonic separately, Zrec,h is approximately unchanged between
 *     f1 and f2. This is an operating-point approximation, not a universal law.
 *   - Forward receiving operation (Rrec1>=0), sufficient steady-state signals,
 *     and correctly aligned voltage/current phase measurements.
 *
 * This can be applied to diode or synchronous receivers when these electrical
 * conditions hold; arbitrary receiver control changes are NOT identified.
 * The receiver ratios are not general linear impedances or DC resistances.
 * Coupling k=|M|/L. Primary-only measurements do not identify dot polarity.
 *
 * The slow solver uses 64-bit long double on C28x for numerical conditioning.
 * It runs only after collection; acquisition and harmonic extraction remain
 * fixed point. Finite multistart optimization does not prove global uniqueness.
 * Solver flags reject detected ambiguity, weak sensitivity, excessive residual
 * or boundary solutions. A low residual does not prove the physical model.
 * WPT_identificationModelWarning remains 1 because these assumptions persist.
 *
 * WPT_estimatedDcLoadMilliOhm is now the INVALID sentinel 0xFFFFFFFF, and
 * WPT_estimatedDcLoadValid is always zero. Measure DC quantities separately.
 *
 * Primary dead-time reconstruction is unchanged. It remains approximate:
 * Coss, switching ringing, diode drops and driver delay are not modeled.
 * Measured primary-voltage overrides are preferred when available.
 *
 * FIXED-POINT REPRESENTATIONS
 * ---------------------------
 * Q15 sine:  -32768..32767 represents approximately -1.0..+1.0.
 * Q32 phase: 0..2^32-1 represents one full 0..360-degree cycle.
 * Q20 constants: a real constant c is stored as round(c*2^20).
 * Q16 crossing position: lower 16 bits store fractional sample position.
 *
 * HOW TO READ THE PROGRAM
 * -----------------------
 * Start at main(). Then read, in order:
 *   WPT_updateCurrentMeasurement()
 *   WPT_analyzeCurrentCapture()
 *   WPT_extractCurrentHarmonics()
 *   WPT_updateIdentificationState()
 *   WPT_solveIdentification().
 */

#include "psfbpcmc.h"
#include "wpt_complex_estimator.h"

//=============================================================================
// Linker symbols used when FLASH builds copy time-critical code into RAM.
// The linker command file supplies the start addresses and number of words.
//=============================================================================
extern uint16_t isrcodefuncsLoadStart;
extern uint16_t isrcodefuncsLoadSize;
extern uint16_t isrcodefuncsRunStart;

//=============================================================================
// Runtime PWM profile values
//=============================================================================
// These are ePWM time-base ticks, not angles. The meaning depends on the
// selected PWM period and on whether the synchronized counter starts counting
// up or down. WPT_loadPwmControllerCase() overwrites all three before PWM setup.
//
// PH_1_4 is EPWM2's TBPHS value, referenced to EPWM1's counter-zero sync.
// Both raw A outputs use the same compare actions. EPWM2 counts DOWN after
// sync, so a positive TBPHS delays its waveform. At 100 MHz, the internal
// sync-to-counter-load delay is two TBCLKs. For case 7, the half-cycle is
// 588 ticks and PH_1_4 = 588 - 2 = 586 aligns opposite raw-A switching edges.
// The final transistor phase also depends on A/B wiring and gate-driver delay.
uint16_t PH_1_4 = 550;

// PH_1_5 places the secondary bridge reference Q5 relative to primary Q1.
uint16_t PH_1_5 = 870;

// PH_5_8 is EPWM5's TBPHS. EPWM4 FORWARDS its incoming sync, so this value
// is referenced to the forwarded EPWM1 sync, not EPWM4's shifted counter zero.
// Its phase relative to EPWM4 therefore depends on BOTH PH_1_5 and PH_5_8.
uint16_t PH_5_8 = 440;

// PWM transition states visible in CCS. ERROR intentionally leaves the bridge
// tripped low; it must not automatically resume with an unknown PWM profile.
#define WPT_PWM_TRANSITION_IDLE                 0U
#define WPT_PWM_TRANSITION_REQUESTED            1U
#define WPT_PWM_TRANSITION_OUTPUTS_OFF          2U
#define WPT_PWM_TRANSITION_COMPLETE             3U
#define WPT_PWM_TRANSITION_ERROR                4U

// Runtime PWM profile and transition diagnostics for CCS Expressions.
// "volatile" forces every read/write to occur in memory so CCS sees live data.
volatile uint16_t WPT_pwmActiveControllerCase = 0U;
volatile uint16_t WPT_pwmRequestedControllerCase =
        WPT_PWM_FIRST_CONTROLLER_CASE;
volatile uint16_t WPT_pwmTransitionRequest = 0U;
volatile uint16_t WPT_pwmTransitionState = WPT_PWM_TRANSITION_IDLE;
volatile uint16_t WPT_pwmPeriodTicks = 0U;
volatile uint32_t WPT_pwmFrequencyHz = 0UL;
volatile uint32_t WPT_pwmTransitionCount = 0UL;
volatile uint16_t WPT_pwmPreexistingTripMask = 0U;

//=============================================================================
// Current-sensor, ADC, sampling, and frequency-estimator constants
//=============================================================================
//
// ACS37030LLZATR-020B3 current-sensor connections from WPT_addedLC.kicad_sch:
//   VOUT -> Iout_adc  -> LaunchPad J1 pin 8 -> ADCINC4
//   VREF -> Iout2_adc -> LaunchPad J1 pin 6 -> ADCINB3
//
// The sensor is bidirectional, has a nominal 66 mV/A sensitivity, and reports
// zero current when VOUT equals VREF. A positive result corresponds to current
// flowing from IP+ to IP-. If the physical current direction is reversed, set
// WPT_CURRENT_POLARITY to -1L.
//
#define WPT_CURRENT_VOUT_ADC_BASE          ADCC_BASE
#define WPT_CURRENT_VOUT_RESULT_BASE       ADCCRESULT_BASE
#define WPT_CURRENT_VOUT_SOC               ADC_SOC_NUMBER15
#define WPT_CURRENT_VOUT_CHANNEL           ADC_CH_ADCIN4

#define WPT_CURRENT_VREF_ADC_BASE          ADCB_BASE
#define WPT_CURRENT_VREF_RESULT_BASE       ADCBRESULT_BASE
#define WPT_CURRENT_VREF_SOC               ADC_SOC_NUMBER14
#define WPT_CURRENT_VREF_CHANNEL           ADC_CH_ADCIN3

#define WPT_CURRENT_SAMPLE_PWM_BASE         PSFB_FB_PWM3_BASE
#define WPT_CURRENT_ADC_TRIGGER             ADC_TRIGGER_EPWM3_SOCA
// EPWM3 counts 0..TBPRD in up-count mode, so:
//   sample_rate = PWM_clock/(TBPRD+1)
//   TBPRD       = PWM_clock/sample_rate - 1.
#define WPT_CURRENT_SAMPLE_PWM_PERIOD       \
    ((uint16_t)((PSFB_PWMSYSCLOCK_FREQ_HZ / WPT_CURRENT_SAMPLE_RATE_HZ) - 1UL))
#define WPT_CURRENT_ADC_ACQPS               20U
#define WPT_CURRENT_ADC_FULL_SCALE_MV       3300L
#define WPT_CURRENT_ADC_COUNTS              4096L

// ACS37030-020B3 scale in fixed point:
//   current_mA = ADC_difference_counts * 34375 / 2816
// This is exactly equal to counts * (3.3 / 4096 / 0.066) * 1000.
#define WPT_CURRENT_MA_NUMERATOR             34375L
#define WPT_CURRENT_MA_DENOMINATOR           2816L
#define WPT_CURRENT_POLARITY                 1L
#define WPT_CURRENT_FILTER_DIVISOR           20L
#define WPT_CURRENT_TRACE_LENGTH            1024U
// Reject nearly flat/noisy buffers before searching for zero crossings.
#define WPT_CURRENT_MIN_P2P_COUNTS           16
// A new rising crossing is armed only after the signal falls this percentage
// of peak-to-peak below the mean. This prevents noise from double-counting.
#define WPT_CURRENT_HYSTERESIS_PERCENT       10L
// Q16 crossing positions retain fractional locations from linear interpolation.
#define WPT_CURRENT_CROSSING_Q_SHIFT         16U
#define WPT_CURRENT_CROSSING_Q_ONE           (1UL << WPT_CURRENT_CROSSING_Q_SHIFT)
#define WPT_CURRENT_MIN_FREQUENCY_HZ          60000UL
#define WPT_CURRENT_MAX_FREQUENCY_HZ         150000UL
#define WPT_CURRENT_MIN_VALID_PERIODS             3U

// Identification state sequence:
// wait for f1 -> settle -> average f1 -> change PWM -> settle -> average f2 ->
// solve. The numbered values are intentionally exposed for CCS debugging.
#define WPT_IDENT_STATE_WAIT_FIRST                0U
#define WPT_IDENT_STATE_SETTLE_FIRST              1U
#define WPT_IDENT_STATE_COLLECT_FIRST             2U
#define WPT_IDENT_STATE_WAIT_CHANGE               3U
#define WPT_IDENT_STATE_SETTLE_SECOND             4U
#define WPT_IDENT_STATE_COLLECT_SECOND            5U
#define WPT_IDENT_STATE_COMPLETE                  6U
#define WPT_IDENT_HARMONIC_OBSERVATION_COUNT      4U
#define WPT_IDENT_ERROR_COMPONENT_COUNT           8ULL

// Fixed-point math constants. Each Q20 constant is round(real_value*2^20).
// WPT_TWO_PI_TIMES_1E6 lets the reactance functions work directly with Hz,
// nH, pF, and mOhm without floating point.
#define WPT_FOUR_OVER_PI_Q20                 1335088L
#define WPT_FOUR_OVER_3PI_Q20                 445029L
#define WPT_TWO_PI_TIMES_1E6                  6283185UL

//=============================================================================
// Live current-capture diagnostics (add these to CCS Expressions)
//=============================================================================
// Raw codes and their voltage/current conversions show whether the sensor and
// ADC wiring are correct before trusting any harmonic or inductance result.
volatile uint16_t WPT_currentVoutRaw = 0U;
volatile uint16_t WPT_currentVrefRaw = 0U;
volatile uint16_t WPT_currentVoutMilliVolts = 0U;
volatile uint16_t WPT_currentVrefMilliVolts = 0U;
volatile int32_t WPT_currentMilliAmps = 0L;
volatile int32_t WPT_currentFilteredMilliAmps = 0L;
// The raw buffer holds signed VOUT-VREF ADC counts. The trace buffer contains
// the same data converted to mA and is convenient for a CCS graph window.
volatile int16_t WPT_currentSampleCounts[WPT_CURRENT_TRACE_LENGTH];
volatile int32_t WPT_currentTraceMilliAmps[WPT_CURRENT_TRACE_LENGTH];
// Capture flags form a simple producer/consumer handshake:
//   index increments for each ADC pair;
//   ready=1 freezes sampling and asks the main loop to analyze the buffer;
//   valid=0 means an ADC overflow occurred while collecting that buffer.
volatile uint16_t WPT_currentCaptureIndex = 0U;
volatile uint16_t WPT_currentCaptureReady = 0U;
volatile uint16_t WPT_currentCaptureValid = 1U;
volatile uint16_t WPT_currentLastCaptureValid = 0U;
volatile uint16_t WPT_currentCrossingCount = 0U;
volatile uint16_t WPT_currentValidPeriodCount = 0U;
volatile uint16_t WPT_currentRejectedPeriodCount = 0U;
volatile uint16_t WPT_currentFrequencyValid = 0U;
volatile uint32_t WPT_currentCaptureNumber = 0UL;
volatile uint32_t WPT_currentAdcOverflowCount = 0UL;
volatile uint32_t WPT_currentFrequencyRejectCount = 0UL;
volatile int32_t WPT_currentAmplitudeMilliAmps = 0L;
volatile int32_t WPT_currentPeakToPeakMilliAmps = 0L;
volatile int32_t WPT_currentDcMilliAmps = 0L;
volatile uint32_t WPT_currentFrequencyHz = 0UL;

//=============================================================================
// PWM-referenced harmonic results
//=============================================================================
// Fixed-point current phasors use PWM1 COUNTER ZERO as the cosine reference.
// The actual bridge-voltage fundamental can have a nonzero phase. Real and imaginary components are peak mA:
//   I = Ireal + j*Iimag
//   amplitude = sqrt(Ireal^2 + Iimag^2)
//   phase     = atan2(Iimag,Ireal).
volatile uint32_t WPT_currentCaptureStartPhaseQ32 = 0UL;
volatile int32_t WPT_currentFundamentalRealMilliAmps = 0L;
volatile int32_t WPT_currentFundamentalImagMilliAmps = 0L;
volatile int32_t WPT_currentThirdRealMilliAmps = 0L;
volatile int32_t WPT_currentThirdImagMilliAmps = 0L;
volatile uint32_t WPT_currentFundamentalAmplitudeMilliAmps = 0UL;
volatile uint32_t WPT_currentThirdAmplitudeMilliAmps = 0UL;
volatile int32_t WPT_currentFundamentalPhaseCentiDegrees = 0L;
volatile int32_t WPT_currentThirdPhaseCentiDegrees = 0L;
volatile uint16_t WPT_currentHarmonicValid = 0U;
volatile uint16_t WPT_currentThirdHarmonicValid = 0U;
// Modeled/measured voltage harmonics from the most recently stored point.
// PWM1 counter zero remains the reference; phases need not be 0/180 degrees.
volatile uint32_t WPT_voltageFundamentalAmplitudeMilliVolts = 0UL;
volatile uint32_t WPT_voltageThirdAmplitudeMilliVolts = 0UL;
volatile int32_t WPT_voltageFundamentalPhaseCentiDegrees = 0L;
volatile int32_t WPT_voltageThirdPhaseCentiDegrees = 18000L;
// Voltage phasors are reconstructed separately for each collected operating
// point; do not reuse a 100 kHz phasor for the saved 85 kHz point.
volatile int32_t WPT_identificationV1RealMilliVolts[2] = {0L, 0L};
volatile int32_t WPT_identificationV1ImagMilliVolts[2] = {0L, 0L};
volatile int32_t WPT_identificationV3RealMilliVolts[2] = {0L, 0L};
volatile int32_t WPT_identificationV3ImagMilliVolts[2] = {0L, 0L};
volatile uint16_t WPT_identificationVoltageValid[2] = {0U, 0U};
volatile uint16_t WPT_identificationVoltageApproximate[2] = {0U, 0U};
volatile uint16_t WPT_identificationDeadtimeUncertain[2] = {0U, 0U};
// User inputs in CCS. Deliberately preserved by reset; all four measured
// phasor components must be updated together before starting a collection.
volatile uint16_t WPT_identificationUseMeasuredVoltage[2] = {0U, 0U};
volatile int32_t WPT_identificationMeasuredV1RealMilliVolts[2] = {0L, 0L};
volatile int32_t WPT_identificationMeasuredV1ImagMilliVolts[2] = {0L, 0L};
volatile int32_t WPT_identificationMeasuredV3RealMilliVolts[2] = {0L, 0L};
volatile int32_t WPT_identificationMeasuredV3ImagMilliVolts[2] = {0L, 0L};

//=============================================================================
// Two-frequency, two-harmonic identification data and final estimates
//=============================================================================
// Array element 0 belongs to the first PWM frequency; element 1 belongs to the
// second. I1 and I3 are averaged peak phasor components. Input-impedance arrays
// are calculated later from Z1=V1/I1 and Z3=V3/I3.
volatile uint16_t WPT_identificationState = WPT_IDENT_STATE_WAIT_FIRST;
volatile uint16_t WPT_identificationResetRequest = 0U;
volatile uint16_t WPT_identificationValid = 0U;
volatile uint16_t WPT_identificationModelWarning = 0U;
volatile uint16_t WPT_identificationStableCount = 0U;
volatile uint16_t WPT_identificationCollectionCount = 0U;
volatile uint32_t WPT_identificationFrequencyHz[2] = {0UL, 0UL};
volatile int32_t WPT_identificationI1RealMilliAmps[2] = {0L, 0L};
volatile int32_t WPT_identificationI1ImagMilliAmps[2] = {0L, 0L};
volatile int32_t WPT_identificationI3RealMilliAmps[2] = {0L, 0L};
volatile int32_t WPT_identificationI3ImagMilliAmps[2] = {0L, 0L};
volatile uint16_t WPT_identificationThirdValid[2] = {0U, 0U};
volatile int32_t WPT_identificationInputResistanceMilliOhms[2] = {0L, 0L};
volatile int32_t WPT_identificationInputReactanceMilliOhms[2] = {0L, 0L};
volatile int32_t WPT_identificationThirdInputResistanceMilliOhms[2] =
        {0L, 0L};
volatile int32_t WPT_identificationThirdInputReactanceMilliOhms[2] =
        {0L, 0L};
// Public results use nH and mOhm for CCS compatibility. The NEW slow solver
// internally uses 64-bit long double; current extraction stays fixed point.
// Coupling permille = round(1000*M/L). Residual is RMS impedance error in mOhm.
volatile uint32_t WPT_estimatedSelfInductanceNanoHenry = 0UL;
volatile uint32_t WPT_estimatedMutualInductanceNanoHenry = 0UL;
volatile uint32_t WPT_estimatedEquivalentAcLoadMilliOhm = 0UL;
// Retained legacy name, but deliberately NOT a numerical estimate.
volatile uint32_t WPT_estimatedDcLoadMilliOhm = 0xFFFFFFFFUL;
volatile uint16_t WPT_estimatedDcLoadValid = 0U;
volatile int32_t WPT_estimatedEquivalentAcReactanceMilliOhm = 0L;
volatile int32_t WPT_estimatedThirdAcResistanceMilliOhm = 0L;
volatile int32_t WPT_estimatedThirdAcReactanceMilliOhm = 0L;
// Solver flags use WPTCE_* definitions in wpt_complex_estimator.h.
volatile uint16_t WPT_identificationSolverFlags = WPTCE_BAD_INPUT;
volatile uint16_t WPT_identificationSolutionsChecked = 0U;
volatile uint32_t WPT_identificationNormalizedResidualPermille = 0UL;
volatile uint32_t WPT_identificationLRelativeSensitivityPermille = 0UL;
volatile uint32_t WPT_identificationMRelativeSensitivityPermille = 0UL;
volatile uint32_t WPT_identificationAlternativeSelfInductanceNanoHenry = 0UL;
volatile uint32_t WPT_identificationAlternativeMutualInductanceNanoHenry = 0UL;
// Engineering uncertainty actually supplied to each fit observation, in mOhm.
volatile uint32_t WPT_identificationImpedanceSigmaMilliOhm[4] = {0UL,0UL,0UL,0UL};
// Static storage keeps the new solver input/result off the small C28x stack.
static WPTCE_Input WPT_complexEstimatorInput;
static WPTCE_Result WPT_complexEstimatorResult;
volatile uint16_t WPT_estimatedCouplingPermille = 0U;
volatile uint32_t WPT_identificationResidualMilliOhm = 0UL;

//=============================================================================
// Fixed-point trigonometry and private accumulation storage
//=============================================================================
// One-cycle Q15 sine table. Entry k approximates sin(2*pi*k/256)*32767.
// The final repeated zero allows interpolation between entry 255 and 256
// without a special wrap-around branch.
static const int16_t WPT_sineTableQ15[257] =
{
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
    6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285,
    32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683,
    27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868,
    18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
    12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179,
    6393, 5602, 4808, 4011, 3212, 2410, 1608, 804,
    0, -804, -1608, -2410, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
    -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
    -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179,
    -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804,
    0
};

// These variables are private working storage. Public CCS result variables are
// declared above. Sums are 64-bit so averaging several 32-bit phasors cannot
// overflow.
static uint32_t WPT_identificationCandidateFrequencyHz = 0UL;
static uint16_t WPT_identificationSettlingCount = 0U;
static int64_t WPT_identificationFrequencySum = 0LL;
static int64_t WPT_identificationI1RealSum = 0LL;
static int64_t WPT_identificationI1ImagSum = 0LL;
static int64_t WPT_identificationI3RealSum = 0LL;
static int64_t WPT_identificationI3ImagSum = 0LL;
static uint16_t WPT_identificationThirdValidCount = 0U;
static void WPT_modelPrimaryVoltage(uint16_t point);
//=============================================================================
// Function declarations
//=============================================================================
void WPT_setupPwms();
uint16_t WPT_loadPwmControllerCase(uint16_t controllerCase);
void WPT_requestPwmControllerCase(uint16_t controllerCase);
void WPT_servicePwmTransition(void);
void WPT_forceBridgePwmLow(void);
void WPT_releaseBridgePwm(void);
void WPT_configureBridgeTripLow(void);
void WPT_setupCurrentSensing(void);
void WPT_updateCurrentMeasurement(void);
void WPT_analyzeCurrentCapture(void);
int32_t WPT_currentCountsToMilliAmps(int32_t currentCounts);
uint32_t WPT_getCurrentSamplePwmPhaseQ32(void);
void WPT_extractCurrentHarmonics(uint32_t frequencyHz, int32_t meanCounts);
void WPT_updateIdentificationState(void);
void WPT_resetIdentification(void);
void WPT_solveIdentification(void);
static void WPT_clearEstimatorResults(void);
uint32_t WPT_integerSquareRoot64(uint64_t value);
int32_t WPT_atan2CentiDegrees(int32_t imaginary, int32_t real);
int16_t WPT_sineQ15(uint32_t phaseQ32);
int32_t WPT_reactanceMilliOhm(uint32_t frequencyHz,
                              uint32_t inductanceNanoHenry);
int32_t WPT_capacitiveReactanceMilliOhm(uint32_t frequencyHz,
                                        uint32_t capacitancePicoFarad);
void WPT_pri_setupPwms(uint32_t base1, uint32_t base2,
                       uint16_t pwm_period_ticks, uint16_t pwm_cmpa_base1,
                       uint16_t pwm_cmpb_base1,
                       uint16_t pwm_dbred_ticks_base1,
                       uint16_t pwm_dbfed_ticks_base1,
                       uint16_t pwm_dbred_ticks_base2,
                       uint16_t pwm_dbfed_ticks_base2,
                       uint16_t phaseshift_Q1_Q4);
void WPT_sec_setupPwms(uint32_t base1, uint32_t base2,
                       uint16_t pwm_period_ticks, uint16_t pwm_cmpa_base1,
                       uint16_t pwm_cmpb_base1,
                       uint16_t pwm_dbred_ticks_base1,
                       uint16_t pwm_dbfed_ticks_base1,
                       uint16_t pwm_dbred_ticks_base2,
                       uint16_t pwm_dbfed_ticks_base2,
                       uint16_t phaseshift_Q1_Q5,
                       uint16_t phaseshift_Q5_Q8);
// Virtual timers inherited from the TI reference project. They are reserved
// for background A/B tasks even though those task calls are currently disabled.
int16_t vTimer0[4];
int16_t vTimer1[4];

/*
 * WPT_loadPwmControllerCase
 * -------------------------
 * Converts a human-selected case number into a complete runtime PWM profile.
 * Each profile supplies operating frequency and three synchronized phase
 * register values. The original floating-point expressions were evaluated
 * once and stored here as integer ticks so changing profiles performs no
 * runtime floating-point arithmetic.
 *
 * Up/down PWM frequency equation:
 *   fPWM   = fTBCLK/(2*TBPRD)
 *   TBPRD = fTBCLK/(2*fPWM).
 *
 * PHASE REVIEW (secondary PWM pins are unused in this experiment):
 *   Case 7 restores the user's primary-leg correction: PH_1_4 = 586.
 *   With a 100 MHz time base, TBPRD1 = floor(100e6/(2*85e3)) = 588,
 *   CMPA1 = CMPA2 = 294, and the sync load delay is two ticks. EPWM2's
 *   down-compare rising edge is at 2 + 586 - 294 = 294, coincident with
 *   EPWM1's up-compare falling edge. Its next falling edge is at
 *   2 + 586 + 294 = 882, coincident with EPWM1's rising edge.
 *   This checks the nominal raw A waveforms, before dead band/driver delay.
 *
 *   The remaining legacy values are retained, not certified by their case
 *   labels: case 7 PH_1_5 = 100 does NOT put EPWM4A 180 degrees from EPWM1A.
 *   EPWM4 forwards incoming sync to EPWM5, so PH_5_8 is not an independent
 *   offset measured from EPWM4A. Case 5 PH_1_4 = 495 gives a nominal
 *   497-tick primary-leg lag, rather than the requested 500-tick half-cycle.
 *   The voltage reconstruction uses the actual primary phase, including
 *   case 5's offset; it does not assume the Q/angle labels are exact.
 *   Secondary PWM phases do not enter this terminal-ratio estimator.
 *
 * Returns 1 for cases 1..8. Returns 0 for any unknown case; the transition
 * service treats that as a fault and leaves the bridge outputs low. A return
 * value of 1 means the case exists, not that its physical phases are verified.
 */
uint16_t WPT_loadPwmControllerCase(uint16_t controllerCase)
{
    uint32_t frequencyHz;

    // These integer values are the truncated results of the original case
    // expressions. Keeping them as integers prevents runtime floating point.
    switch(controllerCase)
    {
        case 1U: // Q1-Q3 180, Q5-Q7 180, Q1 leads Q5 72: 100 kHz
            frequencyHz = 100000UL;
            PH_1_4 = 495U;
            PH_1_5 = 700U;
            PH_5_8 = 698U;
            break;

        case 2U: // Alternate calibrated 100 kHz bridge phase profile
            frequencyHz = 100000UL;
            PH_1_4 = 396U;
            PH_1_5 = 600U;
            PH_5_8 = 530U;
            break;

        case 3U: // Q1-Q3 144, Q5-Q7 144, Q1 leads Q5 36: 100 kHz
            frequencyHz = 100000UL;
            PH_1_4 = 400U;
            PH_1_5 = 594U;
            PH_5_8 = 495U;
            break;

        case 4U: // Q1-Q3 180, Q5-Q7 180, Q1 lags Q5 90: 100 kHz
            frequencyHz = 100000UL;
            PH_1_4 = 495U;
            PH_1_5 = 305U;
            PH_5_8 = 240U;
            break;

        case 5U: // Q1-Q3 180, Q5-Q7 180, Q1 lags Q5 180: 100 kHz
            frequencyHz = 100000UL;
            PH_1_4 = 495U;
            PH_1_5 = 0U;
            PH_5_8 = 0U;
            break;

        case 6U: // Q1-Q3 144, Q5-Q7 144, Q1 leads Q5 36: 120 kHz
            frequencyHz = 120000UL;
            PH_1_4 = 333U;
            PH_1_5 = 495U;
            PH_5_8 = 413U;
            break;

        case 7U: // Q1-Q3 180, Q5-Q7 180, Q1 lags Q5 180: 85 kHz
            frequencyHz = 85000UL;
            // Preserve the user's corrected primary phase on startup AND
            // every runtime transition back to this case: 588 - 2 = 586.
            PH_1_4 = 586U;
            PH_1_5 = 100U;
            PH_5_8 = 0U;
            break;

        case 8U: // Q1-Q3 180, Q5-Q7 180, Q1 lags Q5 90: 100 kHz
            frequencyHz = 100000UL;
            PH_1_4 = 300U;
            PH_1_5 = 390U;
            PH_5_8 = 210U;
            break;

        default:
            return 0U;
    }

    WPT_pwmPeriodTicks = (uint16_t)(PSFB_PWMSYSCLOCK_FREQ_HZ /
                                   (2UL * frequencyHz));
    WPT_pwmFrequencyHz = (uint32_t)((PSFB_PWMSYSCLOCK_FREQ_HZ +
                         WPT_pwmPeriodTicks) / (2UL * WPT_pwmPeriodTicks));
    WPT_pwmActiveControllerCase = controllerCase;
    return 1U;
}

/*
 * WPT_configureBridgeTripLow
 * --------------------------
 * Programs the ePWM one-shot-trip response. TZA controls output A and TZB
 * controls output B. Setting both to LOW guarantees that a software OST trip
 * turns off both gate commands after the dead-band block.
 *
 * EALLOW/EDIS temporarily allow writes to protected device registers.
 */
void WPT_configureBridgeTripLow(void)
{
    EALLOW;

    // A one-shot trip must turn both pins of every half bridge off. This is
    // also the fail-safe action used if a transition requests an invalid case.
    EPWM_setTripZoneAction(PSFB_FB_PWM1_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM1_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM2_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM2_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM4_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM4_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM5_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(PSFB_FB_PWM5_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);

    EDIS;
}

/* Assert the one-shot trip on every bridge PWM. EPWM3 is intentionally absent:
 * it is the ADC sample-clock time base, not a gate-drive PWM in this program. */
void WPT_forceBridgePwmLow(void)
{
    WPT_configureBridgeTripLow();
    EPWM_forceTripZoneEvent(PSFB_FB_PWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PSFB_FB_PWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PSFB_FB_PWM4_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PSFB_FB_PWM5_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

/*
 * Release only trips created by this transition.
 * WPT_pwmPreexistingTripMask remembers any trip that was already active. A
 * preexisting hardware/system fault is therefore never cleared accidentally.
 */
void WPT_releaseBridgePwm(void)
{
    // Do not override an existing system-level disable or a one-shot trip
    // that was already active before this transition began.
    if(PSFB_enable != 0)
    {
        if((WPT_pwmPreexistingTripMask & 0x0001U) == 0U)
        {
            EPWM_clearTripZoneFlag(PSFB_FB_PWM1_BASE, EPWM_TZ_FLAG_OST);
        }
        if((WPT_pwmPreexistingTripMask & 0x0002U) == 0U)
        {
            EPWM_clearTripZoneFlag(PSFB_FB_PWM2_BASE, EPWM_TZ_FLAG_OST);
        }
        if((WPT_pwmPreexistingTripMask & 0x0004U) == 0U)
        {
            EPWM_clearTripZoneFlag(PSFB_FB_PWM4_BASE, EPWM_TZ_FLAG_OST);
        }
        if((WPT_pwmPreexistingTripMask & 0x0008U) == 0U)
        {
            EPWM_clearTripZoneFlag(PSFB_FB_PWM5_BASE, EPWM_TZ_FLAG_OST);
        }
    }
}

/* Queue a nonblocking request. The main loop later performs the safety-critical
 * reconfiguration in WPT_servicePwmTransition(). */
void WPT_requestPwmControllerCase(uint16_t controllerCase)
{
    if(controllerCase == WPT_pwmActiveControllerCase)
    {
        return;
    }

    WPT_pwmRequestedControllerCase = controllerCase;
    WPT_pwmTransitionRequest = 1U;
    WPT_pwmTransitionState = WPT_PWM_TRANSITION_REQUESTED;
}

/*
 * WPT_servicePwmTransition
 * ------------------------
 * Safely changes PWM frequency and phase without writing live bridge timing
 * registers while switches are active.
 *
 * Sequence:
 *   1. Stop new ADC triggers.
 *   2. Remember preexisting trip flags.
 *   3. Force all bridge outputs low and wait PRE_BLANK_US.
 *   4. Stop TBCLKSYNC so all ePWM time bases are frozen together.
 *   5. Load the requested case and rebuild PWM registers.
 *   6. Reset partial/old ADC measurement state.
 *   7. Restart TBCLKSYNC, wait POST_BLANK_US for resonant current to decay.
 *   8. Clear only transition-owned trips and restart sampling.
 *
 * This routine intentionally blocks during the two short blanking delays. ADC
 * triggering is disabled during that time, so the delay cannot overflow the
 * current buffer or contaminate a measurement window.
 */
void WPT_servicePwmTransition(void)
{
    if((WPT_pwmTransitionRequest != 0U) &&
       ((WPT_pwmTransitionState == WPT_PWM_TRANSITION_IDLE) ||
        (WPT_pwmTransitionState == WPT_PWM_TRANSITION_COMPLETE)))
    {
        // This permits a manual CCS request: write the desired case to
        // WPT_pwmRequestedControllerCase, then set WPT_pwmTransitionRequest=1.
        WPT_pwmTransitionState = WPT_PWM_TRANSITION_REQUESTED;
    }

    if(WPT_pwmTransitionState != WPT_PWM_TRANSITION_REQUESTED)
    {
        return;
    }

    WPT_pwmTransitionState = WPT_PWM_TRANSITION_OUTPUTS_OFF;

    // Stop sample acquisition first so the intentional off interval cannot
    // create an ADC overflow or be mistaken for a measurement point.
    EPWM_disableADCTrigger(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);
    WPT_pwmPreexistingTripMask = 0U;
    if((EPWM_getTripZoneFlagStatus(PSFB_FB_PWM1_BASE) &
        EPWM_TZ_FLAG_OST) != 0U)
    {
        WPT_pwmPreexistingTripMask |= 0x0001U;
    }
    if((EPWM_getTripZoneFlagStatus(PSFB_FB_PWM2_BASE) &
        EPWM_TZ_FLAG_OST) != 0U)
    {
        WPT_pwmPreexistingTripMask |= 0x0002U;
    }
    if((EPWM_getTripZoneFlagStatus(PSFB_FB_PWM4_BASE) &
        EPWM_TZ_FLAG_OST) != 0U)
    {
        WPT_pwmPreexistingTripMask |= 0x0004U;
    }
    if((EPWM_getTripZoneFlagStatus(PSFB_FB_PWM5_BASE) &
        EPWM_TZ_FLAG_OST) != 0U)
    {
        WPT_pwmPreexistingTripMask |= 0x0008U;
    }
    WPT_forceBridgePwmLow();
    DEVICE_DELAY_US(WPT_PWM_TRANSITION_PRE_BLANK_US);

    PSFB_HAL_disablePWMCLKCounting();
    if(WPT_loadPwmControllerCase(WPT_pwmRequestedControllerCase) == 0U)
    {
        WPT_pwmTransitionRequest = 0U;
        WPT_pwmTransitionState = WPT_PWM_TRANSITION_ERROR;
        return;
    }

    // Rebuild all four bridge PWMs with the new period and phase values while
    // the time bases are stopped and the one-shot trip remains asserted.
    WPT_setupPwms();
    WPT_forceBridgePwmLow();

    WPT_currentCaptureIndex = 0U;
    WPT_currentCaptureReady = 0U;
    WPT_currentCaptureValid = 1U;
    WPT_currentLastCaptureValid = 0U;
    WPT_currentFrequencyValid = 0U;
    WPT_currentHarmonicValid = 0U;
    WPT_currentThirdHarmonicValid = 0U;
    ADC_clearInterruptStatus(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(WPT_CURRENT_VOUT_ADC_BASE,
                                     ADC_INT_NUMBER1);
    EPWM_clearADCTriggerFlag(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);

    PSFB_HAL_enablePWMCLKCounting();
    DEVICE_DELAY_US(WPT_PWM_TRANSITION_POST_BLANK_US);
    WPT_releaseBridgePwm();
    EPWM_enableADCTrigger(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);

    WPT_pwmTransitionRequest = 0U;
    WPT_pwmTransitionCount++;
    WPT_pwmTransitionState = WPT_PWM_TRANSITION_COMPLETE;
}


/*
 * main
 * ----
 * Performs one-time device initialization and then runs a polling loop.
 * No current-sampling ISR is used: EPWM3 starts both ADC conversions, the loop
 * detects the ADCC completion flag, and it stores the simultaneous ADCC/ADCB
 * results. When 1024 pairs are ready, sampling pauses while the buffer is
 * analyzed.
 */
void main(void)
{
    // STARTUP STEP 1: Load the first runtime PWM profile before configuring the
    // ePWM modules.
    // An invalid case leaves the processor here before any gate signal starts.
    if(WPT_loadPwmControllerCase(WPT_PWM_FIRST_CONTROLLER_CASE) == 0U)
    {
        WPT_pwmTransitionState = WPT_PWM_TRANSITION_ERROR;
        for(;;)
        {
        }
    }
    // STARTUP STEP 2: Initialize system clocks, PLL, watchdog/device services,
    // and CPU timers inherited from the TI reference project.
    PSFB_HAL_setupDevice();

    // STARTUP STEP 3: In a FLASH build, copy the linker-designated time-critical
    // section from FLASH to RAM. Linker symbols supply source, size, and target.
    memcpy(&isrcodefuncsRunStart, &isrcodefuncsLoadStart,
           (size_t)&isrcodefuncsLoadSize);

    // STARTUP STEP 4: Initialize original PSFB project states and enable flags.
    PSFB_globalVariablesInit();

    // STARTUP STEP 5: Configure custom WPT PWMs. The original TI PWM setup is
    // disabled because it would overwrite the custom bridge configuration.
//  PSFB_HAL_setupPwms();
    WPT_setupPwms();

    // The original all-channel ADC initializer is disabled; only the two
    // current-sensor SOCs required by this experiment are configured below.
//  PSFB_HAL_setupADC();

    //
    // Set up the ACS37030 VOUT and VREF channels. EPWM3 is used only as a
    // 1 MHz ADC sample clock; its output pins are not enabled.
    //
    WPT_setupCurrentSensing();

    // The original PWM1 ADC trigger is disabled because EPWM3 now supplies an
    // independent 1 MHz current-sampling trigger.
//  PSFB_HAL_setupADCSOCTrigger(PSFB_FB_PWM1_BASE);

    // General board GPIO setup from the reference project is not used by this
    // reduced experiment.
//  PSFB_HAL_setupGpios();

    // STARTUP STEP 6: Retain TI's Software Frequency Response Analyzer setup.
    // SFRA is independent of the WPT parameter-identification calculations.
    PSFB_setupSFRA();

    // STARTUP STEP 7: Connect GPIO pins to ePWM only after registers and
    // fail-safe trip behavior have been configured.
    PSFB_HAL_setPinsAsPWM();

    // Current capture is polled, so the original control ISR mapping is off.
//  PSFB_HAL_setupInterrupt();

    // STARTUP STEP 8: Apply the project enable/trip state, then start all ePWM
    // time bases together through TBCLKSYNC.
    PSFB_EnablePSFBPWM();
    PSFB_HAL_enablePWMCLKCounting();
    
    for(;;)
    {
        // FOREGROUND LOOP: A pending PWM transition is serviced before reading
        // another ADC result. Buffer analysis runs only after sampling has
        // filled and frozen all 1024 entries.
        // Service a requested case change before accepting more ADC samples.
        WPT_servicePwmTransition();

        // Update the CCS-visible current variables when the latest pair of
        // synchronized ADC conversions has completed.
        WPT_updateCurrentMeasurement();

        if(WPT_currentCaptureReady != 0U)
        {
            WPT_analyzeCurrentCapture();
        }

        // Original TI background task state machine; intentionally disabled.
//      (*Alpha_State_Ptr)();

    }
}

/*
 * WPT_setupCurrentSensing
 * -----------------------
 * Configures two ADC modules to sample the sensor simultaneously:
 *   ADCC SOC15 = ACS37030 VOUT
 *   ADCB SOC14 = ACS37030 VREF
 * Both SOCs use the same EPWM3 SOCA trigger. Subtracting the two results
 * removes the sensor's reference/zero-current voltage.
 *
 * ACQPS=20 means a 21-SYSCLK sample-and-hold window because the hardware uses
 * window_cycles=ACQPS+1. EPWM3 up-count period follows:
 *   TBPRD = fPWMCLK/sample_rate - 1.
 * Its output pin is not enabled; only the internal SOCA event is used.
 */
void WPT_setupCurrentSensing(void)
{
    EALLOW;

    // Both F280049C ADC channels use the internal 3.3 V reference. The
    // F28004x ADC is a fixed 12-bit, single-ended converter.
    ADC_setVREF(WPT_CURRENT_VOUT_ADC_BASE, ADC_REFERENCE_INTERNAL,
                ADC_REFERENCE_3_3V);
    ADC_setVREF(WPT_CURRENT_VREF_ADC_BASE, ADC_REFERENCE_INTERNAL,
                ADC_REFERENCE_3_3V);
    ADC_setPrescaler(WPT_CURRENT_VOUT_ADC_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(WPT_CURRENT_VREF_ADC_BASE, ADC_CLK_DIV_2_0);
    ADC_setInterruptPulseMode(WPT_CURRENT_VOUT_ADC_BASE,
                              ADC_PULSE_END_OF_CONV);

    ADC_enableConverter(WPT_CURRENT_VOUT_ADC_BASE);
    ADC_enableConverter(WPT_CURRENT_VREF_ADC_BASE);

    EDIS;

    DEVICE_DELAY_US(PSFB_ADC_POWERUP_US);

    ADC_setupSOC(WPT_CURRENT_VOUT_ADC_BASE, WPT_CURRENT_VOUT_SOC,
                 WPT_CURRENT_ADC_TRIGGER, WPT_CURRENT_VOUT_CHANNEL,
                 WPT_CURRENT_ADC_ACQPS);
    ADC_setupSOC(WPT_CURRENT_VREF_ADC_BASE, WPT_CURRENT_VREF_SOC,
                 WPT_CURRENT_ADC_TRIGGER, WPT_CURRENT_VREF_CHANNEL,
                 WPT_CURRENT_ADC_ACQPS);

    // ADCC interrupt 1 is used only as a conversion-complete flag. It is
    // polled in the background loop and is not mapped to a CPU ISR.
    ADC_setInterruptSource(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1,
                           WPT_CURRENT_VOUT_SOC);
    ADC_clearInterruptStatus(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(WPT_CURRENT_VOUT_ADC_BASE,
                                     ADC_INT_NUMBER1);
    ADC_enableInterrupt(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1);

    // EPWM3 is a sample clock only. In up-count mode, TBPRD = 99 gives a
    // 1 MHz trigger from the 100 MHz PWM time-base clock. This gives 3.33
    // samples per cycle at the highest requested 300 kHz third harmonic while
    // retaining servicing margin for the present polling implementation.
    EPWM_setPeriodLoadMode(WPT_CURRENT_SAMPLE_PWM_BASE,
                           EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(WPT_CURRENT_SAMPLE_PWM_BASE,
                           WPT_CURRENT_SAMPLE_PWM_PERIOD);
    EPWM_setTimeBaseCounter(WPT_CURRENT_SAMPLE_PWM_BASE, 0U);
    EPWM_setPhaseShift(WPT_CURRENT_SAMPLE_PWM_BASE, 0U);
    EPWM_setTimeBaseCounterMode(WPT_CURRENT_SAMPLE_PWM_BASE,
                               EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(WPT_CURRENT_SAMPLE_PWM_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setADCTriggerSource(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A,
                             EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(WPT_CURRENT_SAMPLE_PWM_BASE,
                                    EPWM_SOC_A, 1U);
    EPWM_clearADCTriggerFlag(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);
    EPWM_enableADCTrigger(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);
}

/*
 * WPT_updateCurrentMeasurement
 * ----------------------------
 * Called repeatedly by main(). When ADCC says the simultaneous conversion is
 * complete, this function reads VOUT and VREF, stores their signed difference,
 * and advances the 1024-sample buffer.
 *
 * On the first sample it records the PWM1 phase at the actual ADC trigger time;
 * this is theta[0] for the later harmonic fit. An ADC overflow means software failed to
 * service a conversion before the next one completed, so the whole capture is
 * marked invalid instead of publishing corrupted harmonic results.
 */
void WPT_updateCurrentMeasurement(void)
{
    int16_t currentCounts;

    if((WPT_currentCaptureReady == 0U) &&
       ADC_getInterruptStatus(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1))
    {
        WPT_currentVoutRaw = ADC_readResult(WPT_CURRENT_VOUT_RESULT_BASE,
                                            WPT_CURRENT_VOUT_SOC);
        WPT_currentVrefRaw = ADC_readResult(WPT_CURRENT_VREF_RESULT_BASE,
                                            WPT_CURRENT_VREF_SOC);

        currentCounts = (int16_t)WPT_currentVoutRaw -
                        (int16_t)WPT_currentVrefRaw;
        if(WPT_currentCaptureIndex == 0U)
        {
            WPT_currentCaptureStartPhaseQ32 =
                    WPT_getCurrentSamplePwmPhaseQ32();
        }
        WPT_currentSampleCounts[WPT_currentCaptureIndex] = currentCounts;
        WPT_currentCaptureIndex++;
        if(WPT_currentCaptureIndex >= WPT_CURRENT_TRACE_LENGTH)
        {
            WPT_currentCaptureReady = 1U;
        }

        ADC_clearInterruptStatus(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1);

        // If CCS halted the CPU long enough to miss a conversion-complete
        // event, clear the overflow so sampling resumes normally.
        if(ADC_getInterruptOverflowStatus(WPT_CURRENT_VOUT_ADC_BASE,
                                          ADC_INT_NUMBER1))
        {
            WPT_currentAdcOverflowCount++;
            WPT_currentCaptureValid = 0U;
            ADC_clearInterruptOverflowStatus(WPT_CURRENT_VOUT_ADC_BASE,
                                             ADC_INT_NUMBER1);
            ADC_clearInterruptStatus(WPT_CURRENT_VOUT_ADC_BASE,
                                     ADC_INT_NUMBER1);
        }
    }
}

/*
 * Convert signed ADC difference counts to signed milliamps.
 *
 *   I_mA = counts * (3300 mV/4096 counts)/(66 mV/A)
 *        = counts * 34375/2816.
 *
 * Half the denominator is added/subtracted before division to round to the
 * nearest integer while preserving correct behavior for negative current.
 */
int32_t WPT_currentCountsToMilliAmps(int32_t currentCounts)
{
    int32_t scaledCurrent = currentCounts * WPT_CURRENT_MA_NUMERATOR;

    // Round to the nearest whole milliamp instead of truncating toward zero.
    if(scaledCurrent >= 0L)
    {
        scaledCurrent += WPT_CURRENT_MA_DENOMINATOR / 2L;
    }
    else
    {
        scaledCurrent -= WPT_CURRENT_MA_DENOMINATOR / 2L;
    }

    return scaledCurrent / WPT_CURRENT_MA_DENOMINATOR;
}

/*
 * Return sin(phase) in Q15 without floating point.
 * Q32 phase uses 2^32 counts per revolution. The top eight bits select one of
 * 256 table intervals and the next eight bits linearly interpolate:
 *   y = y0 + (y1-y0)*fraction/256.
 */
int16_t WPT_sineQ15(uint32_t phaseQ32)
{
    uint16_t tableIndex = (uint16_t)(phaseQ32 >> 24U);
    uint16_t fractionQ8 = (uint16_t)((phaseQ32 >> 16U) & 0xFFUL);
    int32_t firstValue = (int32_t)WPT_sineTableQ15[tableIndex];
    int32_t difference = (int32_t)WPT_sineTableQ15[tableIndex + 1U] -
                         firstValue;

    return (int16_t)(firstValue +
            ((difference * (int32_t)fractionQ8) >> 8U));
}

/*
 * Integer square root, rounded to the nearest integer.
 * This binary restoring algorithm calculates floor(sqrt(value)); the final
 * remainder test rounds upward when that is closer. It is used for phasor
 * magnitude sqrt(real^2+imag^2) and for mutual inductance.
 */
uint32_t WPT_integerSquareRoot64(uint64_t value)
{
    uint64_t remainder = 0ULL;
    uint64_t root = 0ULL;
    uint64_t bit = 1ULL << 62U;

    while(bit > value)
    {
        bit >>= 2U;
    }

    while(bit != 0ULL)
    {
        if(value >= (root + bit))
        {
            value -= root + bit;
            root = (root >> 1U) + bit;
        }
        else
        {
            root >>= 1U;
        }
        bit >>= 2U;
    }

    remainder = value;
    if((remainder > root) && (root < 0xFFFFFFFFULL))
    {
        root++;
    }

    return (uint32_t)root;
}

/*
 * Fixed-point atan2(imaginary, real), returned in centidegrees.
 *
 * atan(imag/real) alone cannot distinguish opposite quadrants and divides by
 * zero when real=0. atan2 uses both signs and returns -18000..+18000, meaning
 * -180.00..+180.00 degrees. The 15 CORDIC rotations use only shifts and adds;
 * atanTableCentiDegrees[k] stores atan(2^-k) in 0.01-degree units.
 */
int32_t WPT_atan2CentiDegrees(int32_t imaginary, int32_t real)
{
    static const int16_t atanTableCentiDegrees[15] =
    {
        4500, 2657, 1404, 713, 358, 179, 90, 45,
        22, 11, 6, 3, 1, 1, 0
    };
    // Guard bits keep small mA/mV phasors from losing direction during the
    // CORDIC shifts. Multiplication also avoids left-shifting negative values.
    int64_t x = (int64_t)real * 65536LL;
    int64_t y = (int64_t)imaginary * 65536LL;
    int64_t nextX;
    int64_t nextY;
    int32_t angle = 0L;
    uint16_t index;

    if((x == 0LL) && (y == 0LL))
    {
        return 0L;
    }

    if(x < 0LL)
    {
        if(y >= 0LL)
        {
            angle = 18000L;
        }
        else
        {
            angle = -18000L;
        }
        x = -x;
        y = -y;
    }

    for(index = 0U; index < 15U; index++)
    {
        if(y == 0LL)
        {
            break;
        }
        if(y > 0LL)
        {
            nextX = x + (y >> index);
            nextY = y - (x >> index);
            angle += (int32_t)atanTableCentiDegrees[index];
        }
        else
        {
            nextX = x - (y >> index);
            nextY = y + (x >> index);
            angle -= (int32_t)atanTableCentiDegrees[index];
        }
        x = nextX;
        y = nextY;
    }

    if(angle > 18000L)
    {
        angle -= 36000L;
    }
    else if(angle <= -18000L)
    {
        angle += 36000L;
    }

    return angle;
}

/*
 * Recover the phase of the first sample relative to PWM1.
 *
 * EPWM3 triggered the ADC at its last zero event, but software reads the
 * registers later. EPWM3's current count estimates that elapsed latency. PWM1
 * is an up/down counter, so its position within a full 2*TBPRD cycle is:
 *   phase_ticks = TBCTR                    while counting up
 *   phase_ticks = 2*TBPRD - TBCTR          while counting down.
 *
 * The trigger-time position is phase_now - elapsed + calibration. Conversion
 * to Q32 is:
 *   phase_Q32 = phase_ticks*2^32/(2*TBPRD).
 */
uint32_t WPT_getCurrentSamplePwmPhaseQ32(void)
{
    uint16_t sampleCounterBefore;
    uint16_t sampleCounterAfter;
    uint16_t sampleElapsedTicks;
    uint16_t pwmCounter;
    uint16_t pwmDirection;
    uint16_t pwmPeriod;
    uint32_t fullPeriodTicks;
    uint32_t phaseTicksNow;
    int32_t samplePhaseTicks;

    // EPWM3 triggered the ADC at its preceding counter-zero event. Reading
    // its current counter value removes variable main-loop polling latency.
    sampleCounterBefore = EPWM_getTimeBaseCounterValue(
            WPT_CURRENT_SAMPLE_PWM_BASE);
    pwmCounter = EPWM_getTimeBaseCounterValue(PSFB_FB_PWM1_BASE);
    pwmDirection = EPWM_getTimeBaseCounterDirection(PSFB_FB_PWM1_BASE);
    sampleCounterAfter = EPWM_getTimeBaseCounterValue(
            WPT_CURRENT_SAMPLE_PWM_BASE);

    if(sampleCounterAfter >= sampleCounterBefore)
    {
        sampleElapsedTicks = (uint16_t)
                (((uint32_t)sampleCounterBefore +
                  (uint32_t)sampleCounterAfter) / 2UL);
    }
    else
    {
        // A wrap here means another trigger occurred during the register
        // reads. The ADC overflow flag will invalidate this capture.
        sampleElapsedTicks = sampleCounterBefore;
    }

    pwmPeriod = EPWM_getTimeBasePeriod(PSFB_FB_PWM1_BASE);
    fullPeriodTicks = (uint32_t)pwmPeriod * 2UL;
    if(fullPeriodTicks == 0UL)
    {
        return 0UL;
    }

    if(pwmDirection == EPWM_TIME_BASE_STATUS_COUNT_UP)
    {
        phaseTicksNow = (uint32_t)pwmCounter;
    }
    else
    {
        phaseTicksNow = fullPeriodTicks - (uint32_t)pwmCounter;
    }

    samplePhaseTicks = (int32_t)phaseTicksNow -
                       (int32_t)sampleElapsedTicks +
                       (int32_t)WPT_CURRENT_PHASE_CAL_TICKS;
    while(samplePhaseTicks < 0L)
    {
        samplePhaseTicks += (int32_t)fullPeriodTicks;
    }
    while((uint32_t)samplePhaseTicks >= fullPeriodTicks)
    {
        samplePhaseTicks -= (int32_t)fullPeriodTicks;
    }

    return (uint32_t)(((uint64_t)(uint32_t)samplePhaseTicks << 32U) /
                      (uint64_t)fullPeriodTicks);
}

/*
 * Fit DC, cos(theta), sin(theta), cos(3*theta), sin(3*theta) together.
 * A 1024-sample buffer need not contain a whole number of PWM cycles, so
 * simply multiplying by 2/N leaks the fundamental into the third harmonic.
 * Least squares solves (B^T B) a = B^T x, including all cross terms.
 * Basis values are Q15; normalized normal equations and coefficients are Q20.
 * A five-by-five Gaussian elimination uses 64-bit intermediates; no float.
 * With >60 fundamental cycles/window these basis columns are well separated.
 */
static uint16_t WPT_fitCurrentHarmonics(uint32_t phaseStep, int32_t meanCounts,
                                      int32_t *i1r, int32_t *i1i,
                                      int32_t *i3r, int32_t *i3i)
{
    int64_t matrix[5][6] = {{0LL}};
    int64_t coefficients[5] = {0LL};
    int32_t basis[5];
    uint32_t phase = WPT_currentCaptureStartPhaseQ32;
    uint16_t n, row, col, pivot;
    int32_t sample;
    int64_t factor;
    int64_t value;
    const int64_t q20 = 1048576LL;
    for(n = 0U; n < WPT_CURRENT_TRACE_LENGTH; n++)
    {
        basis[0] = 32768L;
        basis[1] = WPT_sineQ15(phase + 0x40000000UL);
        basis[2] = WPT_sineQ15(phase);
        basis[3] = WPT_sineQ15((uint32_t)(phase*3UL) + 0x40000000UL);
        basis[4] = WPT_sineQ15((uint32_t)(phase*3UL));
        sample = WPT_CURRENT_POLARITY *
                 ((int32_t)WPT_currentSampleCounts[n] - meanCounts);
        for(row = 0U; row < 5U; row++)
        {
            for(col = 0U; col < 5U; col++)
            {
                matrix[row][col] += (int64_t)basis[row] * basis[col];
            }
            matrix[row][5] += (int64_t)basis[row] * sample;
        }
        phase += phaseStep;
    }
    for(row = 0U; row < 5U; row++)
    {
        for(col = 0U; col < 5U; col++)
        {
            matrix[row][col] /= (int64_t)WPT_CURRENT_TRACE_LENGTH * 1024LL;
        }
        matrix[row][5] = matrix[row][5] * 32LL / WPT_CURRENT_TRACE_LENGTH;
    }
    for(pivot = 0U; pivot < 5U; pivot++)
    {
        // Reject a rank-deficient/ill-conditioned capture rather than divide
        // by a small pivot. Expected DC/AC diagonals are ~1 / ~0.5 in Q20.
        if(matrix[pivot][pivot] < q20/100LL)
        {
            return 0U;
        }
        for(row = (uint16_t)(pivot + 1U); row < 5U; row++)
        {
            factor = matrix[row][pivot] * q20 / matrix[pivot][pivot];
            for(col = (uint16_t)(pivot + 1U); col < 6U; col++)
            {
                matrix[row][col] -= factor * matrix[pivot][col] / q20;
            }
            matrix[row][pivot] = 0LL;
        }
    }
    for(row = 5U; row > 0U; row--)
    {
        pivot = (uint16_t)(row - 1U);
        value = matrix[pivot][5];
        for(col = (uint16_t)(pivot + 1U); col < 5U; col++)
        {
            value -= matrix[pivot][col] * coefficients[col] / q20;
        }
        coefficients[pivot] = value * q20 / matrix[pivot][pivot];
    }
    for(row = 1U; row < 5U; row++)
    {
        value = coefficients[row] * WPT_CURRENT_MA_NUMERATOR;
        factor = q20 * WPT_CURRENT_MA_DENOMINATOR;
        value += (value >= 0LL) ? factor/2LL : -factor/2LL;
        coefficients[row] = value / factor;
        if((coefficients[row] > 100000L) || (coefficients[row] < -100000L))
        {
            return 0U;
        }
    }
    *i1r = (int32_t)coefficients[1];
    *i1i = -(int32_t)coefficients[2];
    *i3r = (int32_t)coefficients[3];
    *i3i = -(int32_t)coefficients[4];
    return 1U;
}

/*
 * Current phasors use PWM1 COUNTER ZERO as the cosine time reference.
 * Use the known ratio of ADC-trigger ticks to bridge-cycle ticks, not a noisy
 * zero-crossing frequency, to advance phase. At 85 kHz the integer PWM period
 * actually gives ~85034 Hz. Using 85000 or a noisy current-frequency estimate
 * would accumulate phase error across the capture, tripled at harmonic 3.
 * The independently measured crossing frequency remains a lock/stability test.
 *
 * I = Ir+j*Ii means i(theta)=Ir*cos(theta)-Ii*sin(theta).
 * The joint least-squares fit avoids finite-window DC/fundamental/third leakage.
 * Higher unmodeled harmonics, ADC timing error and sensor response remain.
 */
void WPT_extractCurrentHarmonics(uint32_t frequencyHz, int32_t meanCounts)
{
    uint32_t phaseStep;
    uint32_t sampleTicks = (uint32_t)WPT_CURRENT_SAMPLE_PWM_PERIOD + 1UL;
    uint32_t fullPeriod = 2UL * WPT_pwmPeriodTicks;
    uint32_t difference;
    int32_t i1r, i1i, i3r, i3i;
    WPT_currentHarmonicValid = 0U;
    WPT_currentThirdHarmonicValid = 0U;
    if((WPT_currentFrequencyValid == 0U) ||
       (WPT_currentCaptureValid == 0U) || (fullPeriod == 0UL))
    {
        return;
    }
    difference = (frequencyHz > WPT_pwmFrequencyHz) ?
        frequencyHz-WPT_pwmFrequencyHz : WPT_pwmFrequencyHz-frequencyHz;
    if((difference > WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ) ||
       (6UL * sampleTicks >= fullPeriod))
    {
        return;
    }
    phaseStep = (uint32_t)(((uint64_t)sampleTicks << 32U) / fullPeriod);
    if(WPT_fitCurrentHarmonics(phaseStep, meanCounts, &i1r, &i1i, &i3r, &i3i)==0U)
    {
        return;
    }
    WPT_currentFundamentalRealMilliAmps = i1r;
    WPT_currentFundamentalImagMilliAmps = i1i;
    WPT_currentThirdRealMilliAmps = i3r;
    WPT_currentThirdImagMilliAmps = i3i;
    WPT_currentFundamentalAmplitudeMilliAmps = WPT_integerSquareRoot64(
        (uint64_t)((int64_t)i1r*i1r + (int64_t)i1i*i1i));
    WPT_currentThirdAmplitudeMilliAmps = WPT_integerSquareRoot64(
        (uint64_t)((int64_t)i3r*i3r + (int64_t)i3i*i3i));
    WPT_currentFundamentalPhaseCentiDegrees = WPT_atan2CentiDegrees(i1i, i1r);
    WPT_currentThirdPhaseCentiDegrees = WPT_atan2CentiDegrees(i3i, i3r);
    if(WPT_currentFundamentalAmplitudeMilliAmps >=
       (uint32_t)WPT_IDENT_MIN_FUNDAMENTAL_CURRENT_MA)
    {
        WPT_currentHarmonicValid = 1U;
    }
    if(WPT_currentThirdAmplitudeMilliAmps >=
       (uint32_t)WPT_IDENT_MIN_THIRD_CURRENT_MA)
    {
        WPT_currentThirdHarmonicValid = 1U;
    }
}

/* Absolute unsigned frequency difference; written without signed subtraction
 * so wrap-around cannot occur when the first argument is smaller. */
static uint32_t WPT_absoluteFrequencyDifference(uint32_t first,
                                                uint32_t second)
{
    return (first >= second) ? (first - second) : (second - first);
}



/*
 * Reconstruct primary bridge voltage using the ACTUAL primary PWM settings.
 * Phase reference is EPWM1 counter zero, not an assumed voltage zero angle.
 *
 * At each TBCLK interval, reproduce the raw AQ states and delayed gate turn-on:
 *   raw A rises -> high-side gate rises after RED, low-side gate turns off;
 *   raw A falls -> low-side gate rises after FED, high-side gate turns off.
 * With both gates off, ideal body diodes set the node according to current.
 * Positive i flows from EPWM1's switching node to EPWM2's switching node:
 *   node1 = 0, node2 = Vbus while both legs are blanked and i > 0;
 *   node1 = Vbus, node2 = 0 while blanked and i < 0.
 * This is NOT the assumption that bridge voltage is zero during dead time.
 * Reconstruct i(theta)=Re{I1 exp(j*theta)+I3 exp(j*3*theta)}.
 * Integrate one settled cycle with midpoint quadrature:
 *   Vh = (2/N) sum(v[k]*(cos(h*theta[k])-j*sin(h*theta[k]))).
 * Three cycles are simulated; the first two initialize AQ/dead-band history.
 *
 * Limitations: instantaneous diode commutation; no Coss/driver propagation,
 * diode drop or current harmonics above the third. Near zero current keep the
 * previous node level and raise DeadtimeUncertain. Approximate remains set
 * even without that flag. Measured complex voltage phasors bypass this model.
 */
static void WPT_modelPrimaryVoltage(uint16_t point)
{
    uint16_t period = WPT_pwmPeriodTicks;
    uint32_t fullPeriod = 2UL * (uint32_t)period;
    uint16_t compare = period / 2U;
    uint16_t slavePeriod = (uint16_t)(period - 1U);
    uint16_t slaveCounter = 0U;
    uint16_t slaveDown = 0U;
    uint16_t raw[2] = {0U, 0U};
    uint16_t previousRaw[2] = {0U, 0U};
    uint16_t node[2] = {0U, 0U};
    uint32_t age[2] = {0UL, 0UL};
    uint32_t risingDelay[2] = {PSFB_FB_DB_PWM_HS_COUNT_BASE1_pri,
                              PSFB_FB_DB_PWM_HS_COUNT_BASE2_pri};
    uint32_t fallingDelay[2] = {PSFB_FB_DB_PWM_LS_COUNT_BASE1_pri,
                               PSFB_FB_DB_PWM_LS_COUNT_BASE2_pri};
    int64_t sums[4] = {0LL, 0LL, 0LL, 0LL};
    int64_t denominator;
    int64_t currentQ15;
    int32_t voltage;
    int32_t components[4];
    int16_t trig[4];
    uint32_t phase;
    uint32_t tick;
    uint16_t cycle;
    uint16_t leg;
    uint16_t h;

    WPT_identificationVoltageValid[point] = 0U;
    WPT_identificationVoltageApproximate[point] = 0U;
    WPT_identificationDeadtimeUncertain[point] = 0U;
    if(WPT_identificationUseMeasuredVoltage[point] != 0U)
    {
        components[0] = WPT_identificationMeasuredV1RealMilliVolts[point];
        components[1] = WPT_identificationMeasuredV1ImagMilliVolts[point];
        components[2] = WPT_identificationMeasuredV3RealMilliVolts[point];
        components[3] = WPT_identificationMeasuredV3ImagMilliVolts[point];
    }
    else
    {
        WPT_identificationVoltageApproximate[point] = 1U;
        if((period < 8U) || (period > WPT_VOLTAGE_MODEL_MAX_PERIOD_TICKS) ||
           (PH_1_4 > slavePeriod) ||
           (WPT_PRIMARY_SYNC_DELAY_TICKS >= compare) ||
           (WPT_PRIMARY_SQUARE_PEAK_MV <= 0L) ||
           (WPT_PRIMARY_SQUARE_PEAK_MV > 1000000L))
        {
            return;
        }
        for(leg = 0U; leg < 2U; leg++)
        {
            if((risingDelay[leg] >= compare) ||
               (fallingDelay[leg] >= compare))
            {
                return;
            }
        }
        for(cycle = 0U; cycle < 3U; cycle++)
        {
            for(tick = 0UL; tick < fullPeriod; tick++)
            {
                // Master AQ: LOW at up-CMPA, HIGH at down-CMPA.
                raw[0] = ((tick < compare) ||
                          (tick >= fullPeriod - compare)) ? 1U : 0U;
                if(tick == WPT_PRIMARY_SYNC_DELAY_TICKS)
                {
                    slaveCounter = PH_1_4;
                    slaveDown = 1U;
                }
                else if(slaveDown != 0U)
                {
                    if(slaveCounter == 0U)
                    {
                        slaveDown = 0U;
                        slaveCounter = 1U;
                    }
                    else
                    {
                        slaveCounter--;
                    }
                }
                else if(slaveCounter >= slavePeriod)
                {
                    slaveDown = 1U;
                    slaveCounter--;
                }
                else
                {
                    slaveCounter++;
                }
                if(slaveCounter == compare)
                {
                    raw[1] = slaveDown;
                }
                // Midpoint of this tick, avoiding a half-tick Fourier bias.
                phase = (uint32_t)((((uint64_t)tick * 2ULL + 1ULL) << 31U) /
                                   (uint64_t)fullPeriod);
                trig[0] = WPT_sineQ15(phase + 0x40000000UL);
                trig[1] = WPT_sineQ15(phase);
                trig[2] = WPT_sineQ15((uint32_t)(phase * 3UL) + 0x40000000UL);
                trig[3] = WPT_sineQ15((uint32_t)(phase * 3UL));
                currentQ15 =
                    (int64_t)WPT_identificationI1RealMilliAmps[point]*trig[0] -
                    (int64_t)WPT_identificationI1ImagMilliAmps[point]*trig[1] +
                    (int64_t)WPT_identificationI3RealMilliAmps[point]*trig[2] -
                    (int64_t)WPT_identificationI3ImagMilliAmps[point]*trig[3];
                for(leg = 0U; leg < 2U; leg++)
                {
                    if(raw[leg] != previousRaw[leg])
                    {
                        age[leg] = 0UL;
                        previousRaw[leg] = raw[leg];
                    }
                    else if(age[leg] < fullPeriod)
                    {
                        age[leg]++;
                    }
                    if((raw[leg] != 0U) && (age[leg] >= risingDelay[leg]))
                    {
                        node[leg] = 1U;
                    }
                    else if((raw[leg] == 0U) && (age[leg] >= fallingDelay[leg]))
                    {
                        node[leg] = 0U;
                    }
                    else if(currentQ15 > (int64_t)WPT_DEADTIME_MIN_CURRENT_MA*32768LL)
                    {
                        node[leg] = leg;  // positive current: node1 low, node2 high
                    }
                    else if(currentQ15 < -(int64_t)WPT_DEADTIME_MIN_CURRENT_MA*32768LL)
                    {
                        node[leg] = (uint16_t)(1U - leg);
                    }
                    else if(cycle == 2U)
                    {
                        WPT_identificationDeadtimeUncertain[point] = 1U;
                    }
                }
                if(cycle == 2U)
                {
                    voltage = WPT_PRIMARY_SQUARE_PEAK_MV *
                              ((int32_t)node[0] - (int32_t)node[1]);
                    for(h = 0U; h < 4U; h++)
                    {
                        sums[h] += (int64_t)voltage * trig[h];
                    }
                }
            }
        }
        denominator = (int64_t)fullPeriod * 32768LL;
        for(h = 0U; h < 4U; h++)
        {
            int64_t value = 2LL * sums[h];
            value += (value >= 0LL) ? denominator/2LL : -denominator/2LL;
            components[h] = (int32_t)(value / denominator);
        }
        components[1] = -components[1];
        components[3] = -components[3];
    }
    for(h = 0U; h < 4U; h++)
    {
        if((components[h] > 2000000L) || (components[h] < -2000000L))
        {
            return;
        }
    }
    if((components[0] == 0L) && (components[1] == 0L))
    {
        return;
    }
    WPT_identificationV1RealMilliVolts[point] = components[0];
    WPT_identificationV1ImagMilliVolts[point] = components[1];
    WPT_identificationV3RealMilliVolts[point] = components[2];
    WPT_identificationV3ImagMilliVolts[point] = components[3];
    WPT_voltageFundamentalAmplitudeMilliVolts = WPT_integerSquareRoot64(
        (uint64_t)((int64_t)components[0]*components[0] +
                   (int64_t)components[1]*components[1]));
    WPT_voltageThirdAmplitudeMilliVolts = WPT_integerSquareRoot64(
        (uint64_t)((int64_t)components[2]*components[2] +
                   (int64_t)components[3]*components[3]));
    WPT_voltageFundamentalPhaseCentiDegrees =
        WPT_atan2CentiDegrees(components[1], components[0]);
    WPT_voltageThirdPhaseCentiDegrees =
        WPT_atan2CentiDegrees(components[3], components[2]);
    WPT_identificationVoltageValid[point] = 1U;
}

/* Reset the sums used to average several complete 1024-sample captures. */
static void WPT_clearIdentificationAccumulator(void)
{
    WPT_identificationCollectionCount = 0U;
    WPT_identificationFrequencySum = 0LL;
    WPT_identificationI1RealSum = 0LL;
    WPT_identificationI1ImagSum = 0LL;
    WPT_identificationI3RealSum = 0LL;
    WPT_identificationI3ImagSum = 0LL;
    WPT_identificationThirdValidCount = 0U;
}

/*
 * Determine whether repeated captures report a stable operating frequency.
 * A new candidate starts whenever the latest value differs by more than the
 * configured tolerance. Otherwise a cumulative mean is updated:
 *
 *   new_mean = (old_mean*count + new_value)/(count+1).
 *
 * The function returns 1 after STABLE_CAPTURE_COUNT consecutive compatible
 * estimates and 0 while it is still waiting.
 */
static uint16_t WPT_updateStableFrequency(uint32_t frequencyHz)
{
    if((WPT_identificationCandidateFrequencyHz == 0UL) ||
       (WPT_absoluteFrequencyDifference(
               frequencyHz, WPT_identificationCandidateFrequencyHz) >
        WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ))
    {
        WPT_identificationCandidateFrequencyHz = frequencyHz;
        WPT_identificationStableCount = 1U;
    }
    else
    {
        WPT_identificationCandidateFrequencyHz = (uint32_t)
                (((uint64_t)WPT_identificationCandidateFrequencyHz *
                  (uint64_t)WPT_identificationStableCount +
                  (uint64_t)frequencyHz) /
                 (uint64_t)(WPT_identificationStableCount + 1U));
        if(WPT_identificationStableCount < 0xFFFFU)
        {
            WPT_identificationStableCount++;
        }
    }

    return (WPT_identificationStableCount >=
            WPT_IDENT_STABLE_CAPTURE_COUNT) ? 1U : 0U;
}

/* Add the latest frequency and phasor components to 64-bit sums. The third
 * harmonic has its own valid count because low-amplitude or aliased captures
 * must not be averaged into I3. */
static void WPT_accumulateIdentificationCapture(void)
{
    WPT_identificationFrequencySum += (int64_t)WPT_pwmFrequencyHz;
    WPT_identificationI1RealSum +=
            (int64_t)WPT_currentFundamentalRealMilliAmps;
    WPT_identificationI1ImagSum +=
            (int64_t)WPT_currentFundamentalImagMilliAmps;
    if(WPT_currentThirdHarmonicValid != 0U)
    {
        WPT_identificationI3RealSum +=
                (int64_t)WPT_currentThirdRealMilliAmps;
        WPT_identificationI3ImagSum +=
                (int64_t)WPT_currentThirdImagMilliAmps;
        WPT_identificationThirdValidCount++;
    }
    WPT_identificationCollectionCount++;
}

/*
 * Store averaged measurement point 0 or 1.
 * Fundamental sums always divide by the total collection count. Third-
 * harmonic sums divide only by valid third-harmonic captures and are accepted
 * only when MIN_THIRD_VALID_CAPTURE_COUNT was reached.
 */
static void WPT_storeIdentificationPoint(uint16_t point)
{
    uint16_t count = WPT_identificationCollectionCount;

    if((point > 1U) || (count == 0U))
    {
        return;
    }

    WPT_identificationFrequencyHz[point] = (uint32_t)
            (WPT_identificationFrequencySum / (int64_t)count);
    WPT_identificationI1RealMilliAmps[point] = (int32_t)
            (WPT_identificationI1RealSum / (int64_t)count);
    WPT_identificationI1ImagMilliAmps[point] = (int32_t)
            (WPT_identificationI1ImagSum / (int64_t)count);

    if(WPT_identificationThirdValidCount >=
       WPT_IDENT_MIN_THIRD_VALID_CAPTURE_COUNT)
    {
        WPT_identificationI3RealMilliAmps[point] = (int32_t)
                (WPT_identificationI3RealSum /
                 (int64_t)WPT_identificationThirdValidCount);
        WPT_identificationI3ImagMilliAmps[point] = (int32_t)
                (WPT_identificationI3ImagSum /
                 (int64_t)WPT_identificationThirdValidCount);
        WPT_identificationThirdValid[point] = 1U;
    }
    else
    {
        WPT_identificationI3RealMilliAmps[point] = 0L;
        WPT_identificationI3ImagMilliAmps[point] = 0L;
        WPT_identificationThirdValid[point] = 0U;
    }

    WPT_modelPrimaryVoltage(point);
    WPT_clearIdentificationAccumulator();
}

/*
 * Clear every identification result and return the collection state machine
 * to its first-frequency state. With automatic PWM transitions enabled, a
 * reset also requests the configured first controller case.
 *
 * In CCS, set WPT_identificationResetRequest=1 to start a new experiment.
 */
void WPT_resetIdentification(void)
{
    uint16_t point;

#if WPT_PWM_AUTO_TRANSITION_ENABLE
    if(WPT_pwmActiveControllerCase != WPT_PWM_FIRST_CONTROLLER_CASE)
    {
        WPT_requestPwmControllerCase(WPT_PWM_FIRST_CONTROLLER_CASE);
    }
#endif

    WPT_identificationState = WPT_IDENT_STATE_WAIT_FIRST;
    WPT_identificationResetRequest = 0U;
    WPT_identificationValid = 0U;
    WPT_identificationModelWarning = 0U;
    WPT_identificationStableCount = 0U;
    WPT_identificationSettlingCount = 0U;
    WPT_identificationCandidateFrequencyHz = 0UL;
    WPT_clearEstimatorResults();
    WPT_clearIdentificationAccumulator();

    for(point = 0U; point < 2U; point++)
    {
        WPT_identificationVoltageValid[point] = 0U;
        WPT_identificationVoltageApproximate[point] = 0U;
        WPT_identificationDeadtimeUncertain[point] = 0U;
        WPT_identificationV1RealMilliVolts[point] = 0L;
        WPT_identificationV1ImagMilliVolts[point] = 0L;
        WPT_identificationV3RealMilliVolts[point] = 0L;
        WPT_identificationV3ImagMilliVolts[point] = 0L;
        WPT_identificationFrequencyHz[point] = 0UL;
        WPT_identificationI1RealMilliAmps[point] = 0L;
        WPT_identificationI1ImagMilliAmps[point] = 0L;
        WPT_identificationI3RealMilliAmps[point] = 0L;
        WPT_identificationI3ImagMilliAmps[point] = 0L;
        WPT_identificationThirdValid[point] = 0U;
        WPT_identificationInputResistanceMilliOhms[point] = 0L;
        WPT_identificationInputReactanceMilliOhms[point] = 0L;
        WPT_identificationThirdInputResistanceMilliOhms[point] = 0L;
        WPT_identificationThirdInputReactanceMilliOhms[point] = 0L;
    }
}

/*
 * WPT_updateIdentificationState
 * -----------------------------
 * Advances only after a complete valid buffer has produced valid fundamental
 * data. Frequency stability, settling, and averaging use counts of complete
 * buffers rather than raw samples.
 *
 * State sequence:
 *   WAIT_FIRST      find a stable f1
 *   SETTLE_FIRST    ignore transient buffers at f1
 *   COLLECT_FIRST   average I1/I3 at f1
 *   WAIT_CHANGE     request/detect the second PWM frequency
 *   SETTLE_SECOND   ignore transient buffers at f2
 *   COLLECT_SECOND  average I1/I3 at f2 and run the solver
 *   COMPLETE        hold results until reset.
 */
void WPT_updateIdentificationState(void)
{
    uint32_t frequencyDifference;

    if(WPT_identificationResetRequest != 0U)
    {
        WPT_resetIdentification();
        return;
    }

    if(WPT_currentHarmonicValid == 0U)
    {
        return;
    }

    switch(WPT_identificationState)
    {
        case WPT_IDENT_STATE_WAIT_FIRST:
            // Require several mutually consistent frequency estimates before
            // declaring the first operating point.
            if(WPT_updateStableFrequency(WPT_currentFrequencyHz) != 0U)
            {
                WPT_identificationFrequencyHz[0] =
                        WPT_identificationCandidateFrequencyHz;
                WPT_identificationSettlingCount = 0U;
                WPT_identificationStableCount = 0U;
                WPT_identificationState = WPT_IDENT_STATE_SETTLE_FIRST;
            }
            break;

        case WPT_IDENT_STATE_SETTLE_FIRST:
            // Discard initial buffers so resonant current reaches steady state.
            frequencyDifference = WPT_absoluteFrequencyDifference(
                    WPT_currentFrequencyHz,
                    WPT_identificationFrequencyHz[0]);
            if(frequencyDifference >
               WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ)
            {
                WPT_resetIdentification();
            }
            else if(++WPT_identificationSettlingCount >=
                    WPT_IDENT_SETTLING_CAPTURE_COUNT)
            {
                WPT_clearIdentificationAccumulator();
                WPT_identificationState = WPT_IDENT_STATE_COLLECT_FIRST;
            }
            break;

        case WPT_IDENT_STATE_COLLECT_FIRST:
            // Average several steady-state frequency and phasor measurements.
            frequencyDifference = WPT_absoluteFrequencyDifference(
                    WPT_currentFrequencyHz,
                    WPT_identificationFrequencyHz[0]);
            if(frequencyDifference >
               WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ)
            {
                WPT_resetIdentification();
            }
            else
            {
                WPT_accumulateIdentificationCapture();
                if(WPT_identificationCollectionCount >=
                   WPT_IDENT_AVERAGE_CAPTURE_COUNT)
                {
                    WPT_storeIdentificationPoint(0U);
                    WPT_identificationCandidateFrequencyHz = 0UL;
                    WPT_identificationStableCount = 0U;
                    WPT_identificationState = WPT_IDENT_STATE_WAIT_CHANGE;
#if WPT_PWM_AUTO_TRANSITION_ENABLE
                    // The main loop services this request after analysis ends.
                    WPT_requestPwmControllerCase(
                            WPT_PWM_SECOND_CONTROLLER_CASE);
#endif
                }
            }
            break;

        case WPT_IDENT_STATE_WAIT_CHANGE:
            // A difference smaller than CHANGE_HZ is treated as normal jitter;
            // a larger stable difference becomes the second operating point.
            frequencyDifference = WPT_absoluteFrequencyDifference(
                    WPT_currentFrequencyHz,
                    WPT_identificationFrequencyHz[0]);
            if(frequencyDifference >= WPT_IDENT_FREQUENCY_CHANGE_HZ)
            {
                if(WPT_updateStableFrequency(WPT_currentFrequencyHz) != 0U)
                {
                    WPT_identificationFrequencyHz[1] =
                            WPT_identificationCandidateFrequencyHz;
                    WPT_identificationSettlingCount = 0U;
                    WPT_identificationStableCount = 0U;
                    WPT_identificationState = WPT_IDENT_STATE_SETTLE_SECOND;
                }
            }
            else
            {
                WPT_identificationCandidateFrequencyHz = 0UL;
                WPT_identificationStableCount = 0U;
            }
            break;

        case WPT_IDENT_STATE_SETTLE_SECOND:
            // A failed second-frequency settle returns to WAIT_CHANGE without
            // erasing the already-completed first measurement.
            frequencyDifference = WPT_absoluteFrequencyDifference(
                    WPT_currentFrequencyHz,
                    WPT_identificationFrequencyHz[1]);
            if(frequencyDifference >
               WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ)
            {
                WPT_identificationCandidateFrequencyHz = 0UL;
                WPT_identificationStableCount = 0U;
                WPT_identificationState = WPT_IDENT_STATE_WAIT_CHANGE;
            }
            else if(++WPT_identificationSettlingCount >=
                    WPT_IDENT_SETTLING_CAPTURE_COUNT)
            {
                WPT_clearIdentificationAccumulator();
                WPT_identificationState = WPT_IDENT_STATE_COLLECT_SECOND;
            }
            break;

        case WPT_IDENT_STATE_COLLECT_SECOND:
            // Once the second average is stored, all four harmonic impedances
            // are available and the slow complex-load parameter solver can run.
            frequencyDifference = WPT_absoluteFrequencyDifference(
                    WPT_currentFrequencyHz,
                    WPT_identificationFrequencyHz[1]);
            if(frequencyDifference >
               WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ)
            {
                WPT_clearIdentificationAccumulator();
                WPT_identificationCandidateFrequencyHz = 0UL;
                WPT_identificationStableCount = 0U;
                WPT_identificationState = WPT_IDENT_STATE_WAIT_CHANGE;
            }
            else
            {
                WPT_accumulateIdentificationCapture();
                if(WPT_identificationCollectionCount >=
                   WPT_IDENT_AVERAGE_CAPTURE_COUNT)
                {
                    WPT_storeIdentificationPoint(1U);
                    WPT_solveIdentification();
                    WPT_identificationState = WPT_IDENT_STATE_COMPLETE;
                }
            }
            break;

        case WPT_IDENT_STATE_COMPLETE:
            // Results remain unchanged until WPT_identificationResetRequest=1.
        default:
            break;
    }
}

/*
 * Inductive reactance in mOhm from frequency in Hz and inductance in nH:
 *
 *   XL_ohm  = 2*pi*f*L_H
 *   XL_mOhm = 2*pi*f*L_nH*10^-6.
 *
 * WPT_TWO_PI_TIMES_1E6 stores 2*pi*10^6 as an integer. The following
 * multiply/divide arrangement implements the 10^-6 scale with rounding.
 */
int32_t WPT_reactanceMilliOhm(uint32_t frequencyHz,
                              uint32_t inductanceNanoHenry)
{
    uint64_t numerator;

    numerator = (uint64_t)WPT_TWO_PI_TIMES_1E6 *
                (uint64_t)frequencyHz *
                (uint64_t)inductanceNanoHenry;
    numerator += 500000000000ULL;
    numerator /= 1000000000000ULL;
    if(numerator > 2147483647ULL)
    {
        return 2147483647L;
    }

    return (int32_t)numerator;
}

/*
 * Capacitive reactance magnitude in mOhm from Hz and pF:
 *
 *   XC_ohm  = 1/(2*pi*f*C_F)
 *   XC_mOhm = 10^15/(2*pi*f*C_pF)
 *           = 159154943091895/(f*C_pF).
 *
 * This function returns the positive magnitude. The caller forms total series
 * reactance as XL-XC. A missing frequency/capacitance returns INT32_MAX so an
 * invalid configuration cannot look like a valid zero reactance.
 */
int32_t WPT_capacitiveReactanceMilliOhm(uint32_t frequencyHz,
                                        uint32_t capacitancePicoFarad)
{
    uint64_t denominator;
    uint64_t reactance;

    if((frequencyHz == 0UL) || (capacitancePicoFarad == 0UL))
    {
        return 2147483647L;
    }

    denominator = (uint64_t)frequencyHz *
                  (uint64_t)capacitancePicoFarad;
    reactance = (159154943091895ULL + (denominator / 2ULL)) /
                denominator;
    if(reactance > 2147483647ULL)
    {
        return 2147483647L;
    }

    return (int32_t)reactance;
}

/*
 * Divide complex voltage by complex current to obtain input impedance.
 *
 *   V = Vr + j*Vi, I = Ir + j*Ii, Z = V/I = R + j*X
 *   R = (Vr*Ir + Vi*Ii)/(Ir^2 + Ii^2)
 *   X = (Vi*Ir - Vr*Ii)/(Ir^2 + Ii^2).
 *
 * mV/mA equals ohms, so multiplication by 1000 returns mOhm. Peak or RMS
 * phasors give the same impedance as long as voltage and current use the same
 * convention. Returns 0 for zero denominator or an out-of-range result.
 */
static uint16_t WPT_calculateInputImpedance(
        int32_t voltageRealMilliVolts,
        int32_t voltageImagMilliVolts,
        int32_t currentRealMilliAmps,
        int32_t currentImagMilliAmps,
        int32_t *resistanceMilliOhms,
        int32_t *reactanceMilliOhms)
{
    int64_t denominator =
            (int64_t)currentRealMilliAmps *
            (int64_t)currentRealMilliAmps +
            (int64_t)currentImagMilliAmps *
            (int64_t)currentImagMilliAmps;
    int64_t realNumerator;
    int64_t imagNumerator;

    if(denominator <= 0LL)
    {
        return 0U;
    }

    realNumerator =
            ((int64_t)voltageRealMilliVolts * currentRealMilliAmps +
             (int64_t)voltageImagMilliVolts * currentImagMilliAmps) *
            1000LL;
    imagNumerator =
            ((int64_t)voltageImagMilliVolts * currentRealMilliAmps -
             (int64_t)voltageRealMilliVolts * currentImagMilliAmps) *
            1000LL;

    realNumerator /= denominator;
    imagNumerator /= denominator;
    if((realNumerator > 2147483647LL) || (realNumerator < -2147483647LL) ||
       (imagNumerator > 2147483647LL) || (imagNumerator < -2147483647LL))
    {
        return 0U;
    }
    *resistanceMilliOhms = (int32_t)realNumerator;
    *reactanceMilliOhms = (int32_t)imagNumerator;
    return 1U;
}

/* BEGIN WPT COMPLEX ESTIMATOR INTEGRATION (also compiled by host tests) */
/* Saturating conversions preserve existing integer CCS display units. */
static uint32_t WPT_estimateToU32(WPTCE_Real value)
{
    if(!WPTCE_finite(value) || value<=0) return 0UL;
    if(value>=WPTCE_R(4294967294.0)) return 0xFFFFFFFEUL;
    return (uint32_t)(value+WPTCE_R(0.5));
}
static int32_t WPT_estimateToI32(WPTCE_Real value)
{
    if(!WPTCE_finite(value)) return 0L;
    if(value>WPTCE_R(2147483646.0)) return 2147483646L;
    if(value<-WPTCE_R(2147483646.0)) return -2147483646L;
    return (int32_t)(value+(value>=0?WPTCE_R(0.5):-WPTCE_R(0.5)));
}

/* Fail closed: no stale accepted result or numerical DC load survives a reset
 * or an unsuccessful solve. Fitted values may later be published for diagnosis,
 * but must never be used unless WPT_identificationValid is one. */
static void WPT_clearEstimatorResults(void)
{
    uint16_t q;
    WPT_identificationValid=0U;
    WPT_estimatedSelfInductanceNanoHenry=0UL;
    WPT_estimatedMutualInductanceNanoHenry=0UL;
    WPT_estimatedEquivalentAcLoadMilliOhm=0UL;
    WPT_estimatedEquivalentAcReactanceMilliOhm=0L;
    WPT_estimatedThirdAcResistanceMilliOhm=0L;
    WPT_estimatedThirdAcReactanceMilliOhm=0L;
    WPT_estimatedDcLoadMilliOhm=0xFFFFFFFFUL;
    WPT_estimatedDcLoadValid=0U;
    WPT_estimatedCouplingPermille=0U;
    WPT_identificationResidualMilliOhm=0UL;
    WPT_identificationSolverFlags=WPTCE_BAD_INPUT;
    WPT_identificationSolutionsChecked=0U;
    WPT_identificationNormalizedResidualPermille=0UL;
    WPT_identificationLRelativeSensitivityPermille=0UL;
    WPT_identificationMRelativeSensitivityPermille=0UL;
    WPT_identificationAlternativeSelfInductanceNanoHenry=0UL;
    WPT_identificationAlternativeMutualInductanceNanoHenry=0UL;
    for(q=0U;q<4U;++q) WPT_identificationImpedanceSigmaMilliOhm[q]=0UL;
}

/* WPT_solveIdentification
 *
 * Use the actual measured/reconstructed complex V1,V3, including their phases.
 * Reuse the existing fixed-point complex division and current capture pipeline.
 * Convert to ohms once, then fit the six-unknown complex-receiver model.
 * No receiver PWM waveform, diode drop, pi^2/8 factor, or DC estimate is used.
 * This function must be called in the existing paused-analysis context, NOT
 * inside the ADC ISR. Target runtime and memory must be checked in CCS.
 */
void WPT_solveIdentification(void)
{
    uint16_t q,point;
    int32_t vr,vi,ir,ii,zr,zi;
    WPTCE_Real currentMagnitude,zMagnitude,sigma,noiseVoltage,noiseCurrent;
    WPTCE_Real *p=WPT_complexEstimatorResult.p;
    WPT_clearEstimatorResults();
    // Even with measured voltage, frequency-invariant receiver ratios and
    // Lp=Ls are modeling assumptions; a fit alone cannot verify them.
    WPT_identificationModelWarning=1U;
    for(point=0U;point<2U;++point) {
        if(WPT_identificationFrequencyHz[point]<WPT_CURRENT_MIN_FREQUENCY_HZ ||
           WPT_identificationFrequencyHz[point]>WPT_CURRENT_MAX_FREQUENCY_HZ ||
           WPT_identificationVoltageValid[point]==0U) return;
        if(WPT_identificationThirdValid[point]==0U) {
            WPT_identificationSolverFlags=WPTCE_MISSING_THIRD;
            return; // Do NOT silently turn this into a fundamental-only fit.
        }
        WPT_complexEstimatorInput.frequency_hz[point]=
            (WPTCE_Real)WPT_identificationFrequencyHz[point];
    }
    if(WPT_IDENT_IMPEDANCE_NOISE_FLOOR_MOHM<=0L ||
       WPT_IDENT_IMPEDANCE_RELATIVE_NOISE_PERMILLE<0L ||
       WPT_IDENT_VOLTAGE_PHASOR_NOISE_MV<0L ||
       WPT_IDENT_CURRENT_PHASOR_NOISE_MA<0L) return;
    for(q=0U;q<4U;++q) {
        point=q%2U;
        if(q<2U) {
            vr=WPT_identificationV1RealMilliVolts[point];
            vi=WPT_identificationV1ImagMilliVolts[point];
            ir=WPT_identificationI1RealMilliAmps[point];
            ii=WPT_identificationI1ImagMilliAmps[point];
        } else {
            vr=WPT_identificationV3RealMilliVolts[point];
            vi=WPT_identificationV3ImagMilliVolts[point];
            ir=WPT_identificationI3RealMilliAmps[point];
            ii=WPT_identificationI3ImagMilliAmps[point];
        }
        // Also guard the inherited fixed-point V/I numerator against overflow.
        if(vr>2000000L || vr<-2000000L || vi>2000000L || vi<-2000000L ||
           ir>1000000L || ir<-1000000L || ii>1000000L || ii<-1000000L) {
            WPT_identificationSolverFlags=WPTCE_BAD_PHASOR; return;
        }
        currentMagnitude=WPTCE_sqrt((WPTCE_Real)ir*ir+(WPTCE_Real)ii*ii);
        if(currentMagnitude < (WPTCE_Real)(q<2U ?
                WPT_IDENT_MIN_FUNDAMENTAL_CURRENT_MA : WPT_IDENT_MIN_THIRD_CURRENT_MA) ||
           WPT_calculateInputImpedance(vr,vi,ir,ii,&zr,&zi)==0U) {
            WPT_identificationSolverFlags=WPTCE_BAD_PHASOR; return;
        }
        if(q<2U) {
            WPT_identificationInputResistanceMilliOhms[point]=zr;
            WPT_identificationInputReactanceMilliOhms[point]=zi;
        } else {
            WPT_identificationThirdInputResistanceMilliOhms[point]=zr;
            WPT_identificationThirdInputReactanceMilliOhms[point]=zi;
        }
        WPT_complexEstimatorInput.zin_real_ohm[q]=(WPTCE_Real)zr/1000;
        WPT_complexEstimatorInput.zin_imag_ohm[q]=(WPTCE_Real)zi/1000;
        zMagnitude=WPTCE_sqrt((WPTCE_Real)zr*zr+(WPTCE_Real)zi*zi)/1000;
        // Propagation for Z=V/I: sigmaZ ~= sqrt(sigmaV^2+|Z|^2*sigmaI^2)/|I|.
        // mV/mA=ohms. Add an absolute floor and relative calibration allowance.
        // These settings are engineering allowances, NOT measured statistics.
        noiseVoltage=(WPTCE_Real)WPT_IDENT_VOLTAGE_PHASOR_NOISE_MV;
        noiseCurrent=zMagnitude*(WPTCE_Real)WPT_IDENT_CURRENT_PHASOR_NOISE_MA;
        sigma=WPTCE_sqrt(noiseVoltage*noiseVoltage+noiseCurrent*noiseCurrent)/currentMagnitude;
        sigma+=(WPTCE_Real)WPT_IDENT_IMPEDANCE_NOISE_FLOOR_MOHM/1000;
        sigma+=zMagnitude*(WPTCE_Real)WPT_IDENT_IMPEDANCE_RELATIVE_NOISE_PERMILLE/1000;
        WPT_complexEstimatorInput.sigma_ohm[q]=sigma;
        WPT_identificationImpedanceSigmaMilliOhm[q]=WPT_estimateToU32(1000*sigma);
    }
    WPT_complexEstimatorInput.cp_pf=(WPTCE_Real)WPT_PRIMARY_CAPACITANCE_PF;
    WPT_complexEstimatorInput.cs_pf=(WPTCE_Real)WPT_SECONDARY_CAPACITANCE_PF;
    WPT_complexEstimatorInput.rp_ohm=(WPTCE_Real)WPT_PRIMARY_RESISTANCE_MOHM/1000;
    WPT_complexEstimatorInput.rs_ohm=(WPTCE_Real)WPT_SECONDARY_RESISTANCE_MOHM/1000;
    WPT_complexEstimatorInput.min_l_uh=(WPTCE_Real)WPT_IDENT_MIN_SELF_INDUCTANCE_NH/1000;
    WPT_complexEstimatorInput.max_l_uh=(WPTCE_Real)WPT_IDENT_MAX_SELF_INDUCTANCE_NH/1000;
    WPT_complexEstimatorInput.min_frequency_separation_hz=(WPTCE_Real)WPT_IDENT_FREQUENCY_CHANGE_HZ;
    WPT_complexEstimatorInput.max_receiver_magnitude_ohm=(WPTCE_Real)WPT_IDENT_MAX_RECEIVER_COMPONENT_MOHM/1000;
    WPT_complexEstimatorInput.max_normalized_rms=(WPTCE_Real)WPT_IDENT_MAX_NORMALIZED_RMS_PERMILLE/1000;
    WPT_complexEstimatorInput.max_relative_sensitivity=(WPTCE_Real)WPT_IDENT_MAX_RELATIVE_SENSITIVITY_PERMILLE/1000;
    WPT_complexEstimatorInput.ambiguity_delta_cost=(WPTCE_Real)WPT_IDENT_AMBIGUITY_DELTA_COST;
    WPTCE_estimate(&WPT_complexEstimatorInput,&WPT_complexEstimatorResult);
    WPT_identificationSolverFlags=WPT_complexEstimatorResult.flags;
    WPT_identificationSolutionsChecked=WPT_complexEstimatorResult.solutions_checked;
    WPT_identificationResidualMilliOhm=WPT_estimateToU32(1000*WPT_complexEstimatorResult.rms_ohm);
    WPT_identificationNormalizedResidualPermille=WPT_estimateToU32(1000*WPT_complexEstimatorResult.normalized_rms);
    WPT_identificationLRelativeSensitivityPermille=WPT_estimateToU32(1000*WPT_complexEstimatorResult.relative_l_sensitivity);
    WPT_identificationMRelativeSensitivityPermille=WPT_estimateToU32(1000*WPT_complexEstimatorResult.relative_m_sensitivity);
    WPT_identificationAlternativeSelfInductanceNanoHenry=WPT_estimateToU32(1000*WPT_complexEstimatorResult.alternative_l_uh);
    WPT_identificationAlternativeMutualInductanceNanoHenry=WPT_estimateToU32(1000*WPT_complexEstimatorResult.alternative_m_uh);
    if(WPT_identificationSolverFlags & (WPTCE_BAD_INPUT|WPTCE_NO_SOLUTION)) return;
    WPT_estimatedSelfInductanceNanoHenry=WPT_estimateToU32(1000*p[0]);
    WPT_estimatedMutualInductanceNanoHenry=WPT_estimateToU32(1000*p[1]);
    WPT_estimatedEquivalentAcLoadMilliOhm=WPT_estimateToU32(1000*p[2]);
    WPT_estimatedEquivalentAcReactanceMilliOhm=WPT_estimateToI32(1000*p[3]);
    WPT_estimatedThirdAcResistanceMilliOhm=WPT_estimateToI32(1000*p[4]);
    WPT_estimatedThirdAcReactanceMilliOhm=WPT_estimateToI32(1000*p[5]);
    if(p[0]>0) WPT_estimatedCouplingPermille=(uint16_t)WPT_estimateToU32(1000*p[1]/p[0]);
    if(WPT_identificationResidualMilliOhm>WPT_IDENT_MAX_IMPEDANCE_RESIDUAL_MOHM)
        WPT_identificationSolverFlags|=WPTCE_HIGH_RESIDUAL;
    WPT_identificationValid=(WPT_identificationSolverFlags==0U)?1U:0U;
    // DC sentinel and DcLoadValid=0 remain untouched. Never use the sentinel
    // in arithmetic. Read all published quantities only when their valid flag
    // permits it; numerical validity does not establish physical accuracy.
}
/* END WPT COMPLEX ESTIMATOR INTEGRATION */

/*
 * WPT_analyzeCurrentCapture
 * -------------------------
 * Consumes one completed 1024-sample buffer. Sampling remains paused until the
 * function finishes so the buffer cannot change during analysis.
 *
 * Processing steps:
 *   1. Find mean, minimum, maximum, DC current, and simple peak-to-peak value.
 *   2. Convert every raw sample to mA for the CCS graph buffer.
 *   3. Estimate frequency from rising crossings of the mean.
 *   4. Extract PWM-referenced fundamental and third-harmonic phasors.
 *   5. Advance the two-frequency identification state machine.
 *   6. Clear flags and restart EPWM3 ADC triggering.
 *
 * Crossing interpolation between samples x0 and x1 is:
 *   fraction = (mean-x0)/(x1-x0)
 *   position = previous_sample_index + fraction.
 * Individual periods outside the configured 60-150 kHz range are rejected
 * before averaging so one noise or missed crossing cannot destroy the result.
 */
void WPT_analyzeCurrentCapture(void)
{
    int16_t minimumCounts = 32767;
    int16_t maximumCounts = -32768;
    int32_t sumCounts = 0L;
    int32_t meanCounts;
    int32_t peakToPeakCounts;
    int32_t lowThresholdCounts;
    int32_t previousCounts;
    int32_t presentCounts;
    int32_t currentSampleMilliAmps;
    uint16_t index;
    uint16_t crossingCount = 0U;
    uint16_t crossingArmed = 0U;
    uint16_t validPeriodCount = 0U;
    uint16_t rejectedPeriodCount = 0U;
    uint32_t crossingFractionQ16;
    uint32_t crossingPositionQ16;
    uint32_t previousCrossingQ16 = 0UL;
    uint32_t periodQ16;
    uint32_t minimumPeriodQ16;
    uint32_t maximumPeriodQ16;
    uint32_t averagePeriodQ16;
    uint32_t candidateFrequencyHz;
    uint64_t periodSumQ16 = 0ULL;
    uint64_t frequencyNumerator;

    // STEP 1: Freeze acquisition and calculate basic buffer statistics.
    // Pause the sample trigger while this completed buffer is analyzed.
    EPWM_disableADCTrigger(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);

    for(index = 0U; index < WPT_CURRENT_TRACE_LENGTH; index++)
    {
        int16_t sampleCounts = WPT_currentSampleCounts[index];
        sumCounts += (int32_t)sampleCounts;

        if(sampleCounts < minimumCounts)
        {
            minimumCounts = sampleCounts;
        }
        if(sampleCounts > maximumCounts)
        {
            maximumCounts = sampleCounts;
        }
    }

    meanCounts = sumCounts / (int32_t)WPT_CURRENT_TRACE_LENGTH;
    peakToPeakCounts = (int32_t)maximumCounts - (int32_t)minimumCounts;

    WPT_currentPeakToPeakMilliAmps =
            WPT_currentCountsToMilliAmps(peakToPeakCounts);
    WPT_currentAmplitudeMilliAmps =
            (WPT_currentPeakToPeakMilliAmps + 1L) / 2L;
    WPT_currentDcMilliAmps = WPT_currentCountsToMilliAmps(
            WPT_CURRENT_POLARITY * meanCounts);

    // STEP 2: Convert the completed capture to mA for display. The simple
    // first-order low-pass display filter is:
    //   filtered += (sample-filtered)/WPT_CURRENT_FILTER_DIVISOR.
    // It is not used by the harmonic or parameter calculations.
    // Convert the completed capture to integer milliamps for the CCS graph.
    for(index = 0U; index < WPT_CURRENT_TRACE_LENGTH; index++)
    {
        currentSampleMilliAmps = WPT_currentCountsToMilliAmps(
                WPT_CURRENT_POLARITY *
                (int32_t)WPT_currentSampleCounts[index]);
        WPT_currentTraceMilliAmps[index] = currentSampleMilliAmps;
        WPT_currentFilteredMilliAmps +=
                (currentSampleMilliAmps - WPT_currentFilteredMilliAmps) /
                WPT_CURRENT_FILTER_DIVISOR;
    }
    WPT_currentMilliAmps =
            WPT_currentTraceMilliAmps[WPT_CURRENT_TRACE_LENGTH - 1U];

    // STEP 3: Estimate frequency from rising mean crossings. Crossing locations use
    // Q16 sample-position fixed point, including linear interpolation between
    // the two ADC samples surrounding each crossing. Each individual period
    // must correspond to 60-150 kHz, so noise crossings and missed crossings
    // cannot corrupt the entire frequency estimate.
    WPT_currentFrequencyValid = 0U;
    minimumPeriodQ16 = (uint32_t)
            (((uint64_t)WPT_CURRENT_SAMPLE_RATE_HZ *
              (uint64_t)WPT_CURRENT_CROSSING_Q_ONE) /
             (uint64_t)WPT_CURRENT_MAX_FREQUENCY_HZ);
    maximumPeriodQ16 = (uint32_t)
            (((uint64_t)WPT_CURRENT_SAMPLE_RATE_HZ *
              (uint64_t)WPT_CURRENT_CROSSING_Q_ONE) /
             (uint64_t)WPT_CURRENT_MIN_FREQUENCY_HZ);

    if(peakToPeakCounts >= WPT_CURRENT_MIN_P2P_COUNTS)
    {
        lowThresholdCounts = meanCounts -
                ((peakToPeakCounts * WPT_CURRENT_HYSTERESIS_PERCENT) / 100L);
        if((int32_t)WPT_currentSampleCounts[0] <= lowThresholdCounts)
        {
            crossingArmed = 1U;
        }

        for(index = 1U; index < WPT_CURRENT_TRACE_LENGTH; index++)
        {
            previousCounts =
                    (int32_t)WPT_currentSampleCounts[index - 1U];
            presentCounts = (int32_t)WPT_currentSampleCounts[index];

            if(presentCounts <= lowThresholdCounts)
            {
                crossingArmed = 1U;
            }

            if((crossingArmed != 0U) &&
               (previousCounts < meanCounts) &&
               (presentCounts >= meanCounts))
            {
                crossingFractionQ16 =
                        ((uint32_t)(meanCounts - previousCounts) <<
                         WPT_CURRENT_CROSSING_Q_SHIFT) /
                        (uint32_t)(presentCounts - previousCounts);
                crossingPositionQ16 =
                        ((uint32_t)(index - 1U) <<
                         WPT_CURRENT_CROSSING_Q_SHIFT) +
                        crossingFractionQ16;

                if(crossingCount != 0U)
                {
                    periodQ16 = crossingPositionQ16 - previousCrossingQ16;
                    if((periodQ16 >= minimumPeriodQ16) &&
                       (periodQ16 <= maximumPeriodQ16))
                    {
                        periodSumQ16 += (uint64_t)periodQ16;
                        validPeriodCount++;
                    }
                    else
                    {
                        rejectedPeriodCount++;
                    }
                }
                previousCrossingQ16 = crossingPositionQ16;
                crossingCount++;
                crossingArmed = 0U;
            }
        }

        if((WPT_currentCaptureValid != 0U) &&
           (validPeriodCount >= WPT_CURRENT_MIN_VALID_PERIODS))
        {
            averagePeriodQ16 = (uint32_t)
                    (periodSumQ16 / (uint64_t)validPeriodCount);
            frequencyNumerator =
                    (uint64_t)WPT_CURRENT_SAMPLE_RATE_HZ *
                    (uint64_t)WPT_CURRENT_CROSSING_Q_ONE;
            candidateFrequencyHz = (uint32_t)
                    ((frequencyNumerator + (averagePeriodQ16 / 2UL)) /
                     averagePeriodQ16);

            if((candidateFrequencyHz >= WPT_CURRENT_MIN_FREQUENCY_HZ) &&
               (candidateFrequencyHz <= WPT_CURRENT_MAX_FREQUENCY_HZ))
            {
                // Keep the last good value across a bad capture instead of
                // replacing it with zero or an out-of-range result.
                WPT_currentFrequencyHz = candidateFrequencyHz;
                WPT_currentFrequencyValid = 1U;
            }
        }
    }

    WPT_currentCrossingCount = crossingCount;
    WPT_currentValidPeriodCount = validPeriodCount;
    WPT_currentRejectedPeriodCount = rejectedPeriodCount;
    if(WPT_currentFrequencyValid == 0U)
    {
        WPT_currentFrequencyRejectCount++;
    }

    // STEPS 4-5: Extract harmonics, then feed this completed capture to the
    // identification state machine.
    // Extract the current fundamental and third harmonic with PWM1 as the
    // phase reference, then advance the two-frequency collection state.
    WPT_extractCurrentHarmonics(WPT_currentFrequencyHz, meanCounts);
    WPT_updateIdentificationState();

    // STEP 6: Publish latest raw ADC voltages and prepare a fresh buffer.
    // ADC millivolts = raw_count*3300/4096.
    WPT_currentVoutMilliVolts = (uint16_t)
            (((uint32_t)WPT_currentVoutRaw *
              (uint32_t)WPT_CURRENT_ADC_FULL_SCALE_MV +
              ((uint32_t)WPT_CURRENT_ADC_COUNTS / 2UL)) /
             (uint32_t)WPT_CURRENT_ADC_COUNTS);
    WPT_currentVrefMilliVolts = (uint16_t)
            (((uint32_t)WPT_currentVrefRaw *
              (uint32_t)WPT_CURRENT_ADC_FULL_SCALE_MV +
              ((uint32_t)WPT_CURRENT_ADC_COUNTS / 2UL)) /
             (uint32_t)WPT_CURRENT_ADC_COUNTS);
    WPT_currentLastCaptureValid = WPT_currentCaptureValid;
    WPT_currentCaptureNumber++;

    ADC_clearInterruptStatus(WPT_CURRENT_VOUT_ADC_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptOverflowStatus(WPT_CURRENT_VOUT_ADC_BASE,
                                     ADC_INT_NUMBER1);
    EPWM_clearADCTriggerFlag(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);
    WPT_currentCaptureIndex = 0U;
    WPT_currentCaptureReady = 0U;
    WPT_currentCaptureValid = 1U;
    EPWM_enableADCTrigger(WPT_CURRENT_SAMPLE_PWM_BASE, EPWM_SOC_A);
}

/*
 * WPT_setupPwms
 * -------------
 * Stops the shared ePWM time-base clock and configures two synchronized full
 * bridges:
 *   primary   = EPWM1 (base leg) + EPWM2 (phase-shifted leg)
 *   secondary = EPWM4 (referenced to EPWM1) + EPWM5 (phase-shifted leg).
 *
 * WPT_pwmPeriodTicks and PH_* must already contain the selected controller
 * case. Dead-time values come from psfbpcmc_user_settings.h. The caller decides
 * when it is safe to restart TBCLKSYNC.
 */
void WPT_setupPwms()
{
    // Freeze the global ePWM time-base clock. This lets every period, compare,
    // phase, dead-band, and trip register be configured before any counter can
    // create an output edge.
    PSFB_HAL_disablePWMCLKCounting();

    // Build the primary full bridge: EPWM1 is the master leg and EPWM2 is the
    // PH_1_4-shifted leg. The four dead-band arguments independently set the
    // rising/falling delays for the two legs.
    WPT_pri_setupPwms(PSFB_FB_PWM1_BASE, PSFB_FB_PWM2_BASE,
                      WPT_pwmPeriodTicks, PSFB_FB_CMPA_PWM_BASE1,
                      PSFB_FB_CMPB_PWM_BASE1, PSFB_FB_DB_PWM_LS_COUNT_BASE1_pri,
                      PSFB_FB_DB_PWM_HS_COUNT_BASE1_pri,
                      PSFB_FB_DB_PWM_LS_COUNT_BASE2_pri,
                      PSFB_FB_DB_PWM_HS_COUNT_BASE2_pri,PH_1_4);

    // Build the second full bridge. PH_1_5 loads EPWM4's phase and PH_5_8
    // loads EPWM5's phase. EPWM4 forwards EPWM1's incoming sync; it does not
    // regenerate sync at its own shifted zero. The relative bridge-leg phase
    // therefore depends on both values. Dead times use the "sec" settings.
    WPT_sec_setupPwms(PSFB_FB_PWM4_BASE, PSFB_FB_PWM5_BASE,
                          WPT_pwmPeriodTicks, PSFB_FB_CMPA_PWM_BASE1,
                          PSFB_FB_CMPB_PWM_BASE1, PSFB_FB_DB_PWM_LS_COUNT_BASE1_sec,
                          PSFB_FB_DB_PWM_HS_COUNT_BASE1_sec,
                          PSFB_FB_DB_PWM_LS_COUNT_BASE2_sec,
                          PSFB_FB_DB_PWM_HS_COUNT_BASE2_sec, PH_1_5,PH_5_8);

    // Make the one-shot trip used by PSFB_EnablePSFBPWM and runtime profile
    // transitions a true all-gates-off state on every driven bridge PWM.
    WPT_configureBridgeTripLow();

}

/*
 * WPT_pri_setupPwms
 * -----------------
 * Configures the primary full bridge. Both modules use symmetric up/down
 * counting and nominal 50% duty. EPWM1 is the master zero-degree reference;
 * EPWM2 loads PH_1_4 on synchronization to create the bridge phase shift.
 *
 * The action qualifier (AQ) defines when raw A/B waveforms change state.
 * The dead-band (DB) block delays complementary rising/falling gate edges so
 * high- and low-side switches are never commanded on simultaneously.
 * The trip-zone (TZ) block is the final safety override.
 *
 * Parameter names ending in base1 apply to EPWM1; base2 values apply to EPWM2.
 * The legacy cmpa/cmpb and phaseshift parameters are retained for interface
 * compatibility; the implementation uses 50% compare values and global PH_*.
 */
void WPT_pri_setupPwms(uint32_t base1, uint32_t base2,
                       uint16_t pwm_period_ticks, uint16_t pwm_cmpa_base1,
                       uint16_t pwm_cmpb_base1, uint16_t pwm_db_ticks_ls_base1,
                       uint16_t pwm_db_ticks_hs_base1, uint16_t pwm_db_ticks_ls_base2,
                       uint16_t pwm_db_ticks_hs_base2,uint16_t phaseshift_Q1_Q4)
{
    // The following ePWM and synchronization registers are protected on C2000.
    // EALLOW enables writes until the matching EDIS at the end of the function.
    EALLOW;

       // PRIMARY BASE1 TIME BASE (EPWM1 MASTER)
       // Direct load makes the supplied period take effect immediately while
       // TBCLKSYNC is stopped. Up-down counting gives a symmetric waveform:
       // the counter runs 0 -> TBPRD -> 0, taking 2*TBPRD clock ticks.
       // Clock divider 1/1 makes TBCLK equal to the configured ePWM clock.
       EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_DIRECT_LOAD);
       EPWM_setTimeBasePeriod(base1, pwm_period_ticks);
       EPWM_setTimeBaseCounter(base1, 0);
       EPWM_setPhaseShift(base1, 0);
       EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
       EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

       // PRIMARY BASE1 COMPARES
       // CMPA=CMPB=TBPRD/2 places switching events halfway through each count
       // ramp. With the actions below this creates nominal 50% raw commands.
       EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A,
                                   pwm_period_ticks/2);
       EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_B,pwm_period_ticks/2);

       // Shadow-load compare values only when the counter returns to zero, so
       // a future runtime update cannot split or shorten the current pulse.
       EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
       EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_B,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);

       // PRIMARY BASE1 ACTION QUALIFIER
       // Clear the old AQ control register before installing the four explicit
       // edge actions. This prevents settings from an earlier project build
       // from surviving in unused AQ fields.
       HWREGH(base1 + EPWM_O_AQCTLA) = 0;

       // On the upward CMPA crossing drive raw A low; on the downward CMPA
       // crossing drive raw A high. The two crossings are half a cycle apart.
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

       // Raw B uses the same edge locations. The dead-band input selection and
       // polarity below turn the final A/B pins into a complementary leg pair.
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

       // PRIMARY BASE2 DIGITAL-COMPARE BLANKING
       // Ignore digital-comparator events during this five-TBCLK window near
       // the end of the PWM period, where switching edges can create spikes.
       EPWM_setDigitalCompareWindowOffset(base2, pwm_period_ticks - 5);
       EPWM_setDigitalCompareWindowLength(base2, 5);

       // PRIMARY BASE2 DIRECTION-SENSITIVE TRIP ACTIONS
       // A cycle-by-cycle event forces A low on the down-count and B low on
       // the up-count. The opposite directions are left unchanged. The basic
       // TZA/TZB actions are disabled here; WPT_configureBridgeTripLow() later
       // programs the separate one-shot software-trip response for both pins.
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZA_D,
                                 EPWM_TZ_ADV_ACTION_LOW);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZA_U,
                                 EPWM_TZ_ADV_ACTION_DISABLE);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZB_U,
                                 EPWM_TZ_ADV_ACTION_LOW);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZB_D,
                                 EPWM_TZ_ADV_ACTION_DISABLE);
       EPWM_setTripZoneAction(base2, EPWM_TZ_ACTION_EVENT_TZA,
                              EPWM_TZ_ACTION_DISABLE);
       EPWM_setTripZoneAction(base2, EPWM_TZ_ACTION_EVENT_TZB,
                              EPWM_TZ_ACTION_DISABLE);

       // A cycle-by-cycle (CBC) fault is automatically released only after the
       // counter has completed a full zero-and-period boundary sequence.
       EPWM_selectCycleByCycleTripZoneClearEvent(base2,
                              EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD);

       // Clear stale digital-compare flags left from reset or a prior setup so
       // the newly configured waveform starts from a known fault state.
       EPWM_clearTripZoneFlag(base1, (EPWM_TZ_INTERRUPT_DCAEVT2 |
               EPWM_TZ_SIGNAL_DCBEVT1));
       EPWM_clearTripZoneFlag(base2, (EPWM_TZ_INTERRUPT_DCAEVT2 |
               EPWM_TZ_SIGNAL_DCBEVT1));

       // PRIMARY BASE2 DEAD BAND
       // RED delays the rising edge; FED delays the falling edge. Both delays
       // use raw EPWMA. FED is inverted later, producing complementary A/B
       // outputs separated by the configured non-overlap time.
       EPWM_setDeadBandCounterClock(base2, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
       EPWM_setRisingEdgeDelayCount(base2, pwm_db_ticks_hs_base2);
       EPWM_setFallingEdgeDelayCount(base2, pwm_db_ticks_ls_base2);
       EPWM_setDeadBandDelayMode(base2, EPWM_DB_RED, true);
       EPWM_setDeadBandDelayMode(base2, EPWM_DB_FED, true);
       EPWM_setRisingEdgeDeadBandDelayInput(base2, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(base2, EPWM_DB_INPUT_EPWMA);
       EPWM_setDeadBandDelayPolarity(base2, EPWM_DB_RED,
                                     EPWM_DB_POLARITY_ACTIVE_HIGH);

       // PRIMARY BASE1 DEAD BAND
       // Here RED uses raw A while FED uses raw B. The active-low FED polarity
       // below supplies the complementary low-side gate command.
       EPWM_setDeadBandCounterClock(base1, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
       EPWM_setRisingEdgeDelayCount(base1, pwm_db_ticks_hs_base1);
       EPWM_setFallingEdgeDelayCount(base1, pwm_db_ticks_ls_base1);

       EPWM_setDeadBandDelayMode(base1, EPWM_DB_RED, true);
       EPWM_setDeadBandDelayMode(base1, EPWM_DB_FED, true);
       EPWM_setRisingEdgeDeadBandDelayInput(base1, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(base1, EPWM_DB_INPUT_EPWMB);
       EPWM_setDeadBandDelayPolarity(base1, EPWM_DB_RED,
                                     EPWM_DB_POLARITY_ACTIVE_HIGH);
       // Invert the falling-edge path so the final B output is complementary.
       EPWM_setDeadBandDelayPolarity(base1, EPWM_DB_FED,
                                     EPWM_DB_POLARITY_ACTIVE_LOW);

       // PRIMARY SYNCHRONIZATION MASTER
       // EPWM1 never loads an external phase. It emits a synchronization pulse
       // every time its counter reaches zero; SYSCTL routes that pulse into the
       // device-wide ePWM synchronization chain.
       EPWM_disablePhaseShiftLoad(base1);
       EPWM_setSyncOutPulseMode(base1, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
       SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
       // PRIMARY BASE2 TIME BASE (EPWM2 SLAVE)
       // The slave also counts up/down at divider 1/1. The historical -1
       // period is retained; it is NOT the two-clock sync-delay correction.
       // Recurring sync reloads the counter. Period mismatch must be included
       // when checking profiles other than the case-7 compare-edge check above.
       EPWM_setPeriodLoadMode(base2, EPWM_PERIOD_DIRECT_LOAD);

       EPWM_setTimeBasePeriod(base2, (pwm_period_ticks - 1));
       EPWM_setTimeBaseCounter(base2, 0);
       EPWM_setPhaseShift(base2, 0);
       EPWM_setTimeBaseCounterMode(base2, EPWM_COUNTER_MODE_UP_DOWN);
       EPWM_setClockPrescaler(base2, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

       // PRIMARY BASE2 COMPARES
       // Half-period compare values make this leg nominally 50% duty. Setting a
       // compare to zero would clamp a raw command, but this code does not do so.
       EPWM_setCounterCompareValue(base2, EPWM_COUNTER_COMPARE_A, pwm_period_ticks/2 );
       EPWM_setCounterCompareValue(base2, EPWM_COUNTER_COMPARE_B, pwm_period_ticks/2);


       EPWM_setCounterCompareShadowLoadMode(base2, EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
       EPWM_setCounterCompareShadowLoadMode(base2, EPWM_COUNTER_COMPARE_B,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);

       // Clear old AQ state before installing the slave-leg edge rules.
       HWREGH(base2 + EPWM_O_AQCTLA) = 0;

       // PRIMARY BASE2 ACTION QUALIFIER
       // Raw A toggles at the upward/downward CMPA crossings, like base1.
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

       // Raw B toggles at period and zero. The dead-band block below derives
       // the final complementary output pair from raw A.
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

       // Reapply base2 digital-compare blanking after the remaining time-base
       // registers have been configured.
       EPWM_setDigitalCompareWindowOffset(base2, pwm_period_ticks - 5);
       EPWM_setDigitalCompareWindowLength(base2, 5);

       // Reapply the direction-sensitive CBC trip behavior for base2.
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZA_D,
                                 EPWM_TZ_ADV_ACTION_LOW);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZA_U,
                                 EPWM_TZ_ADV_ACTION_DISABLE);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZB_U,
                                 EPWM_TZ_ADV_ACTION_LOW);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZB_D,
                                 EPWM_TZ_ADV_ACTION_DISABLE);
       EPWM_setTripZoneAction(base2, EPWM_TZ_ACTION_EVENT_TZA,
                              EPWM_TZ_ACTION_DISABLE);
       EPWM_setTripZoneAction(base2, EPWM_TZ_ACTION_EVENT_TZB,
                              EPWM_TZ_ACTION_DISABLE);

       // Release a CBC fault at the zero/period boundary, then clear stale
       // digital-compare flags on both primary modules.
       EPWM_selectCycleByCycleTripZoneClearEvent(base2,
                              EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD);

       EPWM_clearTripZoneFlag(base1, (EPWM_TZ_INTERRUPT_DCAEVT2 |
               EPWM_TZ_SIGNAL_DCBEVT1));
       EPWM_clearTripZoneFlag(base2, (EPWM_TZ_INTERRUPT_DCAEVT2 |
               EPWM_TZ_SIGNAL_DCBEVT1));

       // PRIMARY BASE2 COMPLEMENTARY DEAD BAND
       EPWM_setDeadBandCounterClock(base2, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
       EPWM_setRisingEdgeDelayCount(base2, pwm_db_ticks_hs_base2);
       EPWM_setFallingEdgeDelayCount(base2, pwm_db_ticks_ls_base2);
       EPWM_setDeadBandDelayMode(base2, EPWM_DB_RED, true);
       EPWM_setDeadBandDelayMode(base2, EPWM_DB_FED, true);
       EPWM_setRisingEdgeDeadBandDelayInput(base2, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(base2, EPWM_DB_INPUT_EPWMA);
       EPWM_setDeadBandDelayPolarity(base2, EPWM_DB_RED,
                                     EPWM_DB_POLARITY_ACTIVE_HIGH);

       // Invert FED so the final B gate is complementary to A.
       EPWM_setDeadBandDelayPolarity(base2, EPWM_DB_FED,
                                     EPWM_DB_POLARITY_ACTIVE_LOW);

       // PRIMARY BASE2 PHASE SLAVE
       // On EPWM1 sync, EPWM2 loads PH_1_4 and then counts down. Because this
       // uses an up/down counter, phase counts are not simply
       // degrees*TBPRD/360; use the calibrated controller-case values.
       EPWM_enablePhaseShiftLoad(base2);
       EPWM_setSyncOutPulseMode(base2, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

       EPWM_setPhaseShift(base2, PH_1_4);
       EPWM_setCountModeAfterSync(base2, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);

       // End access to protected ePWM registers.
       EDIS;
}

/*
 * WPT_sec_setupPwms
 * -----------------
 * Configures the second synchronized full bridge. EPWM4 receives EPWM1's sync
 * and uses PH_1_5. EPWM5 receives that same incoming sync through EPWM4's
 * pass-through and uses PH_5_8. EPWM5 is not synchronized to EPWM4's shifted
 * counter zero, so its phase relative to EPWM4 depends on both phase values.
 * Duty, action qualifier, dead-band and trip-zone setup mirror the primary.
 *
 * If these pins are not connected to active secondary switches in a hardware
 * build, their PWM configuration still exists but does not represent the
 * passive diode bridge used by the parameter model.
 */
void WPT_sec_setupPwms(uint32_t base1, uint32_t base2,
                       uint16_t pwm_period_ticks, uint16_t pwm_cmpa_base1,
                       uint16_t pwm_cmpb_base1, uint16_t pwm_db_ticks_ls_base1,
                       uint16_t pwm_db_ticks_hs_base1, uint16_t pwm_db_ticks_ls_base2,
                       uint16_t pwm_db_ticks_hs_base2,uint16_t phaseshift_Q1_Q5,uint16_t phaseshift_Q5_Q8)
{
    // Enable writes to protected ePWM and synchronization registers.
    EALLOW;

       // SECONDARY BASE1 TIME BASE (EPWM4)
       // EPWM4 counts symmetrically 0 -> TBPRD -> 0 with no clock division.
       // Its phase is initially cleared here and loaded from PH_1_5 when the
       // EPWM1 synchronization configuration is installed below.
       EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_DIRECT_LOAD);
       EPWM_setTimeBasePeriod(base1, pwm_period_ticks);
       EPWM_setTimeBaseCounter(base1, 0);
       EPWM_setPhaseShift(base1, 0);
       EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
       EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

       // SECONDARY BASE1 COMPARES
       // Half-period CMPA/CMPB values create nominal 50% raw waveforms.
       EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A,
                                   pwm_period_ticks/2);
       EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_B, pwm_period_ticks/2);
       // Shadow loading at counter zero avoids mid-cycle compare changes.
       EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
       EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_B,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);

       // Clear any prior action-qualifier settings before defining new edges.
       HWREGH(base1 + EPWM_O_AQCTLA) = 0;

       // SECONDARY BASE1 ACTION QUALIFIER
       // A goes low on the upward CMPA crossing and high on the downward one.
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

       // B uses the same compare locations; the dead-band block converts the
       // raw waveforms to complementary high-side/low-side gate commands.
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
       EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

       // SECONDARY BASE1 DEAD BAND
       // RED delays raw A's rising edge and FED delays raw B's falling edge.
       // Active-low FED polarity creates the complementary B output.
       EPWM_setDeadBandCounterClock(base1, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
       EPWM_setRisingEdgeDelayCount(base1, pwm_db_ticks_hs_base1);
       EPWM_setFallingEdgeDelayCount(base1, pwm_db_ticks_ls_base1);

       EPWM_setDeadBandDelayMode(base1, EPWM_DB_RED, true);
       EPWM_setDeadBandDelayMode(base1, EPWM_DB_FED, true);
       EPWM_setRisingEdgeDeadBandDelayInput(base1, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(base1, EPWM_DB_INPUT_EPWMB);
       EPWM_setDeadBandDelayPolarity(base1, EPWM_DB_RED,
                                     EPWM_DB_POLARITY_ACTIVE_HIGH);
       // Invert the falling-edge path to obtain a complementary low-side gate.
       EPWM_setDeadBandDelayPolarity(base1, EPWM_DB_FED,
                                     EPWM_DB_POLARITY_ACTIVE_LOW);

       // SECONDARY BASE1 SYNCHRONIZATION
       // EPWM4 is a slave of EPWM1, not an independent master. Each EPWM1 sync
       // loads PH_1_5 into EPWM4. EPWM4 then forwards its received sync event.
//       EPWM_disablePhaseShiftLoad(base1);
//       EPWM_setSyncOutPulseMode(base1, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

       EPWM_enablePhaseShiftLoad(base1);
       EPWM_setSyncOutPulseMode(base1, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
       EPWM_setPhaseShift(base1, PH_1_5);
       EPWM_setCountModeAfterSync(base1, 0);

       // Route EPWM1's zero-reference sync pulse into EPWM4.
       SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4,
                                 SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);

       // SECONDARY BASE2 TIME BASE (EPWM5)
       // This leg also counts up/down. The historical -1 period matches the
       // synchronized slave configuration used on the primary bridge.
       EPWM_setPeriodLoadMode(base2, EPWM_PERIOD_DIRECT_LOAD);

       EPWM_setTimeBasePeriod(base2, (pwm_period_ticks - 1));
       EPWM_setTimeBaseCounter(base2, 0);
       EPWM_setPhaseShift(base2, 0);
       EPWM_setTimeBaseCounterMode(base2, EPWM_COUNTER_MODE_UP_DOWN);
       EPWM_setClockPrescaler(base2, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

       // SECONDARY BASE2 COMPARES
       // CMP=TBPRD/2 selects nominal 50% duty. A value of zero would clamp a
       // raw command and TBPRD/1.5 would give a different duty, but neither is
       // used in this configuration.
       EPWM_setCounterCompareValue(base2, EPWM_COUNTER_COMPARE_A, pwm_period_ticks/2);
       EPWM_setCounterCompareValue(base2, EPWM_COUNTER_COMPARE_B, pwm_period_ticks/2);

       // Load future compare changes only when the counter reaches zero.
       EPWM_setCounterCompareShadowLoadMode(base2, EPWM_COUNTER_COMPARE_A,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);
       EPWM_setCounterCompareShadowLoadMode(base2, EPWM_COUNTER_COMPARE_B,
                                            EPWM_COMP_LOAD_ON_CNTR_ZERO);

       // Clear previous action-qualifier state before defining this leg.
       HWREGH(base2 + EPWM_O_AQCTLA) = 0;

       // SECONDARY BASE2 ACTION QUALIFIER
       // Raw A toggles at the CMPA crossings.
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_A ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

       // Raw B toggles at period and zero; the dead-band output is ultimately
       // derived from A and inverted for complementary operation.
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
       EPWM_setActionQualifierAction(base2, EPWM_AQ_OUTPUT_B ,
               EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

       // SECONDARY BASE2 DIGITAL-COMPARE BLANKING
       // Suppress comparator events during the five-TBCLK switching-noise
       // window that starts five counts before the period boundary.
       EPWM_setDigitalCompareWindowOffset(base2, pwm_period_ticks - 5);


       EPWM_setDigitalCompareWindowLength(base2, 5);

       // SECONDARY BASE2 TRIP BEHAVIOR
       // Direction-sensitive CBC events force the conducting command low. The
       // ordinary TZA/TZB action is assigned later for one-shot software trips.
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZA_D,
                                 EPWM_TZ_ADV_ACTION_LOW);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZA_U,
                                 EPWM_TZ_ADV_ACTION_DISABLE);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZB_U,
                                 EPWM_TZ_ADV_ACTION_LOW);
       EPWM_setTripZoneAdvAction(base2, EPWM_TZ_ADV_ACTION_EVENT_TZB_D,
                                 EPWM_TZ_ADV_ACTION_DISABLE);
       EPWM_setTripZoneAction(base2, EPWM_TZ_ACTION_EVENT_TZA,
                              EPWM_TZ_ACTION_DISABLE);
       EPWM_setTripZoneAction(base2, EPWM_TZ_ACTION_EVENT_TZB,
                              EPWM_TZ_ACTION_DISABLE);

       // Clear a CBC fault only after the complete zero/period boundary.
       EPWM_selectCycleByCycleTripZoneClearEvent(base2,
                              EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD);

       // Remove stale digital-compare flags before PWM operation begins.
       EPWM_clearTripZoneFlag(base1, (EPWM_TZ_INTERRUPT_DCAEVT2 |
               EPWM_TZ_SIGNAL_DCBEVT1));
       EPWM_clearTripZoneFlag(base2, (EPWM_TZ_INTERRUPT_DCAEVT2 |
               EPWM_TZ_SIGNAL_DCBEVT1));

       // SECONDARY BASE2 DEAD BAND
       // Both delays use raw EPWMA; FED inversion creates complementary B.
       EPWM_setDeadBandCounterClock(base2, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
       EPWM_setRisingEdgeDelayCount(base2, pwm_db_ticks_hs_base2);
       EPWM_setFallingEdgeDelayCount(base2, pwm_db_ticks_ls_base2);
       EPWM_setDeadBandDelayMode(base2, EPWM_DB_RED, true);
       EPWM_setDeadBandDelayMode(base2, EPWM_DB_FED, true);
       EPWM_setRisingEdgeDeadBandDelayInput(base2, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(base2, EPWM_DB_INPUT_EPWMA);
       EPWM_setDeadBandDelayPolarity(base2, EPWM_DB_RED,
                                     EPWM_DB_POLARITY_ACTIVE_HIGH);

       // Invert the falling-edge path for the complementary low-side output.
       EPWM_setDeadBandDelayPolarity(base2, EPWM_DB_FED,
                                     EPWM_DB_POLARITY_ACTIVE_LOW);

       // SECONDARY BASE2 PHASE SLAVE
       // EPWM5 loads PH_5_8 on its sync event and then counts down. As with
       // PH_1_4, this register value is a calibrated up/down-counter phase tick,
       // not a direct number of degrees.
       EPWM_enablePhaseShiftLoad(base2);
       EPWM_setSyncOutPulseMode(base2, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

       EPWM_setPhaseShift(base2, PH_5_8);
       EPWM_setCountModeAfterSync(base2, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);

       // EPWM5 already receives EPWM4SYNCOUT through the fixed device chain.
       // The disabled alternative is design history, not active configuration.
//       EPWM_enablePhaseShiftLoad(base2);
//       EPWM_setSyncOutPulseMode(base2, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
//
//       EPWM_setPhaseShift(base2, PH_5_8);
//       EPWM_setCountModeAfterSync(base2, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
//
//       // For EPWM1, the sync input is: SYSCTL_SYNC_IN_SRC_EXTSYNCIN1
//              //
//        SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM5, SYSCTL_SYNC_IN_SRC_EPWM4SYNCOUT);


       // End access to protected ePWM registers.
       EDIS;
}
//
// No more.
//
