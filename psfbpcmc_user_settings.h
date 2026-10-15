//#############################################################################
//
// FILE:   psfbpcmc_user_settings.h
//
// TITLE:  psfbpcmc User Settings
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
 * ============================================================================
 * BEGINNER'S GUIDE TO THIS SETTINGS FILE
 * ============================================================================
 *
 * This header contains compile-time constants. A #define does not execute at
 * run time; the C preprocessor substitutes its value wherever the name is used
 * before the compiler builds the program.
 *
 * The file has two kinds of settings:
 *
 *   1. WPT experiment settings
 *      Sampling rate, commanded bridge voltage, compensation capacitors,
 *      winding resistance estimates, automatic frequency transition, and
 *      the limits and acceptance checks of the online complex-load estimator.
 *
 *   2. Hardware mappings inherited from the TI PSFB reference project
 *      CPU timers, ADC inputs, PWM modules/GPIO pins, comparators, and SFRA
 *      serial-interface settings. Many of these are not used by the present
 *      polling-based WPT measurement loop, but they are retained so the
 *      original reference-project support code can still compile.
 *
 * IMPORTANT UNIT CONVENTIONS
 * --------------------------
 *   suffix _HZ       frequency in hertz
 *   suffix _US       time in microseconds
 *   suffix _MV       voltage in millivolts
 *   suffix _MA       current in milliamps
 *   suffix _MOHM     resistance/reactance/impedance in milliohms
 *   suffix _PF       capacitance in picofarads
 *   suffix _NH       inductance in nanohenries
 *   suffix _TICKS    PWM time-base clock counts
 *
 * IDENTIFICATION SETTINGS THAT MUST MATCH THE PHYSICAL HARDWARE
 * ---------------------------------------------------------------------------
 *   WPT_PRIMARY_SQUARE_PEAK_MV
 *       Peak of the bipolar square-wave bridge voltage. A bridge waveform of
 *       +Vpk and -Vpk has Vpp = 2*Vpk.
 *
 *   WPT_PRIMARY_CAPACITANCE_PF and WPT_SECONDARY_CAPACITANCE_PF
 *       Installed primary and secondary series-compensation capacitors.
 *       The estimator deliberately stays disabled while either is zero.
 *
 *   WPT_PRIMARY_RESISTANCE_MOHM and WPT_SECONDARY_RESISTANCE_MOHM
 *       Effective AC series resistance of the two resonant paths, including
 *       winding loss, capacitor ESR, and wiring loss.
 *
 * HARDWARE-MAPPING WARNING
 * ------------------------
 * Do not change ADC channels, EPWM base addresses, or GPIO pin-mux values just
 * to change a numerical experiment setting. Those names describe physical
 * connections on the controller board and must agree with the schematic.
 */

//
// LEGACY PEAK-CURRENT-MODE CONTROL DEFAULTS
//
// These two constants came from the TI phase-shifted-full-bridge reference
// design. PSFB_SLOPE_INITIAL is the starting slope-compensation command.
// PSFB_IRAMPMAX_SET_RATIO is the largest unsigned Q16-like current-reference
// ratio (65534 is one count below full scale). The present open-loop WPT
// waveform measurement does not calculate L or M from these values.
//
#define PSFB_SLOPE_INITIAL 55
#define PSFB_IRAMPMAX_SET_RATIO    65534

//
// BACKGROUND TASK TIMERS
//
// Each TASKx name selects one of the three C28x CPU timers. The *_FREQ_HZ
// values are the intended task service rates:
//
//     timer period (seconds) = 1 / task frequency (hertz)
//
// These timers belong to the original reference-project scheduler. The WPT
// current-capture loop itself is paced by EPWM3, not by these task constants.
//
#define TASKA_CPUTIMER_BASE CPUTIMER0_BASE
#define TASKB_CPUTIMER_BASE CPUTIMER1_BASE
#define TASKC_CPUTIMER_BASE CPUTIMER2_BASE

#define TASKA_FREQ_HZ         ((uint16_t)   2000)
#define TASKB_FREQ_HZ         ((uint16_t)    20)
#define TASKC_FREQ_HZ         ((uint16_t) 10000)

//
// LEGACY INTERRUPT ROUTING
//
// The first pair says which EPWM event requests the control ISR; the second
// pair identifies the EPWM2 trip-zone ISR. The PIE-group constants tell the
// ISR which interrupt-acknowledge group to clear after servicing an event.
// The current WPT test loop polls the ADC and does not use an ADC ISR.
//
#define PSFB_C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE    PSFB_FB_PWM1_BASE
#define PSFB_C28x_CONTROLISR_INTERRUPT                  PSFB_FB_PWM1_INT

#define PSFB_C28x_ISR2_INTERRUPT_TRIG_PWM_BASE          EPWM2_BASE
#define PSFB_C28x_ISR2_INTERRUPT                        INT_EPWM2_TZ

#define PSFB_C28x_CONTROLISR_INTERRUPT_PIE_GROUP_NO     INTERRUPT_ACK_GROUP3
#define PSFB_C28x_ISR2_INTERRUPT_PIE_GROUP_NO           INTERRUPT_ACK_GROUP2

//
// DEFAULT PWM PERIOD AND COMPARE VALUES
//
// The PWM counters run in up-down mode. One electrical switching cycle takes
// 2*TBPRD time-base counts, so
//
//     TBPRD = f_TBCLK / (2*f_switch).
//
// With a 100 MHz PWM clock and f_switch = 85 kHz, TBPRD is about 588 counts.
// PHASE_SHIFT_MAX uses the same count limit. CMPA/CMPB are compare locations
// inside the period and determine the raw edge positions before dead time.
//
#define PSFB_FB_PWM_PERIOD_TICKS (PSFB_PWMSYSCLOCK_FREQ_HZ) / ((float32_t)85000*2)
#define PSFB_PHASE_SHIFT_MAX  (PSFB_PWMSYSCLOCK_FREQ_HZ) / ((float32_t)85000*2)

#define PSFB_FB_CMPA_PWM_BASE1 0
#define PSFB_FB_CMPB_PWM_BASE1 368

//
// LEGACY VOLTAGE-LOOP DIGITAL COMPENSATOR
//
// These are the coefficients of a second-order direct-form controller:
//
//   y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2]
//                    - A1*y[n-1] - A2*y[n-2].
//
// They are retained for the TI closed-loop PSFB build. The WPT harmonic extractor is fixed-point; its new slow parameter
// solver uses long double. Neither uses these legacy controller coefficients.
//
#define PSFB_GV_DF22_B0    (float32_t) 14.0313626*1.6
#define PSFB_GV_DF22_B1    (float32_t) -25.2978386*1.6
#define PSFB_GV_DF22_B2    (float32_t) 11.4016633*1.6
#define PSFB_GV_DF22_A1    (float32_t) -1.8873439
#define PSFB_GV_DF22_A2    (float32_t) 0.8873439


//
// PWM DEAD-BAND COUNTS
//
// RED (rising-edge delay) and FED (falling-edge delay) prevent both switches
// in one bridge leg from conducting simultaneously. The effective delay is
//
//     dead time = dead-band counts / dead-band clock frequency.
//
// At a 100 MHz full-cycle dead-band clock, one count is 10 ns and the present
// value 1*10 gives 100 ns. Verify the actual EPWM clock/divider configuration
// and increase dead time only as required by the gate driver and MOSFETs.
// "pri" constants belong to the primary bridge; "sec" constants belong to the
// secondary bridge outputs used by this project.
//
#define PSFB_FB_DB_PWM_HS_COUNT_BASE2_pri 1*10
#define PSFB_FB_DB_PWM_LS_COUNT_BASE2_pri 1*10
#define PSFB_FB_DB_PWM_HS_COUNT_BASE1_pri 1*10
#define PSFB_FB_DB_PWM_LS_COUNT_BASE1_pri 1*10

#define PSFB_FB_DB_PWM_HS_COUNT_BASE2_sec 1*10
#define PSFB_FB_DB_PWM_LS_COUNT_BASE2_sec 1*10
#define PSFB_FB_DB_PWM_HS_COUNT_BASE1_sec 1*10
#define PSFB_FB_DB_PWM_LS_COUNT_BASE1_sec 1*10
// BOARD STATUS LED GPIO MAPPING
// Each *_IO is the GPIO number and each *_IO_CONF is the matching pin-mux
// selector passed to GPIO_setPinConfig().
#define PSFB_LED_CCARD1_IO       32
#define PSFB_LED_CCARD1_IO_CONF  GPIO_32_GPIO32
#define PSFB_LED_CCARD2_IO       34
#define PSFB_LED_CCARD2_IO_CONF  GPIO_34_GPIO34

//
// GENERAL ADC TIMING
//
// ADC_POWERUP_US is the delay after enabling the ADC before conversions begin.
// PWMtiggerADC_RATIO belongs to the reference design's PWM-triggered samples.
// The WPT current sampler below uses its own EPWM3-to-ADC trigger sequence.
//
#define PSFB_ADC_POWERUP_US      ((uint16_t)1000)
#define PSFB_PWMtiggerADC_RATIO  ((uint16_t)1)

// WPT CURRENT-WAVEFORM SAMPLING RATE
//
// The sample period is Ts = 1/Fs. At 1 MSPS, Ts = 1 us. At 1 MSPS the
// 300 kHz third harmonic from
// a 100 kHz operating point has 3.33 samples per cycle. The F280049C ADC can
// support this rate with the configured acquisition window, but the present
// implementation still polls each conversion, so monitor the overflow count
// before increasing this further.
#define WPT_CURRENT_SAMPLE_RATE_HZ             1000000UL

//
// ONLINE WPT PARAMETER-IDENTIFICATION SETTINGS
//
// The estimator observes the fundamental and third harmonic at two switching
// frequencies. For every harmonic it forms the complex input impedance
//
//     Z_h = V_h / I_h = R_h + j*X_h,
//
// then fits an equal-self-inductance, series-series coupled-coil model. These
// settings supply the quantities that cannot be inferred from primary current
// alone and define when a measurement is accepted.
//
// The commanded primary voltage is a bipolar square wave that switches
// between +WPT_PRIMARY_SQUARE_PEAK_MV and -WPT_PRIMARY_SQUARE_PEAK_MV.
// Therefore its peak-to-peak voltage is twice this setting. An ideal bipolar
// square wave has only odd harmonics:
//
//     V1,peak = 4*Vpk/pi,
//     V3,peak = 4*Vpk/(3*pi), with a NEGATIVE third coefficient when the
//               waveform is sign(cos(theta)). The reference is PWM1 zero.
//
// Enter the actual bridge voltage at the coil terminals, including any known
// bus-voltage correction. This is PEAK voltage, not peak-to-peak or RMS.
//
#define WPT_PRIMARY_SQUARE_PEAK_MV            5000L

// PRIMARY VOLTAGE RECONSTRUCTION (the secondary PWM pins are unused)
// Reconstruct EPWM1/2 gate states at each TBCLK using period, phase, RED/FED.
// During both-gates-off intervals, positive current from the EPWM1 leg to the
// EPWM2 leg clamps the first node LOW and second HIGH through ideal body
// diodes. Negative current reverses the clamps. Reconstruct sign from I1+I3.
// Integrating the resulting voltage gives COMPLEX V1/V3. This accounts for
// commanded dead time and phase but assumes instantaneous commutation: Coss,
// diode drops, driver delays and ringing are not modeled. Near zero current,
// retain the previous node level and flag uncertainty. This is approximate.
#define WPT_PRIMARY_SYNC_DELAY_TICKS              2U
#define WPT_DEADTIME_MIN_CURRENT_MA              25L
#define WPT_VOLTAGE_MODEL_MAX_PERIOD_TICKS      4096U

// Measured voltage override: set WPT_identificationUseMeasuredVoltage[j]=1
// and fill WPT_identificationMeasuredV1RealMilliVolts[j],
// ...MeasuredV1ImagMilliVolts[j], ...MeasuredV3RealMilliVolts[j] and
// ...MeasuredV3ImagMilliVolts[j], j=0/1 for the first/second point.
// Use PEAK phasors of the differential bridge voltage relative to PWM1
// COUNTER ZERO: v(t)=Vr*cos(theta)-Vi*sin(theta). Overrides survive reset.
// Update them if bus voltage, frequency, phase, dead time or load changes.

// COMPLEX-RECEIVER IDENTIFICATION (no rectifier-type switch)
// The solver fits separate receiver ratios Zrec1=R1+jX1 and Zrec3=R3+jX3.
// Each ratio is held approximately constant across the TWO operating points,
// not across harmonics. This allows reactance and negative third resistance.
// It assumes Lp=Ls and forward receiving operation. It is NOT valid for
// arbitrary receiver control/load changes between captures. No DC quantities
// are estimated, and no passive-bridge harmonic-voltage formula is imposed.
// All four measured impedances remain mandatory. A missing third harmonic
// produces WPTCE_MISSING_THIRD rather than an undocumented fallback.
//
// SERIES COMPENSATION CAPACITORS
//
// The capacitor reactance used by the model is
//
//     Xc = -1/(2*pi*f*C).
//
// Identification remains disabled while
// either value is zero, so these placeholders must be replaced with the
// installed capacitances before using the reported L, M, or load values.
#define WPT_PRIMARY_CAPACITANCE_PF               0UL
#define WPT_SECONDARY_CAPACITANCE_PF             0UL

// KNOWN SERIES RESISTANCES
//
// Effective AC series resistance of each resonant path at the operating
// frequency: coil copper/AC resistance, capacitor ESR, and wiring. Units are
// milliohms. The solver reports secondary equivalent load as
//
//     Req = fitted secondary total resistance - Rs.
//
// Zero is permitted as a first approximation, but inaccurate loss estimates
// will be absorbed into the reported load.
#define WPT_PRIMARY_RESISTANCE_MOHM               0L
#define WPT_SECONDARY_RESISTANCE_MOHM             0L

// CURRENT-SAMPLE PHASE CALIBRATION
//
// Phase error caused by a fixed time delay is
//
//     phase error (degrees) = 360*f*delay.
//
// This setting expresses that correction in 100 MHz PWM time-base ticks. The default
// places the ADC sample at the center of its 21-cycle acquisition window.
// Adjust this value if a calibrated voltage/current phase measurement shows
// a fixed sensor or ADC timing offset.
#define WPT_CURRENT_PHASE_CAL_TICKS               10L

// TWO-FREQUENCY COLLECTION ACCEPTANCE RULES
//
// A capture is considered stable when its frequency is within STABLE_TOLERANCE
// of the running target. A difference of at least FREQUENCY_CHANGE announces
// the second operating point. The count constants reject transient captures,
// average several stable captures, and prevent division by near-zero current.
#define WPT_IDENT_FREQUENCY_STABLE_TOLERANCE_HZ 1000UL
#define WPT_IDENT_FREQUENCY_CHANGE_HZ            3000UL
#define WPT_IDENT_STABLE_CAPTURE_COUNT              3U
#define WPT_IDENT_SETTLING_CAPTURE_COUNT            3U
#define WPT_IDENT_AVERAGE_CAPTURE_COUNT              3U
#define WPT_IDENT_MIN_FUNDAMENTAL_CURRENT_MA         50L

// The third harmonic participates through its OWN fitted complex load.
// Reject it below this peak amplitude because division by a very small third-
// harmonic current produces an unusably noisy impedance estimate.
#define WPT_IDENT_MIN_THIRD_CURRENT_MA               25L
#define WPT_IDENT_MIN_THIRD_VALID_CAPTURE_COUNT        2U

// AUTOMATIC PWM PROFILE SEQUENCE
//
// The controller-case numbers refer to WPT_loadPwmControllerCase() in
// psfbpcmc_main.c. That function defines the period and phase shifts for each
// operating point. The automatic sequence changes only after frequency 1 has
// been accepted.
//
// Automatic PWM profile sequence used by the two-frequency identification.
// The default starts with case 7 (85 kHz), collects the first point, blanks
// the bridge, changes to case 5 (100 kHz), and then collects the second point.
// Set WPT_PWM_AUTO_TRANSITION_ENABLE to 0U to change profiles manually.
#define WPT_PWM_AUTO_TRANSITION_ENABLE                1U
#define WPT_PWM_FIRST_CONTROLLER_CASE                 7U
#define WPT_PWM_SECOND_CONTROLLER_CASE                5U

// SAFE PROFILE-TRANSITION BLANKING
//
// Both bridge outputs are held low before and after their time-base and phase
// registers are changed. The post-change interval lets resonant current from
// the old operating point decay before switching resumes.
#define WPT_PWM_TRANSITION_PRE_BLANK_US               20UL
#define WPT_PWM_TRANSITION_POST_BLANK_US            1000UL

// COMMON SELF-INDUCTANCE LIMITS
// Lp=Ls=L is retained. Set these from actual coil bounds, not fitted data.
// The old integer-grid-only solver has been replaced by the commented
// wpt_complex_estimator.h numerical core. Public output units remain integer.
#define WPT_IDENT_MIN_SELF_INDUCTANCE_NH          1000UL
#define WPT_IDENT_MAX_SELF_INDUCTANCE_NH        500000UL

// Receiver component bounds (applied separately to R1,X1,R3,X3).
// Fundamental R1>=0; R3 may be negative because harmonics exchange power.
#define WPT_IDENT_MAX_RECEIVER_COMPONENT_MOHM   10000000L

// IMPEDANCE UNCERTAINTY / ACCEPTANCE SETTINGS
// Initial engineering allowances, NOT verified sensor statistics. Calibrate.
// sigmaZ = floor + relative*|Z| + sqrt(sigmaV^2+|Z|^2*sigmaI^2)/|I|.
// Larger sigma reduces weighting and INCREASES the sensitivity diagnostic.
// Do not loosen these simply to force a valid result. The estimator rejects
// excessive residual, weak local sensitivity, detected ambiguity and bounds.
#define WPT_IDENT_IMPEDANCE_NOISE_FLOOR_MOHM           20L
#define WPT_IDENT_IMPEDANCE_RELATIVE_NOISE_PERMILLE     2L
#define WPT_IDENT_VOLTAGE_PHASOR_NOISE_MV               1L
#define WPT_IDENT_CURRENT_PHASOR_NOISE_MA               1L
#define WPT_IDENT_MAX_NORMALIZED_RMS_PERMILLE       3000UL
#define WPT_IDENT_MAX_RELATIVE_SENSITIVITY_PERMILLE   200UL
#define WPT_IDENT_AMBIGUITY_DELTA_COST                  4L
// Retained additional absolute RMS ceiling, in milliohms.
#define WPT_IDENT_MAX_IMPEDANCE_RESIDUAL_MOHM        5000UL

// ModelWarning stays one after every solve: equal L and frequency-invariant
// receiver ratios remain assumptions, even when measured primary V is used.
// identificationValid means all IMPLEMENTED numerical checks passed, NOT a
// proof of uniqueness, calibration, model correctness or safe power operation.
// The slow solver uses C28x 64-bit long double and runs outside the ISR.
// Allow 64-bit floating operations in CCS; profile execution and memory there.

//
// INPUT/OUTPUT CROSSBAR RESET VALUE
//
// The C2000 crossbar routes internal and external signals to peripherals.
// Zero starts the associated mux register with no optional inputs selected;
// the HAL later enables only the routes required by the board.
//
#define XBAR_MUX_INIT  0x00

//
// COMPARATOR SUBSYSTEM (CMPSS) PROTECTION DEFAULTS
//
// PCMC_DAC_INIT is the initial peak-current comparator reference and
// OCP_DAC_INIT is the over-current threshold in 12-bit DAC counts. The digital
// filter accepts a fault only after THRESHOLD matching samples inside a WINDOW
// sampled every PRESCALE clocks. These values protect hardware and are not
// part of the software current-amplitude calculation.
//
#define PSFB_PCMC_DAC_INIT 512
#define PSFB_OCP_DAC_INIT 3584
#define PSFB_OCP_SAMPLE_PRESCALE 2
#define PSFB_OCP_SAMPLE_WINDOW 31
#define PSFB_OCP_THRESHOLD 25

//
// REFERENCE-PROJECT ADC SIGNAL MAP
//
// Every signal group below supplies the fields required to configure an ADC
// start-of-conversion (SOC):
//
//   *_ADC_MODULE          ADC peripheral (ADCA, ADCB, or ADCC)
//   *_ADC_SOC_NO          SOC slot whose registers hold this setup
//   *_ADC_PIN             analog input channel wired to the signal
//   *_ADC_TRIG_SOURCE     event that starts sampling
//   *_ADC_ACQPS_SYS_CLKS  sample-and-hold window in system-clock counts
//   *_ADCRESULTREGBASE    result-register block read by software
//
// These groups describe the original board telemetry. The added AC current
// sensor is configured explicitly in WPT_setupCurrentSensing() as VOUT=ADCC4
// and VREF=ADCB3, so do not confuse it with ILV_FB below.
//
// LVBUS / VLV_FB: output voltage before the ORing FET.
//
#define PSFB_LVBUS_ADC_MODULE            ADCA_BASE
#define PSFB_LVBUS_ADC_SOC_NO            ADC_SOC_NUMBER0
#define PSFB_LVBUS_ADC_PIN               ADC_CH_ADCIN5
#define PSFB_LVBUS_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_LVBUS_ADC_ACQPS_SYS_CLKS    10
#define PSFB_LVBUS_ADCRESULTREGBASE      ADCARESULT_BASE

//
// LVBUS_1 / VLV_FB1: final output voltage after the ORing FET.
//
#define PSFB_LVBUS_1_ADC_MODULE            ADCA_BASE
#define PSFB_LVBUS_1_ADC_SOC_NO            ADC_SOC_NUMBER1
#define PSFB_LVBUS_1_ADC_PIN               ADC_CH_ADCIN3
#define PSFB_LVBUS_1_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_LVBUS_1_ADC_ACQPS_SYS_CLKS    10
#define PSFB_LVBUS_1_ADCRESULTREGBASE      ADCARESULT_BASE

//
// ACTIVE_EMI_INJ_PD_LEVEL: active-EMI injection power-down monitor.
//
#define PSFB_EMI_INJ_PD_ADC_MODULE            ADCA_BASE
#define PSFB_EMI_INJ_PD_ADC_SOC_NO            ADC_SOC_NUMBER3
#define PSFB_EMI_INJ_PD_ADC_PIN               ADC_CH_ADCIN1
#define PSFB_EMI_INJ_PD_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_EMI_INJ_PD_ADC_ACQPS_SYS_CLKS    10
#define PSFB_EMI_INJ_PD_ADCRESULTREGBASE      ADCARESULT_BASE

//
// ADC MODULE B -- TMP_SEC: secondary-side temperature signal.
//
#define PSFB_TMP_SEC_ADC_MODULE            ADCB_BASE
#define PSFB_TMP_SEC_ADC_SOC_NO            ADC_SOC_NUMBER0
#define PSFB_TMP_SEC_ADC_PIN               ADC_CH_ADCIN0
#define PSFB_TMP_SEC_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_TMP_SEC_ADC_ACQPS_SYS_CLKS    10
#define PSFB_TMP_SEC_ADCRESULTREGBASE      ADCBRESULT_BASE

//
// ILV_FB: original reference-board DC output-current feedback.
//
#define PSFB_ILV_FB_ADC_MODULE            ADCB_BASE
#define PSFB_ILV_FB_ADC_SOC_NO            ADC_SOC_NUMBER1
#define PSFB_ILV_FB_ADC_PIN               ADC_CH_ADCIN1
#define PSFB_ILV_FB_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_ILV_FB_ADC_ACQPS_SYS_CLKS    10
#define PSFB_ILV_FB_ADCRESULTREGBASE      ADCBRESULT_BASE

// CMPSS1 monitors the same original ILV_FB analog path for peak-current-mode
// control and over-current protection. The ASYSCTL mux values select which
// analog pin reaches the high and low comparators.
#define PSFB_PCMC_OCP_CMPSS_BASE                 CMPSS1_BASE
#define PSFB_PCMC_OCP_CMPSS_ASYSCTRL_CMPHPMUX    ASYSCTL_CMPHPMUX_SELECT_1
#define PSFB_PCMC_OCP_CMPSS_ASYSCTRL_CMPLPMUX    ASYSCTL_CMPLPMUX_SELECT_1
#define PSFB_PCMC_OCP_CMPSS_ASYSCTRL_MUX_VALUE   0

//
// ILV_ZCD: output-current zero-crossing detector monitor.
//
#define PSFB_ILV_ZCD_ADC_MODULE            ADCB_BASE
#define PSFB_ILV_ZCD_ADC_SOC_NO            ADC_SOC_NUMBER2
#define PSFB_ILV_ZCD_ADC_PIN               ADC_CH_ADCIN2
#define PSFB_ILV_ZCD_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_ILV_ZCD_ADC_ACQPS_SYS_CLKS    10
#define PSFB_ILV_ZCD_ADCRESULTREGBASE      ADCBRESULT_BASE

//
// FAN_FB: fan tachometer/feedback analog input.
//
#define PSFB_FAN_FB_ADC_MODULE            ADCB_BASE
#define PSFB_FAN_FB_ADC_SOC_NO            ADC_SOC_NUMBER3
#define PSFB_FAN_FB_ADC_PIN               ADC_CH_ADCIN3
#define PSFB_FAN_FB_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_FAN_FB_ADC_ACQPS_SYS_CLKS    10
#define PSFB_FAN_FB_ADCRESULTREGBASE      ADCBRESULT_BASE

//
// ADC MODULE C -- ADC_EMI_SENSE: active-EMI sensing input.
//
#define PSFB_EMI_SENSE_ADC_MODULE            ADCC_BASE
#define PSFB_EMI_SENSE_ADC_SOC_NO            ADC_SOC_NUMBER0
#define PSFB_EMI_SENSE_ADC_PIN               ADC_CH_ADCIN2
#define PSFB_EMI_SENSE_ADC_TRIG_SOURCE       ADC_TRIGGER_EPWM1_SOCA
#define PSFB_EMI_SENSE_ADC_ACQPS_SYS_CLKS    10
#define PSFB_EMI_SENSE_ADCRESULTREGBASE      ADCCRESULT_BASE

//
// EPWM MODULE AND GPIO PIN MAP
//
// Each bridge leg has an EPWM peripheral plus A (high-side command) and B
// (low-side command) GPIO mappings. The *_GPIO value is the physical pin
// number; *_GPIO_PIN_CONFIG selects the EPWM function instead of ordinary
// GPIO. EPWM1/2 form the primary full bridge and EPWM4/5 form the second full
// bridge in the WPT setup. EPWM3 is reserved as the 1 MHz ADC sample clock;
// EPWM7 is retained for other reference builds.
//
#define PSFB_FB_PWM1_BASE                   EPWM1_BASE
#define PSFB_FB_PWM1_DCBEVT1                EPWM_TZ_INTERRUPT_DCBEVT1
#define PSFB_FB_PWM1_INT                    INT_EPWM1

#define PSFB_FB_PWM1_H_GPIO                 0
#define PSFB_FB_PWM1_H_GPIO_PIN_CONFIG      GPIO_0_EPWM1_A

#define PSFB_FB_PWM1_L_GPIO                 1
#define PSFB_FB_PWM1_L_GPIO_PIN_CONFIG      GPIO_1_EPWM1_B


#define PSFB_FB_PWM2_BASE                 EPWM2_BASE

#define PSFB_FB_PWM2_H_GPIO               2
#define PSFB_FB_PWM2_H_GPIO_PIN_CONFIG    GPIO_2_EPWM2_A

#define PSFB_FB_PWM2_L_GPIO               3
#define PSFB_FB_PWM2_L_GPIO_PIN_CONFIG    GPIO_3_EPWM2_B

#define PSFB_FB_PWM3_BASE                   EPWM3_BASE
#define PSFB_FB_PWM3_DCBEVT1                EPWM_TZ_INTERRUPT_DCBEVT3
#define PSFB_FB_PWM3_INT                    INT_EPWM3

#define PSFB_FB_PWM3_H_GPIO                 4
#define PSFB_FB_PWM3_H_GPIO_PIN_CONFIG      GPIO_4_EPWM3_A

#define PSFB_FB_PWM3_L_GPIO                 5
#define PSFB_FB_PWM3_L_GPIO_PIN_CONFIG      GPIO_5_EPWM3_B


#define PSFB_FB_PWM4_BASE                 EPWM4_BASE

#define PSFB_FB_PWM4_H_GPIO               6
#define PSFB_FB_PWM4_H_GPIO_PIN_CONFIG    GPIO_6_EPWM4_A

#define PSFB_FB_PWM4_L_GPIO               7
#define PSFB_FB_PWM4_L_GPIO_PIN_CONFIG    GPIO_7_EPWM4_B

#define PSFB_FB_PWM5_BASE                 EPWM5_BASE

#define PSFB_FB_PWM5_H_GPIO               8
#define PSFB_FB_PWM5_H_GPIO_PIN_CONFIG    GPIO_8_EPWM5_A

#define PSFB_FB_PWM5_L_GPIO               9
#define PSFB_FB_PWM5_L_GPIO_PIN_CONFIG    GPIO_9_EPWM5_B

#define PSFB_FB_PWM7_BASE                 EPWM7_BASE

#define PSFB_FB_PWM7_H_GPIO               28
#define PSFB_FB_PWM7_H_GPIO_PIN_CONFIG    GPIO_28_EPWM7_A

#define PSFB_FB_PWM7_L_GPIO               29
#define PSFB_FB_PWM7_L_GPIO_PIN_CONFIG    GPIO_29_EPWM7_B

// DISABLED REFERENCE-DESIGN OPTIONS
//
// Everything in the following block begins with // and is therefore excluded
// from compilation. It documents earlier oversampling, synchronous-rectifier,
// fan, relay, and auxiliary-converter mappings. Uncommenting one item alone is
// not enough; the matching HAL code and board wiring must also be present.
//#define PSFB_ADC_OVERSAMPLE_PWM_BASE     EPWM5_BASE
//
//
//#define PSFB_SR_PWM_BASE                 EPWM3_BASE
//#define PSFB_SR_PWM_INT                  INT_EPWM3
//
//#define PSFB_SR_PWM_H_GPIO                4
//#define PSFB_SR_PWM_H_GPIO_PIN_CONFIG    GPIO_4_EPWM3_A
//
//#define PSFB_SR_PWM_L_GPIO                5
//#define PSFB_SR_PWM_L_GPIO_PIN_CONFIG    GPIO_5_EPWM3_B


//
//#define PSFB_ON_SWITCH_GPIO              40
//#define PSFB_ON_SWITCH_GPIO_PIN_CONFIG   GPIO_40_GPIO40
//
//#define PSFB_FAN_CTRL_PWM_GPIO             11
//#define PSFB_FAN_CTRL_PWM_GPIO_PIN_CONFIG GPIO_11_EPWM6_B
//
//#define PSFB_SR_FLT_BASE                 EPWM8_BASE
//#define PSFB_SR_FLT_INT                  INT_EPWM8
//
//#define PSFB_ORING_EN_PWM_GPIO             13
//#define PSFB_ORING_EN_PWM_GPIO_PIN_CONFIG GPIO_13_GPIO13
//
//#define PSFB_PSON_GPIO                      17
//#define PSFB_PSON_EN_PWM_GPIO_PIN_CONFIG GPIO_17_GPIO17
//
//
//#define PSFB_ACL_PWM_BASE                EPWM4_BASE
//#define PSFB_PWM_ACL1_GPIO                6
//#define PSFB_PWM_ACL1_GPIO_PIN_CONFIG    GPIO_6_EPWM4_A
//
//#define PSFB_PWM_ACL2_GPIO                7
//#define PSFB_PWM_ACL2_GPIO_PIN_CONFIG    GPIO_7_EPWM4_B
//
//#define PSFB_SR_FLT_L_GPIO                15
//#define PSFB_SR_FLT_L_GPIO_PIN_CONFIG    GPIO_15_EPWM8_B
//
//#define PSFB_SR_DEADBAND_RED_INITIAL     15
//#define PSFB_SR_DEADBAND_FED_INITIAL     15
//
//#define PSFB_ACL_DEADBAND_RED_INITIAL     70
//#define PSFB_ACL_DEADBAND_FED_INITIAL     70


//
// SOFTWARE FREQUENCY RESPONSE ANALYZER (SFRA)
//
// SFRA belongs to the original closed-loop power-supply example. It injects a
// small swept perturbation and reports the control-loop response to a PC GUI.
// It is unrelated to the WPT fundamental/third-harmonic DFT. The ISR rate is
// 100 kHz here.
//
#define PSFB_SFRA_ISR_FREQ_HZ       (((float)100)*1000.0)

//
// SFRA MEASUREMENT TYPE AND GUI SERIAL PORT
//
// SFRA Options: SCIA or SCIB. Since some secondary faults signal status has
// to be communicated to primary using SCIA in HPE server power supply design
// SFRA module must use SCIB
//
#define PSFB_SFRA_VOLTAGE 1
#define PSFB_SFRA_TYPE PSFB_SFRA_VOLTAGE

// SERIAL LINK PARAMETERS
//
// For the F280049 controlCARD, SCIA is connected to the onboard FTDI USB
// bridge. The GUI baud calculation uses a 50 MHz low-speed peripheral clock.
#define PSFB_SFRA_GUI_SCI_BASE SCIA_BASE
//#define PSFB_SFRA_GUI_SCI_BASE SCIB_BASE
#define PSFB_SCI_VBUS_CLK ((float32_t)50 * 1000000)
#define PSFB_SFRA_GUI_SCI_BAUDRATE 57600

// SCIA RX/TX GPIO numbers. The commented SCIB alternatives below apply only
// if the board and SFRA base selection are changed together.
#define PSFB_SFRA_GUI_SCIRX_GPIO 28
#define PSFB_SFRA_GUI_SCITX_GPIO 29

//
//For SCIB_BASE
//#define PSFB_SFRA_GUI_SCIRX_GPIO 55
//#define PSFB_SFRA_GUI_SCITX_GPIO 54

// SCIA pin-mux function selectors corresponding to GPIO28 and GPIO29.
#define PSFB_SFRA_GUI_SCIRX_GPIO_PIN_CONFIG GPIO_28_SCIA_RX
#define PSFB_SFRA_GUI_SCITX_GPIO_PIN_CONFIG GPIO_29_SCIA_TX

//
//For SCIB_BASE
//
//#define PSFB_SFRA_GUI_SCIRX_GPIO_PIN_CONFIG GPIO_55_SCIB_RX
//#define PSFB_SFRA_GUI_SCITX_GPIO_PIN_CONFIG GPIO_54_SCIB_TX

//
// SFRA STATUS LED
// If the first define is 1, the SFRA GUI communication code drives GPIO31 as
// a status LED; if it is 0, the LED indication code is omitted.
//
#define PSFB_SFRA_GUI_LED_INDICATOR 1
#define PSFB_SFRA_GUI_LED_GPIO 31
#define PSFB_SFRA_GUI_LED_GPIO_PIN_CONFIG GPIO_31_GPIO31

//
// SFRA SWEEP SETTINGS
//
// A logarithmic sweep starts at 10 Hz and contains 100 frequency points.
// Consecutive frequencies satisfy f[k+1] = f[k]*step. The factor 1.0798 gives
// about 30 steps per decade. The injected perturbation amplitude is 0.003 in
// the SFRA library's normalized control-variable units.
//
#if PSFB_SFRA_TYPE == PSFB_SFRA_VOLTAGE
#define PSFB_SFRA_FREQ_START 10
#define PSFB_SFRA_FREQ_LENGTH 100

//
// General logarithmic-step equation: multiplier = 10^(1/points-per-decade).
// The configured numeric multiplier is authoritative if this comment and an
// older reference-project annotation disagree.
//
#define PSFB_SFRA_FREQ_STEP_MULTIPLY ((float32_t)1.0798)//25 steps/dec
#define PSFB_SFRA_AMPLITUDE ((float32_t)0.003)
#endif

//
// sfra_gui_scicomms_driverlib.c contains the communication code that consumes
// the SCI base, baud rate, pin, and LED settings above.
//
