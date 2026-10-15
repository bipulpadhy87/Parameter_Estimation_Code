# Complex-receiver inductance estimator

Revision: 2026-09-26

**Before enabling identification, set the actual installed primary and
secondary compensation capacitances.** Both remain `0UL` by default, as in the
supplied firmware; zero capacitance intentionally prevents a valid result.
Do not substitute the simulation's values for hardware measurements.

Also enter/verify the actual primary bridge voltage, effective series losses,
coil-inductance bounds, phase calibration, and uncertainty allowances. The
inherited voltage default is 5000 mV; the inherited resistance defaults are
zero. None of these defaults is a measurement of the hardware.

The numerical core uses `long double` for the slow solve. TI documents this as
64-bit for both C28x COFF and EABI; plain `double` is only 32-bit in COFF. Allow
64-bit floating-point operations in the CCS compiler settings (do not retain
`--float_operations_allowed=none` or `=32` for this translation unit). Keep
strict floating-point semantics for the NaN/finite checks. The C28x runtime
support library supplies the arithmetic. No MATLAB runtime, heap allocation,
complex-number library, or external numerical library is required.

The larger matrices use static storage. The solver is not reentrant and must
not run in an ISR. It executes in the existing paused-acquisition analysis
path after both records are collected. Primary PWM continues as before during
analysis. **Target execution time, stack/RAM/flash use, watchdog servicing and
protection behavior have NOT been measured.** Check the CCS map and profiling
results, and preserve independent hardware protection before a power test.
Start validation with appropriate low-energy, current-limited conditions;
the parameter estimator is not a protection function.

TI references for the numeric implementation:
- https://software-dl.ti.com/docs/esd/SPRU514/c28x-double-and-long-double-floating-point-types-spru5144178.html
- https://software-dl.ti.com/C2000/docs/optimization_guide/phase1/floating_point.html

## Which uploaded method was used

The basis is `parameterestimation.m` from `Parameter estimation.zip`.
An unmodified copy is included as `reference/parameterestimation_original.m`.
That script uses two **fundamental** input impedances to estimate four real
unknowns: common self-inductance L, mutual inductance M, receiver resistance Req,
and receiver reactance Xeq. Its later calculation of DC load resistance still
uses `pi^2/8`, and its DC-current calculation uses `2/pi`.

This package ports the complex-load approach and its analytic quadratic seeds,
**but does not port those DC conversions**. To retain the user's requirement
that the third harmonic participate in identification, it extends the model
to a second, independent complex receiver ratio at the third harmonic.
That extension and the multistart nonlinear optimizer are new work; they are
not an assertion that the uploaded MATLAB script already implemented them.

The alternative `doublefdoublepulse.m` was not used as the estimator core. Its
`f9_dual_asym` enforces a secondary-current zero at a phase derived from receiver
voltage and later uses bridge-waveform voltage/current conversions. Those
constraints are not appropriate for an arbitrary synchronous switching law.
`Lpestimation.m` likewise has a virtual-short assumption for its unsensed
receiver case and optional secondary measurements. No Simulink model in the
ZIP was executed or changed.

## Model and the four input impedances

The four observations, in code order, are

    Zin1(f0), Zin1(f1), Zin3(f0), Zin3(f1).

For harmonic h at operating point j:

    omega_hj = h * 2*pi * f_j
    Zp_hj = Rp + j*(omega_hj*L - 1/(omega_hj*Cp))
    Zs_hj = Rs + j*(omega_hj*L - 1/(omega_hj*Cs))
    Zin_hj = Zp_hj + (omega_hj*M)^2 / (Zs_hj + Zrec_h).

The independent fitted parameters are

    L, |M|, Rrec1, Xrec1, Rrec3, Xrec3.
    Zrec1 = Rrec1 + j*Xrec1
    Zrec3 = Rrec3 + j*Xrec3

There are eight measured real components and six fitted real parameters.
Zrec1 need not equal Zrec3. Rrec3 can be negative: these are terminal phasor
ratios under switching operation, not four independent passive linear loads.
A negative apparent third-harmonic resistance is not itself a fault.
Fundamental receiver resistance is constrained to be nonnegative; reverse
power-flow operation is outside this version.

**This model is rectifier-type independent, not assumption-free.** It can be
applied to diode or synchronous receivers when the following conditions hold:

1. For each harmonic separately, the receiver terminal ratio stays
   approximately constant between the two test frequencies:

       Zrec1(f0) approximately equals Zrec1(f1)
       Zrec3(f0) approximately equals Zrec3(f1).

   This is the essential restriction that makes identification possible.
   It is not the assumption Zrec1 = Zrec3. A receiver controller, DC voltage,
   conduction pattern or load change can violate it. The default inherited
   85/100 kHz pair is NOT certified to satisfy this restriction. Check it
   experimentally or in a trustworthy full switching model. A smaller
   frequency gap may reduce model drift but also worsens identifiability.

2. The resonant network is series-series; Lp=Ls=L. Known Cp, Cs, Rp and Rs must
   describe the actual paths. The supplied constant-loss model neglects
   frequency-dependent losses beyond those constants.

3. L and M are stable across both records and harmonics. Saturation, appreciable
   motion during collection, or changing coil geometry can violate this.

4. Input voltage/current phasors are correctly calibrated and time-aligned,
   both records are in periodic steady state, and both harmonics are strong
   enough to be measured. Receiver harmonic frequency conversion is NOT
   represented by a general harmonic-transfer matrix.

Residual and ambiguity checks cannot detect every possible model violation:
a changed receiver can sometimes imitate different coil parameters. A low
residual or a valid flag is not proof of physical truth. No claim is made that
this works for every possible synchronous controller or diode conduction mode.

## Numerical implementation

The source-script quadratic is retained for initialization, once per harmonic.
Let W=omega*1e-6 so L and M are expressed in microhenries, and let

    D_j = Re(Zin_j) - Rp
    C_j = Im(Zin_j) + 1/(omega_j*Cp)
    K = (W0/W1)^2.

Then the source's constant-complex-load constraint gives

    a = W0^2*(D0-D1)
    b = 2*(D1*W0*C0 - D0*W0^2*C1/W1)
    c = K*D0*(D1^2+C1^2) - D1*(D0^2+C0^2)
    a*L^2 + b*L + c = 0.

All in-range roots are retained. The code does not choose the first positive
root or silently return a nominal inductance when inversion fails. Near-linear
quadratics are handled separately; a cancellation-resistant quadratic formula
is used for ordinary roots.

Additional bounded grid seeds come from eliminating the receiver ratios:

    F_hj = Zin_hj - Zp_hj(L)
    A_hj = W_hj^2/F_hj
    t = M_uH^2
    Zrec_hj = t*A_hj - Zs_hj(L).

Equal receiver ratios across the frequency pair require

    t*(A_h0-A_h1) = j*(Xs_h0-Xs_h1).

A weighted least-squares solution for real t initializes M. The two
reconstructed ratios initialize each harmonic's load. These are only seeds.
The final damped Gauss-Newton iterations minimize the **forward input-impedance
errors**, using all eight real/imaginary components and analytic derivatives.

The search is finite multistart optimization, not a certified global search.
It compares materially different found candidates, rejects bound-limited or
locally weak results, and returns numerical diagnostics. It cannot prove
that an unsearched alternative solution does not exist.

## Measurement uncertainty and thresholds

The adapter computes an engineering allowance for each complex observation:

    sigma_Z = absolute_floor + relative_allowance*abs(Z)
              + sqrt(sigma_V^2 + abs(Z)^2*sigma_I^2)/abs(I).

These are editable settings in `psfbpcmc_user_settings.h`. The supplied defaults
are starting allowances, not characterized ADC/sensor statistics. The
linearized L/M sensitivity is obtained from the local weighted Jacobian and
these allowances; it is not a guaranteed accuracy or confidence interval.
Systematic phase, winding-loss, capacitance and model errors are not fully
represented by these numbers. Do not enlarge allowances simply to suppress
faults or to force an estimate to be accepted.

Default checks include normalized residual RMS <= 3, relative L/M sensitivity
<= 20%, and the inherited additional absolute residual ceiling of 5 ohms.
A second found candidate within four normalized squared-error units and
separated by over 1% in L or 2% in M triggers the ambiguity flag. These are
engineering acceptance rules, not universal statistical thresholds.

## CCS Expressions / outputs

Always inspect `WPT_identificationValid` before using the fitted values.
The solver can expose a rejected candidate's values for diagnosis; they are
not valid measurements when that flag is zero.

| Variable | Meaning / units |
|---|---|
| `WPT_estimatedSelfInductanceNanoHenry` | Common Lp=Ls, nH |
| `WPT_estimatedMutualInductanceNanoHenry` | Magnitude of M, nH |
| `WPT_estimatedCouplingPermille` | 1000*abs(M)/L |
| `WPT_estimatedEquivalentAcLoadMilliOhm` | Fundamental Rrec1, mOhm; legacy name retained |
| `WPT_estimatedEquivalentAcReactanceMilliOhm` | Fundamental Xrec1, signed mOhm |
| `WPT_estimatedThirdAcResistanceMilliOhm` | Third-harmonic Rrec3, signed mOhm |
| `WPT_estimatedThirdAcReactanceMilliOhm` | Third-harmonic Xrec3, signed mOhm |
| `WPT_identificationResidualMilliOhm` | Unweighted RMS across eight components, mOhm |
| `WPT_identificationNormalizedResidualPermille` | 1000 * weighted RMS |
| `WPT_identificationLRelativeSensitivityPermille` | 1000 * local relative L sensitivity |
| `WPT_identificationMRelativeSensitivityPermille` | 1000 * local relative M sensitivity |
| `WPT_identificationImpedanceSigmaMilliOhm[4]` | Per-observation engineering uncertainty, mOhm |
| `WPT_identificationSolverFlags` | Bitmask described below; 0 means checks passed |
| `WPT_identificationSolutionsChecked` | Number of multistart results checked |
| `WPT_identificationAlternativeSelfInductanceNanoHenry` | Found alternative L when ambiguous |
| `WPT_identificationAlternativeMutualInductanceNanoHenry` | Found alternative M when ambiguous |
| `WPT_identificationModelWarning` | Stays 1 after every solve: modeling assumptions remain |
| `WPT_estimatedDcLoadValid` | Always 0; no DC load estimate is available |
| `WPT_estimatedDcLoadMilliOhm` | Always invalid sentinel 0xFFFFFFFF, never a resistance |

Flag bits may be combined:

| Bit value | Meaning |
|---:|---|
| 1 | Invalid configuration/input |
| 2 | No physically acceptable seeded solution |
| 4 | Excessive fit residual |
| 8 | Materially different, similarly compatible solutions found |
| 16 | Weak local L/M identifiability or singular sensitivity calculation |
| 32 | Result at an inductance/coupling bound |
| 64 | Required third-harmonic record missing |
| 128 | Invalid/too-small phasor or guarded arithmetic range |

Set `WPT_identificationResetRequest=1` to start another experiment, as before.
Read the two existing input-impedance arrays for the fundamental and the two
third-harmonic arrays to see the actual four observations supplied to the fit.
No secondary current sensor or secondary telemetry was added.

## PWM, dead time and acquisition

The existing acquisition rate, ADC configuration, harmonic extraction,
automatic case-7 to case-5 transition, PWM setup, phase values, blanking, and
trip handling are retained. In particular, case 7 retains **PH_1_4=586**.
Seventeen existing functions were compared after removing comments and
whitespace; their executable source is unchanged.

Primary voltage reconstruction still uses actual configured PWM timing,
primary dead time and current-dependent clamp polarity. The only executable
change in that reconstruction function permits zero source V3 when a usable
I3 is present; it no longer unnecessarily rejects that observation. The
dead-time arithmetic itself is unchanged.

That voltage reconstruction still neglects Coss, driver propagation delays,
diodes' finite voltage drops and ringing. Measured primary voltage phasor
overrides remain available. No receiver-side PWM phase or diode waveform is
used to calculate the receiver ratios.

## Validation actually performed

These checks are included as code/results in `tests/`:

- Estimator core compiled with GCC C99, `-Wall -Wextra -Werror`, using host
  long double and an explicit 64-bit double configuration.
- Five noiseless synthetic model cases: resistive fundamental load, reactive
  load, negative third-harmonic resistance, strong coupling, and different
  fundamental/third loads. Parameter recovery matched the constructed models
  to numerical precision. This is algebra/software validation, not evidence
  about a real rectifier.
- Analytic Jacobian checked against independent NumPy finite differences.
- Invalid capacitance/frequency/noise settings, NaNs, weak coupling, selected
  receiver changes and corrupt observations tested. Rejected results clear
  stale valid outputs. This does not claim all receiver changes are detectable.
- Forty randomized exact-model cases and forty noisy cases with fixed seed
  260926. All passed the stated test assertions. In the noisy test, each
  impedance component had 0.01-ohm Gaussian noise and the largest L/M errors
  were approximately 0.157% / 0.209% for that one chosen synthetic system.
- The same core tests ran under GCC's undefined-behavior sanitizer.
- The actual new adapter was extracted from the revised main.c and compiled
  with the actual updated settings/header. With quantized synthetic mV/mA
  phasors, true L=180 uH and M=130 uH produced L=179.799 uH and M=129.783 uH;
  the input-impedance RMS residual was 44 mOhm. DC validity stayed false.
- Default zero capacitor settings, missing third data, zero third current,
  oversized phasors and repeated frequencies failed closed in adapter tests.
- Regression comparison confirmed the preserved executable functions and
  case-7 phase correction.

**NOT performed:** full TI CCS compilation/linking, real-time timing or stack
profiling on the F280049C, MATLAB/Simulink execution, time-domain diode or
synchronous-rectifier simulation, sensor calibration, or hardware testing.
Do not interpret the synthetic tests as verification of arbitrary receivers.

To rerun the portable tests, install Python 3, NumPy and a C99 GCC compiler:

    python tests/test_estimator.py
    python tests/test_integration.py
    python tests/test_extended.py

The preservation test additionally needs the unmodified pre-change firmware:

    python tests/test_preservation.py /path/to/original/directory

Test binaries are intentionally not included. Generated `tests/build/` files
are host-test artifacts, not CCS project sources.

## Package contents

The three firmware files are at the package root. Two unified `.patch` files
show changes against the supplied original main/settings files. `reference/`
contains the unmodified chosen MATLAB script. `tests/` contains test sources
and recorded JSON results. No source Library file or original ZIP was modified.
