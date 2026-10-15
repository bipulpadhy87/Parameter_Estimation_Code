#ifndef WPT_COMPLEX_ESTIMATOR_H
#define WPT_COMPLEX_ESTIMATOR_H
/*
 * WPT complex-receiver parameter estimator, 2026-09-26.
 *
 * Basis: parameterestimation.m in the user's Parameter estimation.zip.
 * That script identifies L, M, Req, Xeq from two fundamental impedances.
 * This extension retains BOTH harmonics with independent Zrec,1 and Zrec,3.
 *
 * IMPORTANT: topology-independent is NOT assumption-free. For each h=1,3,
 * Zrec,h(f0) must approximately equal Zrec,h(f1) during the experiment.
 * Zrec,1 and Zrec,3 need NOT equal one another. We do not impose a diode
 * voltage waveform, current-zero switching, a virtual short, or a DC/AC
 * conversion. Negative third-harmonic resistance is allowed: this is a
 * terminal phasor ratio at an operating point, not a passive LTI element.
 * The fundamental load is constrained to nonnegative resistance (forward
 * receiving operation). Reverse power flow is outside this version.
 *
 * Other assumptions: series-series compensation; Lp=Ls=L; known Cp,Cs,Rp,Rs;
 * linear magnetic parameters stable across both records and harmonics;
 * phase-calibrated primary voltage/current phasors in periodic steady state.
 *
 * Observations are ordered [f0, f1, 3*f0, 3*f1]. The six real unknowns are
 * p = [L_uH, M_uH, Rrec1_ohm, Xrec1_ohm, Rrec3_ohm, Xrec3_ohm].
 * Eight real observations give two redundant real constraints, not four
 * freely varying receiver impedances. No formula for Rdc, Idc or Vdc is used.
 *
 * Numerical method: analytic dual-frequency quadratic seeds + a bounded
 * profile grid; multistart damped Gauss-Newton refinement of the FORWARD
 * complex impedances; physical, residual, ambiguity and local-sensitivity
 * checks. This is a finite multistart search, NOT a global uniqueness proof.
 *
 * Header-only to avoid another CCS compilation unit. Include in main.c only.
 * Long double has 64-bit precision under both C28x COFF and EABI. This slow
 * solve is not an ISR. Acquisition/trigonometry in main.c remain fixed point.
 * No heap allocation, complex library, MATLAB runtime, or math library calls.
 */
#include <stdint.h>
#include <float.h>

#ifndef WPTCE_REAL_TYPE
#define WPTCE_REAL_TYPE long double
#endif
typedef WPTCE_REAL_TYPE WPTCE_Real;
#define WPTCE_R(x) ((WPTCE_Real)(x##L))
#define WPTCE_PI WPTCE_R(3.1415926535897932384626433832795)
#define WPTCE_NPAR 6U
#define WPTCE_NOBS 4U
#define WPTCE_NRES 8U
#define WPTCE_MAX_SEEDS 16U
#define WPTCE_GRID_SEEDS 8U
#define WPTCE_MAX_ITERATIONS 80U
#define WPTCE_GRID_STEPS 96U

/* Bit flags: zero means all implemented numerical checks passed. */
#define WPTCE_BAD_INPUT          1U
#define WPTCE_NO_SOLUTION        2U
#define WPTCE_HIGH_RESIDUAL      4U
#define WPTCE_AMBIGUOUS          8U
#define WPTCE_WEAK_IDENTIFICATION 16U
#define WPTCE_AT_BOUND           32U
#define WPTCE_MISSING_THIRD      64U
#define WPTCE_BAD_PHASOR         128U

typedef struct {
    WPTCE_Real frequency_hz[2];
    WPTCE_Real zin_real_ohm[WPTCE_NOBS];
    WPTCE_Real zin_imag_ohm[WPTCE_NOBS];
    /* Per-component engineering uncertainty, in ohms, not a confidence level. */
    WPTCE_Real sigma_ohm[WPTCE_NOBS];
    WPTCE_Real cp_pf, cs_pf, rp_ohm, rs_ohm;
    WPTCE_Real min_l_uh, max_l_uh;
    WPTCE_Real min_frequency_separation_hz;
    WPTCE_Real max_receiver_magnitude_ohm;
    WPTCE_Real max_normalized_rms;
    WPTCE_Real max_relative_sensitivity;
    WPTCE_Real ambiguity_delta_cost;
} WPTCE_Input;

typedef struct {
    WPTCE_Real p[WPTCE_NPAR];
    WPTCE_Real rms_ohm, normalized_rms;
    /* Linearized sensitivity to the supplied sigma; not a guaranteed error. */
    WPTCE_Real relative_l_sensitivity, relative_m_sensitivity;
    WPTCE_Real alternative_l_uh, alternative_m_uh;
    uint16_t flags, solutions_checked;
} WPTCE_Result;

typedef struct { WPTCE_Real re, im; } WPTCE_Complex;
typedef struct { WPTCE_Real p[WPTCE_NPAR], cost; } WPTCE_Candidate;

static WPTCE_Real WPTCE_abs(WPTCE_Real x) { return x < 0 ? -x : x; }
/* Bounds make all ordinary calculations much smaller than this limit.
 * NaN fails x==x; infinities fail the magnitude comparison. */
static uint16_t WPTCE_finite(WPTCE_Real x)
{ return (uint16_t)((x == x) && (WPTCE_abs(x) < WPTCE_R(1e100))); }

/* Bounded, scale-normalized Newton square root avoids ABI-dependent sqrt(). */
static WPTCE_Real WPTCE_sqrt(WPTCE_Real x)
{
    WPTCE_Real scale = WPTCE_R(1.0), y;
    uint16_t k;
    if (!(x > 0) || !WPTCE_finite(x)) return 0;
    while (x > WPTCE_R(4.0)) { x *= WPTCE_R(0.25); scale *= 2; }
    while (x < WPTCE_R(1.0)) { x *= 4; scale *= WPTCE_R(0.5); }
    y = (x + 1) * WPTCE_R(0.5);
    for (k = 0U; k < 12U; ++k) y = (y + x/y) * WPTCE_R(0.5);
    return y * scale;
}
static WPTCE_Complex WPTCE_c(WPTCE_Real a, WPTCE_Real b)
{ WPTCE_Complex z; z.re=a; z.im=b; return z; }
static WPTCE_Complex WPTCE_mul(WPTCE_Complex a, WPTCE_Complex b)
{ return WPTCE_c(a.re*b.re-a.im*b.im, a.re*b.im+a.im*b.re); }
static WPTCE_Real WPTCE_norm2(WPTCE_Complex a)
{ return a.re*a.re+a.im*a.im; }

/* W=omega*1e-6 has units rad/us, so W*L_uH is ohms.
 * Kp=1/(omega*Cp), Ks=1/(omega*Cs) are POSITIVE capacitor magnitudes. */
static void WPTCE_circuit(const WPTCE_Input *in, uint16_t q,
                          WPTCE_Real *w, WPTCE_Real *kp, WPTCE_Real *ks)
{
    WPTCE_Real omega=2*WPTCE_PI*in->frequency_hz[q%2U];
    if (q>=2U) omega*=3;
    *w=omega*WPTCE_R(1e-6);
    *kp=WPTCE_R(1e12)/(omega*in->cp_pf);
    *ks=WPTCE_R(1e12)/(omega*in->cs_pf);
}

/* Physical domain; invalid candidates never become published valid results. */
static uint16_t WPTCE_physical(const WPTCE_Input *in, const WPTCE_Real *p)
{
    uint16_t i;
    for(i=0U;i<WPTCE_NPAR;++i) if(!WPTCE_finite(p[i])) return 0U;
    if(p[0]<in->min_l_uh || p[0]>in->max_l_uh ||
       p[1]<=WPTCE_R(1e-8) || p[1]>=p[0] || p[2]<0) return 0U;
    for(i=2U;i<WPTCE_NPAR;++i)
        if(WPTCE_abs(p[i])>in->max_receiver_magnitude_ohm) return 0U;
    return 1U;
}

/* Forward model and exact analytic Jacobian (derivatives of R and X).
 *
 * D = Rs+Rrec,h + j*(W*L-Ks+Xrec,h), Q=(W*M)^2
 * Zin = Rp+j*(W*L-Kp)+Q/D
 * dZ/dL=j*W*(1-Q/D^2), dZ/dM=2*W^2*M/D
 * dZ/dRrec=-Q/D^2, dZ/dXrec=-j*Q/D^2.
 * Residuals are divided by sigma before optimization. */
static uint16_t WPTCE_evaluate(const WPTCE_Input *in, const WPTCE_Real *p,
                              WPTCE_Real *r, WPTCE_Real j[8][6],
                              WPTCE_Real *cost, WPTCE_Real *raw_cost)
{
    uint16_t q,k,h;
    WPTCE_Real w,kp,ks,d2,qq,er,ei;
    WPTCE_Complex d,inv,inv2,z,der[6],a;
    *cost=0; *raw_cost=0;
    if(!WPTCE_physical(in,p)) return 0U;
    for(q=0U;q<4U;++q) {
        h=q/2U; WPTCE_circuit(in,q,&w,&kp,&ks);
        d=WPTCE_c(in->rs_ohm+p[2U+2U*h],w*p[0]-ks+p[3U+2U*h]);
        d2=WPTCE_norm2(d);
        if(!(d2>WPTCE_R(1e-16))) return 0U;
        inv=WPTCE_c(d.re/d2,-d.im/d2); inv2=WPTCE_mul(inv,inv);
        qq=w*w*p[1]*p[1];
        z=WPTCE_c(in->rp_ohm+qq*inv.re,w*p[0]-kp+qq*inv.im);
        er=z.re-in->zin_real_ohm[q]; ei=z.im-in->zin_imag_ohm[q];
        r[2U*q]=er/in->sigma_ohm[q]; r[2U*q+1U]=ei/in->sigma_ohm[q];
        *raw_cost+=er*er+ei*ei;
        *cost+=r[2U*q]*r[2U*q]+r[2U*q+1U]*r[2U*q+1U];
        if(j) {
            for(k=0U;k<6U;++k) der[k]=WPTCE_c(0,0);
            a=WPTCE_c(1-qq*inv2.re,-qq*inv2.im);
            der[0]=WPTCE_c(-w*a.im,w*a.re);
            der[1]=WPTCE_c(2*w*w*p[1]*inv.re,2*w*w*p[1]*inv.im);
            der[2U+2U*h]=WPTCE_c(-qq*inv2.re,-qq*inv2.im);
            der[3U+2U*h]=WPTCE_c(qq*inv2.im,-qq*inv2.re);
            for(k=0U;k<6U;++k) {
                j[2U*q][k]=der[k].re/in->sigma_ohm[q];
                j[2U*q+1U][k]=der[k].im/in->sigma_ohm[q];
            }
        }
    }
    return WPTCE_finite(*cost);
}

/* 6x6 Gaussian elimination with partial pivoting. Inputs are overwritten.
 * The matrices have column-normalized derivatives, not mixed uH/ohm scales. */
static uint16_t WPTCE_linear(WPTCE_Real a[6][6], WPTCE_Real b[6],
                            WPTCE_Real x[6])
{
    uint16_t i,j,k,p;
    WPTCE_Real v,t,m;
    for(k=0U;k<6U;++k) {
        p=k; v=WPTCE_abs(a[k][k]);
        for(i=k+1U;i<6U;++i) if(WPTCE_abs(a[i][k])>v) {
            p=i; v=WPTCE_abs(a[i][k]);
        }
        if(!(v>WPTCE_R(1e-12)) || !WPTCE_finite(v)) return 0U;
        if(p!=k) {
            for(j=k;j<6U;++j) { t=a[k][j]; a[k][j]=a[p][j]; a[p][j]=t; }
            t=b[k]; b[k]=b[p]; b[p]=t;
        }
        for(i=k+1U;i<6U;++i) {
            m=a[i][k]/a[k][k];
            for(j=k;j<6U;++j) a[i][j]-=m*a[k][j];
            b[i]-=m*b[k];
        }
    }
    for(i=6U;i>0U;--i) {
        k=i-1U; t=b[k];
        for(j=k+1U;j<6U;++j) t-=a[k][j]*x[j];
        x[k]=t/a[k][k]; if(!WPTCE_finite(x[k])) return 0U;
    }
    return 1U;
}

/* Construct normalized J'J, J'r and column scales. */
static void WPTCE_normal(WPTCE_Real j[8][6], const WPTCE_Real *r,
                        WPTCE_Real a[6][6], WPTCE_Real g[6], WPTCE_Real s[6])
{
    uint16_t i,k,n;
    for(i=0U;i<6U;++i) {
        s[i]=0;
        for(n=0U;n<8U;++n) s[i]+=j[n][i]*j[n][i];
        s[i]=WPTCE_sqrt(s[i]);
        if(s[i]<WPTCE_R(1e-15)) s[i]=WPTCE_R(1e-15);
    }
    for(i=0U;i<6U;++i) {
        g[i]=0;
        for(n=0U;n<8U;++n) g[i]+=j[n][i]*r[n]/s[i];
        for(k=0U;k<6U;++k) {
            a[i][k]=0;
            for(n=0U;n<8U;++n) a[i][k]+=(j[n][i]/s[i])*(j[n][k]/s[k]);
        }
    }
}

/* Minimize measured input-impedance residuals, not reconstructed receiver
 * residuals. Increasing damping handles rejected/nonphysical trial steps. */
static void WPTCE_refine(const WPTCE_Input *in, WPTCE_Candidate *c)
{
    static WPTCE_Real r[8],j[8][6],a[6][6],g[6],s[6],dx[6],trial[6];
    WPTCE_Real tr[8],cost,raw,ncost,nraw,lambda=WPTCE_R(0.001),step;
    uint16_t it,k;
    if(!WPTCE_evaluate(in,c->p,r,j,&cost,&raw)) { c->cost=WPTCE_R(1e90); return; }
    for(it=0U;it<WPTCE_MAX_ITERATIONS;++it) {
        WPTCE_normal(j,r,a,g,s);
        for(k=0U;k<6U;++k) { a[k][k]+=lambda; g[k]=-g[k]; }
        if(!WPTCE_linear(a,g,dx)) break;
        step=0;
        for(k=0U;k<6U;++k) {
            trial[k]=c->p[k]+dx[k]/s[k];
            step+=WPTCE_abs(dx[k]/s[k])/(1+WPTCE_abs(c->p[k]));
        }
        if(WPTCE_evaluate(in,trial,tr,0,&ncost,&nraw) && ncost<cost) {
            for(k=0U;k<6U;++k) c->p[k]=trial[k];
            lambda*=WPTCE_R(0.25);
            if(lambda<WPTCE_R(1e-12)) lambda=WPTCE_R(1e-12);
            if(!WPTCE_evaluate(in,c->p,r,j,&cost,&raw)) break;
            if(step<WPTCE_R(1e-10) || cost<WPTCE_R(1e-18)) break;
        } else {
            lambda*=10;
            if(lambda>WPTCE_R(1e14) || step<WPTCE_R(1e-12)) break;
        }
    }
    c->cost=cost;
}

/* Seed construction by eliminating the two unknown complex loads.
 * Fhj=Zinhj-Zphj(L), Ahj=Whj^2/Fhj, t=M_uH^2.
 * Zrechj=t*Ahj-Zshj(L). Equal receiver ratios across the two frequencies
 * require t*(Ah0-Ah1)=j*(Xsh0-Xsh1).
 * A weighted linear least-squares estimate of real t gives a seed, then
 * each Zrec,h is the average of its two reconstructed ratios. This is only
 * initialization; the final optimizer fits the forward impedances above. */
static uint16_t WPTCE_seed(const WPTCE_Input *in, WPTCE_Real l,
                           WPTCE_Candidate *c)
{
    WPTCE_Complex aa[4],delta;
    WPTCE_Real xs[4],ww[4],w,kp,ks,fr,fi,d2,num=0,den=0,weight,t;
    WPTCE_Real r[8],raw;
    uint16_t q,h;
    if(l<in->min_l_uh || l>in->max_l_uh) return 0U;
    for(q=0U;q<4U;++q) {
        WPTCE_circuit(in,q,&w,&kp,&ks); ww[q]=w; xs[q]=w*l-ks;
        fr=in->zin_real_ohm[q]-in->rp_ohm;
        fi=in->zin_imag_ohm[q]-(w*l-kp); d2=fr*fr+fi*fi;
        if(d2<WPTCE_R(1e-16)) return 0U;
        aa[q]=WPTCE_c(w*w*fr/d2,-w*w*fi/d2);
    }
    for(h=0U;h<2U;++h) {
        q=2U*h; delta=WPTCE_c(aa[q].re-aa[q+1U].re,aa[q].im-aa[q+1U].im);
        /* First-order noise amplification of A=w^2/F. */
        weight=WPTCE_norm2(aa[q])*WPTCE_norm2(aa[q])*
                   in->sigma_ohm[q]*in->sigma_ohm[q]/(ww[q]*ww[q]*ww[q]*ww[q]);
        weight+=WPTCE_norm2(aa[q+1U])*WPTCE_norm2(aa[q+1U])*
                   in->sigma_ohm[q+1U]*in->sigma_ohm[q+1U]/
                   (ww[q+1U]*ww[q+1U]*ww[q+1U]*ww[q+1U]);
        weight=1/(weight+WPTCE_R(1e-30));
        num+=weight*delta.im*(xs[q]-xs[q+1U]);
        den+=weight*WPTCE_norm2(delta);
    }
    if(!(den>WPTCE_R(1e-30))) return 0U;
    t=num/den; if(!(t>0) || t>=l*l) return 0U;
    c->p[0]=l; c->p[1]=WPTCE_sqrt(t);
    for(h=0U;h<2U;++h) {
        q=2U*h;
        c->p[2U+2U*h]=t*(aa[q].re+aa[q+1U].re)/2-in->rs_ohm;
        c->p[3U+2U*h]=(t*(aa[q].im+aa[q+1U].im)-xs[q]-xs[q+1U])/2;
    }
    if(c->p[2]<0 && c->p[2]>-WPTCE_R(0.05)) c->p[2]=0;
    return WPTCE_evaluate(in,c->p,r,0,&c->cost,&raw);
}

/* The source MATLAB script's quadratic, applied separately to h=1 and h=3.
 * D=Re(Zin)-Rp, C=Im(Zin)+Kp, K=(W0/W1)^2.
 * a=W0^2(D0-D1)
 * b=2(D1 W0 C0-D0 W0^2 C1/W1)
 * c=K D0(D1^2+C1^2)-D1(D0^2+C0^2).
 * ALL in-range roots are retained. No "first positive"/nearest-nominal rule. */
static uint16_t WPTCE_roots(const WPTCE_Input *in, uint16_t h,
                            WPTCE_Real roots[2])
{
    WPTCE_Real w0,w1,kp0,kp1,ks,d0,d1,c0,c1,a,b,c,disc,q,scale;
    uint16_t i,n=0U,base=2U*h;
    WPTCE_circuit(in,base,&w0,&kp0,&ks);
    WPTCE_circuit(in,base+1U,&w1,&kp1,&ks);
    d0=in->zin_real_ohm[base]-in->rp_ohm;
    d1=in->zin_real_ohm[base+1U]-in->rp_ohm;
    c0=in->zin_imag_ohm[base]+kp0; c1=in->zin_imag_ohm[base+1U]+kp1;
    a=w0*w0*(d0-d1);
    b=2*(d1*w0*c0-d0*w0*w0*c1/w1);
    c=(w0/w1)*(w0/w1)*d0*(d1*d1+c1*c1)-d1*(d0*d0+c0*c0);
    /* Relative degeneracy checks use the actual configured L interval. */
    scale=WPTCE_abs(a)*in->max_l_uh*in->max_l_uh+
          WPTCE_abs(b)*in->max_l_uh+WPTCE_abs(c);
    if(!(scale>WPTCE_R(1e-24))) return 0U;
    if(WPTCE_abs(a)*in->max_l_uh*in->max_l_uh<WPTCE_R(1e-12)*scale) {
        if(WPTCE_abs(b)*in->max_l_uh<WPTCE_R(1e-12)*scale) return 0U;
        roots[n++]=-c/b;
    } else {
        disc=b*b-4*a*c;
        if(disc<0) return 0U;
        q=-WPTCE_R(0.5)*(b+(b>=0 ? WPTCE_sqrt(disc) : -WPTCE_sqrt(disc)));
        if(WPTCE_abs(q)<WPTCE_R(1e-24)) roots[n++]=-b/(2*a);
        else { roots[n++]=q/a; roots[n++]=c/q; }
    }
    for(i=0U;i<n;) {
        if(!WPTCE_finite(roots[i]) || roots[i]<in->min_l_uh || roots[i]>in->max_l_uh)
            roots[i]=roots[--n];
        else ++i;
    }
    return n;
}

/* Input checks include uncertainty and frequency separation, so a zero sigma
 * or two copies of the same record cannot silently produce a valid result. */
static uint16_t WPTCE_input_ok(const WPTCE_Input *in)
{
    uint16_t q;
    if(!in) return 0U;
    for(q=0U;q<2U;++q)
        if(!WPTCE_finite(in->frequency_hz[q]) || in->frequency_hz[q]<1000 ||
           in->frequency_hz[q]>WPTCE_R(1e6)) return 0U;
    if(!WPTCE_finite(in->cp_pf) || !WPTCE_finite(in->cs_pf) ||
       in->cp_pf<=0 || in->cs_pf<=0 || in->cp_pf>WPTCE_R(1e9) || in->cs_pf>WPTCE_R(1e9) ||
       !WPTCE_finite(in->rp_ohm) || !WPTCE_finite(in->rs_ohm) ||
       in->rp_ohm<0 || in->rs_ohm<0 || in->rp_ohm>WPTCE_R(1e4) || in->rs_ohm>WPTCE_R(1e4) ||
       !WPTCE_finite(in->min_l_uh) || !WPTCE_finite(in->max_l_uh) ||
       in->min_l_uh<=0 || in->max_l_uh<=in->min_l_uh || in->max_l_uh>WPTCE_R(1e4) ||
       !WPTCE_finite(in->min_frequency_separation_hz) || in->min_frequency_separation_hz<=0 ||
       WPTCE_abs(in->frequency_hz[0]-in->frequency_hz[1])<in->min_frequency_separation_hz ||
       !WPTCE_finite(in->max_receiver_magnitude_ohm) || in->max_receiver_magnitude_ohm<=0 ||
       in->max_receiver_magnitude_ohm>WPTCE_R(1e6) ||
       !WPTCE_finite(in->max_normalized_rms) || in->max_normalized_rms<=0 ||
       !WPTCE_finite(in->max_relative_sensitivity) || in->max_relative_sensitivity<=0 ||
       !WPTCE_finite(in->ambiguity_delta_cost) || in->ambiguity_delta_cost<0) return 0U;
    for(q=0U;q<4U;++q)
        if(!WPTCE_finite(in->zin_real_ohm[q]) || !WPTCE_finite(in->zin_imag_ohm[q]) ||
           WPTCE_abs(in->zin_real_ohm[q])>WPTCE_R(1e6) ||
           WPTCE_abs(in->zin_imag_ohm[q])>WPTCE_R(1e6) ||
           !WPTCE_finite(in->sigma_ohm[q]) || in->sigma_ohm[q]<WPTCE_R(1e-9) ||
           in->sigma_ohm[q]>WPTCE_R(1e6)) return 0U;
    return 1U;
}

/* Public entry point. Work arrays are static: no large C28x stack frame.
 * This function is deliberately NOT reentrant and must not be called in ISR.
 * It never changes PWM, GPIO, ADC, protection hardware, or any caller input. */
static void WPTCE_estimate(const WPTCE_Input *in, WPTCE_Result *out)
{
    static WPTCE_Candidate cand[WPTCE_MAX_SEEDS];
    static WPTCE_Real j[8][6],a[6][6],acopy[6][6];
    WPTCE_Candidate trial;
    WPTCE_Real roots[2],r[8],g[6],s[6],b[6],x[6],cost,raw,l,v;
    uint16_t h,i,k,n=0U,nr,ng=0U,idx,worst,best=0U,c;
    if(!out) return;
    for(k=0U;k<6U;++k) out->p[k]=0;
    out->rms_ohm=0; out->normalized_rms=0;
    out->relative_l_sensitivity=0; out->relative_m_sensitivity=0;
    out->alternative_l_uh=0; out->alternative_m_uh=0;
    out->solutions_checked=0U; out->flags=WPTCE_BAD_INPUT;
    if(!WPTCE_input_ok(in)) return;
    /* Analytic seeds are kept even when a grid seed has a lower initial cost. */
    for(h=0U;h<2U;++h) {
        nr=WPTCE_roots(in,h,roots);
        for(i=0U;i<nr;++i)
            if(WPTCE_seed(in,roots[i],&trial)) cand[n++]=trial;
    }
    /* Keep eight best grid seeds, including the endpoints. */
    for(i=0U;i<=WPTCE_GRID_STEPS;++i) {
        l=in->min_l_uh+(in->max_l_uh-in->min_l_uh)*i/WPTCE_GRID_STEPS;
        if(!WPTCE_seed(in,l,&trial)) continue;
        if(ng<WPTCE_GRID_SEEDS) cand[n+ng++]=trial;
        else {
            worst=n;
            for(k=n+1U;k<n+ng;++k) if(cand[k].cost>cand[worst].cost) worst=k;
            if(trial.cost<cand[worst].cost) cand[worst]=trial;
        }
    }
    n+=ng;
    if(n==0U) { out->flags=WPTCE_NO_SOLUTION; return; }
    for(i=0U;i<n;++i) {
        WPTCE_refine(in,&cand[i]);
        if(cand[i].cost<cand[best].cost) best=i;
    }
    out->solutions_checked=n;
    for(k=0U;k<6U;++k) out->p[k]=cand[best].p[k];
    if(!WPTCE_evaluate(in,out->p,r,j,&cost,&raw)) {
        out->flags=WPTCE_NO_SOLUTION; return;
    }
    out->flags=0U;
    out->rms_ohm=WPTCE_sqrt(raw/8); out->normalized_rms=WPTCE_sqrt(cost/8);
    if(out->normalized_rms>in->max_normalized_rms) out->flags|=WPTCE_HIGH_RESIDUAL;
    if(out->p[0]-in->min_l_uh<WPTCE_R(1e-4)*out->p[0] ||
       in->max_l_uh-out->p[0]<WPTCE_R(1e-4)*out->p[0] ||
       out->p[1]>WPTCE_R(0.9999)*out->p[0]) out->flags|=WPTCE_AT_BOUND;
    /* Do not publish a unique estimate when a materially different candidate
     * is almost as compatible with the supplied measurement uncertainty. */
    for(i=0U;i<n;++i) {
        if(cand[i].cost>cost+in->ambiguity_delta_cost) continue;
        if(WPTCE_abs(cand[i].p[0]-out->p[0])>WPTCE_R(0.01)*out->p[0] ||
           WPTCE_abs(cand[i].p[1]-out->p[1])>WPTCE_R(0.02)*out->p[1]) {
            out->flags|=WPTCE_AMBIGUOUS;
            out->alternative_l_uh=cand[i].p[0]; out->alternative_m_uh=cand[i].p[1];
            break;
        }
    }
    /* Local sensitivity: sqrt(diag((J'J)^-1))/parameter. A low residual alone
     * is insufficient when all the measurements weakly constrain L or M. */
    WPTCE_normal(j,r,a,g,s);
    for(idx=0U;idx<2U;++idx) {
        for(k=0U;k<6U;++k) {
            b[k]=(k==idx)?1:0;
            for(c=0U;c<6U;++c) acopy[k][c]=a[k][c];
        }
        if(!WPTCE_linear(acopy,b,x) || !(x[idx]>0)) {
            v=WPTCE_R(1e6); out->flags|=WPTCE_WEAK_IDENTIFICATION;
        } else v=WPTCE_sqrt(x[idx])/s[idx]/out->p[idx];
        if(idx==0U) out->relative_l_sensitivity=v; else out->relative_m_sensitivity=v;
        if(v>in->max_relative_sensitivity) out->flags|=WPTCE_WEAK_IDENTIFICATION;
    }
}
#endif
