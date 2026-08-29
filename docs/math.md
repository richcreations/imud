# imud — Mathematical Reference (as implemented)

**Document purpose.** This is an audit reference for the *actual numerical
methods executed by the imud source code*, at the release this file ships
with. It is deliberately **not** a derivation of the methods imud "should"
use, nor a restatement of the papers it draws on: every equation below was
transcribed from the code and is annotated with the function, variable, and
struct-field names so a reviewer can place each symbol against its
implementation. Where the code approximates, deviates from, or specializes
its cited source, an **As-implemented** note says so explicitly.

**Typeset copy.** `docs/math.pdf` in the source repository renders these
equations properly for reading away from a terminal. It is committed but not
packaged — it would only duplicate this file — and it is not linked from here,
because a relative link to it would be dead in the installed doc tree. It is
kept honest by `make check-math-pdf-stamp`, which compares it against this
file by hash; after editing here, run `make math-pdf` and commit both. That
needs `pandoc` and either `tectonic` (a single binary) or a XeLaTeX.

**Intended reader.** Someone auditing the mathematics itself — an estimation
or geophysics specialist — who wants to check the equations and their data
flow without reading C. Familiarity with quaternion attitude estimation,
Kalman filtering, and spherical-harmonic geomagnetism is assumed.

**Scope.** All numeric routines in the daemon and its calibration tool:
the MEKF attitude/heading filter, accelerometer/magnetometer measurement
models, the heave and sea-state estimators, compass-health and
engine-vibration metrics, timestamp anchoring, the offline magnetometer /
Allan-variance / gyro-temperature calibration fits, and the World Magnetic
Model evaluation. The WMM section states the *implementation shape* and cites
the defining report rather than re-deriving spherical-harmonic theory.

**Source-of-truth files.**
`src/fusion.c`, `include/fusion.h` (MEKF, heave, sea state);
`src/imu.c` (live calibration application, Euler rates, rate of turn,
timestamp anchor, engine detector, startup bias);
`src/cal_math.c`, `include/cal_math.h` (offline fits, Allan variance);
`src/wmm.c`, `include/wmm.h` (geomagnetic model);
`src/cal.c` (calibration file I/O).

**Citation convention.** Each section ends with a **Source** line. Because
imud implements standard methods rather than novel mathematics, most sources
are the canonical reference for the technique. A citation marked *(code
comment)* is the reference named in the source itself; one marked
*(canonical)* is the standard literature reference for a technique the code
does not cite in-line. See [§15 References](#15-references). Citations the
maintainer may wish to verify against the exact edition are flagged in that
section.

---

## 1. Conventions and reference frames

**Frames.**
- **Body frame** `b`: the sensor board axes after the optional mount
  rotation (§3). NED-compatible: X forward, Y starboard, Z down.
- **Navigation frame** `NED`: local North-East-Down.

**Attitude quaternion.** `mekf_t.q` $= q = [q_w, q_x, q_y, q_z]$ is a unit
Hamilton quaternion, **scalar-first**, representing the **body→NED**
rotation:

$$ v_{NED} = R(q)\, v_{b}, \qquad R(q)\in SO(3). $$

The rotation matrix `q_to_R()` (`fusion.c:280`) is

$$
R(q)=\begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y)\\
2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x)\\
2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}.
$$

The Hamilton product `q_mul()` ($c = a\otimes b$, `fusion.c:243`) uses the
standard scalar-first convention (Solà eq. 16).

**Units.** Gyroscope rad·s⁻¹; accelerometer m·s⁻²; magnetometer µT on the
wire, converted to **Gauss** (×0.01) inside the filter; angles rad; time s.
Standard gravity `G_MS2` $=g=9.80665\,\mathrm{m\,s^{-2}}$ (`fusion.c:43`).

**Gravity reference.** $g_{ref}=[0,0,1]$ (unit, NED, Z-down). The
accelerometer's *specific force* at rest reads $-g$ on Z; the filter works
with the **gravity direction** $z_a=-\widehat{a}_b$ (§4.7).

**As-implemented.** MEKF state — quaternion, bias, wave acceleration, and the
covariance $P$ — is single precision (`float`); calibration fits and WMM are
double precision. The quaternion is renormalized (`q_normalize`,
`fusion.c:266`) after every multiplicative update, guarding unit-norm drift
from float round-off, and $P$ uses the Joseph form for the same reason
(§4.5): the simple $(I-KH)P$ form slowly loses symmetry and
positive-definiteness at 833 Hz over multi-day runs.

**Where that split stops holding, and why the accumulators are double.** The
rule is not "filter state is float" but *nothing that accumulates over an
unbounded number of steps is float*. Every leaky integrator and exponentially
weighted statistic here advances by $\alpha\,\delta$ with
$\alpha = \Delta t/\tau$, and that gain shrinks with the sample rate **and**
with the time constant. Once $\alpha$ falls below half a float32 ULP of the
state it is being added to, the update rounds to an exact no-op — not a
gradual loss, a full stop — and $\varepsilon_{32} = 1.19\times10^{-7}$ is
reachable from `imud.conf`: at 32 kHz a 1200 s sea-state window gives
$\alpha = 2.6\times10^{-8}$.

Measured against a double-precision reference of the same arithmetic, both fed
identical float-rounded inputs so only accumulator width differs:

```
rm -f test_fusion && make test_fusion CFLAGS="-D_GNU_SOURCE -O2 -Wall \
    -Wextra -std=c11 -pthread -Iinclude -DBENCH_SWEEP_PRECISION" && ./test_fusion
```

| $\Delta t/\tau$ | configuration | Hs error | $T_z$ error |
|---|---|---|---|
| $1.0\times10^{-5}$ | 833 Hz / 120 s **(default)** | 0.000 % | 0.000 % |
| $2.1\times10^{-7}$ | 8 kHz / 600 s | 0.128 % | 0.491 % |
| $5.2\times10^{-8}$ | 32 kHz / 600 s | 1.823 % | 0.982 % |
| $2.6\times10^{-8}$ | 32 kHz / 1200 s | **17.995 %** | **6.865 %** |

Heave, on its own $\tau$ axis, failed harder. Its leak is what bounds the
drift of a double integration of accelerometer noise; when the leak becomes a
no-op the estimator is a *plain* double integrator:

| $\Delta t/\tau$ | configuration | recovered amplitude error |
|---|---|---|
| $2.6\times10^{-6}$ | 32 kHz / 12 s **(default)** | 0.225 % |
| $5.2\times10^{-7}$ | 32 kHz / 60 s | 1.225 % |
| $1.0\times10^{-7}$ | 32 kHz / 300 s | 7.263 % |
| $3.5\times10^{-8}$ | 32 kHz / 900 s | **6 543 975 %** — 253 m of "heave" |

Both estimators' accumulators are therefore `double`
(`seastate_t`'s six mean/variance pairs, `heave_t`'s `vel`/`disp`/`hp_y`),
along with their gains and heave's gravity subtraction — a cancellation of
$9.8$ against $9.8$ down to a signal three orders smaller. That moves the
cliff to $\Delta t/\tau \sim 10^{-16}$, which no rate and window reach. Every
cell above reads 0.000 % after. Inputs, outputs and the wire stay float:
sensor resolution is coarser than float32 either way, so widening those would
flatter the measurement without changing the hardware.

Independent confirmation from the default-on sweeps: `test_seastate_across_rates`
was flat at 1.015 % Hs from 100 Hz to 16 kHz and ticked up to 1.046 % at
32 kHz. §4.7.2 recorded that endpoint as float32 accumulation. It is now
1.016 % — the plateau simply continues, and the uptick is gone.

One elapsed-time gain is worth naming separately. `mekf_ema_alpha`
($\alpha = 1 - e^{-\Delta t/\tau}$, exact for any feed rate) must be computed
with `expm1f`: spelled `1.0f - expf(-x)` it subtracts two numbers differing by
less than an ULP of 1.0, quantising the answer to multiples of
$5.96\times10^{-8}$ — 2.7 % error at 32 kHz against the 30 s health constant,
with only 17 representable values.

**Source:** Solà 2017 *(code comment)*; Shuster 1993 for quaternion
conventions *(canonical)*; Higham 2002 for the $1-e^{-x}$ cancellation
*(canonical)*. Measurements from `-DBENCH_SWEEP_PRECISION`.

---

## 2. Signal flow

One IMU sample traverses, in order:

1. **Driver read** → raw `imu_sample_t{accel[3], gyro[3], temp_c, chip_ts}`
   in board axes.
2. **Mount rotation** `apply_mount_rot_if_set()` (§3.1) → body axes.
3. **Calibration** `apply_imu_cal()` (§3.2): gyro temperature compensation,
   accel offset/scale. (Magnetometer: `apply_mag_cal()`, hard/soft iron.)
4. **MEKF** (§4): `mekf_predict` on every IMU sample; `mekf_update_accel`
   every sample; `mekf_update_mag` per magnetometer sample.
5. **Derived outputs**: Euler angles and heading (§4.10); Euler rates and
   rate of turn (§5); heave (§6); sea-state statistics (§7); compass-health
   (§8) and engine (§9) metrics.

Gyro **bias** is *not* removed in step 3; the MEKF estimates and subtracts it
(§4.4). Timestamps are reconstructed from the chip counter (§11).

---

## 3. Calibration application (live path)

### 3.1 Mount rotation

`apply_mount_rot_if_set()` (`imu_math.c:393`) applies a fixed 3×3 board→body
matrix $R_{mount}=$ `cfg->mount_rot` in place, when `cfg->mount_set`:

$$ v \leftarrow R_{mount}\, v, \qquad v\in\{a_b,\ \omega_b,\ m_b\}. $$

Applied identically to accel, gyro, and magnetometer vectors. Accumulated in
double, stored back as float.

**As-implemented.** $R_{mount}$ is supplied by configuration (a fixed
installation rotation, e.g. yaw 180° for a stern-facing board); imud does not
estimate it. It may be given as Euler angles (`rotation_euler_deg`), a named
`preset`, or directly as a 3×3 `rotation_matrix`. The first two are orthonormal
by construction; a directly-supplied matrix is validated at config load against
both $R^\top R\approx I$ and $\det R>0$ (rejecting reflections, which an
orthogonality-only test would accept) and the daemon refuses to start if it
fails.

### 3.2 Inertial calibration

`apply_imu_cal()` (`imu_math.c:26`), per axis $i$:

- **Gyro temperature compensation** (when `cal->has_gyro_temp`):
  $$ \omega_i \leftarrow \omega_i - c_i\,(T - T_{ref}), $$
  with $c_i=$ `gyro_temp_coeff[i]`, $T=$ `temp_c`,
  $T_{ref}=$ `gyro_temp_ref_c` (fit in §12.6).
- **Accel offset/scale** (when `cal->has_accel`):
  $$ a_i \leftarrow (a_i - o_i)\, s_i, $$
  with $o_i=$ `accel_offset[i]`, $s_i=$ `accel_scale[i]`.

### 3.3 Magnetometer hard/soft-iron

`apply_mag_cal()` (`imu_math.c:44`), when `cal->has_mag`:

$$ m_{cal} = S_{soft}\,(m_{raw} - b_{hard}), $$

with $b_{hard}=$ `mag_hard_iron[3]`, $S_{soft}=$ `mag_soft_iron[3][3]`. The
correction parameters are produced offline (§12.2–12.3).

**As-implemented.** The accel model is diagonal scale + offset (no
off-diagonal cross-axis / misalignment terms). The magnetometer model is a
full 3×3 soft-iron matrix times a hard-iron offset, but that matrix is fitted
from a **2-D** swing (§12.3), so its out-of-plane (dip) accuracy is limited —
which is exactly why the filter defaults to heading-only magnetometer fusion
(§4.8).

**Source:** standard sensor error model; Titterton & Weston 2004 §12
*(canonical)*.

---

## 4. Multiplicative Extended Kalman Filter

The attitude/heading core is an **error-state (multiplicative) EKF**. The
nominal state carries the full quaternion and gyro bias; the Kalman filter
operates on a minimal 6-D **error state**, and corrections are injected
multiplicatively so the quaternion never leaves the unit sphere.

### 4.1 State and covariance

Nominal state (`mekf_t`, `fusion.h:46`):
- $q$ — unit quaternion, body→NED (`q[4]`).
- $b$ — gyro bias, rad·s⁻¹ (`bias[3]`).
- $a_w$ — wave acceleration, body frame, normalized gravity units
  (`wave_acc[3]`); see §4.1.1.

Error state (never stored explicitly; the axis of `P`):

$$ \delta x = [\,\delta\theta\ (3)\ \mid\ \delta b\ (3)\ \mid\ \delta a_w\ (3)\,]
   \in \mathbb{R}^9, $$

$\delta\theta$ = small-angle rotation error (rad), $\delta b$ = bias error,
$\delta a_w$ = wave-acceleration error.
Covariance $P\in\mathbb{R}^{9\times9}$ (`P[MEKF_N][MEKF_N]`, `MEKF_N = 9`),
symmetric PD; top-left 3×3 is attitude-error covariance (rad²), the middle
3×3 is bias-error covariance ((rad·s⁻¹)²), the trailing 3×3 is
wave-acceleration-error covariance (g²).

The wave block is **appended**, not inserted, so the attitude and bias blocks
keep their indices: `mekf_get_state()` reads `P[0:3][0:3]` and `P[3][3]`,
`P[4][4]`, `P[5][5]` exactly as before and the wire format is unchanged.

#### 4.1.1 The wave-acceleration state

The gravity measurement is contaminated in a seaway by wave-orbital
acceleration, which is **correlated over ~0.5–1.5 s, not white**. A white $R_a$
therefore tells the filter that 833 samples per second are 833 independent
measurements of gravity when they are closer to one per wave. The consequence
is not subtle: the filter's covariance collapses, its gain vanishes, and it
diverges into its own over-confidence (measured: attitude RMS 9.5°/11.4°,
NIS ≈ 56 — §4.7).

$a_w$ is modelled as a **first-order Gauss–Markov (Ornstein–Uhlenbeck)
process** with steady-state standard deviation $\sigma$ and correlation time
$\tau$:

$$ \dot a_w = -\frac{1}{\tau}a_w + w,\qquad
   \mathbb{E}[a_w a_w^{\mathsf T}] = \sigma^2 I_3 . $$

$\sigma$ is configured in m·s⁻² (`mekf_wave_accel`, the units a spec sheet and
`imud-cal fit-ra` both speak) and stored as the variance $\sigma_g^2 =
(\sigma/g)^2$ in `wave_sig2`; $\tau$ is `mekf_wave_accel_tau_s`. Either at 0
disables the state, and the disabled filter is bit-for-bit the pre-1.7 6-state
filter — $P$'s wave rows and columns stay identically zero, $\Phi$'s block is
$I$, and the measurement Jacobian's block is never executed.

**Frame.** $a_w$ is carried in the **body** frame. Wave-orbital acceleration is
arguably a property of the seaway and so NED-stationary, which would add a
transport term $\Phi_w = e^{-dt/\tau}(I-[\omega\,dt]_\times)$. That was
implemented and measured (`-DWAVE_TRANSPORT` in `fusion.c`): it changes the
benchmark in the third decimal and is slightly *worse* on yaw-only attitude
RMS (2.308° → 2.325°), because at ±15° of roll the frame rotation is small
compared with $\tau$. The body-frame form is kept for the simpler Jacobian.

**Observability.** $\delta\theta$ and $\delta a_w$ both act in the tangent
plane of the measured direction, so they are separated *only* by their
dynamics — attitude error is a random walk driven by gyro noise, wave
acceleration decays with $\tau$. Too large a $\sigma$ or $\tau$ lets the wave
state absorb genuine tilt error; §4.7 records the sweep that bounds this.

The nominal/error relationship is the **right** (local) perturbation
$q_{true} = \hat q \otimes \delta q(\delta\theta)$, established by the sign of
the measurement Jacobian in §4.5.

**Source:** Solà 2017 §5.4 *(code comment)*; Markley & Crassidis 2014 §6.1;
Trawny & Roumeliotis 2005 *(canonical)*.

### 4.2 Initialization — `mekf_init()` (`fusion.c:1019`)

Nominal: $q=[1,0,0,0]$, $b=$ `gyro_bias_init` (from the startup still window,
§10; may be 0).

Initial covariance (diagonal):
$$ P_{0}[0{:}3] = (0.175)^2\ \mathrm{rad^2}\ (\approx10°),\qquad
   P_{0}[3{:}6] = (0.001)^2\ (\mathrm{rad\,s^{-1}})^2,\qquad
   P_{0}[6{:}9] = \sigma_g^2 . $$

The wave block starts at its **steady state**, not at a wide acquisition
value: the Gauss–Markov process is stationary, so there is no transient to
model. (It is seeded to 0 when the state is disabled, which is what keeps the
block inert.)

Discrete process-noise variances, step $dt=1/\text{ODR}$:
$$ Q_g = N_g^2\,dt,\qquad Q_b = N_b^2\,dt, $$
with $N_g=$ `mekf_gyro_noise`, $N_b=$ `mekf_gyro_bias` (datasheet noise
densities). Stored as `f->Qg`, `f->Qb`.

Measurement-noise variances:
$$ R_a = \left(\frac{N_a}{g}\right)^2 \text{ODR},\qquad
   R_m = N_m^2\,f_{s,\text{mag}}, $$
`f->Ra` (`mekf_derive_tuning`, `fusion.c:897`) in normalized
(gravity-direction) units, `f->Rm` (`:898`) in Gauss², with
$N_a=$ `mekf_accel_noise`, $N_m=$ `mekf_mag_noise`,
$f_{s,\text{mag}}=$ `mag_odr_hz`.

Derived thresholds: accel skip band
$[\,1-s_k,\ 1+s_k\,]$ with $s_k=$ `accel_skip_thresh`;
$mag\_reject\_sq = (\text{mag\_reject\_gauss})^2$; convergence threshold
$conv\_thresh = 3\,\theta_c^2$ with
$\theta_c = \max(0.5°,\ 0.30\,\arcsin\sigma_g)$ — 1.40° at the default
$\sigma$. It scales with $\sigma_g$ rather than being flat because with the
wave state $P$ reports a believable steady-state attitude variance, and that
has a floor near 0.9°/axis: a fixed 0.5° threshold would never be reached
(§4.1.1). m_ref EMA gain
$mref\_alpha = 1/(\tau_{mref}\, f_{s,\text{mag}})$ with $\tau_{mref}=300$ s
(`:941`).

**As-implemented.** $Q_g,Q_b$ are *per nominal step*; during prediction they
are rescaled by the actual $dt$ (§4.4). The bias process is modeled as a
random walk whose density $N_b$ is a **tuning constant deliberately held
above** the measured in-run bias instability (see `docs/capture.md`); it is
not driven by the Allan-variance characterization of §12.5.

**Source:** Solà 2017 §4.1 for discrete process noise *(code comment)*.

### 4.3 Alignment — `mekf_align()` (`fusion.c:1077`)

Deterministic initial attitude from one static accel+mag pair (a
tilt-then-heading decomposition, TRIAD-family).

**The caller supplies a window mean, not one sample** (`imu.c`, the align
loop): the accelerometer and magnetometer are averaged over
`align_window_sec` before this is called. That window matters far more than
its obscurity suggests, because whatever tilt error survives it is baked
permanently into $m_{ref}$'s dip (§4.8.1) and, in 3-D mode, becomes a constant
roll/pitch bias. One second — the hardcoded value before 1.7 — is about a
fifth of a typical roll period, so it averages an arbitrary fraction of the
cycle. Measured over the 12-seed wave benchmark, attitude RMS:

| window | yaw-only (default) | 3-D | 3-D NEES(strict) |
|---|---|---|---|
| 1 s | 47.7° | 6.72° | 339 |
| 2 s | 2.28° | 5.78° | 250 |
| 3 s | 2.11° | 2.38° | 38.9 |
| **5 s (default)** | **2.19°** | **1.18°** | **5.74** |
| 15 s | 2.19° | 0.90° | 1.71 |
| 30 s | 2.24° | 0.93° | 1.94 |

The marine default is flat from ~2 s; 3-D keeps improving to ~15 s. The default
is 5 s, and the cost of a longer window is purely startup latency before the
filter produces usable attitude — worth paying at a mooring, not underway.

**Tilt from accelerometer.** With $\widehat g_b=-a_b/\lVert a_b\rVert$ the
gravity direction in body (guard: reject if $\lVert a_b\rVert<0.5g$):

$$ \phi = \operatorname{atan2}(\widehat g_{b,y},\ \widehat g_{b,z}),\qquad
   \theta = \operatorname{atan2}\!\big(-\widehat g_{b,x},\
   \sqrt{\widehat g_{b,y}^2+\widehat g_{b,z}^2}\big). $$

Tilt quaternion $q_{tilt}=q_\theta\otimes q_\phi$ built directly from
half-angles (`mekf_align`, `fusion.c:1094`).

**Heading from magnetometer.** Rotate $m_b$ by $R(q_{tilt})$ into the
tilt-levelled frame; with horizontal components $m_x,m_y$ there,

$$ \psi = \operatorname{atan2}(-m_y,\ m_x). $$

Full attitude $q = q_\psi\otimes q_{tilt}$, $q_\psi=[\cos\tfrac\psi2,0,0,
\sin\tfrac\psi2]$.

**Magnetic reference.** On first alignment, `m_ref` (NED, Gauss) is set from
the measurement rotated to NED and scaled µT→Gauss:
$$ m_{ref} = 0.01\, R(q)\, m_b. $$

**As-implemented / audit note.** The heading sign is
$\operatorname{atan2}(-m_y,m_x)$, **not** $\operatorname{atan2}(+m_y,m_x)$;
the levelled field is the true NED field rotated by $-\psi$, so the negative
sign recovers vessel yaw. (The `+m_y` form returns $-\psi$, mirrored about
north — a historical bug invisible when aligning while pointing north; see
the in-code comment at `fusion.c:1104`.) The initial `m_ref` inherits any
alignment tilt error in its magnitude/dip, which the quiescence-gated EMA
(§4.8) and the WMM invariants (§4.9) later remove; its horizontal
**direction** is the heading datum and is never subsequently adapted.

**Source:** tilt/heading coarse alignment — Farrell 2008 §10; TRIAD lineage
Shuster & Oh 1981 *(canonical)*.

### 4.4 Prediction — `mekf_predict()` (`fusion.c:1134`)

Per IMU sample, with measured interval $dt$ (from hardware timestamps, §11;
falls back to nominal $1/\text{ODR}$).

**Bias-corrected rate:** $\omega = s.\text{gyro} - b$.

**Quaternion propagation** by the exponential map (`q_from_rotvec`,
`fusion.c:293`):
$$ q \leftarrow q \otimes \exp\!\big(\tfrac12\,[\,0,\ \omega\,dt\,]\big),
\qquad
\exp(\phi) = \Big[\cos\tfrac{|\vartheta|}{2},\
\tfrac{\sin(|\vartheta|/2)}{|\vartheta|}\,\vartheta\Big], $$
$\vartheta=\omega\,dt$, with the small-angle branch
$\delta q\approx[1,\tfrac12\vartheta]$ for $|\vartheta|<10^{-7}$. Renormalized
after.

**Covariance propagation** $P \leftarrow \Phi P \Phi^\top + Q_d$, first-order
discrete transition (`mekf_predict`, `fusion.c:1162`):

$$
\Phi = \begin{bmatrix} I_3 - [\omega]_\times\,dt & -I_3\,dt & 0_3 \\
0_3 & I_3 & 0_3 \\ 0_3 & 0_3 & \varphi\,I_3\end{bmatrix},\qquad
Q_d = \operatorname{diag}\!\big(Q_g\tfrac{dt}{dt_0} I_3,\
Q_b\tfrac{dt}{dt_0} I_3,\ Q_w I_3\big),
$$

$[\omega]_\times$ the skew-symmetric cross-product matrix; the gyro/bias noise
is rescaled by $dt/dt_0$ ($dt_0=$ nominal step) so variance grows linearly with
the real interval. $P$ is symmetrized after. Convergence flag
$\operatorname{tr}(P[0{:}3]) < conv\_thresh$.

**The wave block is discretized exactly**, not to first order:

$$ \varphi = e^{-dt/\tau},\qquad Q_w = \sigma_g^2\,(1 - \varphi^2),\qquad
   \hat a_w \leftarrow \varphi\,\hat a_w . $$

This is not fastidiousness. $\tau$ is of order 0.5 s while the $|a|$ skip band
routinely leaves gaps of the same order, so $dt/\tau$ is **not** small on
exactly the steps that matter, and a first-order expansion would both
over-decay the state and mis-size its noise while bridging a gap. The exact
form is stationary for any $dt$: feed it a step of $10\tau$ and it returns the
state to zero with variance $\sigma_g^2$, which is the correct answer — so a
scheduling hiccup cannot destabilize the filter. It is also why $Q_w$ is *not*
put through the $dt/dt_0$ rescaling: it is already exact for this $dt$.

With the state disabled $\varphi=1$ and $Q_w=0$, leaving an identity block
that touches nothing.

**As-implemented.** Prediction uses the plain $\Phi P\Phi^\top+Q$ form (not
Joseph — the Joseph stabilization is applied on the *updates*, §4.5).
$\Phi$ is a first-order (zero-hold) approximation of $\exp(F_c\,dt)$; valid
because $\lVert\omega\rVert dt \ll 1$ at supported ODRs. The
$[\omega]_\times$ sign in $\Phi_{[0:3,0:3]}$ is that of the right-perturbation
error convention.

**Source:** Solà 2017 eq. 259 (integration), eq. 268 (covariance)
*(code comment)*.

### 4.5 Generic vector measurement update — `eskf_update()` (`fusion.c:495`)

Measurement of a known NED reference observed in body: predicted $h$, actual
$z$, isotropic noise $R_{noise}$, gate `chi2_gate`.

**Jacobian** (3×9), attitude block only (`dir_jacobian`):
$$ H = \big[\, [h]_\times \ \big|\ 0_3 \ \big|\ 0_3 \,\big],\qquad
[h]_\times=\begin{bmatrix}0&-h_2&h_1\\ h_2&0&-h_0\\ -h_1&h_0&0\end{bmatrix}. $$

The **gravity** update uses a different Jacobian when the wave state is
enabled — see §4.5.1. The mag channels use this one, unchanged.

**Innovation covariance and gain:**
$$ S = H P H^\top + R_{noise} I_3,\qquad K = P H^\top S^{-1}, $$
$S^{-1}$ by Cramer's rule (`m33_inv`, `fusion.c:95`; singular ⇒ skip).

**The singularity test is relative, not absolute** (1.7). $S$ carries physical
units, and for the gravity update they are tiny: with $R_a\approx4.2\times
10^{-5}$ and the attitude block converged to ~0.4°, $\det S \approx R_a
\lambda^2 \approx 6\times10^{-13}$ at a condition number of about 3. The
previous absolute test $|\det S| < 10^{-12}$ therefore declared a
well-conditioned matrix singular and made `eskf_update` return −1, **silently
dropping 87% of accel updates in the wave benchmark** and pinning attitude
uncertainty at whatever value made $\det S\approx10^{-12}$ rather than letting
it converge. `m33_inv` now compares $|\det|$ against $10^{-6}\,\lVert
A\rVert_{\max}^3$, which is unit-free, tests rank deficiency (what "singular"
means), and still sits well above float's $\approx1.2\times10^{-7}$ epsilon.
§4.7 records what this exposed.

**Innovation** $\nu = z - \hat z$ ($\hat z = h$ here), with **robust
(Huber-style) gating** on the
Mahalanobis distance $d^2 = \nu^\top S^{-1}\nu$:

$$
\begin{cases}
\text{reject the update} & d^2 > 9\,\gamma \\[2pt]
\nu \leftarrow \nu\,\sqrt{\gamma/d^2} & \gamma < d^2 \le 9\gamma \\[2pt]
\nu \text{ unchanged} & d^2 \le \gamma
\end{cases}
$$

$\gamma=$ `chi2_gate`. The middle branch caps the innovation's influence at
the gate distance rather than discarding it.

**Correction and injection:**
$$ \delta x = K\,\nu,\qquad q \leftarrow q\otimes\exp(\delta\theta),\qquad
   b \leftarrow b + \delta b,\qquad a_w \leftarrow a_w + \delta a_w, $$
$\delta\theta=\delta x[0{:}3]$, $\delta b=\delta x[3{:}6]$,
$\delta a_w=\delta x[6{:}9]$; quaternion renormalized. The $a_w$ injection is
unconditional: with the state disabled $P$'s wave rows are zero, so $K$'s are
too and it adds exactly $0.0f$. The *mag* channels inject it as well — their
$H$ has no wave block, but the cross-covariance the accel path builds up means
$K$ does, and that is correct Kalman book-keeping.

**Covariance — Joseph form, with error-state reset folded in:**
$$ P \leftarrow \big[G(I-KH)\big]\,P\,\big[G(I-KH)\big]^\top + R_{noise}\,(GK)(GK)^\top, $$
then symmetrized. (Isotropic $R$ ⇒ $KRK^\top=R_{noise}KK^\top$.)

**Error-state reset.** Injecting $\delta\theta$ into the quaternion moves the
linearisation point, so $P$ is rotated by the reset Jacobian
$G=I-\tfrac12[\delta\theta]_\times$ (Solà eq. 285). Only the attitude block
resets — the gyro-bias and wave-acceleration errors are additive and carry
over — so the applied transform is $G_{full}=\mathrm{diag}(G,I_3,I_3)$,
implemented by premultiplying
the first three rows of $K$ and $(I-KH)$. Cost is ~81 multiply-adds against
the Joseph block's ~450. Measured over the 12-seed wave benchmark this
improves yaw-only attitude RMS 4.10° → 3.57° and covariance consistency
(NEES 32.4 → 28.4), with 3-D attitude neutral (+2%).

**Huber cap.** The cap replaces a hard $\chi^2$ reject so that wave-orbital
acceleration (which swings gravity *direction* while keeping $|a|\approx g$)
deweights rather than starves the filter; only gross outliers
($d^2>25\gamma$, `GROSS_REJECT_MULT`) are rejected. Because $S$ contains $P$,
the cap is naturally inactive during acquisition (large $P$) and engages only
once confident — no explicit convergence gate needed.

**Gross-reject threshold (1.7: $9\gamma\to25\gamma$).** The original $9\gamma$
was too tight to be a fault gate. Over the 12-seed benchmark it discarded 26%
of 3-D accel updates — routine wave motion, not outliers — which starved the
filter, and the resulting drift produced larger innovations that tripped the
gate more often still. Widening it (3-D / yaw-only pairs):

| reject | 3-D att | yaw att | NEES(tr) 3-D/yaw | NIS 3-D/yaw | reject rate |
|---|---|---|---|---|---|
| $9\gamma$ | 6.85° | 3.57° | 28.9 / 21.1 | 30.7 / 27.3 | .262 / .043 |
| $16\gamma$ | 5.45° | 2.32° | 16.7 / 7.9 | 22.3 / 25.2 | .012 / .000 |
| $25\gamma$ | 5.65° | 2.31° | 18.3 / 7.8 | 19.3 / 25.2 | .007 / .000 |
| $64\gamma$ | 5.62° | 2.31° | 18.3 / 7.8 | 16.7 / 25.2 | .000 / .000 |

Attitude RMS improves 17% (3-D) and 35% (yaw-only), NEES roughly halves, and
the worst-draw spread collapses (3-D 13.7°→7.0°, yaw 9.6°→2.3°) — the spread
§10.8 had attributed to scenario luck. The benefit saturates by $25\gamma$,
which keeps a genuine fault gate at 5× the cap radius instead of 3×.

**Known inconsistency (measured, deliberately retained).** The cap scales
$\nu$ but leaves $K$ — and hence the covariance update — at full confidence,
so $P$ contracts as though the measurement had been fully trusted even when
the correction was attenuated up to 3×. Making this consistent was tried three
ways and measured over the 12-seed wave benchmark, where NEES $\approx 1$ would
indicate a self-consistent filter:

Re-measured after the $25\gamma$ change above (the earlier figures were taken
while the reject gate was corrupting every run). Rebuild with
`-DHUBER_VARIANT=n`:

| n | variant | 3-D att | yaw att | NEES(tr) 3-D/yaw | NIS 3-D/yaw |
|---|---|---|---|---|---|
| 0 | ν-capping (shipped) | 5.65° | 2.31° | 18.3 / 7.8 | 19.3 / 25.2 |
| 1 | $K\leftarrow wK$ (exact Joseph, suboptimal gain) | 8.85° | 3.09° | 39.9 / 12.4 | 17.0 / 28.5 |
| 2 | $R\to R/w$ (IRLS inflation) | 7.79° | 5.57° | 31.5 / 40.5 | 15.1 / 43.3 |
| 3 | $R\to R/w^2$ | 8.48° | 4.74° | 35.0 / 27.9 | 18.2 / 36.3 |

Every variant is **both** less accurate and less self-consistent — by a wider
margin than before. Inflating $R$ is additionally wrong on its own terms
because $P\gg R$ here, a regime where $K=P/(P+R)$ is near unity and $R\to R/w$
barely moves it, losing the bounded per-sample influence the seaway robustness
depends on.

The reason these fail is **not** that $R_a$ is mistuned. $R_a$ *is*
over-optimistic (NIS ≈ 20, §4.7), but retuning it does not rescue the family —
they stay worse across the whole sweep. The cap is a *robustness* device, not
a statistical one: contracting $P$ by the attenuated gain faithfully records
having learned less from that sample, but the consequence is a larger $P$,
hence a larger gain on the **following** samples — which in a seaway carry the
same wave contamination. The inconsistency does useful work, holding the gain
down exactly when measurements are least trustworthy. `innov_weight` /
`innov_reject` on the wire (§8.1) expose how hard the cap is being leaned on;
`nis_accel` / `nis_mag` (§8.2) expose whether the model under it is right.

**Source:** EKF update Kalman 1960; Joseph form Bucy & Joseph 1968; robust
innovation capping Huber 1964; Mahalanobis gating Bar-Shalom et al. 2001
*(canonical)*. Jacobian sign Solà 2017 §7 *(code comment)*.

#### 4.5.1 Wave-aware gravity Jacobian — `wave_jacobian()`

When the Gauss–Markov state is enabled, the gravity update no longer predicts
$h$; it predicts the *contaminated* direction the accelerometer actually sees.
With the specific force written in normalized gravity units,

$$ v \equiv h - \hat a_w,\qquad \hat z = \frac{v}{\lVert v\rVert},\qquad
   \nu = z - \hat z . $$

Perturbing both contributions ($h_{true}=\hat h+[\hat h]_\times\delta\theta$,
$a_{true}=\hat a_w+\delta a_w$) gives $\delta v = [\hat h]_\times\delta\theta -
\delta a_w$, and differentiating the normalization introduces the **tangent
projector** $P_{\hat z} = I - \hat z\hat z^{\mathsf T}$:

$$ \delta\hat z = \frac{P_{\hat z}}{\lVert v\rVert}\,\delta v
\quad\Longrightarrow\quad
H = \Big[\ \tfrac{1}{\lVert v\rVert}P_{\hat z}[h]_\times \ \Big|\ 0_3 \ \Big|\
        -\tfrac{1}{\lVert v\rVert}P_{\hat z} \ \Big]. $$

**The projector is load-bearing, not tidiness.** It is what makes
$\hat z^{\mathsf T}H = 0$ hold *exactly*, since $P_{\hat z}\hat z = 0$. That in
turn makes $\hat z$ an eigenvector of $S$ with eigenvalue $R$, which is the
sole premise of the $\mathbb{E}[d^2]=2$ derivation in §8.2 — i.e. of the
`nis_accel` wire field meaning "1.0 = consistent". Writing the Jacobian the
way a first pass naturally would, $[\hat h]_\times/\lVert v\rVert$ and
$-I/\lVert v\rVert$, leaves $\hat z^{\mathsf T}H = O(\lVert\hat a_w\rVert)$;
the radial component then leaks into $d^2$ and NIS is biased. Measured: the
unprojected form moves the ground-truth benchmark NIS from 0.99 to 0.76 while
leaving every other test in the suite passing
(`test_wave_gm_ground_truth`).

At $\hat a_w = 0$ this reduces algebraically to `dir_jacobian`: $v=h$ is
already a unit vector and $(I-hh^{\mathsf T})[h]_\times = [h]_\times$ because
$h^{\mathsf T}[h]_\times = 0$.

$\lVert v\rVert$ is guarded at 0.5; below that the acceleration is comparable
to gravity itself and the linearisation is meaningless, so the update falls
back to the plain direction form.

### 4.6 Scalar heading update — `eskf_update_yaw()` (`fusion.c:758`)

Heading-only correction (a 1-D measurement). Innovation $y$ (rad, wrapped to
$\pm\pi$), variance $R_{noise}$. The Jacobian projects the error onto the
NED-down axis expressed in body:
$$ H = \big[\,-R[2][:]\ \mid\ 0_3\,\big]\in\mathbb{R}^{1\times6}, $$
$R[2][:]$ = third row of $R(q)$. Then
$$ S = H P H^\top + R_{noise},\qquad K = P H^\top/S, $$
with the same Huber policy on $d^2=y^2/S$ against
$\gamma_\psi=6.63$ ($\chi^2_1$ 99%), rank-1 Joseph covariance update.

**Source:** scalar-measurement EKF specialization of §4.5 *(canonical)*.

### 4.7 Accelerometer update — `mekf_update_accel()` (`fusion.c:1374`)

**Speed-aided centripetal correction.** In a turn the accelerometer senses
$a = a_{platform} - g$, and with speed-over-ground $v$ (body $\approx[v,0,0]$,
leeway neglected) the centripetal term is $\omega\times v = [0,\ \omega_z v,\
-\omega_y v]$. When `speed_mps` $>0.1$:
$$ a_y \leftarrow a_y - \omega_z\,v,\qquad a_z \leftarrow a_z + \omega_y\,v, $$
$\omega_{y,z}$ bias-corrected. `speed_mps=0` (no GPS) ⇒ no-op.

**Quiescence EMA** (updated every sample, incl. gated), $\tau\approx2$ s:
$$ q_{quiet} \leftarrow q_{quiet} + \big((a_g-1)^2 - q_{quiet}\big)\tfrac{dt}{2},
\qquad a_g = \lVert a\rVert/g. $$

**Skip gate.** If $a_g\notin[$`accel_skip_lo`, `accel_skip_hi`$]$, skip
(linear acceleration).

**Measurement.** Gravity direction $z=-a/\lVert a\rVert$; prediction
$h = R^\top g_{ref} = R[2][:]$ (third row). Effective noise
$R_{eff}=R_a\cdot$ `Ra_scale`. Then `eskf_update(h, z, R_eff, 11.34)` with
$\gamma=11.34$ ($\chi^2_3$ 99%). The gravity direction $z$ is stashed as
`g_body` (attitude-independent dip reference for §4.8).

**Protected axis.** $h$ is also passed as `eskf_update`'s `protect` argument,
which projects the attitude rows of $K$ onto the plane orthogonal to it:
$$ K_{\delta\theta} \leftarrow (I - hh^\top)\,K_{\delta\theta}. $$
Two properties follow, and they are what the update is entitled to claim:
$$ \delta\theta \cdot h = 0, \qquad h^\top P^+ h = h^\top P^- h, $$
the second because $K^\top(h,0,0) = (h^\top K_{\delta\theta})^\top = 0$
leaves both $(I-KH)^\top$ and $KRK^\top$ inert along $h$. Joseph form holds
for any gain, so nothing else in §4.5 changes.

$H_{\delta\theta}h = 0$ already holds exactly for both Jacobians — the
projection subtracts nothing at small angle. It exists for the large-angle
regime. Once $\operatorname{tr}P[0{:}3]$ reaches $O(1)$ rad², which is where an
unaided yaw axis ends up, the gain is large enough that a single innovation
moves $q$ further than the first-order reset $G=I-\tfrac12[\delta\theta]_\times$
tracks $h$; the misalignment leaks the unobservable variance into the observable
subspace, the next update annihilates it, and the filter reports a collapsed
covariance and a heading correction gravity cannot justify.

**Attitude rows only.** A gyro-bias component along $h$ is equally unobservable
from this measurement at this instant, but $h$ rotates in the body frame as the
platform moves, so that component becomes observable a moment later. A yaw error
does not: $\Phi$ rotates $\delta\theta$ with the body and $h$ rotates with the
body, so $\delta\theta \parallel h$ stays parallel to $h$ for all time.
Projecting rows 0–2 is therefore correct and projecting rows 3–5 would not be.
The magnetometer passes `protect = NULL` (§4.8): it is the measurement that does
carry heading.

**As-implemented.** `Ra_scale` is set to 4 by the fusion thread while engine
vibration is detected (§9) — vibration is high-frequency, near-zero-mean, so
deweighting (not gating) is correct. Wave *direction* disturbance is handled
by the Huber cap (§4.5), **not** by magnitude-based deweighting, which
benchmarking showed rectifies wave-phase-correlated error into an
attitude/bias offset. The centripetal model assumes zero leeway and no
vertical velocity.

**Health-EMA gain (1.7 fix).** `innov_weight`, `innov_reject` and `nis_accel`
are fed only by samples that clear the skip gate. In a seaway that gate
discards 85–95% of samples, so a per-update gain sized from the IMU ODR
(`gate_alpha` $=1/(\tau f_{odr})$) stretched the intended $\tau=30$ s to ten
minutes or more, and the metrics reported their seed values rather than the
data. The accel path now derives the gain from elapsed time instead:
$$ \alpha = 1-\exp(-\Delta t/\tau), $$
$\Delta t$ accumulated over *every* sample, accepted or skipped. This is exact
at any feed rate and reduces to $\Delta t/\tau$ when updates are frequent.

**What $R_a$ can and cannot do.** $R_a$ derives from the
datasheet noise floor, and the filter is measurably over-confident about it:
mean NIS over the wave benchmark is 19 (3-D) / 25 (yaw-only) where a
consistent model reads 1. The natural remedy — raise `mekf_accel_noise` until
NIS ≈ 1 — does not work. Sweeping it (12 seeds, both mag modes):

| $N_a$ | 3-D att | yaw att | NEES(tr) 3-D/yaw | NIS 3-D/yaw |
|---|---|---|---|---|
| 0.0022 (default) | 5.65° | 2.31° | 18.3 / 7.8 | 19.3 / 25.2 |
| 0.004 | 7.93° | 11.13° | 114 / 304 | 33.7 / 39.6 |
| 0.008 | 7.57° | 11.09° | 102 / 285 | 10.6 / 13.5 |
| 0.015 | 6.98° | 10.29° | 81 / 241 | 3.2 / 4.2 |
| **0.030** | 5.74° | **8.58°** | 50 / 156 | **0.82 / 1.03** |
| 0.050 | 5.07° | 7.01° | 34 / 87 | 0.32 / 0.34 |
| 0.120 | 4.72° | 4.33° | 17 / 17 | 0.06 / 0.05 |
| 0.300 | 4.47° | 2.48° | 6.4 / 2.2 | 0.01 / 0.01 |

Three things to read off it:

1. **NIS ≈ 1 costs accuracy.** $N_a=0.03$ makes the innovations statistically
   consistent, but the marine (yaw-only) default degrades 2.31° → 8.58° and
   its NEES rises 7.8 → 156. Weakening the gravity correction makes the
   filter lean on gyro integration, so the *state* error grows even as the
   *innovations* start to look honest.
2. **The two consistency criteria disagree.** NIS falls monotonically with
   $N_a$ while NEES(trace) has its minimum near the default. No single scalar
   satisfies both.
3. **The inconsistency is structural, not a scale error.** NEES(strict) for
   3-D stays pinned in the 44–64 band across the entire sweep — four orders
   of magnitude of $R_a$ — so $P$'s *shape* is wrong, and no isotropic scalar
   can fix that.

The cause is that the seaway residual is wave-orbital: correlated with wave
phase, with a measured correlation time of order a second (`imud-cal fit-ra`),
not white. A white isotropic $R$ cannot describe a coloured disturbance;
making its *variance* right necessarily gets its *spectrum* wrong. The real
fix is to model the correlation, which is what §4.7.1 does.

The default is therefore unchanged, and is additionally a sharp local optimum:
$N_a\in[0.003,0.03]$ is markedly **worse** than either side of it, so hand-
tuning it upward "a little" lands in the worst available region.

> **Caveat on the table above.** These rows were measured before the `m33_inv`
> singularity bug of §4.5 was found, i.e. through a filter that was discarding
> 87% of its accel updates. The *conclusions* survive — no scalar $R_a$ fixes a
> coloured disturbance, and the 44–64 NEES(strict) band really was structural —
> but the absolute numbers are not reproducible on current code and the table is
> kept as the historical record of why §10.5 was undertaken.

#### 4.7.1 The Gauss–Markov wave state — outcome

Fixing `m33_inv` removed an accidental 87% decimation of accel updates. That
decimation had been doing real work: it crudely decorrelated the
wave-contaminated samples, which is why the filter looked as good as it did.
With every sample fed to a white $R$, the filter believes it has 833
independent gravity measurements per second, $P$ collapses, and it diverges.
Modelling the disturbance as a first-order Gauss–Markov process (§4.1.1) makes
repeated correlated samples correctly stop adding information. Over the
12-seed wave benchmark:

| | 3-D att | 3-D hdg | yaw att | yaw hdg | NEES(tr) 3-D/yaw | NEES(st) 3-D/yaw | NIS 3-D/yaw | reject |
|---|---|---|---|---|---|---|---|---|
| old baseline (bug present) | 5.653° | 3.065° | 2.309° | 1.961° | 18.3 / 7.8 | 57 / 38 | 19.3 / 25.2 | .007/.000 |
| `m33_inv` fixed, no wave state | 9.488° | 7.255° | 11.424° | 8.801° | 259 / 252 | 933 / 1086 | 56.5 / 56.9 | .148/.160 |
| **+ wave state (shipped)** | **4.452°** | **0.828°** | **2.308°** | **1.016°** | **3.47 / 0.99** | 335 / **10.1** | **1.01 / 0.69** | .000/.000 |

(For 3-D attitude and NEES(strict), §4.8.1 carries the authoritative figures:
it measures them with the benchmark's alignment corrected. The wave-state
conclusion here rests on the yaw-only default and on the NIS column.)

Against the old baseline: 3-D attitude −21%, 3-D
heading −73%, yaw heading −48%, yaw attitude a dead heat, NIS from 19–25 to
≈1, NEES(trace) from 18.3/7.8 to 3.47/0.99, and both the Huber cap and the
gross-reject gate go completely idle (weight 1.000, reject 0.0000) — the
filter no longer needs robustness machinery to survive an ordinary seaway.

**Tuning, and why it is not curve-fitting.** $\sigma$ and $\tau$ were chosen on
a *broadband* scenario (`SCEN_GM` in `test_fusion.c`) whose disturbance is a
genuine Gauss–Markov process of known $\sigma$ and $\tau$, because a single
tone has no correlation time and fitting $\tau$ to one is fitting the
benchmark. The tone is the held-out validation. Three independent lines agree
on $\sigma\approx0.8$ m·s⁻²:

- it is the broadband scenario's ground truth, at which the filter reports
  NIS $=0.99$ (`test_wave_gm_ground_truth`);
- it is exactly the tone scenario's true per-axis RMS ($1.2/\sqrt2$);
- `imud-cal fit-ra` independently recovers $0.67$ m·s⁻² from a capture of that
  tone, seen only through a file, a replayed filter and a 67%-decimating skip
  band.

$\tau = 0.5$ s sits inside the range fit-ra measures. Neither knob is a free
parameter. The full grid is reproducible with

```
rm -f test_fusion && make test_fusion CFLAGS="-D_GNU_SOURCE -O2 -Wall \
    -Wextra -std=c11 -pthread -Iinclude -DBENCH_SWEEP_WAVE" && ./test_fusion
```

**The degeneracy ridge.** $\delta\theta$ and $\delta a_w$ are separated only by
their dynamics, so an over-large $\tau$ lets the wave state absorb real tilt.
The sweep shows exactly that: at $\sigma=0.9$ the yaw-only attitude RMS runs
2.17° / 2.29° / 2.40° / 2.72° for $\tau =$ 0.3 / 0.5 / 0.6 / 0.8 s, and by
$\tau=2$ s it has collapsed to 7.14°. Short $\tau$ is the safe side; the
shipped 0.5 s sits at the knee. A visible symptom of straying too far is
`bias_z` drifting, which the benchmark bounds independently.

**The column that looked worse: 3-D NEES(strict), 57 → 335.** Two things were
going on, and the first attribution of this to "the swing-circle mag
calibration is structurally 2-D" was **wrong** — the benchmark synthesises its
magnetometer data from the true attitude with nothing but white noise, so no
calibration defect can be what it measures. See §4.8.1: it is the magnetic
reference's DIP error, and ~96% of it was the benchmark aligning from a single
instantaneous sample where the daemon averages a window.

#### 4.7.2 The filter across the whole rate ladder

Every figure above, and every figure elsewhere in this document, was measured at
one point: 833 Hz IMU with a ~104 Hz magnetometer. The drivers advertise **29
IMU rates from 12 Hz to 32 kHz and 13 magnetometer rates from 1 Hz to 1 kHz**,
and each of those 377 pairings is a configuration `imud.conf` will accept. This
section is what the other 376 do.

Two instruments, because the questions are different sizes.
`test_rate_derivations` walks all 377 pairings on every build, asserting the
derived tuning is correct and the filter stays numerically sane — it deliberately
asserts nothing about accuracy, since $R_a \propto f_{odr}$ retunes the filter at
every rung and the 833 Hz bounds are meaningless elsewhere. Accuracy comes from
an opt-in sweep along two axes:

```
rm -f test_fusion && make test_fusion CFLAGS="-D_GNU_SOURCE -O2 -Wall \
    -Wextra -std=c11 -pthread -Iinclude -DBENCH_SWEEP_ODR" && ./test_fusion
```

**Accuracy is very nearly rate-independent.** 3-D attitude RMS over the IMU axis,
mag pinned at 100 Hz:

| $f_s$ (Hz) | 12 | 26 | 52 | 104 | 208 | 416 | **833** | 1660 | 3332 | 6664 | 8000 | 16000 | 32000 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 3-D att RMS | 1.210° | 1.163° | 1.156° | 1.181° | 1.157° | 1.176° | **1.178°** | 1.192° | 1.210° | 1.224° | 1.252° | 1.280° | 1.299° |
| yaw att RMS | 7.673° | 2.370° | 1.980° | 2.036° | 2.083° | 2.105° | **2.185°** | 2.203° | 2.226° | 2.241° | 2.232° | 2.241° | 2.124° |
| NIS$_a$ (3-D) | 15.66 | 7.40 | 3.84 | 2.11 | 1.26 | 0.82 | **0.63** | 0.53 | 0.47 | 0.42 | 0.42 | 0.35 | 0.23 |

±6% about the midpoint across a 2667× range in 3-D — 1.156° at 52 Hz to 1.299°
at 32 kHz. A low-power board at 104 Hz gives up essentially nothing, which was
the question that prompted the work.

The top two rungs arrived later, with the ODR-coverage work that added
icm42688p's 16 kHz and 32 kHz. They did not change the conclusion, and they
did not reveal a new mechanism either: 3-D attitude RMS has climbed steadily
above the 833 Hz default for the whole length of this table, and it simply
keeps climbing. Through 8 kHz the spread was ±4%; the two new rungs supply the
rest. NIS$_a$ keeps falling too, to 0.23 at 32 kHz — nearly 3× more
under-confident than at the default, on the same trend the next paragraph
describes rather than a break in it.

Worth separating from a related finding, because they are *not* the same
effect. `test_heave_across_rates` measures float32 accumulation error in the
heave integrator turning upward above 6664 Hz (0.112% there, 0.247% at 32 kHz;
see the comment on that test). The MEKF's attitude degradation here begins at
833 Hz, well below that inflection, and is a property of the tuning across the
ladder rather than of float32 accumulation. Two different curves that happen
to both bend upward at the top.

**But NIS$_a$ is not flat, and crosses 1.0 near 150–200 Hz.** The shipped
measurement model is therefore most self-consistent around 200 Hz; at the 833 Hz
default the filter is mildly *under*-confident, and below ~100 Hz increasingly
over-confident. Accuracy does not track this at all.

**And that is not a tuning choice.** Re-running the $(\sigma,\tau)$ grid at
104 Hz: $\tau = 0.5$ s is still at the knee — the eight-fold drop in samples per
correlation time, 416 to 52, does not move it — but *no point in the grid* brings
NIS$_a$ near 1. The best is ≈1.69 at $\sigma = 1.8$, twice the shipped value and
at a real accuracy cost. The decisive evidence is `SCEN_GM`, the one
configuration whose right answer is known in advance: with the knobs set to the
truth it reports NIS$_a = 0.87$ at 833 Hz and **≈3.4 at 104 Hz**. Whatever this
is, it is structural, and it is the open question this section leaves behind.

It is *not* the harness. The benchmark injects per-sample sigmas while the filter
models a density × bandwidth, so the scenario scales its sensor draws by
$\sqrt{f_s/833}$ (and the magnetometer independently by $\sqrt{f_{mag}/100}$).
Cross-checked, $R_a$ over-estimates the injected per-sample variance by 4.480× at
833 Hz and 4.480× at 104 Hz — identical to four figures.

**The floor is at 12 Hz, only in yaw-only mode, and it is the gate.**

| $f_s$ (Hz) | 12 | 14 | 16 | 20 | 25 | 26 | 40 | 52 |
|---|---|---|---|---|---|---|---|---|
| yaw att RMS | **7.673°** | 3.949° | 3.404° | 2.838° | 2.408° | 2.370° | 2.043° | 1.980° |
| NEES(strict) | **402** | 102 | 72.9 | 46.9 | 30.0 | 28.4 | 15.3 | 12.1 |
| reject | **.1051** | .0000 | .0000 | .0000 | .0000 | .0000 | .0000 | .0000 |

Degradation from 52 Hz down to 14 Hz is smooth; 12 Hz is a cliff. The mechanism
is visible in the last row — the gross-outlier gate goes from idle to rejecting
10.5% of updates, which is the rejection-feedback regime §4.7 describes: lost
corrections cause drift, drift enlarges innovations, and those trip the gate
more often still. 3-D mode at 12 Hz is unaffected (1.210°).

Note also that yaw-only is *best* around 40–52 Hz (1.980°), not at the 833 Hz
default (2.185°).

**This does not test the $\Phi = I + F_c\,\Delta t$ linearisation**, and the
claim in §4.4 that $\lVert\omega\rVert\Delta t \ll 1$ at supported ODRs remains
unmeasured. The wave scenario's own peak rate is 18.8 °/s, so
$\lVert\omega\rVert\Delta t = 0.027$ rad even at 12 Hz — nowhere near the limit.
At the 2000 °/s full scale the config permits it would be 2.9 rad at 12 Hz.
Probing that needs a high-rate-rotation scenario that does not exist.

**Two further results worth having.** The Gauss–Markov wave state is
load-bearing at *every* rate, not a fix specific to 833 Hz: disabled, attitude
RMS runs **5.0–11.4°** across the whole ladder — against 1.16–1.30° in 3-D with
it enabled — and the gross-outlier gate goes from idle to rejecting **17–29%**
of updates at 416 Hz and below. Both ranges are from the re-run that added the
16 kHz and 32 kHz rungs, and both are wider than the 8–11° and 12–27%
previously recorded here; the mag-axis figures in the next paragraph reproduce
to the digit against the same run, so this is a stale entry rather than a
non-deterministic bench. Its *direction* also needs qualifying: the wave state
mattering more as the rate falls is true of the rejection rate, not of
accuracy. By RMS ratio it is worth least at 52 Hz (4.4×) and most around the
833 Hz default (8.6×), tapering to 7.8× at 32 kHz. And the magnetometer ladder
is benign for
accuracy — 3-D attitude is flat at 1.171–1.223° from a 1 Hz mag to a 1 kHz one —
while being hostile to the NIS instrument: NIS$_m$ runs 9.84 at 1 Hz against 0.46
at 1 kHz, because $R_m = N_m^2 f_{mag}$ shrinks with the rate while the
attitude-error component of the innovation does not. NEES(strict) correspondingly
*improves* at low mag rates (0.59 at 1 Hz against 5.74 at 100 Hz), which is
independent corroboration of the dip-error diagnosis in §4.8.1: fewer 3-D mag
updates inject less alignment dip bias into roll and pitch.

**The gyro pad.** `mekf_gyro_noise = 0.007` is a ~58×
pad standing in for wave-induced angular dynamics. If that were all it covered it
should be rate-invariant, since $Q_g = N_g^2\Delta t$ already delivers the same
rad²/second at any rate. Measured, the RMS-optimal $N_g$ is **0.002 at 833 Hz and
0.004 at 104 Hz** — eight times the $\Delta t$, twice the pad, close to the
$\sqrt{\Delta t}$ that $N_g^2\Delta t$ implies for an error growing linearly in
wall-clock time. That supports intra-sample rotation nonlinearity as (part of)
what the pad absorbs.

It does **not** justify changing the default, for the reason §4.7 already
documents for $R_a$: at 833 Hz, $N_g = 0.002$ gives the best attitude RMS in the
grid (1.074° against 1.178° shipped) *and* the worst NEES(strict) near it (18.5
against 5.74). No scalar satisfies both. Worth recording that the shipped 0.007
is essentially optimal for yaw-only (2.185° against a best of 2.178°) while ~9%
off optimal for 3-D — a deliberate compromise favouring the marine default, not
an arbitrary number. Reproduce with `-DBENCH_SWEEP_NG`, composable with
`-DBENCH_ODR_HZ=<rate>`.

### 4.8 Magnetometer update — `mekf_update_mag()` (`fusion.c:1438`)

Measurement in Gauss ($m = 0.01\,m_{\mu T}$). Predicted body field
$h_{raw}=R^\top m_{ref}$, magnitude $|h_{raw}|$.

**Normalization.** Both predicted and measured vectors are unit-normalized
before the update; the mag noise is rescaled $R_{m,n}=R_m/|h_{raw}|^2$ to
preserve the optimal gain. (Rationale: in Gauss² units $\det S\approx R_m^3
\approx 4\times10^{-15}$ falls below the inverse's singularity threshold once
$P_{att}$ is small; normalizing makes $S$ dimensionless and well-conditioned.)
Residual $\nu = z-h$, $\text{res\_sq}=\lVert\nu\rVert^2$.

**Compass-health metrics** — computed **before** the gates (§8).

**Gates.** Magnitude ratio $r=|m|/|h_{raw}|$ must lie in $[0.5,2]$; direction
$\text{res\_sq}<4$ (≤90° correction); and, once converged, the tight anomaly
gate $\text{res\_sq} > \text{mag\_reject\_sq}/|h_{raw}|^2$ ⇒ skip.

**Reference adaptation (m_ref EMA), magnitude + dip only.** When
`mref_alpha`>0, `g_body_valid`, $\text{res\_sq}<0.1$, and
$q_{quiet}<2\times10^{-4}$ (platform calm), adapt the field magnitude and dip
from **attitude-independent invariants**: $\sin(\text{dip}) = (m\cdot
g_{body})/|m|$,
$$ m_{ref}^{h}\!\leftarrow m_{ref}^{h}\big(1+\alpha_{mref}(|m|\cos\text{dip}/m_{ref}^{h}-1)\big),\quad
   m_{ref,z}\!\leftarrow m_{ref,z}+\alpha_{mref}(|m|\sin\text{dip}-m_{ref,z}). $$
The horizontal **direction** is never touched.

**Update.** If `mag_yaw_only` (marine default): rotate $m$ to NED, form the
heading innovation
$$ y = \operatorname{atan2}(m^{NED}_y,m^{NED}_x) -
       \operatorname{atan2}(m_{ref,y},m_{ref,x})\ (\text{wrapped}), $$
noise $R_\psi = R_m/(m_{ref}^{h})^2$, and call the scalar update §4.6.
Otherwise call the full vector update §4.5 with $(h,z,R,11.34)$, where $R$
carries the anisotropic dip term of §4.8.1. A near-vertical field
($m^h<0.2|h_{raw}|$) carries no heading information and is skipped.

#### 4.8.1 The dip-reference error, and the anisotropic $R$

In 3-D vector mode the field's **dip** constrains roll and pitch. That is the
mode's whole value over `mag_yaw_only`, and also its exposure: the dip of
$m_{ref}$ is the one part of the reference the filter cannot establish
accurately on its own.

$m_{ref}$ is fixed once, at alignment, from the tilt estimate available at that
moment (§4.3). In a seaway that tilt is wrong, and the error is baked in
permanently — in 3-D mode it becomes a **constant roll/pitch bias** that $P$ has
no term for, which is what a large NEES(strict) is reporting. Measured over the
12-seed benchmark — these are the authoritative 3-D figures:

| 3-D mode | dip error | att RMS | hdg RMS | NEES(tr) | NEES(st) |
|---|---|---|---|---|---|
| align from 1 sample | −4.38° | 4.452° | 0.828° | 3.47 | 335 |
| align over the window | +0.86° | 1.204° | 0.745° | 0.21 | 12.8 |
| + dip reference exact (WMM) | 0.00° | 0.841° | 0.737° | 0.11 | 0.22 |

The per-axis decomposition is unambiguous: at the top row the mean attitude
error is $[-3.85°, -2.00°, -0.28°]$ against an RMS of $[3.87°, 2.04°, 0.87°]$ —
almost pure bias, not spread.

**It cannot be healed in run.** The m_ref EMA above exists for exactly this and
is gated on $q_{quiet}<2\times10^{-4}$, which a seaway never reaches
($q_{quiet}\approx5\times10^{-3}$). That gate is correct, not an oversight.
Raising it looks like a fix over 120 s and is refuted over 30 minutes:

| 3-D, 1800 s window | dip error | $\lvert m_{ref}^h\rvert$ error | att RMS | NEES(st) |
|---|---|---|---|---|
| gate $2\times10^{-4}$ (shipped) | +0.862° | −5.04% | 1.151° | 12.84 |
| gate $2\times10^{-2}$ | −1.460° | +4.24% | 1.725° | 42.10 |
| gate off | −1.469° | +4.27% | 1.729° | 42.31 |

The reference sails past truth and keeps going, because the samples clearing
the $|a|$ band are wave-phase correlated: learning from that subset walks
$m_{ref}$ away rather than toward. (De-contaminating $g_{body}$ with the
Gauss–Markov $\hat a_w$ was also tried, and is worse — $\hat a_w$ has itself
absorbed some tilt, so adding it back re-injects a correlated error.)

So the dip error is **not observable from seaway data**. It is removed at the
source by WMM invariants (`mekf_set_mref_invariants`), or it is admitted into
$P$.

**The anisotropic term.** A dip error $d\delta$ rotates $m_{ref}$ about
$\hat a_{NED} = \hat h_{hor}\times\hat e_D$, so in body frame it perturbs the
normalised prediction along one unit tangent direction:

$$ \hat a_{body} = R^\top\hat a_{NED},\qquad u = \hat a_{body}\times\hat h,
   \qquad \lVert u\rVert = 1,\ \ u^{\mathsf T}\hat h = 0, $$

(the last two because $\hat m_{ref}$ lies in the plane spanned by
$\hat h_{hor}$ and $\hat e_D$, hence $\hat m_{ref}\perp\hat a_{NED}$). The
uncertainty is therefore exactly rank-1:

$$ R = R_{m,n} I_3 + \sigma_{dip}^2\,u u^{\mathsf T}, $$

with $\sigma_{dip}$ = `mekf_mag_dip_sigma_deg` in radians (a dip error of
$\delta$ displaces the predicted unit direction by $\delta$, so the units match
directly). Default 1.0°, which is the measured +0.86° residual rounded up —
**not** a value fitted to the metric.

**Why rank-1 and not just a larger $R_m$.** Isotropic inflation reaches the same
NEES: $R_m\times64$ gives NEES(strict) 12.83 → 1.57 with attitude and heading
both marginally better. But it deweights the heading-carrying components too,
and `nis_mag` falls to **0.01** — the wire's magnetometer-health instrument
stops being able to report a fault. The rank-1 form leaves those components
alone. Sweep, 3-D mode with a window-aligned reference:

| $\sigma_{dip}$ | att RMS | hdg RMS | NEES(st) | `nis_mag` |
|---|---|---|---|---|
| 0° | 1.204° | 0.745° | 12.83 | 0.52 |
| 0.5° | 1.186° | 0.723° | 9.06 | 0.39 |
| **1.0° (shipped)** | **1.178°** | **0.714°** | **5.74** | **0.31** |
| 2.0° | 1.177° | 0.715° | 3.14 | 0.28 |
| 3.0° | 1.179° | 0.720° | 2.15 | 0.27 |

**dof = 2 survives.** Because $u$ is tangent, $R\hat h = R_{m,n}\hat h$, so
$\hat h$ remains an eigenvector of $S = HPH^\top + R$ with eigenvalue
$R_{m,n}$; combined with $H^\top\hat h = 0$ the §8.2 derivation
$E[d^2] = 3 - R_{m,n}\hat h^\top S^{-1}\hat h = 2$ is unchanged. An anisotropy
with any radial component would break it silently — and would also stop the
term working at all, since the radial direction of a normalised measurement
carries no information (measured: a contaminated $u$ widens $P$ by 1% instead
of 9.5% and absorbs 53% of the pull instead of 93%).

Note `nis_mag` settling near **0.5** once $\sigma_{dip}$ dominates is the
correct reading, not a regression: one of the two tangent degrees of freedom is
deliberately deweighted, so a consistent filter reads about half.

**What this does not do.** It does not reach NEES(strict) = 1, and cannot. The
dip error is a *bias*; a covariance term can only partly stand in for one, and
the accelerometer legitimately keeps $P$ tight in roll and pitch. Driving the
number to 1 would need $\sigma_{dip}\approx4°$, i.e. inventing uncertainty to
satisfy a statistic. For an install that cares, the answer is a position
source.

**As-implemented / audit note.** Adapting **only** magnitude and dip is
deliberate: the horizontal direction *is* the heading reference, so learning
it from the filter's own attitude would feed the heading estimate back into
its own datum and let both random-walk together (gauge drift). The adaptation
targets are body-frame invariants (total magnitude; dip from the mag–gravity
angle), so the filter's attitude never enters. Heading-only fusion is the
default because the soft-iron matrix is fitted from a 2-D swing (§3.3, §12.3)
and its dip channel is least trustworthy.

**Source:** heading-only / tilt-compensated magnetometer fusion — standard
marine AHRS practice, Farrell 2008 *(canonical)*; gauge-feedback avoidance is
imud-specific design (see the in-code rationale in `mekf_update_mag`,
`fusion.c:1550`).

### 4.9 WMM reference invariants — `mekf_set_mref_invariants()` (`fusion.c:1057`)

Given WMM horizontal magnitude $H$ and vertical $Z$ (Gauss) at a known
position (from §13), rescale `m_ref` to those invariants while preserving
horizontal direction: with $m_h=\sqrt{m_{ref,x}^2+m_{ref,y}^2}$ and
$s=H/m_h$,
$$ m_{ref,x}\!\leftarrow s\,m_{ref,x},\quad m_{ref,y}\!\leftarrow s\,m_{ref,y},
   \quad m_{ref,z}\!\leftarrow Z. $$
No-op before alignment or if $H\le0$.

**Source:** imud-specific (WMM supplies the invariants of §13).

### 4.10 State extraction — `mekf_get_state()` (`fusion.c:1988`)

Euler angles from $R(q)$ (NED 3-2-1 aerospace):
$$ \theta=\arcsin(-R[2][0]),\quad \phi=\operatorname{atan2}(R[2][1],R[2][2]),
   \quad \psi=\operatorname{atan2}(R[1][0],R[0][0]). $$
Magnetic heading $\psi$ wrapped to $[0,360°)$. Attitude covariance
(`cov[9]`) = $P[0{:}3,0{:}3]$; bias variance = $\operatorname{diag}
P[3{:}6]$; `quiescence` $=q_{quiet}$. `rate_of_turn` is left 0 here and filled
by the fusion thread (§5).

**Source:** quaternion→Euler, Diebel 2006 *(canonical)*.

### 4.11 Reconfigure — `mekf_reconfigure()` (`fusion.c:1918`)

Recomputes $Q_g,Q_b,R_a,R_m$, the skip band, `mag_reject_sq`,
`mag_yaw_only`, `mref_alpha`, `conv_thresh`, and the gate-health EMA rate from
a new config on hot-reload; $q$, $P$, $b$, $dt$ and the runtime scalars
(`Ra_scale`, `acc_quiet_ema`, the health EMAs) are untouched, so the filter
keeps running. Both this and `mekf_init()` derive their tuning through one
shared helper (`mekf_derive_tuning`, `fusion.c:864`), so no value can be
updated in one path and forgotten in the other — `mref_alpha` previously was,
leaving the m_ref EMA at a stale rate after any `mag_odr_hz` change.

---

## 5. Euler rates and rate of turn

Computed in the fusion thread (`fusion_thread`, `imu.c:1429`–`1476`) from the
bias-corrected body rate $\omega = s.\text{gyro}-b$ and the current Euler
angles, using the inverse of the 3-2-1 kinematic relation.

**Roll rate** (for sea-state, §7), with a near-gimbal-lock fallback:
$$ \dot\phi = \omega_x + \tan\theta\,(\omega_y\sin\phi + \omega_z\cos\phi),
\qquad \dot\phi=\omega_x \text{ if } |\cos\theta|\le0.2. $$

**Pitch rate** (no singularity):
$$ \dot\theta = \omega_y\cos\phi - \omega_z\sin\phi. $$

**Rate of turn** (heading rate, output in deg·min⁻¹):
$$ \dot\psi = \frac{\omega_y\sin\phi + \omega_z\cos\phi}{\cos\theta},
\qquad \dot\psi=\omega_z \text{ if } |\cos\theta|\le0.2,\qquad
\text{ROT}=\dot\psi\cdot\frac{180}{\pi}\cdot 60. $$

**As-implemented.** ROT is the *Euler yaw rate*, not raw body-Z gyro: at
20° roll the body-Z rate under-reads heading rate and couples in pitch rate,
which autopilots notice. Near $\pm90°$ pitch ($|\cos\theta|\le0.2$) the code
falls back to the body-axis rate to avoid division blow-up.

**Source:** 3-2-1 Euler kinematics, Titterton & Weston 2004 §3.6; Diebel 2006
*(canonical)*.

---

## 6. Heave estimator — `heave_update()` (`fusion.c:1771`)

Vertical displacement from vertical acceleration, by leaky double integration
followed by a true first-order high-pass.

**Vertical acceleration (NED down, zero at rest):**
$$ a_D = \big(R(q)\,a_b\big)_D + g = R[2][:]\cdot a_b + g. $$

**Leaky double integration**, leak $\ell = dt/\tau$:
$$
v \leftarrow (v + a_D\,dt)(1-\ell),\qquad
d \leftarrow (d + v\,dt)(1-\ell).
$$

**Output high-pass** (exact zero at DC), $\alpha_{hp}=\tau/(\tau+dt)$:
$$ y \leftarrow \alpha_{hp}\,(y + d - d_{prev}),\qquad d_{prev}\leftarrow d, $$
heave (positive **up**) $= -y$. `heave_rate` output $=-v$. Settled after
$\ge 10\tau$ (`settled` flag).

**As-implemented / audit note.** Two leaky integrators bound drift but retain
a DC gain of $\sim\tau^2$ ($\approx144$ m per m·s⁻² at the default
$\tau=12$ s); any slow residual (accel bias, small tilt error, start-up
transient) would appear at metre scale. The output high-pass has a
**structural** zero at DC, killing those residuals rather than estimating
them. Passband amplitude error $\approx2\%$ at 8 s, $\approx4\%$ at 12 s for
$\tau=12$ s (band-limited: very long-period swell is attenuated). $\tau=0$
disables. The estimate is a filtered kinematic quantity, not a
statistically-optimal one.

**Source:** leaky integrator + first-order high-pass are standard
discrete-time filter forms, Oppenheim & Schafer 2010; the double-integration-
plus-high-pass heave technique is long-standing in wave-buoy practice
(Datawell / Longuet-Higgins tradition) *(canonical)*.

---

## 7. Sea-state statistics — `seastate_*` (`fusion.c:1829`–`1916`)

Windowed spectral moments over the heave, roll, and pitch oscillations via
exponentially-weighted mean/variance pairs — no FFT, no sample storage.

**EW mean/variance recursion** (`ew_stat`, `fusion.c:1847`), $\alpha=dt/\tau$:
$$ \mu \leftarrow \mu + \alpha\,d,\qquad
   \sigma^2 \leftarrow \sigma^2 + \alpha\big((1-\alpha)\,d^2 - \sigma^2\big),
   \qquad d = x-\mu. $$
Applied to six signals: heave, heave-rate, roll, roll-rate, pitch,
pitch-rate. Fed **only while heave is settled** (`seastate_update`,
`fusion.c:1854`).

**Outputs** (from the variances; $m_0=\operatorname{var}(x)$,
$m_2=\operatorname{var}(\dot x)$):
$$
H_s = 4\sqrt{m_0^{heave}},\qquad
T_z = 2\pi\sqrt{\frac{m_0^{heave}}{m_2^{heave}}},\qquad
T_{roll} = 2\pi\sqrt{\frac{m_0^{roll}}{m_2^{roll}}},
$$
$$
A_{roll} = 2\sqrt{m_0^{roll}},\qquad
A_{pitch} = 2\sqrt{m_0^{pitch}},\qquad
T_{pitch} = 2\pi\sqrt{\frac{m_0^{pitch}}{m_2^{pitch}}}.
$$

Guards: all return 0 until settled ($\ge2\tau$); periods additionally return
0 below the oscillation floors $\sigma(\text{heave})<2$ cm
(`SEASTATE_MIN_HEAVE_SIG`) or $\sigma(\text{angle})<0.3°$
(`SEASTATE_MIN_ANGLE_SIG`), or non-positive rate variance.

**As-implemented / audit note.** $H_s=4\sqrt{m_0}$ is the significant wave
height for a narrow-band Gaussian sea (Longuet-Higgins). The period identity
$T=2\pi\sqrt{m_0/m_2}$ is Rice's mean up-crossing period from the zeroth and
second spectral moments — **exact for a pure sine** ($\operatorname{var}=A^2/2$,
$\operatorname{var}(\dot x)=A^2\omega^2/2$, ratio $=1/\omega^2$) and an
estimator for a real seaway. Amplitudes are reported as **significant single
amplitudes** ($2\sigma$, seakeeping convention); wave height is the
significant **double** amplitude ($4\sigma$). The EW variance is biased toward
the window's recent history and toward the EW mean (so a steady heel / trim
is removed from roll/pitch, appearing in $\mu$, not $\sigma$). The default
$\tau=120$ s is far shorter than the 10–20 min oceanographic record; it
favours a responsive live display. Roll/pitch **rates** are Euler rates (§5),
not body rates.

**Source:** $H_s=4\sqrt{m_0}$ Longuet-Higgins 1952; spectral-moment
up-crossing period Rice 1944–45; engineering treatment Tucker & Pitt 2001;
roll-period/seakeeping convention Lloyd 1989 *(canonical)*. EW variance
recursion, West 1979 / Finch 2009 *(canonical)*.

---

## 8. Compass-health diagnostics — `mekf_update_mag()` (`fusion.c:1438`)

Two EMAs ($\alpha=1/3000$, $\tau\approx30$ s at 100 Hz mag ODR), updated
**before** the rejection gates so that gated-out anomalies still register:

**Field-magnitude anomaly** (attitude-independent):
$$ \text{mag\_anom} \leftarrow \text{mag\_anom} +
   \alpha\left(\frac{\lvert\, |m|-|h_{raw}|\,\rvert}{|h_{raw}|}
   - \text{mag\_anom}\right). $$

**Heading residual** (mode-independent; $m^{NED}=R(q)\,m$):
$$ y = \operatorname{atan2}(m^{NED}_y,m^{NED}_x) -
       \operatorname{atan2}(m_{ref,y},m_{ref,x})\ (\text{wrapped to }\pm\pi),
   \qquad
   \text{mag\_resid} \leftarrow \text{mag\_resid} +
   \alpha\big(|y| - \text{mag\_resid}\big), $$
gated on both horizontal magnitudes exceeding $0.2|h_{raw}|$.

**As-implemented.** Diagnostics only — they never modify the filter. Fed from
pre-gate innovations because the rejected samples are precisely the anomalies
these metrics exist to surface. The 30 s time constant scales inversely with a
non-standard mag rate (acceptable for a health indicator).

### 8.1 Update-gate health — `gate_health()` (`fusion.c:159`)

Two further EMAs ($\tau\approx30$ s, gain per §4.7), written by every
`eskf_update` / `eskf_update_yaw` call — accepted, capped and rejected alike,
with $\gamma_r=25\gamma$ the gross-reject threshold:

$$ \text{innov\_weight} \leftarrow \text{innov\_weight} +
   \alpha\big(w - \text{innov\_weight}\big), \qquad
   w=\begin{cases}1 & d^2\le\gamma\\ \sqrt{\gamma/d^2} & \gamma<d^2\le\gamma_r\\
   0 & d^2>\gamma_r\end{cases} $$

$$ \text{innov\_reject} \leftarrow \text{innov\_reject} +
   \alpha\big(\mathbb{1}[d^2>\gamma_r] - \text{innov\_reject}\big). $$

`innov_weight` near 1 means the filter is accepting its measurements at face
value; a sustained drift toward $1/5$ means the Huber cap is doing continuous
work, which is the regime in which the covariance inconsistency noted in §4.5
matters. `innov_reject` separates "capping hard" from "throwing measurements
away outright". Both are exported on the wire (§8 of `spec.md`, offsets 256
and 260) so the behaviour can be observed in the field rather than inferred
from a synthetic benchmark.

**Source:** exponential moving average, standard *(canonical)*; innovation
consistency monitoring Bar-Shalom et al. 2001 *(canonical)*.

### 8.2 Measurement-model consistency — `nis_record()` (`fusion.c:197`)

Where §8.1 reports how hard the robustness machinery is *working*, this
reports whether the noise model underneath it is *right*. Two EMAs of the
normalised innovation squared, per channel (wire offsets 264 and 268):

$$ \text{nis} \leftarrow \text{nis} + \alpha\Big(\min\big(d^2/n_{dof},\,100\big)
   - \text{nis}\Big),\qquad d^2=\nu^\top S^{-1}\nu. $$

Accumulated **before** the Huber cap and **including** rejected updates: the
cap censors $d^2$ at $\gamma$, so a post-cap average would be bounded by
construction and could never report the inconsistency it exists to measure.
The channels are split because they run at very different rates (833 Hz accel
vs ~104 Hz mag); a combined EMA would be ~8:1 accel.

**Degrees of freedom.** The vector updates carry $n_{dof}=2$, not 3, because
the measurement is a *unit* vector — normalising $z$ removes the radial
component, so the noise lives only in the tangent plane. Write $\hat z$ for the
predicted direction ($\hat z = h$ without the wave state, $\hat z =
(h-\hat a_w)/\lVert h-\hat a_w\rVert$ with it, §4.5.1). The true innovation
covariance is $E[\nu\nu^\top]=HPH^\top+R(I-\hat z\hat z^\top)$ while
$S=HPH^\top+R\,I$:

$$ E[d^2]=\operatorname{tr}\!\big(S^{-1}E[\nu\nu^\top]\big)
        =\operatorname{tr}\!\big(S^{-1}(S-R\,\hat z\hat z^\top)\big)
        =3-R\,\hat z^\top S^{-1}\hat z, $$

and in **both** cases $H^\top\hat z=0$ — for the plain form because
$H=[h]_\times$ gives $[h]_\times h=0$, and for the wave-aware form because
every row of $H$ is left-multiplied by the tangent projector
$P_{\hat z}=I-\hat z\hat z^\top$ and $P_{\hat z}\hat z=0$. So $\hat z$ is an
eigenvector of $S$ with eigenvalue $R$, $S^{-1}\hat z=\hat z/R$, and the
correction term is exactly 1:
$$ E[d^2]=2 \quad\text{for both the gravity and 3-D mag updates.} $$
The yaw-only scalar update has $n_{dof}=1$. Dividing by 3 instead would peg a
perfectly consistent filter at 0.67.

This identity is the reason the projector in §4.5.1 cannot be dropped: it is
the *only* thing that keeps $H^\top\hat z=0$ once $\hat a_w\ne0$, and without
it the wire field silently stops meaning what its documentation says.

Measured values over the wave benchmark are 1.01 (3-D) / 0.69 (yaw-only) with
the shipped Gauss–Markov wave state — i.e. the model is now consistent, and
mildly conservative in yaw-only mode. They were 19 / 25 before §4.7.1, and 56
/ 57 with the `m33_inv` fix but no wave state. `imud-cal fit-ra` computes the
same statistic offline from a capture, through the filter's own
`mekf_accel_probe()`, so the live number can be checked against real water.

**Source:** NIS consistency test Bar-Shalom et al. 2001 §5.4 *(canonical)*.

---

## 9. Engine-vibration detector — `ism_reader_thread()` (`imu.c`)

EMA of squared specific-force deviation from 1 g, stepped once per sample:
$$ e \leftarrow e + \alpha\big((\lVert a\rVert - g)^2 - e\big),\qquad
   \alpha = \frac{1}{\tau\,f_{ODR}},\ \ \tau = 1\,\mathrm{s}. $$

$\alpha$ is derived from the **resolved sample rate**, not fixed. Because the
EMA advances once per sample but samples arrive in FIFO bursts, a constant
$\alpha$ made the effective time constant a function of burst depth (a deeper
burst = more steps per read = a faster EMA) rather than the intended 1 s.

Assertion uses **hysteresis** rather than a bare threshold:
$$ \text{engine\_on} = \begin{cases}
   \text{true}  & e > \text{engine\_vibration\_g2}\\
   \text{false} & e < 0.7\,\text{engine\_vibration\_g2}\\
   \text{unchanged} & \text{otherwise}
   \end{cases} $$

A single threshold chattered at the boundary, and each toggle steps
`Ra_scale`, rewrites the skip band, emits a log line, and flips
`FLAG_ENGINE_ON` on the wire — all externally visible. `engine_on` raises the
accel noise ×4 and widens the skip band (§4.7); the fusion thread re-asserts
both on every sample, so a config reload cannot leave the skip band and
`Ra_scale` disagreeing.

**Source:** EMA threshold detector with Schmitt hysteresis, standard
*(canonical)*.

---

## 10. Startup gyro-bias estimation — `fusion_thread()` (`imu.c:934`)

Mean of the gyro over a still window ($N=$ `gyro_bias_sec`·ODR samples):
$$ \hat b_k = \frac1N\sum_{i=1}^{N} \omega_{i,k}. $$
A per-axis motion check computes the sample standard deviation
$\sigma_k=\sqrt{\overline{\omega^2}-\bar\omega^2}$; if
$\max_k\sigma_k > 0.00873$ rad·s⁻¹ ($0.5°$·s⁻¹) the window is **doubled once**
(a longer average spans more wave cycles). The result seeds `mekf_init`
(§4.2); the MEKF refines it online regardless.

**Source:** sample mean/variance, elementary *(canonical)*.

---

## 11. Timestamp anchoring and per-sample dt — `imu_math.c:58`–`194`

A single anchor $(\text{chip\_ticks}, \text{wall\_ns}, \text{tai\_ns},
\text{gen})$ maps the sensor's free-running counter to system time. Per
sample (`chip_to_wall`, `imu_math.c:178`), with `tick_ns` the counter period:
$$ \text{offset} = (\text{chip\_ts} - \text{chip\_ticks})\cdot\text{tick\_ns},
\qquad t_{wall} = \text{wall\_ns} + \text{offset}, $$
$t_{tai}$ likewise. Subtraction is unsigned 32-bit (wrap-safe within one
anchor interval). The anchor is refreshed on first read, every 60 s with a
hardware counter, or every burst without one.

**Per-sample interval** for prediction (`imu.c`): the wall-time delta between
consecutive samples of the same anchor generation, **clamped to
$[0.5,\,2]\times$ nominal**; outside that (FIFO gap, anchor reset) the nominal
$1/\text{ODR}$ is used. This keeps oscillator tolerance (few % on the chip
clock) from scaling integrated rotation while rejecting gap artefacts.

**As-implemented.** Linear (constant-rate) interpolation between anchors; no
clock-drift model beyond the 60 s re-anchor. `tick_ns`=0 for chips without a
hardware timestamp degenerates the offset to 0 (the anchor wall-time is then
the per-sample time).

**Source:** imud-specific timestamp reconstruction.

---

## 12. Offline calibration fits (`imud-cal`, `cal_math.c`)

### 12.1 Linear solver — `gauss4()` (`cal_math.c:25`)

Gaussian elimination with **partial pivoting** on the augmented $4\times5$
system, then back-substitution. Returns $-1$ if any pivot $<10^{-12}$.
Double precision.

**Source:** Golub & Van Loan 2013 §3.4 *(canonical)*.

### 12.2 Hard-iron sphere fit — `sphere_fit()` (`cal_math.c:153`)

Algebraic (linearized) sphere fit. The sphere
$(x-c_x)^2+(y-c_y)^2+(z-c_z)^2=r^2$ is linearized as
$$ x^2+y^2+z^2 = 2c_x x + 2c_y y + 2c_z z + (r^2 - \lVert c\rVert^2), $$
and the normal equations $A^\top A\,p = A^\top b$ are accumulated
incrementally (`sphere_add`, `cal_math.c:59`) over sums
$\Sigma x,\Sigma x^2,\Sigma xy,\Sigma x r^2,\dots$. Solved by `gauss4`:
$$ A^\top A =
\begin{bmatrix}
4S_{xx}&4S_{xy}&4S_{xz}&2S_x\\
4S_{xy}&4S_{yy}&4S_{yz}&2S_y\\
4S_{xz}&4S_{yz}&4S_{zz}&2S_z\\
2S_x&2S_y&2S_z&n
\end{bmatrix},\quad
A^\top b=\begin{bmatrix}2S_{xr}\\2S_{yr}\\2S_{zr}\\S_{xx}{+}S_{yy}{+}S_{zz}\end{bmatrix}, $$
center $c=p[0{:}3]$, $r=\sqrt{p_3+\lVert c\rVert^2}$. Center = hard-iron
offset; $r$ = reference field magnitude for §12.3.

**As-implemented.** This is the *algebraic* fit (minimizes an algebraic
residual, not orthogonal geometric distance); fast, closed-form, and adequate
for well-sampled swings, but statistically biased for sparse/noisy data
versus a geometric fit.

**Source:** algebraic sphere/magnetometer fit, Renaudin et al. 2010; Coope
1993 *(canonical)*.

### 12.3 Soft-iron 2-D ellipse fit — `ellipse_fit()` (`cal_math.c:106`)

Conic least-squares on the horizontal magnetometer locus
$A x^2 + B xy + C y^2 = 1$ (regressors $x^2,xy,y^2$; normal equations from
$S_{40},S_{31},S_{22},S_{13},S_{04},S_{20},S_{11},S_{02}$ via `ellipse_add`,
solved by `gauss4` with an identity pad row). Requires the conic
$M=\begin{pmatrix}A&B/2\\B/2&C\end{pmatrix}$ positive-definite
($A>0$, $\det>0$).

Soft-iron matrix $S = r\cdot M^{1/2}$ via the $2\times2$ symmetric
eigendecomposition: eigenvalues $\lambda_{1,2}=\tfrac{A+C}2\pm
\sqrt{\tfrac14(A-C)^2+\tfrac14 B^2}$, unit eigenvector $v$ for $\lambda_1$,
and
$$ S = s_1\,v v^\top + s_2\,v_\perp v_\perp^\top,\qquad
   s_{1,2}=r\sqrt{\lambda_{1,2}},\ v_\perp=(-v_y,v_x). $$

**As-implemented / audit note.** The soft-iron correction is **2-D** (X–Y
plane only). This is a deliberate specialization for the heading-only fusion
default (§4.8): the swing is a horizontal circle, so the vertical/dip channel
is not calibrated. `ellipse_fit` includes a diagonal fallback when the conic
is degenerate. The full `mag_soft_iron[3][3]` applied live (§3.3) therefore
has its Z couplings from configuration/identity, not from this fit.

**Source:** algebraic conic/ellipse fit, Fitzgibbon, Pilu & Fisher 1999;
magnetometer soft-iron, Renaudin et al. 2010 *(canonical)*.

### 12.4 Heading-circle coverage — `cal_cov_mark()` (`cal_math.c:308`)

Guided-swing coverage: the heading circle is split into `nsec` sectors; the
sample $(x,y)$ relative to center $(c_x,c_y)$ is binned by
$$ s = \Big\lfloor \frac{\operatorname{atan2}(y-c_y,\,x-c_x)\bmod 2\pi}
{2\pi/\text{nsec}} \Big\rfloor. $$
`cal_cov_count` sums the marked sectors. Pure bookkeeping (no fit).

### 12.5 Allan variance — `allan_deviation()` (`cal_math.c:327`)

**Overlapping** Allan deviation. With the cumulative integral
$\theta[k]=\sum_{j<k} x_j\,dt$ and octave cluster lengths $m=1,2,4,\dots$
($\tau=m\,dt$):
$$ \sigma^2(\tau) = \frac{1}{2\tau^2\,(N-2m+1)}
   \sum_{k=0}^{N-2m}\big(\theta[k{+}2m]-2\theta[k{+}m]+\theta[k]\big)^2. $$

**Characterization** (`allan_characterize`, `cal_math.c:360`): white-noise
density from the shortest cluster,
$$ N = \sigma(\tau_0)\sqrt{\tau_0}, $$
bias instability from the curve minimum,
$$ B = 0.664\cdot\min_\tau \sigma(\tau). $$

**As-implemented.** Octave (not fully-overlapping all-$\tau$) spacing;
$N$ is read at $\tau_0$ (assumes white noise dominates there); $B$ uses the
IEEE-952 factor $0.664$ at the deviation minimum (an upper bound if the flat
region is not reached). These characterize the sensor; they do **not** feed
the filter tuning (§4.2 note).

**Source:** Allan 1966; overlapping estimator Riley 2008 (NIST SP 1065);
$N$/$B$ extraction and $0.664$, IEEE Std 952 *(code comment: "IEEE 952")*.

### 12.6 Gyro-bias / temperature fit — `gyro_temp_fit()` (`cal_math.c:384`)

Ordinary least-squares line of gyro bias vs. temperature about a reference
$T_{ref}$: with $t_i=T_i-T_{ref}$,
$$ \text{coeff} = \frac{n\sum t_i y_i - \sum t_i\sum y_i}
{n\sum t_i^2 - (\sum t_i)^2},\qquad
\text{bias\_ref} = \frac{\sum y_i - \text{coeff}\sum t_i}{n}. $$
Rejects fits with temperature span $<1$ °C. `coeff` → `gyro_temp_coeff`,
applied live in §3.2.

**Source:** ordinary least squares, elementary *(canonical)*.

---

## 13. World Magnetic Model — `wmm_field_ned()` (`wmm.c:119`)

Degree/order 12 spherical-harmonic geomagnetic field. This section states the
implemented computation; the defining theory is the WMM report cited below.

**Secular variation** to the decimal year $t$ (`wmm_decimal_year`,
`wmm.c:73`), $\Delta t = t - t_{epoch}$:
$$ g_n^m(t) = g_n^m + \dot g_n^m\,\Delta t,\qquad
   h_n^m(t) = h_n^m + \dot h_n^m\,\Delta t. $$

**Geodetic (WGS-84) → geocentric spherical.** With prime-vertical radius
$N=a/\sqrt{1-e^2\sin^2\varphi}$ ($a=6378137$ m, $e^2$ from
$f=1/298.257223563$):
$$ p_x=(N+\text{alt})\cos\varphi,\quad p_z=(N(1-e^2)+\text{alt})\sin\varphi,$$
$$ r=\sqrt{p_x^2+p_z^2},\quad \theta=\arccos(p_z/r)\ (\text{co-latitude}),
   \quad \lambda=\text{lon}. $$

**Schmidt quasi-normalized associated Legendre functions** $P_n^m$ and
derivatives $dP_n^m/d\theta$, as coded (`wmm.c:178`): seeds $P_0^0=1$,
$P_1^0=\cos\theta$, $P_1^1=\sin\theta$; for $n\ge2$,
$$
P_n^n = \sqrt{\tfrac{2n-1}{2n}}\,\sin\theta\,P_{n-1}^{n-1},\qquad
P_n^m = \frac{2n-1}{\sqrt{n^2-m^2}}\cos\theta\,P_{n-1}^m
        - \sqrt{\tfrac{(n{-}m{-}1)(n{+}m{-}1)}{(n{-}m)(n{+}m)}}\,P_{n-2}^m,
$$
the second covering zonal ($m=0$), sub-diagonal ($m=n-1$, second term 0), and
tesseral terms; derivatives follow by differentiating the same recursion.

**Field synthesis (geocentric NED), $a=6371.2$ km:**
$$
X_{gc}=\sum_{n=1}^{12}\Big(\tfrac{a}{r}\Big)^{n+2}\!\sum_{m=0}^{n}
 \big(g_n^m\cos m\lambda + h_n^m\sin m\lambda\big)\,\frac{dP_n^m}{d\theta},
$$
$$
Y_{gc}=\frac{1}{\sin\theta}\sum_{n=1}^{12}\Big(\tfrac{a}{r}\Big)^{n+2}\!\sum_{m=0}^{n}
 m\big(g_n^m\sin m\lambda - h_n^m\cos m\lambda\big)\,P_n^m,
$$
$$
Z_{gc}=-\sum_{n=1}^{12}(n{+}1)\Big(\tfrac{a}{r}\Big)^{n+2}\!\sum_{m=0}^{n}
 \big(g_n^m\cos m\lambda + h_n^m\sin m\lambda\big)\,P_n^m.
$$

**Geocentric → geodetic NED** rotation by $\delta=\varphi-\varphi_{gc}$
($\varphi_{gc}=\pi/2-\theta$):
$$ X_{NED}=X_{gc}\cos\delta + Z_{gc}\sin\delta,\quad Y_{NED}=Y_{gc},\quad
   Z_{NED}=Z_{gc}\cos\delta - X_{gc}\sin\delta. $$

**Declination** (`wmm_declination`, `wmm.c:251`):
$D=\operatorname{atan2}(Y_{NED},X_{NED})$. The horizontal magnitude
$H=\sqrt{X^2+Y^2}$ and vertical $Z$ supply the §4.9 magnetic invariants.

**As-implemented.** Secular variation is linear extrapolation from the epoch
coefficients (valid over the model's 5-year validity window). `wmm_load`
requires exactly the 90 degree-12 coefficient rows and rejects truncated
files. Powers $(a/r)^{n+2}$ are formed by iterative multiplication, not
`pow`. Poles are guarded by clamping $\sin\theta\ge10^{-10}$.

**Source:** WMM2025 Technical Report, Chulliat et al. 2024 (NCEI/BGS) — the
defining document; ALF recursion Langel 1987 / WMM Technical Note 28
*(code comment)*; WGS-84 NIMA TR8350.2 *(canonical)*.

---

## 14. Symbol ↔ code map

| Symbol | Meaning | Code |
|---|---|---|
| $q$ | attitude quaternion, body→NED, scalar-first | `mekf_t.q[4]` |
| $b$ | gyro bias | `mekf_t.bias[3]` |
| $a_w$ | wave acceleration, body, g units | `mekf_t.wave_acc[3]` |
| $P$ | 9×9 error covariance $[\delta\theta\mid\delta b\mid\delta a_w]$ | `mekf_t.P[MEKF_N][MEKF_N]` |
| $R(q)$ | body→NED rotation matrix | `q_to_R()` |
| $Q_g,Q_b$ | process-noise variances / step | `mekf_t.Qg, .Qb` |
| $R_a,R_m$ | accel / mag meas. noise var | `mekf_t.Ra, .Rm` |
| $\sigma,\tau$ | Gauss–Markov wave σ (m·s⁻²), correlation time (s) | `mekf_wave_accel`, `mekf_wave_accel_tau_s` |
| $\sigma_g^2,\varphi,Q_w$ | wave variance (g²), GM transition, GM noise | `mekf_t.wave_sig2`, `mekf_predict` |
| $P_{\hat z}$ | tangent projector $I-\hat z\hat z^\top$ | `wave_jacobian` |
| $H$ | measurement Jacobian | `eskf_update`, `eskf_update_yaw` |
| $\gamma$ | $\chi^2$ innovation gate | `ACCEL_CHI2_GATE`=11.34, `YAW_CHI2_GATE`=6.63 |
| $m_{ref}$ | Earth-field reference, NED, Gauss | `mekf_t.m_ref[3]` |
| $q_{quiet}$ | quiescence EMA $(|a|/g-1)^2$ | `mekf_t.acc_quiet_ema` |
| $g_{body}$ | accel gravity direction, body | `mekf_t.g_body[3]` |
| $H_s,T_z$ | sig. wave height, zero-cross period | `seastate_wave_height/_period` |
| $a_D$ | NED-down linear accel | `heave_update` |
| $N,B$ | Allan noise density, bias instability | `allan_characterize` |
| $g_n^m,h_n^m$ | Gauss coefficients | `wmm_t.g/.h`, `.g_sv/.h_sv` |
| $D$ | magnetic declination | `wmm_declination` |

---

## 15. References

Citations are to the canonical source for each method; imud implements
standard techniques rather than novel mathematics. Items the maintainer may
wish to verify against a specific edition/page are flagged **[verify]**.

1. **J. Solà**, "Quaternion kinematics for the error-state Kalman filter,"
   arXiv:1711.02508, 2017. *(MEKF, quaternion kinematics, error-state
   covariance — named in `fusion.c`.)*
2. **F. L. Markley, J. L. Crassidis**, *Fundamentals of Spacecraft Attitude
   Determination and Control*, Springer, 2014. *(MEKF.)*
3. **N. Trawny, S. Roumeliotis**, "Indirect Kalman Filter for 3D Attitude
   Estimation," Univ. Minnesota TR-2005-002. *(error-state attitude EKF.)*
4. **R. E. Kalman**, "A New Approach to Linear Filtering and Prediction
   Problems," *J. Basic Eng.*, 1960. *(Kalman update.)*
5. **R. S. Bucy, P. D. Joseph**, *Filtering for Stochastic Processes with
   Applications to Guidance*, 1968. *(Joseph-form covariance.)* **[verify]**
6. **P. J. Huber**, "Robust Estimation of a Location Parameter," *Ann. Math.
   Statist.*, 1964. *(innovation influence capping.)*
7. **Y. Bar-Shalom, X.-R. Li, T. Kirubarajan**, *Estimation with Applications
   to Tracking and Navigation*, Wiley, 2001. *(Mahalanobis $\chi^2$ gating.)*
8. **J. A. Farrell**, *Aided Navigation: GPS with High Rate Sensors*,
   McGraw-Hill, 2008. *(coarse alignment, tilt-compensated heading,
   specific-force model.)*
9. **D. Titterton, J. Weston**, *Strapdown Inertial Navigation Technology*,
   2nd ed., IET, 2004. *(Euler kinematics, sensor error models,
   coordinated-turn specific force.)*
10. **M. D. Shuster, S. D. Oh**, "Three-Axis Attitude Determination from
    Vector Observations," *J. Guidance & Control*, 1981. *(TRIAD lineage.)*
11. **J. Diebel**, "Representing Attitude: Euler Angles, Unit Quaternions,
    and Rotation Vectors," Stanford, 2006. *(quaternion↔Euler.)*
12. **M. S. Longuet-Higgins**, "On the statistical distribution of the
    heights of sea waves," *J. Marine Res.*, 1952. *($H_s=4\sqrt{m_0}$.)*
13. **S. O. Rice**, "Mathematical Analysis of Random Noise," *Bell Syst.
    Tech. J.*, 1944–45. *(zero-crossing rate / spectral-moment period.)*
14. **M. J. Tucker, E. G. Pitt**, *Waves in Ocean Engineering*, Elsevier,
    2001. *(spectral moments, sea-state parameters.)* **[verify]**
15. **A. R. J. M. Lloyd**, *Seakeeping: Ship Behaviour in Rough Weather*,
    Ellis Horwood, 1989. *(roll period, significant single amplitude.)*
    **[verify]**
16. **D. H. D. West**, "Updating mean and variance estimates: an improved
    method," *Comm. ACM*, 1979; **T. Finch**, "Incremental calculation of
    weighted mean and variance," 2009. *(EW mean/variance recursion.)*
17. **A. V. Oppenheim, R. W. Schafer**, *Discrete-Time Signal Processing*,
    3rd ed., 2010. *(leaky integrator, first-order high-pass.)*
18. **D. W. Allan**, "Statistics of Atomic Frequency Standards," *Proc.
    IEEE*, 1966; **W. J. Riley**, *Handbook of Frequency Stability Analysis*,
    NIST SP 1065, 2008. *(overlapping Allan deviation.)*
19. **IEEE Std 952-2020** (and 952-1997), *Standard Specification Format
    Guide and Test Procedure for Single-Axis Interferometric Fiber Optic
    Gyros*, Annex C. *(noise density, bias instability, factor 0.664 — cited
    in `cal_math.c`.)* **[verify edition]**
20. **A. W. Fitzgibbon, M. Pilu, R. B. Fisher**, "Direct Least Square Fitting
    of Ellipses," *IEEE TPAMI*, 1999. *(algebraic conic fit.)*
21. **V. Renaudin, M. H. Afzal, G. Lachapelle**, "Complete Triaxis
    Magnetometer Calibration in the Magnetic Domain," *J. Sensors*, 2010.
    *(hard/soft-iron sphere & ellipsoid fits.)*
22. **I. D. Coope**, "Circle fitting by linear and nonlinear least squares,"
    *J. Optim. Theory Appl.*, 1993. *(algebraic sphere fit.)* **[verify]**
23. **G. H. Golub, C. F. Van Loan**, *Matrix Computations*, 4th ed., 2013.
    *(Gaussian elimination with partial pivoting.)*
24. **A. Chulliat et al.**, *The US/UK World Magnetic Model for 2025–2030*,
    NOAA Technical Report, NCEI/BGS, 2024. *(defining WMM document.)*
    **[verify exact title/authors of the 2025 report]**
25. **R. A. Langel**, "The Main Field," in *Geomagnetism* (J. A. Jacobs,
    ed.), Academic Press, 1987. *(Schmidt-normalized ALF recursion — cited
    in `wmm.c` via WMM Technical Note 28.)*
26. **NIMA TR8350.2**, *Department of Defense World Geodetic System 1984*,
    3rd ed. *(WGS-84 ellipsoid constants.)*
27. **N. J. Higham**, *Accuracy and Stability of Numerical Algorithms*,
    2nd ed., SIAM, 2002. *(Cancellation in expressions of the form
    $1-e^{-x}$, and why an accumulator's width is set by the smallest
    increment it must still register rather than by the precision of its
    output.)* **[verify]** — section numbers not checked against a copy.

---

*This document reflects the code at the release it ships with. When a
numerical routine changes, update the corresponding section and its
**As-implemented** note in the same commit.*
