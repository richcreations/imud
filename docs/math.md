# imud — Mathematical Reference (as implemented)

**Document purpose.** This is an audit reference for the *actual numerical
methods executed by the imud source code*, at the release this file ships
with. It is deliberately **not** a derivation of the methods imud "should"
use, nor a restatement of the papers it draws on: every equation below was
transcribed from the code and is annotated with the function, variable, and
struct-field names so a reviewer can place each symbol against its
implementation. Where the code approximates, deviates from, or specializes
its cited source, an **As-implemented** note says so explicitly.

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

The rotation matrix `q_to_R()` (`fusion.c:136`) is

$$
R(q)=\begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y)\\
2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x)\\
2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}.
$$

The Hamilton product `q_mul()` ($c = a\otimes b$, `fusion.c:118`) uses the
standard scalar-first convention (Solà eq. 16).

**Units.** Gyroscope rad·s⁻¹; accelerometer m·s⁻²; magnetometer µT on the
wire, converted to **Gauss** (×0.01) inside the filter; angles rad; time s.
Standard gravity `G_MS2` $=g=9.80665\,\mathrm{m\,s^{-2}}$ (`fusion.c:42`).

**Gravity reference.** $g_{ref}=[0,0,1]$ (unit, NED, Z-down). The
accelerometer's *specific force* at rest reads $-g$ on Z; the filter works
with the **gravity direction** $z_a=-\widehat{a}_b$ (§4.7).

**As-implemented.** All filter state is single precision (`float`);
calibration fits and WMM are double precision. The quaternion is
renormalized (`q_normalize`, `fusion.c:126`) after every multiplicative
update, guarding unit-norm drift from float round-off.

**Source:** Solà 2017 *(code comment)*; Shuster 1993 for quaternion
conventions *(canonical)*.

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

`apply_mount_rot_if_set()` (`imu.c:222`) applies a fixed 3×3 board→body
matrix $R_{mount}=$ `cfg->mount_rot` in place, when `cfg->mount_set`:

$$ v \leftarrow R_{mount}\, v, \qquad v\in\{a_b,\ \omega_b,\ m_b\}. $$

Applied identically to accel, gyro, and magnetometer vectors. Accumulated in
double, stored back as float.

**As-implemented.** $R_{mount}$ is supplied by configuration (a fixed
installation rotation, e.g. yaw 180° for a stern-facing board); imud does not
estimate it. No orthonormalization is performed on the configured matrix.

### 3.2 Inertial calibration

`apply_imu_cal()` (`imu.c:141`), per axis $i$:

- **Gyro temperature compensation** (when `cal->has_gyro_temp`):
  $$ \omega_i \leftarrow \omega_i - c_i\,(T - T_{ref}), $$
  with $c_i=$ `gyro_temp_coeff[i]`, $T=$ `temp_c`,
  $T_{ref}=$ `gyro_temp_ref_c` (fit in §12.6).
- **Accel offset/scale** (when `cal->has_accel`):
  $$ a_i \leftarrow (a_i - o_i)\, s_i, $$
  with $o_i=$ `accel_offset[i]`, $s_i=$ `accel_scale[i]`.

### 3.3 Magnetometer hard/soft-iron

`apply_mag_cal()` (`imu.c:159`), when `cal->has_mag`:

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

Nominal state (`mekf_t`, `fusion.h:31`):
- $q$ — unit quaternion, body→NED (`q[4]`).
- $b$ — gyro bias, rad·s⁻¹ (`bias[3]`).

Error state (never stored explicitly; the axis of `P`):

$$ \delta x = [\,\delta\theta\ (3)\ \mid\ \delta b\ (3)\,] \in \mathbb{R}^6, $$

$\delta\theta$ = small-angle rotation error (rad), $\delta b$ = bias error.
Covariance $P\in\mathbb{R}^{6\times6}$ (`P[6][6]`), symmetric PD; top-left
3×3 is attitude-error covariance (rad²), bottom-right 3×3 is bias-error
covariance ((rad·s⁻¹)²).

The nominal/error relationship is the **right** (local) perturbation
$q_{true} = \hat q \otimes \delta q(\delta\theta)$, established by the sign of
the measurement Jacobian in §4.5.

**Source:** Solà 2017 §5.4 *(code comment)*; Markley & Crassidis 2014 §6.1;
Trawny & Roumeliotis 2005 *(canonical)*.

### 4.2 Initialization — `mekf_init()` (`fusion.c:400`)

Nominal: $q=[1,0,0,0]$, $b=$ `gyro_bias_init` (from the startup still window,
§10; may be 0).

Initial covariance (diagonal):
$$ P_{0}[0{:}3] = (0.175)^2\ \mathrm{rad^2}\ (\approx10°),\qquad
   P_{0}[3{:}6] = (0.001)^2\ (\mathrm{rad\,s^{-1}})^2. $$

Discrete process-noise variances, step $dt=1/\text{ODR}$:
$$ Q_g = N_g^2\,dt,\qquad Q_b = N_b^2\,dt, $$
with $N_g=$ `mekf_gyro_noise`, $N_b=$ `mekf_gyro_bias` (datasheet noise
densities). Stored as `f->Qg`, `f->Qb`.

Measurement-noise variances:
$$ R_a = \left(\frac{N_a}{g}\right)^2 \text{ODR},\qquad
   R_m = N_m^2\,f_{s,\text{mag}}, $$
`f->Ra` (`fusion.c:448`) in normalized (gravity-direction) units,
`f->Rm` (`:449`) in Gauss², with $N_a=$ `mekf_accel_noise`,
$N_m=$ `mekf_mag_noise`, $f_{s,\text{mag}}=$ `mag_odr_hz`.

Derived thresholds: accel skip band
$[\,1-s_k,\ 1+s_k\,]$ with $s_k=$ `accel_skip_thresh`;
$mag\_reject\_sq = (\text{mag\_reject\_gauss})^2$; convergence threshold
$conv\_thresh = 3\,(0.5°\text{ in rad})^2$ (`:471`); m_ref EMA gain
$mref\_alpha = 1/(\tau_{mref}\, f_{s,\text{mag}})$ with $\tau_{mref}=300$ s
(`:467`).

**As-implemented.** $Q_g,Q_b$ are *per nominal step*; during prediction they
are rescaled by the actual $dt$ (§4.4). The bias process is modeled as a
random walk whose density $N_b$ is a **tuning constant deliberately held
above** the measured in-run bias instability (see `docs/capture.md`); it is
not driven by the Allan-variance characterization of §12.5.

**Source:** Solà 2017 §4.1 for discrete process noise *(code comment)*.

### 4.3 Alignment — `mekf_align()` (`fusion.c:494`)

Deterministic initial attitude from one static accel+mag pair (a
tilt-then-heading decomposition, TRIAD-family).

**Tilt from accelerometer.** With $\widehat g_b=-a_b/\lVert a_b\rVert$ the
gravity direction in body (guard: reject if $\lVert a_b\rVert<0.5g$):

$$ \phi = \operatorname{atan2}(\widehat g_{b,y},\ \widehat g_{b,z}),\qquad
   \theta = \operatorname{atan2}\!\big(-\widehat g_{b,x},\
   \sqrt{\widehat g_{b,y}^2+\widehat g_{b,z}^2}\big). $$

Tilt quaternion $q_{tilt}=q_\theta\otimes q_\phi$ built directly from
half-angles (`fusion.c:508`).

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
the in-code comment at `fusion.c:519`.) The initial `m_ref` inherits any
alignment tilt error in its magnitude/dip, which the quiescence-gated EMA
(§4.8) and the WMM invariants (§4.9) later remove; its horizontal
**direction** is the heading datum and is never subsequently adapted.

**Source:** tilt/heading coarse alignment — Farrell 2008 §10; TRIAD lineage
Shuster & Oh 1981 *(canonical)*.

### 4.4 Prediction — `mekf_predict()` (`fusion.c:551`)

Per IMU sample, with measured interval $dt$ (from hardware timestamps, §11;
falls back to nominal $1/\text{ODR}$).

**Bias-corrected rate:** $\omega = s.\text{gyro} - b$.

**Quaternion propagation** by the exponential map (`q_from_rotvec`,
`fusion.c:149`):
$$ q \leftarrow q \otimes \exp\!\big(\tfrac12\,[\,0,\ \omega\,dt\,]\big),
\qquad
\exp(\phi) = \Big[\cos\tfrac{|\vartheta|}{2},\
\tfrac{\sin(|\vartheta|/2)}{|\vartheta|}\,\vartheta\Big], $$
$\vartheta=\omega\,dt$, with the small-angle branch
$\delta q\approx[1,\tfrac12\vartheta]$ for $|\vartheta|<10^{-7}$. Renormalized
after.

**Covariance propagation** $P \leftarrow \Phi P \Phi^\top + Q_d$, first-order
discrete transition (`fusion.c:588`):

$$
\Phi = \begin{bmatrix} I_3 - [\omega]_\times\,dt & -I_3\,dt \\
0_3 & I_3 \end{bmatrix},\qquad
Q_d = \operatorname{diag}\!\big(Q_g\tfrac{dt}{dt_0} I_3,\
Q_b\tfrac{dt}{dt_0} I_3\big),
$$

$[\omega]_\times$ the skew-symmetric cross-product matrix; the noise is
rescaled by $dt/dt_0$ ($dt_0=$ nominal step) so variance grows linearly with
the real interval. $P$ is symmetrized after (`:610`). Convergence flag
$\operatorname{tr}(P[0{:}3]) < conv\_thresh$ (`:619`).

**As-implemented.** Prediction uses the plain $\Phi P\Phi^\top+Q$ form (not
Joseph — the Joseph stabilization is applied on the *updates*, §4.5).
$\Phi$ is a first-order (zero-hold) approximation of $\exp(F_c\,dt)$; valid
because $\lVert\omega\rVert dt \ll 1$ at supported ODRs. The
$[\omega]_\times$ sign in $\Phi_{[0:3,0:3]}$ is that of the right-perturbation
error convention.

**Source:** Solà 2017 eq. 259 (integration), eq. 268 (covariance)
*(code comment)*.

### 4.5 Generic vector measurement update — `eskf_update()` (`fusion.c:185`)

Measurement of a known NED reference observed in body: predicted $h$, actual
$z$, isotropic noise $R_{noise}$, gate `chi2_gate`.

**Jacobian** (3×6), attitude block only:
$$ H = \big[\, [h]_\times \ \big|\ 0_3 \,\big],\qquad
[h]_\times=\begin{bmatrix}0&-h_2&h_1\\ h_2&0&-h_0\\ -h_1&h_0&0\end{bmatrix}. $$

**Innovation covariance and gain:**
$$ S = H P H^\top + R_{noise} I_3,\qquad K = P H^\top S^{-1}, $$
$S^{-1}$ by Cramer's rule (`m33_inv`, `fusion.c:73`; singular ⇒ skip).

**Innovation** $\nu = z - h$, with **robust (Huber-style) gating** on the
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
   b \leftarrow b + \delta b, $$
$\delta\theta=\delta x[0{:}3]$, $\delta b=\delta x[3{:}6]$; quaternion
renormalized.

**Covariance — Joseph form** (`fusion.c:284`):
$$ P \leftarrow (I-KH)\,P\,(I-KH)^\top + R_{noise}\,K K^\top, $$
then symmetrized. (Isotropic $R$ ⇒ $KRK^\top=R_{noise}KK^\top$.)

**As-implemented / audit note.** The error is *reset* into the quaternion
each update but $P$ is **not** rotated by the reset Jacobian
$G=I-\tfrac12[\delta\theta]_\times$ (the common first-order reset). At the
per-sample correction magnitudes here $G\approx I$; the omission is a
deliberate first-order simplification. The Huber cap replaces a hard $\chi^2$
reject so that wave-orbital acceleration (which swings gravity *direction*
while keeping $|a|\approx g$) deweights rather than starves the filter; only
gross outliers ($d^2>9\gamma$) are rejected. Because $S$ contains $P$, the cap
is naturally inactive during acquisition (large $P$) and engages only once
confident — no explicit convergence gate needed.

**Source:** EKF update Kalman 1960; Joseph form Bucy & Joseph 1968; robust
innovation capping Huber 1964; Mahalanobis gating Bar-Shalom et al. 2001
*(canonical)*. Jacobian sign Solà 2017 §7 *(code comment)*.

### 4.6 Scalar heading update — `eskf_update_yaw()` (`fusion.c:337`)

Heading-only correction (a 1-D measurement). Innovation $y$ (rad, wrapped to
$\pm\pi$), variance $R_{noise}$. The Jacobian projects the error onto the
NED-down axis expressed in body:
$$ H = \big[\,-R[2][:]\ \mid\ 0_3\,\big]\in\mathbb{R}^{1\times6}, $$
$R[2][:]$ = third row of $R(q)$. Then
$$ S = H P H^\top + R_{noise},\qquad K = P H^\top/S, $$
with the same Huber policy on $d^2=y^2/S$ against
$\gamma_\psi=6.63$ ($\chi^2_1$ 99%), rank-1 Joseph covariance update.

**Source:** scalar-measurement EKF specialization of §4.5 *(canonical)*.

### 4.7 Accelerometer update — `mekf_update_accel()` (`fusion.c:628`)

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

**As-implemented.** `Ra_scale` is set to 4 by the fusion thread while engine
vibration is detected (§9) — vibration is high-frequency, near-zero-mean, so
deweighting (not gating) is correct. Wave *direction* disturbance is handled
by the Huber cap (§4.5), **not** by magnitude-based deweighting, which
benchmarking showed rectifies wave-phase-correlated error into an
attitude/bias offset. The centripetal model assumes zero leeway and no
vertical velocity.

**Source:** specific-force / coordinated-turn model Titterton & Weston 2004;
Farrell 2008 *(canonical)*.

### 4.8 Magnetometer update — `mekf_update_mag()` (`fusion.c:697`)

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
Otherwise call the full vector update §4.5 with $(h,z,R_{m,n},11.34)$. A near-
vertical field ($m^h<0.2|h_{raw}|$) carries no heading information and is
skipped.

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
imud-specific design (see in-code rationale `fusion.c:781`).

### 4.9 WMM reference invariants — `mekf_set_mref_invariants()` (`fusion.c:474`)

Given WMM horizontal magnitude $H$ and vertical $Z$ (Gauss) at a known
position (from §13), rescale `m_ref` to those invariants while preserving
horizontal direction: with $m_h=\sqrt{m_{ref,x}^2+m_{ref,y}^2}$ and
$s=H/m_h$,
$$ m_{ref,x}\!\leftarrow s\,m_{ref,x},\quad m_{ref,y}\!\leftarrow s\,m_{ref,y},
   \quad m_{ref,z}\!\leftarrow Z. $$
No-op before alignment or if $H\le0$.

**Source:** imud-specific (WMM supplies the invariants of §13).

### 4.10 State extraction — `mekf_get_state()` (`fusion.c:1030`)

Euler angles from $R(q)$ (NED 3-2-1 aerospace):
$$ \theta=\arcsin(-R[2][0]),\quad \phi=\operatorname{atan2}(R[2][1],R[2][2]),
   \quad \psi=\operatorname{atan2}(R[1][0],R[0][0]). $$
Magnetic heading $\psi$ wrapped to $[0,360°)$. Attitude covariance
(`cov[9]`) = $P[0{:}3,0{:}3]$; bias variance = $\operatorname{diag}
P[3{:}6]$; `quiescence` $=q_{quiet}$. `rate_of_turn` is left 0 here and filled
by the fusion thread (§5).

**Source:** quaternion→Euler, Diebel 2006 *(canonical)*.

### 4.11 Reconfigure — `mekf_reconfigure()` (`fusion.c:1005`)

Recomputes $Q_g,Q_b,R_a,R_m$, the skip band, `mag_reject_sq`, and
`mag_yaw_only` from a new config on hot-reload; $q$, $P$, $b$, $dt$ untouched
(the filter keeps running).

---

## 5. Euler rates and rate of turn

Computed in the fusion thread (`imu.c:1004`–`1051`) from the bias-corrected
body rate $\omega = s.\text{gyro}-b$ and the current Euler angles, using the
inverse of the 3-2-1 kinematic relation.

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

## 6. Heave estimator — `heave_update()` (`fusion.c:880`)

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

## 7. Sea-state statistics — `seastate_*` (`fusion.c:907`–`1003`)

Windowed spectral moments over the heave, roll, and pitch oscillations via
exponentially-weighted mean/variance pairs — no FFT, no sample storage.

**EW mean/variance recursion** (`ew_stat`, `fusion.c:937`), $\alpha=dt/\tau$:
$$ \mu \leftarrow \mu + \alpha\,d,\qquad
   \sigma^2 \leftarrow \sigma^2 + \alpha\big((1-\alpha)\,d^2 - \sigma^2\big),
   \qquad d = x-\mu. $$
Applied to six signals: heave, heave-rate, roll, roll-rate, pitch,
pitch-rate. Fed **only while heave is settled** (`seastate_update`,
`fusion.c:944`).

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

## 8. Compass-health diagnostics — `mekf_update_mag()` (`fusion.c:749`)

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

**Source:** exponential moving average, standard *(canonical)*.

---

## 9. Engine-vibration detector — `ism_reader_thread()` (`imu.c`)

EMA of squared specific-force deviation from 1 g, $\alpha=0.01$
($\tau\approx1$ s at 100 Hz burst rate), per accepted sample:
$$ e \leftarrow e + \alpha\big((\lVert a\rVert - g)^2 - e\big),\qquad
   \text{engine\_on} = \big[\,e > \text{engine\_vibration\_g2}\,\big]. $$
`engine_on` raises the accel noise ×4 and widens the skip band (§4.7).

**Source:** EMA threshold detector, standard *(canonical)*.

---

## 10. Startup gyro-bias estimation — `fusion_thread()` (`imu.c:729`)

Mean of the gyro over a still window ($N=$ `gyro_bias_sec`·ODR samples):
$$ \hat b_k = \frac1N\sum_{i=1}^{N} \omega_{i,k}. $$
A per-axis motion check computes the sample standard deviation
$\sigma_k=\sqrt{\overline{\omega^2}-\bar\omega^2}$; if
$\max_k\sigma_k > 0.00873$ rad·s⁻¹ ($0.5°$·s⁻¹) the window is **doubled once**
(a longer average spans more wave cycles). The result seeds `mekf_init`
(§4.2); the MEKF refines it online regardless.

**Source:** sample mean/variance, elementary *(canonical)*.

---

## 11. Timestamp anchoring and per-sample dt — `imu.c:173`–`207`

A single anchor $(\text{chip\_ticks}, \text{wall\_ns}, \text{tai\_ns},
\text{gen})$ maps the sensor's free-running counter to system time. Per
sample (`chip_to_wall`, `imu.c:197`), with `tick_ns` the counter period:
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

### 12.1 Linear solver — `gauss4()` (`cal_math.c:24`)

Gaussian elimination with **partial pivoting** on the augmented $4\times5$
system, then back-substitution. Returns $-1$ if any pivot $<10^{-12}$.
Double precision.

**Source:** Golub & Van Loan 2013 §3.4 *(canonical)*.

### 12.2 Hard-iron sphere fit — `sphere_fit()` (`cal_math.c:125`)

Algebraic (linearized) sphere fit. The sphere
$(x-c_x)^2+(y-c_y)^2+(z-c_z)^2=r^2$ is linearized as
$$ x^2+y^2+z^2 = 2c_x x + 2c_y y + 2c_z z + (r^2 - \lVert c\rVert^2), $$
and the normal equations $A^\top A\,p = A^\top b$ are accumulated
incrementally (`sphere_add`, `cal_math.c:58`) over sums
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

### 12.3 Soft-iron 2-D ellipse fit — `ellipse_fit()` (`cal_math.c:78`)

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

### 12.4 Heading-circle coverage — `cal_cov_mark()` (`cal_math.c:151`)

Guided-swing coverage: the heading circle is split into `nsec` sectors; the
sample $(x,y)$ relative to center $(c_x,c_y)$ is binned by
$$ s = \Big\lfloor \frac{\operatorname{atan2}(y-c_y,\,x-c_x)\bmod 2\pi}
{2\pi/\text{nsec}} \Big\rfloor. $$
`cal_cov_count` sums the marked sectors. Pure bookkeeping (no fit).

### 12.5 Allan variance — `allan_deviation()` (`cal_math.c:170`)

**Overlapping** Allan deviation. With the cumulative integral
$\theta[k]=\sum_{j<k} x_j\,dt$ and octave cluster lengths $m=1,2,4,\dots$
($\tau=m\,dt$):
$$ \sigma^2(\tau) = \frac{1}{2\tau^2\,(N-2m+1)}
   \sum_{k=0}^{N-2m}\big(\theta[k{+}2m]-2\theta[k{+}m]+\theta[k]\big)^2. $$

**Characterization** (`allan_characterize`, `cal_math.c:203`): white-noise
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

### 12.6 Gyro-bias / temperature fit — `gyro_temp_fit()` (`cal_math.c:227`)

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
| $P$ | 6×6 error covariance $[\delta\theta\mid\delta b]$ | `mekf_t.P[6][6]` |
| $R(q)$ | body→NED rotation matrix | `q_to_R()` |
| $Q_g,Q_b$ | process-noise variances / step | `mekf_t.Qg, .Qb` |
| $R_a,R_m$ | accel / mag meas. noise var | `mekf_t.Ra, .Rm` |
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

---

*This document reflects the code at the release it ships with. When a
numerical routine changes, update the corresponding section and its
**As-implemented** note in the same commit.*
