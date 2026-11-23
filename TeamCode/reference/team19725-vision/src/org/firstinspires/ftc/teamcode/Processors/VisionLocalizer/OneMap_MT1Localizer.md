# Single-Map Pose Estimation

## 1. Frames and Known Conditions

- **Field frame {W}**: origin at the field center, x to the right, y forward, z up.
- **HIVE pivot**: axis along the y axis, passing through $p_{\text{pivot}} = (0,0,h)$, with unit direction $\mathbf{u} = (0,1,0)$.
- **Map**: stores the poses of our 8 AprilTags when the HIVE is in the **horizontal reference state** (denoted $\theta=0$).
- **Actual HIVE**: rotated by angle $\theta$ about the axis relative to the horizontal state.
- **Limelight output**: robot pose estimated with the horizontal map, denoted $T' = \begin{bmatrix} R' & p' \ 0 & 1 \end{bmatrix} $, where $p' = (x', y', z')^T$.
- **True pose**: actual robot pose, denoted $T = \begin{bmatrix} R & p \ 0 & 1 \end{bmatrix} $, where $p = (x, y, z)^T$.

***

## 2. Relation Between the Erroneous Pose and the True Pose

The homogeneous transform for a rotation about the y axis by angle $\phi$ is:

$$
T\_{\text{rot}}(\phi) =
\begin{bmatrix}
\cos\phi & 0 & \sin\phi & -h\sin\phi \\
0 & 1 & 0 & 0 \\
-\sin\phi & 0 & \cos\phi & h(1-\cos\phi) \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Derivation: rotation matrix $ R\_y(\phi) = \begin{bmatrix} \cos\phi & 0 & \sin\phi \ 0 & 1 & 0 \ -\sin\phi & 0 & \cos\phi \end{bmatrix} $, and the translation part is $ (I - R\_y(\phi)) p\_{\text{pivot}} = (-h\sin\phi, 0, h(1-\cos\phi))^T $.

From the earlier general derivation, the erroneous pose and the true pose satisfy:
$$
T' = T\_{\text{rot}}(\phi) , T
$$
where $ \phi = -\theta $ ($ \theta $ is the actual HIVE rotation angle). In other words, the botpose returned by Limelight is the true pose rotated about the pivot axis by $ \phi = -\theta $.

Conversely:
$$
T = T\_{\text{rot}}(-\phi) , T'
$$

***

## 3. Position Correction Formula

From $ T' = T\_{\text{rot}}(\phi) T $:
$$
p' = R\_y(\phi) , p + (-h\sin\phi, 0, h(1-\cos\phi))^T
$$

Expanded component-wise:
$$
\begin{aligned}
x' &= x \cos\phi + z \sin\phi - h \sin\phi \\
y' &= y \\
z' &= -x \sin\phi + z \cos\phi + h(1-\cos\phi)
\end{aligned}
$$

Conversely, recovering the true position $ p $ from $ p' $:
$$
p = R\_y(-\phi) \left( p' - (-h\sin\phi, 0, h(1-\cos\phi))^T \right)
$$
i.e.:
$$
\begin{aligned}
x &= x' \cos\phi - z' \sin\phi + h \sin\phi \\
y &= y' \\
z &= x' \sin\phi + z' \cos\phi + h(1-\cos\phi)
\end{aligned}
$$

***

## 4. Solving for $\phi $ Using the Robot Ground-Contact Constraint

The robot moves on the ground, so its true height is $z \approx 0$. Setting $z = 0$:
$$
x' \sin\phi + z' \cos\phi + h(1-\cos\phi) = 0
$$
Rearranged:
$$
x' \sin\phi + (z' - h) \cos\phi + h = 0
$$

Let $A = x' $, $B = z' - h$, $C = -h$; then:
$$
A \sin\phi + B \cos\phi = C
$$
This is a standard trigonometric equation whose **two solution families** are:
$$
\phi_1 = \alpha - \operatorname{atan2}(B, A),\qquad
\phi_2 = \pi - \alpha - \operatorname{atan2}(B, A),\qquad \alpha = \arcsin\!\left(\frac{C}{\sqrt{A^2+B^2}}\right)
$$
The two families always differ by $\pi - 2\alpha$, so normally only one family falls in the reasonable range. The HIVE maximum tilt is about $30^\circ$,
so filter by $|\phi| \le$ `hiveCellUpAngleDeg`; if both families fall inside the window, take the one with smaller $|\phi|$.
(Solvability precondition: $|C| \le \sqrt{A^2+B^2}$.)

> **Degenerate condition**: with the estimation model $F(\phi) = x'\sin\phi + (z'-h)\cos\phi + h$, we always have $F(0) = z'$,
> so when $|z'| \approx 0$ the position-path solution necessarily degenerates to $\phi = 0$ ($\theta = 0$), regardless of the true HIVE tilt.
> See [MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/MT1Localizer.md) Sec. 3.2-3 for the discrimination method (attitude-path cross-check).

After solving for $\phi$, substitute it into the position correction formula to get the true position.

***

## 5. Attitude Correction Formula

From ( R' = R\_y(\phi) R ):
$$
R = R\_y(-\phi) R'
$$
where $ R\_y(-\phi) = \begin{bmatrix} \cos\phi & 0 & -\sin\phi \ 0 & 1 & 0 \ \sin\phi & 0 & \cos\phi \end{bmatrix} $.

If only the 2D pose $ (x, y, \text{yaw}) $ is needed, yaw can be extracted from the corrected rotation matrix.
The first column of $R'$ is the direction of the robot x axis in the field frame (SDK convention: intrinsic yaw -> pitch -> roll,
so $R_{00} = \cos\psi'\cos P'$, $R_{10} = \sin\psi'\cos P'$, $R_{20} = -\sin P'$):
$$
\text{yaw} = \operatorname{atan2}\big(R_{10},\ \cos\phi\,R_{00} - \sin\phi\,R_{20}\big)
$$
> An earlier version of this document wrote $\text{yaw} \approx \operatorname{atan2}(R_{21}, R_{11})$, which is always 0 when the robot is level; that was a typo.

***

## 6. Summary Procedure

1. Get botpose $ (x', y', z') $ and attitude $ R' $ from Limelight.
2. Solve $ x' \sin\phi + (z' - h) \cos\phi + h = 0 $ for $\phi$ (i.e. $ -\theta $).
3. Compute the true position:
   $$
   \begin{aligned}
   x &= x' \cos\phi - z' \sin\phi + h \sin\phi \\
   y &= y' \\
   z &= x' \sin\phi + z' \cos\phi + h(1-\cos\phi)
   \end{aligned}
   $$
   Note: for an analytic root, $z \equiv 0$ is an **identity** ($z = C + h = 0$) and cannot serve as a quality check;
   use the attitude-path cross-check instead (see [MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/MT1Localizer.md) Sec. 3.2-3).
4. Compute the true attitude: $ R = R\_y(-\phi) R' $, and extract $\text{yaw}$ with the formula in Sec. 5.
5. Feed the corrected $ (x, y, \text{yaw}) $ into the EKF or use it directly.

With this, **a single horizontal-state map** adapts to arbitrary HIVE rotation angles, with no need to upload maps dynamically. Note: only tags of the **own-side HIVE** may be used; mixing red and blue HIVE tags makes the rotation axes inconsistent.

**Precondition (confirmed on the real robot): Limelight `snap to ground` must be set to `No`.** When that option is on,
Limelight clamps the estimated pose onto the ground plane (equivalent to $z' \equiv 0$),
the tilt information in the position path is completely erased, and this scheme always outputs $\theta = 0$.
