# Physics-Informed Prediction and Fan Planning for Radon Ventilation

## 1. Project Situation

The target site is an underground pump room inside a subway station. The room contains
equipment that pumps out underground water. Because the space is underground and is
connected to water, soil, cracks, drainage paths, and service voids, the radon concentration
can become high when the space is not ventilated.

The room has one powerful ventilator. At present, this ventilator is manually toggled.
The operational goal is not to keep the room continuously safe at all times. Instead, the
goal is to make the room safe during scheduled worker entry windows while avoiding
unnecessary fan operation outside those windows.

Let the scheduled worker entry windows be

$$
\mathcal{W}
= \bigcup_{i=1}^{N}
\left[t_i^{\mathrm{in}},\, t_i^{\mathrm{out}}\right].
$$

For each interval

$$
\left[t_i^{\mathrm{in}},\, t_i^{\mathrm{out}}\right],
$$

the controller must make the radon concentration satisfy

$$
R(t) \le R_{\mathrm{safe}},
\qquad
\forall t \in
\left[t_i^{\mathrm{in}},\, t_i^{\mathrm{out}}\right].
$$

Outside these windows,

$$
t \notin \mathcal{W},
$$

high radon concentration may be operationally acceptable, provided that it does not prevent
the system from preparing the room safely before the next scheduled entry.

The key difficulty is that the fan does not reduce radon instantaneously. If the fan is turned
on at time

$$
t = t_i^{\mathrm{in}},
$$

the room may still be unsafe when workers enter. The fan must therefore be activated at an
earlier time

$$
t_i^{\mathrm{on}} < t_i^{\mathrm{in}},
$$

such that the predicted concentration satisfies

$$
\widehat{R}(t_i^{\mathrm{in}})
\le
R_{\mathrm{safe}} - m_{\mathrm{safety}},
$$

where

$$
m_{\mathrm{safety}} > 0
$$

is a conservative safety margin.

The project is therefore not only a prediction problem. It is a two-layer engineering
problem:

$$
\boxed{
\text{Prediction}
\quad + \quad
\text{Planning}
}
$$

The prediction layer learns how this specific room accumulates and clears radon. The
planning layer uses that prediction model to choose a binary fan toggle schedule that makes
the room safe during worker-entry windows while minimizing fan runtime and avoiding
excessive switching.

The fan command is binary:

$$
u(t) \in \{0,1\},
$$

where

$$
u(t)=0
\quad \text{means fan off,}
$$

and

$$
u(t)=1
\quad \text{means fan on.}
$$

The desired system is not merely a detector and not merely a timer. It is a predictive
planner:

$$
\left[
\text{current radon state}
\right]
+
\left[
\text{weather and room conditions}
\right]
+
\left[
\text{worker schedule}
\right]
\longrightarrow
\left[
\text{optimized fan schedule}
\right].
$$

## 2. Available Data as a Time-Dependent Vector

At each time

$$
t,
$$

we assume that the system observes the underground room, the ventilator, and the outside
atmosphere. A practical measurement vector is

$$
\mathbf{z}(t)
=
\begin{bmatrix}
R_{\mathrm{room}}(t) \\
T_{\mathrm{room}}(t) \\
H_{\mathrm{room}}(t) \\
P_{\mathrm{room}}(t) \\
Q_{\mathrm{fan}}(t) \\
\Delta P_{\mathrm{fan}}(t) \\
T_{\mathrm{out}}(t) \\
H_{\mathrm{out}}(t) \\
P_{\mathrm{out}}(t)
\end{bmatrix}.
$$

The entries are:

$$
R_{\mathrm{room}}(t)
:
\text{measured radon concentration in the pump room},
$$

$$
T_{\mathrm{room}}(t)
:
\text{room temperature},
$$

$$
H_{\mathrm{room}}(t)
:
\text{room relative humidity},
$$

$$
P_{\mathrm{room}}(t)
:
\text{room pressure},
$$

$$
Q_{\mathrm{fan}}(t)
:
\text{fan volumetric flow rate},
$$

$$
\Delta P_{\mathrm{fan}}(t)
:
\text{fan pressure differential},
$$

$$
T_{\mathrm{out}}(t)
:
\text{outside temperature from weather API},
$$

$$
H_{\mathrm{out}}(t)
:
\text{outside humidity from weather API},
$$

and

$$
P_{\mathrm{out}}(t)
:
\text{outside atmospheric pressure from weather API}.
$$

The control input is

$$
u(t) \in \{0,1\}.
$$

The effective fan-driven ventilation can be written as

$$
Q_{\mathrm{eff}}(t)
= u(t)\,Q_{\mathrm{fan}}(t).
$$

If the measured fan flow is not available directly, it may be approximated by a learned
relationship

$$
Q_{\mathrm{eff}}(t)
=
u(t)\,
\Phi_Q
\left(
\Delta P_{\mathrm{fan}}(t),
P_{\mathrm{room}}(t)-P_{\mathrm{out}}(t),
T_{\mathrm{room}}(t)-T_{\mathrm{out}}(t),
H_{\mathrm{room}}(t),
H_{\mathrm{out}}(t)
\right).
$$

The dataset is a time-continuum, or a sampled approximation of a time-continuum:

$$
\mathcal{D}
=
\left\{
\mathbf{z}(t), u(t)
\right\}_{t \in [0,T]}.
$$

In sampled form,

$$
\mathcal{D}_K
=
\left\{
\mathbf{z}_k,u_k,t_k
\right\}_{k=0}^{K},
$$

where

$$
\mathbf{z}_k = \mathbf{z}(t_k),
\qquad
u_k = u(t_k).
$$

If the radon sensor reports a moving average rather than instantaneous concentration, the
observed signal may be

$$
Y_{\tau}(t)
=
\frac{1}{\tau}
\int_{t-\tau}^{t}
R(s)\,ds,
$$

where

$$
\tau = 4\ \mathrm{hours}
$$

or

$$
\tau = 24\ \mathrm{hours}.
$$

This distinction is important. The controller needs to reason about the hidden instantaneous
or short-window concentration

$$
R(t),
$$

even if the sensor reports

$$
Y_{\tau}(t).
$$

## 3. Engineering Problem Definition: Prediction + Planning

After defining the measured vector

$$
\mathbf{z}(t),
$$

the engineering problem should be written as two coupled mathematical problems:

$$
\boxed{
\mathcal{P}_1:
\text{predict radon dynamics}
}
\qquad
\boxed{
\mathcal{P}_2:
\text{plan the fan toggle sequence}
}
$$

The first problem learns a predictive model. The second problem exploits that predictive
model to choose

$$
u(t)\in\{0,1\}.
$$

The worker-entry plan is also binary:

$$
w(t)\in\{0,1\},
$$

where

$$
w(t)=1
\quad
\text{means workers are scheduled to be inside the room,}
$$

and

$$
w(t)=0
\quad
\text{means no scheduled worker entry.}
$$

In sampled form:

$$
w_k = w(t_k),
\qquad
u_k = u(t_k),
\qquad
R_k = R(t_k).
$$

The complete operational map is therefore:

$$
\boxed{
\left(
\mathcal{D}_{0:k},
\mathbf{z}_{k:k+H}^{+},
w_{k:k+H}
\right)
\longrightarrow
u_{k:k+H}^{\star}
}
$$

where

$$
\mathcal{D}_{0:k}
=
\left\{
\mathbf{z}_j,u_j,y_j
\right\}_{j=0}^{k}
$$

is the historical data,

$$
\mathbf{z}_{k:k+H}^{+}
$$

is the future or forecasted exogenous input over the planning horizon, and

$$
u_{k:k+H}^{\star}
$$

is the optimized fan plan.

### 3.1 Problem 1: Predict

The prediction problem is to learn a model

$$
\mathcal{M}_{\theta}
$$

that maps current state, future exogenous variables, and a proposed fan sequence into a
future radon trajectory:

$$
\boxed{
\widehat{R}_{k:k+H}
=
\mathcal{M}_{\theta}
\left(
\widehat{\mathbf{x}}_k,
\mathbf{z}_{k:k+H}^{+},
u_{k:k+H}
\right)
}
$$

where

$$
\widehat{\mathbf{x}}_k
=
\begin{bmatrix}
\widehat{R}_k\\
\widehat{\mathbf{h}}_k
\end{bmatrix}
$$

is the estimated current state, including both radon concentration and latent room state.

The state estimate is obtained from past data:

$$
\widehat{\mathbf{x}}_k
=
\mathcal{E}_{\theta}
\left(
\mathcal{D}_{0:k}
\right).
$$

The model may output a deterministic forecast:

$$
\widehat{R}_{k+\ell}
\approx
R_{k+\ell},
\qquad
\ell=1,\ldots,H,
$$

or, preferably for safety, a probabilistic forecast:

$$
p_{\theta}
\left(
R_{k+\ell}
\mid
\mathcal{D}_{0:k},
\mathbf{z}_{k:k+\ell}^{+},
u_{k:k+\ell}
\right).
$$

Thus the prediction task is:

$$
\boxed{
\theta^{\star}
=
\arg\min_{\theta}
\mathcal{L}_{\mathrm{pred}}(\theta)
}
$$

with a typical objective:

$$
\mathcal{L}_{\mathrm{pred}}(\theta)
=
\sum_{k}
\sum_{\ell=1}^{H}
\left\|
\widehat{y}_{k+\ell\mid k,\theta}
-
y_{k+\ell}
\right\|^2
+
\mathcal{L}_{\mathrm{phys}}(\theta)
+
\mathcal{L}_{\mathrm{reg}}(\theta).
$$

The prediction model is not the final product by itself. It is the simulator used by the fan
planner.

### 3.2 Problem 2: Plan

The planning problem is to choose a binary fan sequence:

$$
\boxed{
u_{k:k+H}^{\star}
=
\left(
u_k^{\star},
u_{k+1}^{\star},
\ldots,
u_{k+H}^{\star}
\right),
\qquad
u_j^{\star}\in\{0,1\}.
}
$$

The planner uses the learned prediction model:

$$
\widehat{\mathbf{x}}_{j+1}
=
F_{\theta}
\left(
\widehat{\mathbf{x}}_j,
\mathbf{z}_j^{+},
u_j
\right),
\qquad
j=k,\ldots,k+H-1.
$$

It minimizes fan use, switching, and safety violations:

$$
\boxed{
\min_{u_{k:k+H}}
\sum_{j=k}^{k+H}
c_Eu_j
+
c_{\Delta}
\sum_{j=k}^{k+H}
\left|
u_j-u_{j-1}
\right|
+
c_{\xi}
\sum_{j=k}^{k+H}
\xi_j
}
$$

subject to:

$$
u_j\in\{0,1\},
\qquad
\xi_j\ge 0,
$$

and the worker-entry safety constraint:

$$
\boxed{
w_j=1
\quad\Longrightarrow\quad
R_j
\le
R_{\mathrm{safe}}
-
m_{\mathrm{safety}}
+
\xi_j.
}
$$

Equivalently:

$$
w_j
\left(
R_j
-
R_{\mathrm{safe}}
+
m_{\mathrm{safety}}
\right)
\le
\xi_j.
$$

For probabilistic safety:

$$
\boxed{
w_j=1
\quad\Longrightarrow\quad
Q_{1-\alpha}
\left[
R_j
\mid
\mathcal{D}_{0:k},
\mathbf{z}_{k:j}^{+},
u_{k:j}
\right]
\le
R_{\mathrm{safe}}.
}
$$

The fan should also avoid excessive toggling. Define the switch indicator:

$$
s_j
=
\left|u_j-u_{j-1}\right|,
\qquad
s_j\in\{0,1\}.
$$

A minimum debounce or dwell time of

$$
N_{\mathrm{debounce}}
$$

steps can be enforced by:

$$
\boxed{
\sum_{r=j}^{j+N_{\mathrm{debounce}}-1}
s_r
\le
1,
\qquad
j=k,\ldots,k+H-N_{\mathrm{debounce}}+1.
}
$$

This means that once the fan toggles, no second toggle is allowed within the debounce
window.

The planner is therefore not a second neural model. It is a constrained binary optimization
problem wrapped around the prediction model:

$$
\boxed{
\text{learn } \mathcal{M}_{\theta}
\quad
\text{then optimize } u_{k:k+H}.
}
$$

In control language, this is a mixed-integer model predictive control problem:

$$
\boxed{
\text{UDE prediction model}
\quad + \quad
\text{binary MPC fan planner}.
}
$$

## 4. Why Conventional CFD Is Not the Right Starting Point

A full computational fluid dynamics model would try to solve the local concentration field

$$
c(\mathbf{x},t),
\qquad
\mathbf{x} \in \Omega,
$$

where

$$
\Omega \subset \mathbb{R}^3
$$

is the three-dimensional pump room domain.

A classical advection-diffusion-reaction PDE for radon would be

$$
\frac{\partial c(\mathbf{x},t)}{\partial t}
=
-\nabla \cdot
\left(
\mathbf{v}(\mathbf{x},t)c(\mathbf{x},t)
\right)
+
\nabla \cdot
\left(
D(\mathbf{x},t)\nabla c(\mathbf{x},t)
\right)
+
s(\mathbf{x},t)
-
\lambda_{\mathrm{Rn}}c(\mathbf{x},t).
$$

Here:

$$
c(\mathbf{x},t)
:
\text{radon concentration field},
$$

$$
\mathbf{v}(\mathbf{x},t)
:
\text{air velocity field},
$$

$$
D(\mathbf{x},t)
:
\text{effective turbulent diffusion or mixing coefficient},
$$

$$
s(\mathbf{x},t)
:
\text{radon source term},
$$

and

$$
\lambda_{\mathrm{Rn}}
:
\text{radon decay constant}.
$$

For radon-222,

$$
\lambda_{\mathrm{Rn}}
=
\frac{\ln 2}{T_{1/2}},
$$

with

$$
T_{1/2} \approx 3.8\ \mathrm{days}.
$$

The mathematical structure is simple. The practical identification is not. A CFD model would
need accurate geometry, boundary conditions, inlet paths, leakage paths, pressure gradients,
water-surface source behavior, fan curves, equipment blockage, local turbulence, thermal
stratification, and sensor placement effects.

The hardest quantities are exactly the ones that matter most:

$$
\mathbf{v}(\mathbf{x},t),
\qquad
D(\mathbf{x},t),
\qquad
s(\mathbf{x},t),
\qquad
\partial\Omega_{\mathrm{leak}},
\qquad
\partial\Omega_{\mathrm{inflow}}.
$$

The room may have multiple uncontrolled air inflow paths. The fan may induce different
effective flow patterns depending on door state, pressure difference, outside weather, humidity,
and water-pump operation. Therefore, a first-principles CFD model is likely to be expensive,
slow to calibrate, and brittle under real operating conditions.

The better mathematical framing is:

$$
\text{keep the conservation law,}
\qquad
\text{learn the unknown effective terms.}
$$

That leads to the chosen option.

## 5. Prediction Model: Universal Differential Equation / Grey-Box Physics Model

The recommended approach is a Universal Differential Equation, also called a grey-box
scientific machine learning model.

The idea is:

$$
\text{known physics}
+
\text{learned unknown terms}
=
\text{usable predictive model}.
$$

Instead of attempting to reconstruct the full spatial PDE, we integrate the PDE over the room
volume and obtain a reduced model for the room-average radon concentration.

Define the room volume:

$$
V = |\Omega|.
$$

Define the room-average radon concentration:

$$
R(t)
=
\frac{1}{V}
\int_{\Omega}
c(\mathbf{x},t)\,d\mathbf{x}.
$$

Integrating the PDE over

$$
\Omega
$$

gives

$$
\frac{d}{dt}
\int_{\Omega}
c(\mathbf{x},t)\,d\mathbf{x}
=
\int_{\Omega}
s(\mathbf{x},t)\,d\mathbf{x}
-
\int_{\partial\Omega}
\left(
\mathbf{v}c
\right)
\cdot \mathbf{n}\,d\Gamma
+
\int_{\partial\Omega}
D\nabla c\cdot\mathbf{n}\,d\Gamma
-
\lambda_{\mathrm{Rn}}
\int_{\Omega}
c\,d\mathbf{x}.
$$

This can be reduced to an effective mass-balance model:

$$
\frac{dR}{dt}
=
S_{\mathrm{eff}}(t)
-
A_{\mathrm{eff}}(t)R(t)
-
\lambda_{\mathrm{Rn}}R(t)
+
B_{\mathrm{in}}(t).
$$

Where:

$$
S_{\mathrm{eff}}(t)
=
\frac{1}{V}
\int_{\Omega}
s(\mathbf{x},t)\,d\mathbf{x}
$$

is the effective room-average radon generation term.

$$
A_{\mathrm{eff}}(t)
$$

is the effective removal rate due to ventilation, leakage, mixing, and pressure-driven exchange.

$$
B_{\mathrm{in}}(t)
$$

is any radon contribution from incoming air. In many settings this may be small compared with
underground radon generation, but it should remain in the model until data proves it negligible.

The problem is that

$$
S_{\mathrm{eff}}(t),
\qquad
A_{\mathrm{eff}}(t),
\qquad
B_{\mathrm{in}}(t)
$$

are unknown, nonlinear, and site-specific. This is where ML is used.

We write:

$$
\frac{dR}{dt}
=
S_{\theta}(\mathbf{z}(t),\mathbf{h}(t))
-
A_{\theta}(\mathbf{z}(t),u(t),\mathbf{h}(t))R(t)
-
\lambda_{\mathrm{Rn}}R(t)
+
B_{\theta}(\mathbf{z}(t),u(t),\mathbf{h}(t)).
$$

The neural network does not replace the physical equation. It only represents the unknown
terms:

$$
S_{\theta}
\approx
S_{\mathrm{eff}},
$$

$$
A_{\theta}
\approx
A_{\mathrm{eff}},
$$

and

$$
B_{\theta}
\approx
B_{\mathrm{in}}.
$$

The vector

$$
\mathbf{h}(t)
$$

is an optional latent state. It represents slow hidden conditions that are not directly measured:

$$
\mathbf{h}(t)
=
\begin{bmatrix}
h_1(t)\\
h_2(t)\\
\vdots\\
h_m(t)
\end{bmatrix}.
$$

These latent variables may absorb effects such as:

$$
\text{water table behavior},
\qquad
\text{recent pump activity},
\qquad
\text{wall or floor emanation memory},
\qquad
\text{humidity-dependent source strength},
\qquad
\text{unmeasured door or leakage state}.
$$

The latent state can be modeled dynamically:

$$
\frac{d\mathbf{h}}{dt}
=
F_{\theta}
\left(
\mathbf{h}(t),
\mathbf{z}(t),
u(t)
\right).
$$

The complete grey-box model is therefore:

$$
\boxed{
\begin{aligned}
\frac{dR}{dt}
&=
S_{\theta}(\mathbf{z},\mathbf{h})
-
A_{\theta}(\mathbf{z},u,\mathbf{h})R
-
\lambda_{\mathrm{Rn}}R
+
B_{\theta}(\mathbf{z},u,\mathbf{h}),
\\
\frac{d\mathbf{h}}{dt}
&=
F_{\theta}(\mathbf{h},\mathbf{z},u).
\end{aligned}
}
$$

This is a Universal Differential Equation because part of the differential equation is known
and part is learned:

$$
\frac{d\mathbf{x}}{dt}
=
f_{\mathrm{phys}}(\mathbf{x},\mathbf{z},u;\mathbf{p})
+
f_{\theta}(\mathbf{x},\mathbf{z},u).
$$

For this project, the state may be

$$
\mathbf{x}(t)
=
\begin{bmatrix}
R(t)\\
\mathbf{h}(t)
\end{bmatrix}.
$$

The physical component is:

$$
f_{\mathrm{phys}}
=
\begin{bmatrix}
-\lambda_{\mathrm{Rn}}R\\
\mathbf{0}
\end{bmatrix},
$$

and the learned component is:

$$
f_{\theta}
=
\begin{bmatrix}
S_{\theta}(\mathbf{z},\mathbf{h})
-
A_{\theta}(\mathbf{z},u,\mathbf{h})R
+
B_{\theta}(\mathbf{z},u,\mathbf{h})
\\
F_{\theta}(\mathbf{h},\mathbf{z},u)
\end{bmatrix}.
$$

Additional known physical structure can be included if available. For example:

$$
A_{\theta}(\mathbf{z},u,\mathbf{h})
=
A_{\mathrm{nat},\theta}(\mathbf{z},\mathbf{h})
+
u\,A_{\mathrm{fan},\theta}(\mathbf{z},\mathbf{h}).
$$

This decomposition is valuable because it separates natural air exchange from fan-driven
air exchange.

The natural component may depend on pressure and temperature differences:

$$
A_{\mathrm{nat},\theta}
=
\Psi_{\mathrm{nat},\theta}
\left(
P_{\mathrm{room}}-P_{\mathrm{out}},
T_{\mathrm{room}}-T_{\mathrm{out}},
H_{\mathrm{room}},
H_{\mathrm{out}},
\mathbf{h}
\right).
$$

The fan component may depend on fan flow and pressure:

$$
A_{\mathrm{fan},\theta}
=
\Psi_{\mathrm{fan},\theta}
\left(
Q_{\mathrm{fan}},
\Delta P_{\mathrm{fan}},
P_{\mathrm{room}}-P_{\mathrm{out}},
\mathbf{h}
\right).
$$

A simple physical prior is:

$$
A_{\mathrm{fan},\theta}
\approx
\eta_{\theta}(\mathbf{z},\mathbf{h})
\frac{Q_{\mathrm{fan}}}{V},
$$

where

$$
0 \le \eta_{\theta}(\mathbf{z},\mathbf{h}) \le 1
$$

is an effective ventilation efficiency. This term accounts for short-circuiting, dead zones,
recirculation, and non-ideal mixing.

Thus:

$$
\frac{dR}{dt}
=
S_{\theta}(\mathbf{z},\mathbf{h})
-
\left[
A_{\mathrm{nat},\theta}(\mathbf{z},\mathbf{h})
+
u(t)\,
\eta_{\theta}(\mathbf{z},\mathbf{h})
\frac{Q_{\mathrm{fan}}(t)}{V}
+
\lambda_{\mathrm{Rn}}
\right]R(t)
+
B_{\theta}(\mathbf{z},u,\mathbf{h}).
$$

This equation is compact, interpretable, trainable, and directly useful for control.

## 6. Why Option 1 Is Preferred

Option 1 is preferred because the site is a single real room with complex geometry, limited
spatial sensing, uncertain airflow paths, and a practical operational objective.

The chosen model has the right bias:

$$
\text{conservation of mass}
\quad
\text{is trusted,}
$$

while

$$
\text{source strength, leakage, mixing, and ventilation efficiency}
\quad
\text{are learned from data.}
$$

This is important because the room probably does not provide enough information to identify
the full spatial field

$$
c(\mathbf{x},t)
$$

or the full airflow field

$$
\mathbf{v}(\mathbf{x},t).
$$

But it can provide enough information to learn the operationally relevant map:

$$
\left(
\mathbf{z}(t),u(t),R(t)
\right)
\longrightarrow
\frac{dR}{dt}.
$$

The system does not need perfect CFD. It needs a reliable estimate of:

$$
\widehat{R}(t+\tau)
\quad
\text{for}
\quad
\tau \in [0,H],
$$

where

$$
H
$$

is the control horizon, such as the time until the next scheduled entry.

The chosen UDE model also supports uncertainty-aware control:

$$
\widehat{R}(t+\tau)
\quad
\longrightarrow
\quad
p\left(R(t+\tau)\mid\mathcal{D}_t\right).
$$

Instead of asking only

$$
\widehat{R}(t_i^{\mathrm{in}}) \le R_{\mathrm{safe}},
$$

the safety logic can ask:

$$
\Pr
\left(
R(t) \le R_{\mathrm{safe}}
\;\;
\forall t \in
\left[t_i^{\mathrm{in}},t_i^{\mathrm{out}}\right]
\right)
\ge
1-\alpha.
$$

This is a much better operational target than a single deterministic forecast.

## 7. Brief Alternatives and Why They Are Not the First Choice

### 6.1 Full PINN on the Spatial PDE

A Physics-Informed Neural Network could represent:

$$
\widehat{c}_{\theta}(\mathbf{x},t)
\approx
c(\mathbf{x},t),
$$

and penalize the PDE residual:

$$
\mathcal{R}_{\theta}(\mathbf{x},t)
=
\frac{\partial \widehat{c}_{\theta}}{\partial t}
+
\nabla \cdot
\left(
\mathbf{v}_{\theta}\widehat{c}_{\theta}
\right)
-
\nabla \cdot
\left(
D_{\theta}\nabla \widehat{c}_{\theta}
\right)
-
s_{\theta}
+
\lambda_{\mathrm{Rn}}\widehat{c}_{\theta}.
$$

The training loss would include:

$$
\mathcal{L}_{\mathrm{PDE}}
=
\frac{1}{N_r}
\sum_{j=1}^{N_r}
\left|
\mathcal{R}_{\theta}(\mathbf{x}_j,t_j)
\right|^2.
$$

This is mathematically attractive, but it is not the best first choice here because the room
probably does not have dense spatial radon measurements. With only one or a few sensors,
many different spatial fields

$$
c(\mathbf{x},t)
$$

can produce the same sensor readings. The inverse problem is underdetermined.

PINNs also require boundary conditions, or at least useful approximations:

$$
c(\mathbf{x},t)
\big|_{\partial\Omega},
\qquad
\mathbf{v}(\mathbf{x},t)
\big|_{\partial\Omega},
\qquad
D(\mathbf{x},t)
\big|_{\partial\Omega}.
$$

Those are precisely the quantities that are uncertain in the station room.

PINNs may become useful later if multiple sensors are installed across the room and if the
goal becomes spatial risk mapping. For the first deployable controller, they are too ambitious.

### 6.2 Neural Operators: FNO or DeepONet

Neural operators learn mappings between functions. For example, one could try to learn:

$$
\mathcal{G}_{\theta}:
\left(
u(t),
P_{\mathrm{out}}(t),
T_{\mathrm{out}}(t),
H_{\mathrm{out}}(t),
R(0)
\right)
\mapsto
R(t).
$$

More generally:

$$
\mathcal{G}_{\theta}:
a(\cdot)
\mapsto
s(\cdot),
$$

where

$$
a(\cdot)
$$

is an input function and

$$
s(\cdot)
$$

is a solution function.

Fourier Neural Operators and DeepONets are powerful when there are many examples from a
family of related PDE problems:

$$
\left\{
a_j(\cdot),s_j(\cdot)
\right\}_{j=1}^{M}.
$$

They are especially attractive when one has:

$$
M \gg 1,
$$

such as many CFD simulations, many rooms, many geometries, or many parameterized
boundary conditions.

For this project, however, there may be only one room and one real time series. In that case,
operator learning may be data-hungry and unnecessarily complex. Neural operators are a
good future option if the subway authority expands the project to many pump rooms or if a
simulation campaign is later created.

### 6.3 Pure Black-Box Sequence Model

A black-box recurrent model, temporal convolution, or transformer could learn:

$$
\left[
\mathbf{z}(t-k),u(t-k)
\right]_{k=0}^{L}
\mapsto
R(t+\tau).
$$

This may work empirically, but it has weaker extrapolation behavior. It does not know that
radon mass must be conserved. It does not know that the fan should not increase the removal
time constant negatively. It does not know that source terms should be nonnegative.

A black-box model may fit the training data while violating basic physical expectations:

$$
A_{\theta} < 0,
\qquad
S_{\theta} < 0,
\qquad
\frac{\partial R(t+\tau)}{\partial u(t)} > 0
\quad
\text{under conditions where fan removal should dominate}.
$$

For a safety-related control system, this is not ideal.

### 6.4 Classical ARX, ARMAX, or Kalman State-Space Model

A classical linear model might use:

$$
R_{k+1}
=
aR_k
+
\mathbf{b}^{\top}\mathbf{z}_k
+
c u_k
+
\epsilon_k.
$$

This is a reasonable baseline. It is interpretable and easy to validate. However, the real system
is likely nonlinear:

$$
A_{\mathrm{eff}}
=
A_{\mathrm{eff}}
\left(
u,
\Delta P,
T_{\mathrm{room}}-T_{\mathrm{out}},
H_{\mathrm{room}},
P_{\mathrm{room}}-P_{\mathrm{out}}
\right).
$$

The source term may also be nonlinear and history-dependent:

$$
S_{\mathrm{eff}}(t)
\ne
\text{constant}.
$$

A classical model can be a useful benchmark, but the UDE model is better aligned with the
physics and the control goal.

## 8. Technical Formulation of the Prediction Model

### 7.1 Spatial PDE Before Reduction

Let

$$
\Omega
\subset
\mathbb{R}^3
$$

represent the pump room.

Let its boundary be decomposed as

$$
\partial\Omega
=
\Gamma_{\mathrm{fan}}
\cup
\Gamma_{\mathrm{leak}}
\cup
\Gamma_{\mathrm{wall}}
\cup
\Gamma_{\mathrm{water}}
\cup
\Gamma_{\mathrm{door}}.
$$

The radon concentration field is:

$$
c:
\Omega \times [0,T]
\rightarrow
\mathbb{R}_{\ge 0}.
$$

The PDE is:

$$
\partial_t c
=
-\nabla \cdot (\mathbf{v}c)
+
\nabla \cdot(D\nabla c)
+
s
-
\lambda_{\mathrm{Rn}}c.
$$

The fan affects the boundary condition on

$$
\Gamma_{\mathrm{fan}}.
$$

For example:

$$
\mathbf{v}(\mathbf{x},t)\cdot\mathbf{n}
=
u(t)\frac{Q_{\mathrm{fan}}(t)}{|\Gamma_{\mathrm{fan}}|}
+
q_{\mathrm{nat}}(\mathbf{x},t),
\qquad
\mathbf{x}\in\Gamma_{\mathrm{fan}}.
$$

Natural leakage can be represented on

$$
\Gamma_{\mathrm{leak}}
$$

as:

$$
\mathbf{v}(\mathbf{x},t)\cdot\mathbf{n}
=
q_{\theta}
\left(
\mathbf{x},
P_{\mathrm{room}}-P_{\mathrm{out}},
T_{\mathrm{room}}-T_{\mathrm{out}},
H_{\mathrm{room}},
H_{\mathrm{out}}
\right).
$$

Radon source from water or surfaces can be represented as:

$$
s(\mathbf{x},t)
=
s_{\mathrm{water}}(\mathbf{x},t)
+
s_{\mathrm{wall}}(\mathbf{x},t)
+
s_{\mathrm{floor}}(\mathbf{x},t).
$$

A learned source model could be:

$$
s_{\theta}(\mathbf{x},t)
=
s_{\theta}
\left(
\mathbf{x},
\mathbf{z}(t),
\mathbf{h}(t)
\right).
$$

The full PDE problem would require:

$$
\left\{
\Omega,
\partial\Omega,
\Gamma_{\mathrm{fan}},
\Gamma_{\mathrm{leak}},
\Gamma_{\mathrm{water}},
\mathbf{v},
D,
s,
c(\mathbf{x},0)
\right\}.
$$

These are not reliably known. Therefore the PDE is used as a structural guide, not as a
literal full-field CFD model.

### 7.2 Volume-Averaged Conservation Law

The room-average concentration is:

$$
R(t)
=
\frac{1}{V}
\int_{\Omega}
c(\mathbf{x},t)\,d\mathbf{x}.
$$

The total radon mass proxy is:

$$
M(t)
=
\int_{\Omega}
c(\mathbf{x},t)\,d\mathbf{x}
=
VR(t).
$$

From the PDE:

$$
\frac{dM}{dt}
=
G(t)
-
E(t)
-
\lambda_{\mathrm{Rn}}M(t)
+
I(t),
$$

where

$$
G(t)
=
\int_{\Omega}
s(\mathbf{x},t)\,d\mathbf{x},
$$

$$
E(t)
=
\int_{\partial\Omega_{\mathrm{out}}}
c(\mathbf{x},t)
\mathbf{v}(\mathbf{x},t)\cdot\mathbf{n}
\,d\Gamma,
$$

and

$$
I(t)
=
\int_{\partial\Omega_{\mathrm{in}}}
c_{\mathrm{in}}(\mathbf{x},t)
\left|
\mathbf{v}(\mathbf{x},t)\cdot\mathbf{n}
\right|
\,d\Gamma.
$$

Divide by

$$
V
$$

to get:

$$
\frac{dR}{dt}
=
\frac{G(t)}{V}
-
\frac{E(t)}{V}
-
\lambda_{\mathrm{Rn}}R(t)
+
\frac{I(t)}{V}.
$$

If the room is treated as approximately well-mixed at the control scale, then:

$$
E(t)
\approx
Q_{\mathrm{out,eff}}(t)R(t).
$$

Thus:

$$
\frac{dR}{dt}
=
\frac{G(t)}{V}
-
\frac{Q_{\mathrm{out,eff}}(t)}{V}R(t)
-
\lambda_{\mathrm{Rn}}R(t)
+
\frac{Q_{\mathrm{in,eff}}(t)}{V}R_{\mathrm{in}}(t).
$$

This can be written:

$$
\frac{dR}{dt}
=
S(t)
-
K(t)R(t)
+
B(t),
$$

where

$$
S(t)=\frac{G(t)}{V},
$$

$$
K(t)=\frac{Q_{\mathrm{out,eff}}(t)}{V}+\lambda_{\mathrm{Rn}},
$$

and

$$
B(t)=\frac{Q_{\mathrm{in,eff}}(t)}{V}R_{\mathrm{in}}(t).
$$

The UDE replaces

$$
S(t), K(t), B(t)
$$

with learned but constrained functions.

### 7.3 Learned Terms With Physical Constraints

The learned source should be nonnegative:

$$
S_{\theta}(\mathbf{z},\mathbf{h}) \ge 0.
$$

The learned removal rate should be nonnegative:

$$
A_{\theta}(\mathbf{z},u,\mathbf{h}) \ge 0.
$$

The fan-on removal rate should not be lower than the fan-off removal rate, except under
explicitly modeled failure conditions:

$$
A_{\theta}(\mathbf{z},1,\mathbf{h})
\ge
A_{\theta}(\mathbf{z},0,\mathbf{h}).
$$

A useful decomposition is:

$$
A_{\theta}(\mathbf{z},u,\mathbf{h})
=
A_{0,\theta}(\mathbf{z},\mathbf{h})
+
uA_{1,\theta}(\mathbf{z},\mathbf{h}),
$$

with

$$
A_{0,\theta}(\mathbf{z},\mathbf{h}) \ge 0,
\qquad
A_{1,\theta}(\mathbf{z},\mathbf{h}) \ge 0.
$$

Then:

$$
A_{\theta}(\mathbf{z},1,\mathbf{h})
-
A_{\theta}(\mathbf{z},0,\mathbf{h})
=
A_{1,\theta}(\mathbf{z},\mathbf{h})
\ge
0.
$$

The model becomes:

$$
\frac{dR}{dt}
=
S_{\theta}(\mathbf{z},\mathbf{h})
-
\left[
A_{0,\theta}(\mathbf{z},\mathbf{h})
+
uA_{1,\theta}(\mathbf{z},\mathbf{h})
+
\lambda_{\mathrm{Rn}}
\right]R
+
B_{\theta}(\mathbf{z},u,\mathbf{h}).
$$

If outside radon is negligible or unavailable, one may initially set:

$$
B_{\theta}(\mathbf{z},u,\mathbf{h}) \approx 0,
$$

but a more conservative model keeps it:

$$
B_{\theta}(\mathbf{z},u,\mathbf{h}) \ge 0.
$$

The learned ventilation term can include a physically meaningful fan efficiency:

$$
A_{1,\theta}(\mathbf{z},\mathbf{h})
=
\eta_{\theta}(\mathbf{z},\mathbf{h})
\frac{Q_{\mathrm{fan}}}{V},
$$

with

$$
0 \le \eta_{\theta}(\mathbf{z},\mathbf{h}) \le \eta_{\max}.
$$

The parameter

$$
\eta_{\theta}
$$

captures how much of the fan's nominal flow actually removes radon from the sensor-relevant
air volume.

### 7.4 Latent Source and Mixing Memory

Radon generation and removal may have memory. For example, humidity changes may alter
surface emanation. Pump activity may expose or agitate water. Pressure changes may open
or suppress underground inflow paths. The measured vector

$$
\mathbf{z}(t)
$$

may not fully describe these effects.

Introduce a latent state:

$$
\mathbf{h}(t)
\in
\mathbb{R}^{m}.
$$

A physically interpretable version is:

$$
\mathbf{h}(t)
=
\begin{bmatrix}
S_{\mathrm{lat}}(t)\\
M_{\mathrm{mix}}(t)\\
L_{\mathrm{leak}}(t)
\end{bmatrix},
$$

where:

$$
S_{\mathrm{lat}}(t)
:
\text{slow source intensity memory},
$$

$$
M_{\mathrm{mix}}(t)
:
\text{mixing or dead-zone memory},
$$

and

$$
L_{\mathrm{leak}}(t)
:
\text{unobserved leakage state}.
$$

A relaxation model for source memory is:

$$
\frac{dS_{\mathrm{lat}}}{dt}
=
\frac{
S_{\mathrm{eq},\theta}(\mathbf{z})
-
S_{\mathrm{lat}}
}{
\tau_{S,\theta}(\mathbf{z})
}.
$$

Then the source term can be:

$$
S_{\theta}(\mathbf{z},\mathbf{h})
=
S_{\mathrm{base},\theta}(\mathbf{z})
+
S_{\mathrm{lat}}(t).
$$

A mixing-memory term can affect fan efficiency:

$$
\eta_{\theta}(\mathbf{z},\mathbf{h})
=
\eta_{\theta}
\left(
\mathbf{z},
M_{\mathrm{mix}},
L_{\mathrm{leak}}
\right).
$$

This gives the model enough flexibility to represent slow changes without abandoning
physical interpretability.

### 7.5 Sensor Observation Model

The actual sensor may not read the exact room average

$$
R(t).
$$

If the sensor is located at position

$$
\mathbf{x}_s,
$$

then the raw concentration near the sensor is:

$$
y_{\mathrm{raw}}(t)
=
c(\mathbf{x}_s,t)
+
\varepsilon(t).
$$

Under the reduced model, this is approximated as:

$$
y_{\mathrm{raw}}(t)
=
R(t)
+
\delta_{\mathrm{loc}}(t)
+
\varepsilon(t),
$$

where

$$
\delta_{\mathrm{loc}}(t)
$$

is a location bias due to imperfect mixing.

If the sensor reports a rolling average:

$$
y_{\tau}(t)
=
\frac{1}{\tau}
\int_{t-\tau}^{t}
R(s)\,ds
+
\varepsilon(t).
$$

The exact derivative of this rolling average is:

$$
\frac{dy_{\tau}}{dt}
=
\frac{R(t)-R(t-\tau)}{\tau}.
$$

For controller design, an exponential approximation is often more convenient:

$$
\frac{d\bar{R}_{\tau}}{dt}
=
\frac{R(t)-\bar{R}_{\tau}(t)}{\tau}.
$$

Then the model state can include:

$$
\mathbf{x}(t)
=
\begin{bmatrix}
R(t)\\
\bar{R}_{4h}(t)\\
\bar{R}_{24h}(t)\\
\mathbf{h}(t)
\end{bmatrix}.
$$

With:

$$
\frac{d\bar{R}_{4h}}{dt}
=
\frac{R-\bar{R}_{4h}}{\tau_{4h}},
$$

and:

$$
\frac{d\bar{R}_{24h}}{dt}
=
\frac{R-\bar{R}_{24h}}{\tau_{24h}}.
$$

The observation equation becomes:

$$
y(t)
=
g(\mathbf{x}(t))
+
\varepsilon(t).
$$

For a 4-hour average sensor:

$$
g(\mathbf{x}(t))
=
\bar{R}_{4h}(t).
$$

For a raw or short-interval sensor:

$$
g(\mathbf{x}(t))
=
R(t).
$$

This distinction matters because a controller that only sees

$$
\bar{R}_{4h}(t)
$$

may respond too slowly unless it estimates the hidden state

$$
R(t).
$$

### 7.6 Training Objective

Let the model prediction be:

$$
\widehat{\mathbf{x}}_{\theta}(t).
$$

Let the predicted sensor output be:

$$
\widehat{y}_{\theta}(t)
=
g(\widehat{\mathbf{x}}_{\theta}(t)).
$$

Given measurements

$$
y_k = y(t_k),
$$

the data loss is:

$$
\mathcal{L}_{\mathrm{data}}(\theta)
=
\sum_{k=1}^{K}
\frac{
\left(
\widehat{y}_{\theta}(t_k)-y_k
\right)^2
}{
\sigma_y^2
}.
$$

The state evolves according to:

$$
\widehat{\mathbf{x}}_{\theta}(t_{k+1})
=
\widehat{\mathbf{x}}_{\theta}(t_k)
+
\int_{t_k}^{t_{k+1}}
f_{\theta}
\left(
\widehat{\mathbf{x}}_{\theta}(s),
\mathbf{z}(s),
u(s)
\right)
ds.
$$

The differential-equation consistency is built into the model integration. Additional physical
penalties can be added:

$$
\mathcal{L}_{\mathrm{phys}}
=
\rho_S
\sum_k
\left[
\max
\left(
0,
-S_{\theta}(\mathbf{z}_k,\mathbf{h}_k)
\right)
\right]^2
+
\rho_A
\sum_k
\left[
\max
\left(
0,
-A_{\theta}(\mathbf{z}_k,u_k,\mathbf{h}_k)
\right)
\right]^2.
$$

Fan monotonicity can be encouraged by:

$$
\mathcal{L}_{\mathrm{fan}}
=
\rho_F
\sum_k
\left[
\max
\left(
0,
A_{\theta}(\mathbf{z}_k,0,\mathbf{h}_k)
-
A_{\theta}(\mathbf{z}_k,1,\mathbf{h}_k)
\right)
\right]^2.
$$

A smoothness prior can be placed on slowly varying source behavior:

$$
\mathcal{L}_{\mathrm{smooth}}
=
\rho_H
\int_0^T
\left\|
\frac{d\mathbf{h}}{dt}
\right\|^2
dt.
$$

The total training objective is:

$$
\boxed{
\mathcal{L}(\theta)
=
\mathcal{L}_{\mathrm{data}}
+
\mathcal{L}_{\mathrm{phys}}
+
\mathcal{L}_{\mathrm{fan}}
+
\mathcal{L}_{\mathrm{smooth}}
+
\mathcal{L}_{\mathrm{reg}}.
}
$$

This formulation makes the model data-driven while still discouraging physically impossible
behavior.

### 7.7 Multi-Compartment Extension

If the room is not well-mixed, a single average state may be insufficient. A practical extension
is a multi-compartment model.

Divide the room into

$$
n
$$

effective zones:

$$
R_1(t),R_2(t),\ldots,R_n(t).
$$

The model becomes:

$$
\frac{dR_i}{dt}
=
S_{i,\theta}
-
\left(
A_{i,\theta}
+
\lambda_{\mathrm{Rn}}
\right)R_i
+
\sum_{j\ne i}
K_{ij,\theta}(R_j-R_i)
+
B_{i,\theta}.
$$

In vector form:

$$
\frac{d\mathbf{R}}{dt}
=
\mathbf{S}_{\theta}
-
\left(
\mathbf{A}_{\theta}
+
\lambda_{\mathrm{Rn}}\mathbf{I}
\right)\mathbf{R}
+
\mathbf{K}_{\theta}\mathbf{R}
+
\mathbf{B}_{\theta}.
$$

Where:

$$
\mathbf{R}(t)
=
\begin{bmatrix}
R_1(t)\\
R_2(t)\\
\vdots\\
R_n(t)
\end{bmatrix}.
$$

The matrix

$$
\mathbf{K}_{\theta}
$$

models inter-zone exchange. This is a finite-volume approximation of the PDE, but with
learned effective transport coefficients.

This extension is especially useful if multiple radon sensors are installed:

$$
y_j(t)
=
R_{\pi(j)}(t)+\varepsilon_j(t),
$$

where

$$
\pi(j)
$$

maps sensor

$$
j
$$

to zone

$$
i.
$$

The multi-compartment model is still option 1. It is not full CFD. It is a grey-box UDE with
more spatial structure.

## 9. Fan Planning Formulation: Binary MPC

The prediction model is exploited by a planning layer. The planner does not need to learn a
second neural dynamics model. It uses the trained UDE model as a differentiable or
simulatable transition model and solves a constrained binary optimization problem.

At current decision time

$$
t_0,
$$

define a planning horizon:

$$
[t_0,t_0+H].
$$

The sampled decision variables are:

$$
\mathbf{u}
=
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_H
\end{bmatrix},
\qquad
u_k\in\{0,1\}.
$$

The sampled worker-entry plan is:

$$
\mathbf{w}
=
\begin{bmatrix}
w_0\\
w_1\\
\vdots\\
w_H
\end{bmatrix},
\qquad
w_k\in\{0,1\}.
$$

where:

$$
w_k=1
\quad
\Longleftrightarrow
\quad
\text{workers are scheduled to be inside at step } k.
$$

The historical data is used to estimate the current model state:

$$
\widehat{\mathbf{x}}_0
=
\mathcal{E}_{\theta}
\left(
\mathcal{D}_{\le 0}
\right),
$$

where

$$
\mathcal{D}_{\le 0}
=
\left\{
\mathbf{z}_{j},u_j,y_j
\right\}_{j\le 0}.
$$

After this state-estimation step, the optimizer does not need to directly consume all past data.
It uses:

$$
\boxed{
\widehat{\mathbf{x}}_0,
\qquad
\mathbf{z}_{0:H}^{+},
\qquad
\mathbf{w}_{0:H},
\qquad
F_{\theta}.
}
$$

The UDE prediction model supplies the rollout dynamics:

$$
\widehat{\mathbf{x}}_{k+1}
=
F_{\theta}
\left(
\widehat{\mathbf{x}}_k,
\mathbf{z}_k^{+},
u_k
\right),
\qquad
k=0,\ldots,H-1.
$$

The predicted radon concentration is extracted from the state:

$$
\widehat{R}_k
=
C_R\widehat{\mathbf{x}}_k.
$$

The basic binary planning problem is:

$$
\boxed{
\begin{aligned}
\mathbf{u}^{\star}
=
\arg\min_{\mathbf{u},\boldsymbol{\xi}}
\quad
&
\sum_{k=0}^{H}
\left(
c_Eu_k
+
c_{\Delta}s_k
+
c_{\xi}\xi_k
\right)
\\
\mathrm{s.t.}
\quad
&
\widehat{\mathbf{x}}_{k+1}
=
F_{\theta}
\left(
\widehat{\mathbf{x}}_k,
\mathbf{z}_k^{+},
u_k
\right),
\\
&
u_k\in\{0,1\},
\\
&
\xi_k\ge 0,
\\
&
w_k
\left(
\widehat{R}_k
-
R_{\mathrm{safe}}
+
m_{\mathrm{safety}}
\right)
\le
\xi_k.
\end{aligned}
}
$$

The term

$$
c_Eu_k
$$

penalizes fan runtime. The term

$$
c_{\Delta}s_k
$$

penalizes toggling. The slack term

$$
c_{\xi}\xi_k
$$

penalizes safety violation and should have the largest cost:

$$
c_{\xi}
\gg
c_E,
\qquad
c_{\xi}
\gg
c_{\Delta}.
$$

The switch variable can be defined as:

$$
s_k
=
\left|
u_k-u_{k-1}
\right|.
$$

For a mixed-integer linear representation, introduce:

$$
s_k\in\{0,1\},
$$

with:

$$
s_k \ge u_k-u_{k-1},
\qquad
s_k \ge u_{k-1}-u_k,
$$

and:

$$
s_k \le u_k+u_{k-1},
\qquad
s_k \le 2-u_k-u_{k-1}.
$$

### 9.1 Minimum Debounce or Dwell Time

The subway officials may not want the fan to toggle too frequently. Let:

$$
N_{\mathrm{debounce}}
$$

be the minimum number of discrete time steps between two toggles.

The debounce constraint is:

$$
\boxed{
\sum_{r=k}^{k+N_{\mathrm{debounce}}-1}
s_r
\le
1,
\qquad
k=0,\ldots,H-N_{\mathrm{debounce}}+1.
}
$$

This ensures that if:

$$
s_k=1,
$$

then:

$$
s_{k+1}=s_{k+2}=\cdots=s_{k+N_{\mathrm{debounce}}-1}=0.
$$

If separate minimum-on and minimum-off times are required, define the start and stop
variables:

$$
a_k
=
\max(0,u_k-u_{k-1}),
\qquad
b_k
=
\max(0,u_{k-1}-u_k).
$$

Then:

$$
a_k=1
\quad
\Longrightarrow
\quad
u_{k+r}=1,
\qquad
r=0,\ldots,N_{\mathrm{on}}-1,
$$

and:

$$
b_k=1
\quad
\Longrightarrow
\quad
u_{k+r}=0,
\qquad
r=0,\ldots,N_{\mathrm{off}}-1.
$$

These can be expressed as:

$$
\sum_{r=k}^{k+N_{\mathrm{on}}-1}
u_r
\ge
N_{\mathrm{on}}a_k,
$$

and:

$$
\sum_{r=k}^{k+N_{\mathrm{off}}-1}
(1-u_r)
\ge
N_{\mathrm{off}}b_k.
$$

### 9.2 Planning With Probabilistic Safety

For worker safety, the deterministic constraint:

$$
\widehat{R}_k
\le
R_{\mathrm{safe}}
-
m_{\mathrm{safety}}
$$

can be replaced by a probabilistic or quantile constraint:

$$
\boxed{
w_k=1
\quad
\Longrightarrow
\quad
Q_{1-\alpha}
\left[
R_k
\mid
\widehat{\mathbf{x}}_0,
\mathbf{z}_{0:k}^{+},
u_{0:k}
\right]
\le
R_{\mathrm{safe}}.
}
$$

This makes the planner conservative when the model is uncertain. A deterministic equivalent
can be written as:

$$
\mu_{R,k}
+
\beta\sigma_{R,k}
\le
R_{\mathrm{safe}},
\qquad
w_k=1,
$$

where:

$$
\mu_{R,k}
=
\mathbb{E}[R_k],
\qquad
\sigma_{R,k}^2
=
\mathrm{Var}(R_k).
$$

### 9.3 Preemptive Fan Start

The planner naturally discovers a preemptive fan start time. If the next worker-entry time is:

$$
t_i^{\mathrm{in}},
$$

then the planner searches for a sequence satisfying:

$$
R(t)
\le
R_{\mathrm{safe}},
\qquad
t\in
\left[
t_i^{\mathrm{in}},
t_i^{\mathrm{out}}
\right],
$$

while minimizing:

$$
\int_{t_0}^{t_i^{\mathrm{out}}}
u(t)\,dt.
$$

In the simplest single-entry case, this is equivalent to finding the latest feasible fan start:

$$
t_i^{\mathrm{on},\star}
=
\max
\left\{
t:
Q_{1-\alpha}
\left[
R(t_i^{\mathrm{in}})
\mid
u(s)=1,\ s\in[t,t_i^{\mathrm{in}}]
\right]
\le
R_{\mathrm{safe}}
\right\}.
$$

For multiple worker windows, the full binary MPC formulation is preferable because it can
reuse ventilation across nearby windows, account for fan debounce limits, and avoid short
unnecessary on/off cycles.

The final planning output is:

$$
\boxed{
\mathbf{u}^{\star}
=
\left[
0,0,0,1,1,1,1,0,\ldots
\right],
}
$$

which is the optimized fan toggle plan.

## 10. Uncertainty and Safety Margins

Because this is a worker-safety application, the model should not output only a mean:

$$
\mathbb{E}[R(t)].
$$

It should output predictive uncertainty:

$$
p(R(t)\mid\mathcal{D}_{t_0}).
$$

Useful uncertainty sources include:

$$
\text{sensor noise},
\qquad
\text{model parameter uncertainty},
\qquad
\text{unmeasured door or leakage state},
\qquad
\text{weather forecast uncertainty},
\qquad
\text{fan performance uncertainty}.
$$

The safety rule should be conservative:

$$
R_{\mathrm{control}}
(t)
=
Q_{1-\alpha}[R(t)]
$$

and:

$$
R_{\mathrm{control}}(t)
\le
R_{\mathrm{safe}}.
$$

For example, with:

$$
\alpha=0.05,
$$

the controller uses the 95th percentile forecast, not the mean forecast.

A safety margin can also be added:

$$
R_{\mathrm{control}}(t)
\le
R_{\mathrm{safe}}-m_{\mathrm{safety}}.
$$

The margin can be adaptive:

$$
m_{\mathrm{safety}}(t)
=
\beta\sigma_R(t)
+
m_0,
$$

where:

$$
\sigma_R(t)
$$

is the predictive standard deviation,

$$
\beta>0
$$

is a conservatism factor, and

$$
m_0
$$

is a fixed regulatory or engineering buffer.

## 11. Data Strategy

The model can learn only if it observes changes caused by the fan and by natural conditions.
The useful training data includes:

$$
\left(
R_{\mathrm{room}},
T_{\mathrm{room}},
H_{\mathrm{room}},
P_{\mathrm{room}},
Q_{\mathrm{fan}},
\Delta P_{\mathrm{fan}},
T_{\mathrm{out}},
H_{\mathrm{out}},
P_{\mathrm{out}},
u
\right)(t).
$$

Additional high-value signals, if available, are:

$$
\text{water pump state},
\qquad
\text{water level},
\qquad
\text{door open or closed state},
\qquad
\text{worker access logs},
\qquad
\text{fan current or power},
\qquad
\text{local differential pressure to adjacent spaces}.
$$

The training data should include:

$$
u(t)=0
$$

periods, to learn natural accumulation:

$$
\frac{dR}{dt}
\approx
S_{\theta}
-
\left(A_{0,\theta}+\lambda_{\mathrm{Rn}}\right)R.
$$

It should also include:

$$
u(t)=1
$$

periods, to learn fan-driven removal:

$$
\frac{dR}{dt}
\approx
S_{\theta}
-
\left(A_{0,\theta}+A_{1,\theta}+\lambda_{\mathrm{Rn}}\right)R.
$$

The most informative events are transitions:

$$
u:0\rightarrow 1,
\qquad
u:1\rightarrow 0.
$$

These transitions reveal time constants:

$$
\tau_{\mathrm{off}}
\approx
\frac{1}{A_{0,\theta}+\lambda_{\mathrm{Rn}}},
$$

and:

$$
\tau_{\mathrm{on}}
\approx
\frac{1}{A_{0,\theta}+A_{1,\theta}+\lambda_{\mathrm{Rn}}}.
$$

If safe and operationally acceptable, planned fan test pulses can improve identifiability:

$$
u(t)
=
\begin{cases}
1, & t\in[t_a,t_b],\\
0, & \text{otherwise}.
\end{cases}
$$

The response:

$$
R(t_b+\tau)-R(t_a)
$$

helps estimate the removal dynamics under real conditions.

## 12. Deployment Concept

A practical deployment sequence is:

1. Collect baseline data with current manual fan operation.
2. Add synchronized weather API records and room sensor records.
3. Fit a simple baseline mass-balance model.
4. Fit the UDE model with physical constraints.
5. Validate forecasts on held-out time periods.
6. Run the controller in shadow mode, where it recommends fan actions but does not actuate.
7. Compare recommended actions against actual radon outcomes.
8. Move to supervised automation.
9. Move to automatic control with manual override and fault detection.

The model should be judged by operational metrics:

$$
\text{miss rate}
=
\Pr
\left(
R(t)>R_{\mathrm{safe}}
\mid
t\in\mathcal{W}
\right),
$$

$$
\text{fan runtime}
=
\int_0^T u(t)\,dt,
$$

$$
\text{preparation success}
=
\Pr
\left(
R(t_i^{\mathrm{in}})
\le
R_{\mathrm{safe}}
\right),
$$

and:

$$
\text{unnecessary ventilation}
=
\int_{t\notin\mathcal{W}}
u(t)\,dt.
$$

For worker safety, the priority order should be:

$$
\text{safety during entry windows}
\succ
\text{robustness to sensor/model uncertainty}
\succ
\text{energy optimization}
\succ
\text{fan wear minimization}.
$$

## 13. Final Recommendation

The final system should be defined as two coupled mathematical components:

$$
\boxed{
\text{Component 1: physics-informed prediction}
}
\qquad
\boxed{
\text{Component 2: constrained binary fan planning}
}
$$

The best prediction model is a grey-box Universal Differential Equation:

$$
\boxed{
\frac{dR}{dt}
=
S_{\theta}(\mathbf{z},\mathbf{h})
-
\left[
A_{0,\theta}(\mathbf{z},\mathbf{h})
+
uA_{1,\theta}(\mathbf{z},\mathbf{h})
+
\lambda_{\mathrm{Rn}}
\right]R
+
B_{\theta}(\mathbf{z},u,\mathbf{h})
}
$$

with latent dynamics:

$$
\boxed{
\frac{d\mathbf{h}}{dt}
=
F_{\theta}(\mathbf{h},\mathbf{z},u)
}
$$

and sensor observation:

$$
\boxed{
y(t)
=
g(\mathbf{x}(t))
+
\varepsilon(t)
}
$$

This prediction model is the right compromise:

$$
\text{more physical than a black-box forecast model,}
$$

$$
\text{more feasible than CFD,}
$$

$$
\text{less data-hungry than neural operators,}
$$

and

$$
\text{more deployable than a full spatial PINN.}
$$

But the prediction model alone is not the final operational answer. It must be wrapped by a
binary fan planner:

$$
\boxed{
\mathbf{u}^{\star}
=
\arg\min_{\mathbf{u},\mathbf{s},\boldsymbol{\xi}}
\left[
\sum_{k=0}^{H}c_Eu_k
+
\sum_{k=0}^{H}c_{\Delta}s_k
+
\sum_{k=0}^{H}c_{\xi}\xi_k
\right]
}
$$

with:

$$
u_k\in\{0,1\},
\qquad
s_k\in\{0,1\},
\qquad
\xi_k\ge 0,
$$

subject to UDE rollout dynamics:

$$
\widehat{\mathbf{x}}_{k+1}
=
F_{\theta}
\left(
\widehat{\mathbf{x}}_k,
\mathbf{z}_k^{+},
u_k
\right),
$$

worker-entry safety:

$$
\boxed{
w_k=1
\quad
\Longrightarrow
\quad
Q_{1-\alpha}
\left[
R_k
\right]
\le
R_{\mathrm{safe}},
}
$$

and debounce constraints:

$$
\boxed{
\sum_{r=k}^{k+N_{\mathrm{debounce}}-1}
s_r
\le
1.
}
$$

Thus the final engineering object is:

$$
\boxed{
\left(
\mathcal{D}_{\le k},
\mathbf{z}_{k:k+H}^{+},
\mathbf{w}_{k:k+H}
\right)
\longrightarrow
\mathbf{u}_{k:k+H}^{\star}.
}
$$

In plain terms:

$$
\text{learn how this exact room accumulates and clears radon,}
$$

$$
\text{use that learned model to simulate candidate fan plans,}
$$

and

$$
\text{choose the safest low-runtime 0/1 fan schedule with limited toggling.}
$$

## 14. Method References

The selected direction is closest to Universal Differential Equations and scientific machine
learning:

- Rackauckas et al., "Universal Differential Equations for Scientific Machine Learning":
  https://arxiv.org/abs/2001.04385
- Raissi, Perdikaris, and Karniadakis, "Physics-informed neural networks":
  https://doi.org/10.1016/j.jcp.2018.10.045
- Li et al., "Fourier Neural Operator for Parametric Partial Differential Equations":
  https://arxiv.org/abs/2010.08895
- Lu et al., "Learning nonlinear operators via DeepONet":
  https://www.nature.com/articles/s42256-021-00302-5
