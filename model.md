# SlipperyRoomba POMDP Model

A robot vacuum cleaner navigates a room under **partial observability and stochastic slipping motion**. The robot does not know its exact location and must localize itself using noisy sensor readings while navigating toward a goal and avoiding stairs.

---

## State Space

The state is a `RoombaState` tuple:

| Field | Type | Description |
|-------|------|-------------|
| `x` | Float64 | x position [m] |
| `y` | Float64 | y position [m] |
| `theta` | Float64 | heading angle [rad], wrapped to (-π, π] |
| `status` | Float64 | terminal indicator: `0` = navigating, `1` = goal reached, `-1` = stairs reached |

Two state-space variants are available:
- **`ContinuousRoombaStateSpace`** (default) — continuous (x, y, θ)
- **`DiscreteRoombaStateSpace(num_x_pts, num_y_pts, num_theta_pts)`** — uniform grid over x ∈ [-25, 15] m, y ∈ [-20, 5] m, θ ∈ [-π, π]

---

## Action Space

Actions are `RoombaAct` pairs:

| Field | Type | Description |
|-------|------|-------------|
| `v` | Float64 | linear velocity [m/s] |
| `omega` | Float64 | angular velocity [rad/s] |

Velocities are clamped internally: `v ∈ [0, v_max]`, `ω ∈ [-om_max, om_max]`.

A typical discrete action set (used in experiments):

```julia
vlist  = [0.0, 1.0, 2.0]        # m/s
omlist = [-1.0, 0.0, 1.0]       # rad/s
aspace = vec([RoombaAct(v, om) for v in vlist, om in omlist])
```

---

## Transition Model

### Chain Rule Factorization

The transition has two stochastic components — heading and step distance — and one deterministic component — position update:

$$p(s' \mid s, a) = p(\theta' \mid \theta, \omega)\; p(\text{step} \mid v, \theta')$$

$$x' = x + \text{step}\cos\theta', \quad y' = y + \text{step}\sin\theta'$$

### Rotation

$$p(\theta' \mid \theta, \omega) = \text{VonMises}\!\left(\mu_\theta,\; \kappa\right)$$

$$\mu_\theta = \theta + \omega \cdot dt, \qquad \kappa = \frac{1}{\left(\theta_\text{std} \cdot \bar\omega\right)^2}, \qquad \bar\omega = \text{clamp}(|\omega|,\; 0.01,\; \omega_\text{max})$$

The VonMises is a circular analogue of the Gaussian. Higher `theta_std` → lower $\kappa$ → wider distribution → more rotational slip. $\bar\omega$ is clamped to keep $\kappa$ finite when $\omega = 0$.

### Translation

Given $\theta'$, let $d_\text{wall}$ be the distance to the nearest wall along the heading direction (via ray-casting). The step distance follows a **censored truncated-normal**:

$$p(\text{step} \mid v, \theta') = \begin{cases} f_{\mathcal{N}^+}(\text{step};\; \mu_s,\, \sigma_s) & \text{step} < d_\text{wall} \\ 1 - F_{\mathcal{N}^+}(d_\text{wall};\; \mu_s,\, \sigma_s) & \text{step} = d_\text{wall} \end{cases}$$

where $\mathcal{N}^+(\mu_s, \sigma_s)$ denotes a normal distribution truncated at 0 from below:

$$\mu_s = v \cdot dt, \qquad \sigma_s = \text{trans\_noise\_coeff} \cdot v \cdot dt$$

The continuous part $f_{\mathcal{N}^+}$ is the truncated-normal pdf; the point mass at $d_\text{wall}$ collects all remaining probability — it represents the robot pressing against the wall. Truncation at 0 prevents backward motion.

Equivalently, the generative form is:

$$\text{step} = \min\!\left(d,\; d_\text{wall}\right), \qquad d \sim \mathcal{N}^+(\mu_s,\, \sigma_s)$$

**Assumptions:** `v > 0` and `theta_std > 0` are required.

### Default Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dt` | 0.5 s | time step |
| `theta_std` | 0.1 | rotation noise std [rad] |
| `trans_noise_coeff` | 0.1 | translation noise coefficient |
| `v_max` | 2.0 m/s | maximum linear velocity |
| `om_max` | 1.0 rad/s | maximum angular velocity |

---

## Observation Models

### Lidar (default)

A single forward-facing range sensor. The true range is measured along the robot's heading, and corrupted by proportional Gaussian noise:

$$o \sim \text{TruncNormal}\!\left(r_\text{true},\; \sigma_o,\; 0,\; \infty\right), \quad \sigma_o = \text{ray\_stdev} \cdot \max(r_\text{true},\, 0.01)$$

where $r_\text{true}$ is the ray-cast distance to the nearest wall in the heading direction.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ray_stdev` | 0.1 | proportional noise coefficient |

### Bumper

Deterministic binary contact sensor:

$$o = \text{wall\_contact}(\text{room},\, [x, y]) \in \{\text{true},\, \text{false}\}$$

Returns `true` if any wall is within robot radius of the center position.

### Discrete Lidar

Discretizes the continuous Lidar distribution into $N+1$ intervals defined by cutpoints `disc_points ∈ ℝᴺ`. The observation is the interval index.

---

## Reward Function

$$R(s, a, s') = r_\text{time} + r_\text{wall} \cdot \mathbb{1}[\text{new contact}] + r_\text{goal} \cdot \mathbb{1}[s'.\text{status} = 1] + r_\text{stairs} \cdot \mathbb{1}[s'.\text{status} = -1]$$

The wall contact penalty is incurred only on the *first* contact (transition from no-contact to contact), not for consecutive contact frames.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `time_pen` | -0.1 | penalty per timestep |
| `contact_pen` | -1.0 | penalty for hitting a wall |
| `goal_reward` | 10.0 | reward for reaching goal |
| `stairs_penalty` | -10.0 | penalty for reaching stairs |
| `discount` | 0.95 | discount factor γ |

---

## Initial State Distribution

- Position (x, y) sampled uniformly from the room interior (weighted by rectangle area)
- Heading θ ~ Uniform(-π, π)
- status = 0.0 (non-terminal always)

---

## Terminal States

The episode ends when `abs(s.status) > 0`:
- `status = 1.0` — goal reached
- `status = -1.0` — stairs reached (failure)

---

## Room Layouts

Three layouts are available via the `layout` keyword of `Room(sspace; layout=..., num_rooms=...)`:

### `baseline`
Four connected rectangles forming a 2×2 arrangement, total span ≈ 40 × 25 m. Goal and stairs locations vary by `config ∈ {1, 2, 3}`.

### `one_sided_corridor`
A row of `num_rooms` upper rooms connected by a corridor, with a single lower room containing stairs. Goal at the far end of the upper row.

### `two_sided_corridor`
A corridor with `num_rooms` upper rooms and `num_rooms - 1` lower rooms. The first two lower rooms are merged into a wider space. Goal at the far end of the upper row, stairs at the far end of the lower row.

**Robot geometry:** `ROBOT_W = 1.0 m` — robot diameter used for wall clearance in collision detection and initialization.

---

## Constructor Summary

```julia
RoombaMDP(;
    v_max            = 2.0,    # [m/s]    max linear velocity
    om_max           = 1.0,    # [rad/s]  max angular velocity
    dt               = 0.5,    # [s]      time step
    contact_pen      = -1.0,   #          wall contact penalty
    time_pen         = -0.1,   #          per-step time penalty
    goal_reward      = 10.0,   #          goal terminal reward
    stairs_penalty   = -10.0,  #          stairs terminal penalty
    discount         = 0.95,   #          discount factor
    theta_std        = 0.1,    # [rad]    VonMises rotation noise
    trans_noise_coeff= 0.1,    #          translation noise coefficient
    config           = 1,      #          room configuration {1,2,3}
    sspace           = ContinuousRoombaStateSpace(),
    room             = Room(sspace, layout=baseline, config=config),
    aspace           = RoombaActions(),
)

RoombaPOMDP(; sensor=Lidar(), mdp=RoombaMDP())
```
