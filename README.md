# PID Controller Simulation

A modular PID control simulation in Python. The controller is tested on two simple physical plants (a 1-D car and a drone moving along its vertical axis) with actuator saturation, anti-windup, sensor noise, derivative-on-measurement, and automated Ziegler–Nichols tuning. Responses are evaluated with standard step-response metrics.

Below is the comparison of controller performance on 1-D drone dynamics between manually tuned PID gains and those tuned using the Ziegler-Nichols method

<table>
  <tr>
    <td align="center"><b>Manual tuning</b></td>
    <td align="center"><b>Ziegler–Nichols tuning</b></td>
  </tr>
  <tr>
    <td><img src="results/manual_tuning.png" alt="Manual tuning response" width="100%"></td>
    <td><img src="results/ZN_tuning.png" alt="Ziegler-Nichols tuning response" width="100%"></td>
  </tr>
</table>

Each plot shows position, velocity, error, and the P / I / D / total control contributions over time.

## Features

- **PID controller** with derivative-on-measurement and conditional-integration anti-windup
- **Two plants:** a 1-D car (mass + viscous friction) and a drone along its vertical axis (gravity, thrust limit, ground contact)
- **Actuator saturation** and **Gaussian sensor noise** (configurable)
- **Automated Ziegler–Nichols tuning:** finds the ultimate gain and oscillation period by simulation, then computes PID gains
- **Performance report:** rise time, overshoot, settling time, steady-state error
- **Diagnostic plots** of the state, error, and each PID term

## Project structure

| File | Description |
|------|-------------|
| `pid.py` | `PID` class: control law, anti-windup, history of P/I/D terms and error |
| `plant.py` | `Plant_Car` and `Plant_Drone` models |
| `simulator.py` | `Simulator`: closed-loop simulation, plotting, and performance metrics |
| `ZieglerNichols.py` | Contains methods `find_ultimate_gain()` and `ziegler_nichols_pid(Ku, Tu)` which automate the Ziegler-Nichols tuning method|
| `main.py` | Demo script: compares manual tuning against Ziegler–Nichols on the drone |
| `results/` | Generated plots and the printed performance report |

## How it works

### Control law

With error `e = r − y` (setpoint minus measured output):

$$u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\, d\tau - K_d \frac{dy}{dt}$$

- **Derivative on measurement.** The D term differentiates the measured output rather than the error, so a change in setpoint does not cause a derivative spike. The first call initializes the previous output to avoid a startup spike.
- **Anti-windup (conditional integration).** When the command would exceed the actuator limits and the error would push it further into saturation, the integrator is frozen. The plant clips the command to its actuator limits.

### Plants

State is `[position, velocity]`, integrated with a fixed time step (semi-implicit Euler).

- **Car:** `m·a = u − b·v`
- **Drone (vertical axis):** `m·a = u − b·v − m·g` with `g = 10`, a thrust limit, and a ground at `y = 0`

### Ziegler–Nichols tuning

The closed-loop method, automated in simulation:

1. Set `Ki = Kd = 0` and increase `Kp` in small steps.
2. Stop when the response shows sustained oscillation (the last four peak amplitudes agree within a tolerance). That `Kp` is the ultimate gain `Ku`, and the peak spacing gives the period `Tu`.
3. Compute the gains: `Kp = 0.6·Ku`, `Ki = 1.2·Ku / Tu`, `Kd = 0.075·Ku·Tu`.

## Quick start

Requires Python 3.9 or newer.

```bash
git clone https://github.com/johnjooyoungkim/pid-controller.git
cd pid-controller
pip install -r requirements.txt
python main.py
```

Plots are saved to `results/` and the performance report is printed to the terminal.

To change the experiment, edit the parameters at the top of `main.py`: setpoint (`cmd`), time step (`dt`), number of steps (`n`), actuator limits (`u_min`, `u_max`), sensor noise (`noise_std`), and the PID gains.

## Results

Experiment: drone plant (mass 1, friction 0.05), setpoint 150, `dt = 0.1 s`, 200 s simulation, thrust limited to ±15, no sensor noise.

| Tuning | Kp | Ki | Kd | Rise time | Overshoot | Settling time (2%) | Steady-state error |
|--------|------|--------|-------|-----------|-----------|--------------------|--------------------|
| Manual | 0.50 | 0.02 | 0.50 | 5.6 s | 0.0 % | 53.4 s | 0.007 |
| Ziegler–Nichols | 0.114 | 0.0158 | 0.205 | 6.3 s | 15.6 % | 63.5 s | −0.001 |

Ziegler–Nichols found `Ku = 0.19` and `Tu = 14.4 s`.

Metric definitions: rise time is the 10% → 90% time, settling time is the last time the response leaves a ±2% band around the setpoint, and steady-state error is the average over the last 5 s.

**Observations**

- The integral term converges to about 10, which equals `m·g`: the integrator effectively learns the hover thrust needed to cancel gravity. This is why the manually tuned response, which rises quickly without overshooting the setpoint, still has a long settling tail while the integrator winds up to that value.
- Ziegler–Nichols gives a lightly damped, oscillatory response (15.6 % overshoot) that settles later but with a smaller steady-state error. This is the characteristic behavior of the Ziegler–Nichols rule, which is meant as a fast starting point for further hand tuning rather than a finished design.

## Limitations and next steps

- Single-axis plants only; no coupled or multi-axis dynamics
- No low-pass filter on the derivative term, so the D term amplifies sensor noise
- Fixed-step Euler integration and step-response metrics only (no disturbance injection yet)
- Planned: noise and disturbance sweeps, a filtered derivative, and a comparison against state-space control in [lqr-control-sim](https://github.com/johnjooyoungkim/lqr-control-sim)

## Author

Jooyoung Kim, Electrical Engineering at Georgia Tech. Built while studying control theory independently.
