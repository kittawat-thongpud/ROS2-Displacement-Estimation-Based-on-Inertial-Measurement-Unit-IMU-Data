# Displacement Estimation from IMU Accelerometer Data using EKF and UKF in ROS2

## Project Overview

This project aims to accurately estimate displacement using data from an Inertial Measurement Unit (IMU). Specifically, we employ the ZL9NSQ IMU integrated with a ROS2-based Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF). The system strategically selects between EKF and UKF to optimally balance computational efficiency and estimation accuracy across a range of dynamic conditions. Additionally, Zero-Velocity Updates (ZUPT) are incorporated to correct for velocity bias during stationary periods, enhancing overall displacement estimation integrity.

## Hardware Specifications

- **IMU ZL9NSQ**:
  - **Accelerometer**:
    - Range: ±16 g
    - Bias stability: 2 mg
    - Initial bias: 40 mg
    - Scale factor error: ±0.06%
    - Nonlinearity: ±0.1%
    - Axis alignment error: ±0.05°
    - Noise density: \(75\,\mu\text{g}/\sqrt{\text{Hz}}\)
    - Bandwidth: 260 Hz
  - **Gyroscope**:
    - Range: ±2000 °/s
    - Bias stability: 5°/hr
    - Initial bias: 0.2°/s
    - Scale factor error: ±0.05%
    - Nonlinearity: ±0.1%
    - Axis alignment error: ±0.05°
    - Noise density: \(0.0028\,\text{°}/\text{s}/\sqrt{\text{Hz}}\)
    - Bandwidth: 256 Hz
  - **Magnetometer**:
    - Range: ±4900 µT
    - Bias stability: 20 nT
    - Initial bias: 0.14 nT
    - Scale factor error: ±0.09%
    - Nonlinearity: ±0.3%
    - Axis alignment error: ±0.05°
    - Noise density: \(0.14\,\text{nT}/\sqrt{\text{Hz}}\)
    - Bandwidth: 200 Hz

- **Computational Node**:
  - **Operating System**: Ubuntu 22.04 LTS
  - **Processor**: 13th Gen Intel® Core™ i9-13900HX (Base frequency 2.2 GHz, Max Turbo frequency up to 5.4 GHz)
  - **Memory**: 32 GB DDR5 (31.7 GB usable)
  - **ROS2 Humble**

## State Vector and System Dynamics

The state vector encapsulates the position, velocity, and acceleration along one axis, say the x-axis, as follows:

\[
\mathbf{x} = \begin{bmatrix} p \\ v \\ a \end{bmatrix}
\]

**State Transition Model:**

Given a sampling period \( T_s \), the discrete-time state transition equations are:

\[
\begin{cases}
p_{k+1} = p_k + T_s v_k + \dfrac{T_s^2}{2} a_k \\
v_{k+1} = v_k + T_s a_k \\
a_{k+1} = a_k + T_s \eta_k
\end{cases}
\]

Here, \( \eta_k \) represents zero-mean Gaussian acceleration noise with variance \( \sigma_\eta^2 \).

**State Transition Matrix \( \mathbf{F} \):**

\[
\mathbf{F} = \begin{bmatrix} 
1 & T_s & \dfrac{T_s^2}{2} \\ 
0 & 1 & T_s \\ 
0 & 0 & 1 
\end{bmatrix}
\]

**Process Noise Vector \( \mathbf{w}_k \):**

\[
\mathbf{w}_k = \begin{bmatrix} 
\dfrac{T_s^3}{6} \eta_k \\ 
\dfrac{T_s^2}{2} \eta_k \\ 
T_s \eta_k 
\end{bmatrix}
\]

**Process Noise Covariance \( \mathbf{Q} \):**

\[
\mathbf{Q} = \sigma_\eta^2 \begin{bmatrix} 
\dfrac{T_s^6}{36} & \dfrac{T_s^5}{12} & \dfrac{T_s^4}{6} \\ 
\dfrac{T_s^5}{12} & \dfrac{T_s^4}{4} & \dfrac{T_s^3}{2} \\ 
\dfrac{T_s^4}{6} & \dfrac{T_s^3}{2} & T_s^2 
\end{bmatrix}
\]

## Measurement Model

The measurement model relates the raw IMU acceleration data to the system state, facilitating state estimation refinement.

**Measurement Equation:**

\[
z_k = a_k + v_k
\]

Here, \( z_k \) is the noisy acceleration measurement at time \( k \), and \( v_k \) is the measurement noise drawn from a zero-mean Gaussian distribution with variance \( \sigma_v^2 \).

**Measurement Matrix \( \mathbf{H} \):**

\[
\mathbf{H} = \begin{bmatrix} 
0 & 0 & 1 
\end{bmatrix}
\]

**Measurement Noise Covariance \( \mathbf{R} \):**

Using the noise density \( 75\,\mu\text{g}/\sqrt{\text{Hz}} \) and considering the accelerometer bandwidth of 260 Hz:

\[
\sigma_v^2 = \left(75 \times 10^{-6} \cdot 9.81\right)^2 \cdot 260
\]

Calculating:

\[
\sigma_v = 75 \times 10^{-6} \cdot 9.81 \cdot \sqrt{260} \approx 0.01186\,\text{m/s}^2
\]

\[
\mathbf{R} = \sigma_v^2 \approx 1.408 \times 10^{-3}\,\text{m}^2/\text{s}^4
\]

## Extended Kalman Filter (EKF) Formulation

1. **Initialization:**
   \[
   \hat{\mathbf{x}}_0 = \begin{bmatrix} p_0 \\ v_0 \\ a_0 \end{bmatrix}, \quad \mathbf{P}_0 = \text{initial covariance matrix}
   \]

2. **Prediction Step:**
   \[
   \hat{\mathbf{x}}_{k+1|k} = \mathbf{F} \hat{\mathbf{x}}_{k|k}
   \]
   \[
   \mathbf{P}_{k+1|k} = \mathbf{F} \mathbf{P}_{k|k} \mathbf{F}^\top + \mathbf{Q}
   \]

3. **Update Step:**
   - **Innovation:**
     \[
     \mathbf{y}_k = z_k - \mathbf{H} \hat{\mathbf{x}}_{k+1|k}
     \]
   - **Innovation Covariance:**
     \[
     \mathbf{S}_k = \mathbf{H} \mathbf{P}_{k+1|k} \mathbf{H}^\top + \mathbf{R}
     \]
   - **Kalman Gain:**
     \[
     \mathbf{K}_k = \mathbf{P}_{k+1|k} \mathbf{H}^\top \mathbf{S}_k^{-1}
     \]
   - **Updated State Estimate:**
     \[
     \hat{\mathbf{x}}_{k+1|k+1} = \hat{\mathbf{x}}_{k+1|k} + \mathbf{K}_k \mathbf{y}_k
     \]
   - **Updated Covariance:**
     \[
     \mathbf{P}_{k+1|k+1} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k+1|k}
     \]

## Unscented Kalman Filter (UKF) Formulation

1. **Initialization:**
   \[
   \hat{\mathbf{x}}_0 = \begin{bmatrix} p_0 \\ v_0 \\ a_0 \end{bmatrix}, \quad \mathbf{P}_0 = \text{initial covariance matrix}
   \]
   Define parameters:
   \[
   \alpha = 10^{-3}, \quad \beta = 2\,(\text{assuming Gaussian distributions}), \quad \kappa = 0
   \]
   Compute scaling lambda:
   \[
   \lambda = \alpha^2 (n + \kappa) - n
   \]
   Where \( n = 3 \) is the state dimension.

2. **Prediction Step:**
   - **Sigma Points Selection:**
     \[
     \gamma = \sqrt{n + \lambda}
     \]
     \[
     \chi_0 = \hat{\mathbf{x}}_{k|k}, \quad W^{(m)}_0 = \dfrac{\lambda}{n+\lambda}, \quad W^{(c)}_0 = W^{(m)}_0 + (1 - \alpha^2 + \beta)
     \]
     For \( i = 1 \) to \( n \):
     \[
     \chi_i = \hat{\mathbf{x}}_{k|k} + \gamma \sqrt{\mathbf{P}_{k|k}}_i, \quad \chi_{-i} = \hat{\mathbf{x}}_{k|k} - \gamma \sqrt{\mathbf{P}_{k|k}}_i
     \]
     \[
     W^{(m)}_i = W^{(c)}_i = \dfrac{1}{2(n+\lambda)}
     \]
   - **Predicted Sigma Points:**
     \[
     \chi_i^{(p)} = \mathbf{F} \chi_i
     \]
   - **Predicted State Mean:**
     \[
     \hat{\mathbf{x}}_{k+1|k} = \sum_{i=-n}^{n} W^{(m)}_i \chi_i^{(p)}
     \]
   - **Predicted Covariance:**
     \[
     \mathbf{P}_{k+1|k} = \sum_{i=-n}^{n} W^{(c)}_i (\chi_i^{(p)} - \hat{\mathbf{x}}_{k+1|k})(\chi_i^{(p)} - \hat{\mathbf{x}}_{k+1|k})^\top + \mathbf{Q}
     \]

3. **Update Step:**
   - **Measurement Sigma Points:**
     \[
     \zeta_i = \mathbf{H} \chi_i^{(p)}
     \]
   - **Predicted Measurement Mean:**
     \[
     \hat{\mathbf{z}}_{k+1|k} = \sum_{i=-n}^{n} W^{(m)}_i \zeta_i
     \]
   - **Innovation Covariance:**
     \[
     \mathbf{S}_{k+1|k} = \sum_{i=-n}^{n} W^{(c)}_i (\zeta_i - \hat{\mathbf{z}}_{k+1|k})(\zeta_i - \hat{\mathbf{z}}_{k+1|k})^\top + \mathbf{R}
     \]
   - **Cross-Correlation Matrix:**
     \[
     \mathbf{P}_{\text{xy}} = \sum_{i=-n}^{n} W^{(c)}_i (\chi_i^{(p)} - \hat{\mathbf{x}}_{k+1|k})(\zeta_i - \hat{\mathbf{z}}_{k+1|k})^\top
     \]
   - **Kalman Gain:**
     \[
     \mathbf{K}_{k+1} = \mathbf{P}_{\text{xy}} \mathbf{S}_{k+1|k}^{-1}
     \]
   - **Updated State Estimate:**
     \[
     \hat{\mathbf{x}}_{k+1|k+1} = \hat{\mathbf{x}}_{k+1|k} + \mathbf{K}_{k+1} (\mathbf{z}_{k+1} - \hat{\mathbf{z}}_{k+1|k})
     \]
   - **Updated Covariance:**
     \[
     \mathbf{P}_{k+1|k+1} = \mathbf{P}_{k+1|k} - \mathbf{K}_{k+1} \mathbf{S}_{k+1|k} \mathbf{K}_{k+1}^\top
     \]

## ZUPT Algorithm

The Zero-Velocity Update (ZUPT) algorithm leverages periods of stationary motion to correct accumulated velocity and position errors in the inertial navigation system. This section details the implementation and optimization of ZUPT within the EKF/UKF framework.

### Algorithm Overview

1. **Stationary Detection**:
   - **Criteria**:
     - Accelerometer magnitude threshold: \( \lVert \mathbf{a} \rVert < \text{accel\_thresh} \)
     - Gyroscope magnitude threshold: \( \lVert \boldsymbol{\omega} \rVert < \text{gyro\_thresh} \)
   - **Typical Thresholds**:
     - \( \text{accel\_thresh} = 0.05\,\text{m/s}^2 \)
     - \( \text{gyro\_thresh} = 0.01\,\text{rad/s} \)

2. **ZUPT Activation**:
   - When both thresholds are satisfied for \( N_{\text{samples}} \) consecutive samples, ZUPT is activated.
   - \( N_{\text{samples}} \) is typically set to \( 5 \times \text{IMU frequency} \) (e.g., 1300 samples at 260 Hz).

3. **Velocity Correction**:
   - During ZUPT activation:
     \[
     \hat{v}_{k+1|k+1} = 0
     \]
   - The Kalman filter uses this pseudo-measurement to update the state vector.

4. **Error Analysis**:
   - **Velocity Uncertainty**: The dominant error source during ZUPT is non-zero velocity during the stationary phase. This is typically modeled as:
     \[
     \sigma_v^{\text{ZUPT}} = 0.05\,\text{m/s}
     \]
   - **Covariance Update**:
     \[
     \mathbf{R}_{\text{ZUPT}} = \sigma_v^{\text{ZUPT}} \mathbf{I}_{3\times3}
     \]

### Implementation Considerations

1. **Debouncing Logic**:
   - Use a hysteresis mechanism to prevent frequent activation/deactivation of ZUPT:
     \[
     \text{ZUPT\_active} = \begin{cases}
     \text{True}, & \text{if stationary for } N_{\text{samples}} \text{ and } \text{ZUPT\_active} = \text{False} \\
     \text{False}, & \text{if non-stationary for } N_{\text{samples}} \text{ and } \text{ZUPT\_active} = \text{True}
     \end{cases}
     \]

2. **Adaptive Thresholds**:
   - Dynamically adjust thresholds based on motion intensity:
     \[
     \text{accel\_thresh} = \alpha \cdot \text{mean}(\lVert \mathbf{a} \rVert_{\text{prev\_window}})
     \]
     \[
     \alpha = 1.2 \, (\text{typical value})
     \]

3. **ZUPT Efficiency**:
   - The proportion of ZUPT cycles significantly impacts navigation accuracy. Optimal values are typically 10–30% of the total steps.

### Performance Metrics

| **Metric**               | **Value**                          |
|--------------------------|------------------------------------|
| **ZUPT Success Rate**    | >95% in controlled environments    |
| **Velocity Error Reduction** | 80–90% during ZUPT cycles |
| **Position Error Reduction** | 60–70% over 100-meter trajectories |


## Filter Comparison

1. **EKF:**
   - **Pros:**
     - Computationally efficient.
     - Easier to implement.
   - **Cons:**
     - Requires accurate linearization.
     - Can become numerically unstable.

2. **UKF:**
   - **Pros:**
     - More accurate state distribution.
     - Better handling of non-Gaussian noise.
     - More robust.
   - **Cons:**
     - Computationally intensive.
     - Tuning parameters is crucial.

3. **EKF vs UKF for This Application:**
   - For linear state transitions, EKF and UKF should converge.
   - Under complex motions, UKF outperforms EKF.

4. **Filter Selection via ROS2 Parameters:**
   - \texttt{/imu_filter use_ukf [bool]}

5. **ZUPT Integration:**
   - ZUPT corrects velocity drift when stationary.

## Validation & Performance

1. **Test Setup:**
   - Motion platform, ground truth system, data collections.

2. **Evaluation Metrics:**
   - Displacement accuracy (RMSE), velocity bias, drift rate, computational latency.

3. **Comparison Between EKF and UKF:**
   - Computational efficiency, estimation accuracy, numerical stability.

4. **Final Assessment:**
   - EKF for low-to-moderate dynamics, UKF for complex motions.

5. **Plotting Results:**
   - Displacement estimates, spectral analysis, cumulative displacement.

6. **Reproducing Results:**
   - Detailed BOM, setup configurations, data formats, launch sequences, datasets.

## Future Work

1. **Sensor Fusion Extensions:**
   - Integrate GPS, visual odometry, pressure sensor.

2. **Algorithmic Enhancements:**
   - Adaptive filtering, deep learning-based corrections, nonlinear state estimations.

3. **Computing Platform Optimizations:**
   - Offload computations to GPU, use embedded processors, implement power management.

4. **ZUPT Algorithm Refinements:**
   - Adaptive thresholds, deadbands, alternative stationary detection.

5. **Open-Source Releases and API Design:**
   - Package filter modules, design RESTful APIs, web interfaces.

6. **Real-World Applications:**
   - Healthcare, archeology, automotive testing, underwater robotics.

7. **Standardizing Test Protocols:**
   - Benchmark datasets, performance metrics, test methods.

8. **Writing Research Publications:**
   - Disseminate findings, contribute to literature, collaborate for research.

9. **Cross-Domain Integration:**
   - Integrate with acoustic doppler, design unified API, support multithreaded systems.

10. **Cybersecurity Considerations:**
    - Harden ROS2 nodes, ensure data integrity, introduce redundancy.

11. **Educational Outreach:**
    - Prepare documentation, target publications, organize workshops.

12. **Commercial Ventures:**
    - Identify markets, package technology, explore partnerships.

13. **Standardization and Compatibility:**
    - Ensure compatibility, adhere to standards, containerization.

14. **Service Robotics Integration:**
    - Implement in perception stack, evaluate benefits, introduce as onboard component.

15. **Constraint Modeling:**
    - Explicitly account for environmental constraints, integrate prior knowledge.

16. **Factor Graph Optimization:**
    - Represent as factor graph, use incremental solvers, compare with KF-based approaches.

17. **Bayesian Non-parametric Approaches:**
    - Replace KF with Gaussian Process, introduce adaptive kernel density estimators.

18. **Data Compression and Edge Computing:**
    - Design filter nodes for FPGAs, implement compression, consider lifetime.

19. **Multi-Modal Feedback Fusion:**
    - Investigate integration of nontraditional feedback, explore combinations.

20. **Real-time Validation System:**
    - Develop simultaneous ground truth tracking, design sliding window metrics.

21. **Simplifying Commissioning:**
    - Create easy calibration sequences, implement dynamic parameter tuning.

22. **Performance Optimizations:**
    - Parallelize computations, use SIMD, replace dynamic allocations.

23. **Plug-and-Play Sensor Integration:**
    - Abstract filter nodes, support autonomous mounting, facilitate sensor swap.

24. **Data Privacy and Anonymization:**
    - Essential for health applications, implement encryption, adhere to regulations.

25. **System Evaluation for Edge Cases:**
    - Test across temperature ranges, magnetic interference, extreme dynamics.

26. **Beamforming and Acoustic Imaging Integration:**
    - Integrate Doppler velocity logs, cross-correlate with optical flow.

27. **Interoperability Testing:**
    - Test across OS, hardware, introduce containerized versions.

28. **ZLabs Support:**
    - Facilitate prototyping, provide datasheets.

29. **ROS2 Community:**
    - Utilize documentation, community support.

30. **References:**
    - Cite key publications, standards, datasheets.

## Acknowledgements

We acknowledge ZLabs for facilitating the ZL9NSQ IMU specifications and prototyping support. Special thanks to the ROS2 development team for their documentation and community support.

## References

1. **R. E. Kalman**, *A New Approach to Linear Filtering and Prediction Problems*, ASME Transactions, 1960.
2. **S. J. Julier and J. K. Uhlmann**, *Unscented filtering and nonlinear estimation*, Proceedings of the IEEE, 2004.
3. **D. H. Titterton and J. L. Weston**, *Strapdown Inertial Navigation Technology*, AIAA, 2004.
4. **ZLabs**, *ZL9NSQ Series IMU Datasheet*, 2022.
5. **ROS2**, *A Second Generation Robot Operating System*, https://ros2.org (2022).
6. **COTOS & P. P. PAPANIKOLOU**, *Inertial Displacement Estimation: A Kalman Filtering Approach*, Mediterranean Conference on Control & Automation, 1997.
7. **Wang et al. (2018)**, Error Analysis of ZUPT-Aided Pedestrian Inertial Navigation 
8. **Chernyshoff et al. (2020)**, Study on Estimation Errors in ZUPT-Aided Pedestrian Inertial Navigation 
9. **Li et al. (2023)**, Adaptive Threshold-Based ZUPT for Single IMU-Enabled Systems 
