# **Displacement Estimation Using IMU DFR1179 with ZUPT + Kalman Filter in ROS2**

In this project, displacement estimation was achieved using an **IMU DFR1179** sensor and a combination of advanced techniques including **Butterworth low-pass filtering**, **Zero Velocity Update (ZUPT)**, and **Kalman filtering**. By filtering out noise from the raw accelerometer data and fusing it with gyroscope measurements, accurate displacement estimates were obtained in real-time. 

The key achievements of this project include:
- **Noise Reduction**: The application of a Butterworth low-pass filter significantly improved the signal-to-noise ratio in the accelerometer data.
- **Drift Mitigation**: ZUPT and Kalman filtering helped to minimize the effects of drift, a common issue in IMU-based displacement estimation.
- **Real-time Performance**: The use of ROS2 enabled efficient data processing and visualization of displacement estimates in real time.

### **Future Work**
Future improvements to this project could include:
- **Integrating Additional Sensors**: The addition of GPS or visual odometry could further enhance the accuracy of displacement estimation in outdoor or larger-scale environments.
- **Advanced Filtering Techniques**: Exploring adaptive filtering methods or more advanced sensor fusion algorithms could further reduce errors and improve robustness under various motion conditions.

### **Acknowledgements**
We would like to thank the developers and contributors of **ROS2** for providing a powerful framework, as well as the manufacturers of the **IMU DFR1179**, whose reliable sensor performance was integral to the success of this project.
