# 005 Drop Landing Right Leg
Measure and train knee flexion angle, trunk side angle, and trunk forward angle during single right leg drop landing. 

### Nodes Required: 5
- Sensing (3): 
  - shank: lateral, switch pointing up
  - thigh: lateral, switch pointing up
  - trunk: back, switch pointing up
- Feedback (2): 
  - min feedback: Node used to provide haptic feedback when the participant is below the minimum threshold set in the app configuration panel.
  - max feedback: Node used to provide haptic feedback when the participant exceeds the maximum threshold set in the app configuration panel.

## Algorithm & Calibration
### Algorithm Information
The raw quaternions from the IMU are converted to Euler angles, and the roll angle is extracted using well established mathematical principles. If you'd like to learn more about quaternion to Euler angles calculations, we suggest starting with this Wikipedia page: [Conversion between quaternions and Euler angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

### Calibration Process:
The participant should be standing vertically upright, No other initial static calibration is performed to compensate for misalignment with the segments.

## Description of Data in Downloaded File
### Calculated Fields
- time (sec): time since trial start
- Angle: angle being measured.
- landphase: pre_land (before initial contact), land (after initial contact during land task), post_land (after land task is complete)
- TSA (deg): trunk side angle (medial-lateral), positive is to the right
- TFA (deg): trunk forward angle (anterior-posterior), positive is forward
- KFA (deg): knee flexion angle for the right leg
- KAA (deg): knee adduction angle for the right leg
- min_threshold: The lower threshold set by the user in the App configuration Panel.
- max_threshold: The upper threshold set by the user in the App configuration Panel.
- min_feedback_state: feedback status for if the sensor has crossed the min angle threshold. 
  - 0 is “feedback off”
  - 1 is “feedback on” 
- max_feedback_state: feedback status for if the sensor has crossed the max angle threshold. 
  - 0 is “feedback off”
  - 1 is “feedback on” 
### Sensor Raw Data Values 
Please Note: Each of the columns listed below will be repeated for each sensor
  - SensorIndex: index of raw sensor data
  - AccelX/Y/Z (m/s^2): raw acceleration data
  - GyroX/Y/Z (deg/s): raw gyroscope data
  - MagX/Y/Z (μT): raw magnetometer data
  - Quat1/2/3/4: quaternion data (Scaler first order)
  - Sampletime: timestamp of the sensor value
  - Package: package number of the sensor value


# Development and App Processing Loop
The best place to start with developing an or modifying an app, is the [SageMotion Documentation](http://docs.sagemotion.com/index.html) page.
