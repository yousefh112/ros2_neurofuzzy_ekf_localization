# ros2_neurofuzzy_ekf_localization
ROS 2 Project for hybrid mobile robot localization . It utilizes dual Extended Kalman Filters for outdoor GPS-INS-odometry fusion . During GPS outages, it transitions to an ANN for position prediction, utilizing a Fuzzy Logic System to compute dynamic blending parameters and mitigate wheel slippage .
