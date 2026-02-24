// This file is called once.
const double H = 1.0; // [s] Sampling time.
const double R_TRUE = 0.033; // Radius of the wheel in meters
const double L_TRUE = 0.160; // Distance between the wheels in meters
const double K_PSI_1 = ((L_TRUE)/(H*R_TRUE)); // In the middle of the stability range.
printf("The controller gain K_PSI_1 is set to %f\n", K_PSI_1);
const double K_OMEGA_1 = 1/(2*H*R_TRUE); // Gain for the angular velocity in the pose control.
printf("The controller gain K_OMEGA_1 is set to %f\n", K_OMEGA_1);
double theta_R = 0; // Desired orientation of the robot in degrees.
double omega = 0; // Angular velocity of the robot in degrees per second.
double angle_error = 0;
double v = 0; // Translational velocity of the robot in cm/s.
// Define the velocity vector
double v_c[] = {0, 0}; // Desired velocity vector of the robot in the x and y direction in cm/s.
double delta_0[] = {0, 0}; // Position error in the x and y direction in cm.
double d_0 = 0; // Inner product of the velocity vector and the position error in cm^2/s.
double left_0 = 0; // [1°/s] Control input to the left wheel for the rotation control.
double right_0 = 0; // [1°/s] Control input to the right wheel for the rotation control.
double left_1 = 0; // [1°/s] Control input to the left wheel for the pose control.
double right_1 = 0; // [1°/s] Control input to the right wheel for the pose control.