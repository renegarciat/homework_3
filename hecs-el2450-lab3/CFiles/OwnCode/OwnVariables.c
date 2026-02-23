// This file is called once.
const double R_TRUE = 0.033; // Radius of the wheel in meters
const double L_TRUE = 0.160; // Distance between the wheels in meters
const double H = 1.0; // Sampling time in seconds.
const double K_PSI_1 = ((L_TRUE)/(2*H*R_TRUE)); // In the middle of the stability range.
printf("The controller gain K_PSI_1 is set to %f\n", K_PSI_1);
const double K_OMEGA_1 = 1/(2*H*R_TRUE); // Gain for the angular velocity in the pose control.
printf("The controller gain K_OMEGA_1 is set to %f\n", K_OMEGA_1);
int theta_R = 0; // Desired orientation of the robot in degrees.
int theta_next = 0; //
int omega = 0; // Angular velocity of the robot in degrees per second.
int v = 0; // Translational velocity of the robot in cm/s.
// Define the velocity vector
double v_c[] = {0, 0}; // Desired velocity vector of the robot in the x and y direction in cm/s.
double delta_0[] = {0, 0}; // Position error in the x and y direction in cm.
double d_0 = 0; // Inner product of the velocity vector and the position error in cm^2/s.  