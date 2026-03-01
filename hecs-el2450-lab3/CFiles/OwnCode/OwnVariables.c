// This file is called once.
const double H = 1.0;                             // [s] Sampling time.
const double R_TRUE = 0.033;                      // Radius of the wheel in meters
const double L_TRUE = 0.160;                      // Distance between the wheels in meters
const double K_PSI_1 = ((L_TRUE) / (H * R_TRUE)); // In the middle of the stability range.
printf("The controller gain K_PSI_1 is set to %f\n", K_PSI_1);
const double K_OMEGA_1 = 1 / (H * R_TRUE); // Gain for the angular velocity in the pose control.
printf("The controller gain K_OMEGA_1 is set to %f\n", K_OMEGA_1);
double theta_R = 0; // Desired orientation of the robot in degrees.
double omega = 0;   // Angular velocity of the robot in degrees per second.
double angle_error = 0;
double v = 0; // Translational velocity of the robot in 1°/s.
// Define the velocity vector
double v_c[] = {0, 0};     // Desired velocity vector of the robot in the x and y direction in cm/s.
double delta_0[] = {0, 0}; // Position error in the x and y direction in cm.
double d_0 = 0;            // Inner product of the velocity vector and the position error in cm^2/s.
double left_0 = 0;         // [1°/s] Control input to the left wheel for the rotation control.
double right_0 = 0;        // [1°/s] Control input to the right wheel for the rotation control.
double left_1 = 0;         // [1°/s] Control input to the left wheel for the pose control.
double right_1 = 0;        // [1°/s] Control input to the right wheel for the pose control.
double dx = 0;
double dy = 0;
double line_norm = 0;
double vg_perp_x = 0;
double vg_perp_y = 0;
double theta_rad = 0;
double xp = 0;
double yp = 0;
double vp_x = 0;
double vp_y = 0;
double d_p = 0;
const double R_CM = 100 * R_TRUE; // Convert radius to cm

// Stability condition: 0 < H*R*K < 2
// Choose middle of interval: HRK = 1
const double K_OMEGA_2 = 1 / (H * R_CM);
printf("The controller gain K_OMEGA_2 is set to %f\n", K_OMEGA_2);
// Variables for Task 11
double delta_g[] = {0, 0}; // Goal position error in cm
double v_g[] = {0, 0};     // Unit direction toward goal
double d_g = 0;            // Projected distance to goal
double theta_g = 0;        // Desired angle in radians
// Task 13
double P = 0.5; // around 2 times the body length
double dp = 0;
const double alpha = 0.5;
double K_PSI_2 = alpha * (L_TRUE / (H * R_TRUE * P));
printf("The controller gain K_PSI_2 is set to %f\n", K_PSI_2);
double invariant_pole = 0;

// ----------------------
// Task 17
// ----------------------
int task17_state = 0;
double pos_tol = 0.05;   // 5 cm = 0.05 m
double ang_tol = 5.0;    // 5 degrees
double min_speed = 10.0; // 10 degrees/s
double task9_left = 0;
double task9_right = 0;
double task15_left = 0;
double task15_right = 0;
double pos_error = 0;

// Task 18
// -----------------------
int task18_active = 1;      // chosen here as 1 if Task 18 is active, 0 otherwise
int new_goal_to_be_set = 1; // 1 if a new goal should be set, 0 otherwise
int path_node_num = 0;      // Index of the node in the path most recentrly visited by the robot
// double path_node_pos[2] = {0, 0}; // Position of the node in the path most recently visited by the robot in meters
// double goal_positions[4][2] = {{0.5, 0.5}, {0.5, -0.5}, {-0.5, -0.5}, {-0.5, 0.5}}; // Goal positions in meters
// double prefix_nodes[7] = {6, 7, 18, 17, 16, 21, 20}; // Node indices for the prefix path
// double suffix_nodes[2] = {21, 20}; // Node indices for the suffix path
double prefix[7][2] = {{-125, 125}, {-75, 125}, {-25, 125}, {-25, 75}, {-25, 25}, {25, 25}, {25, 75}}; // Prefix for goal positions in cm
double suffix[2][2] = {{25, 25}, {25, 75}};                                                            // Suffix for goal positions in cm
// -----------------------
