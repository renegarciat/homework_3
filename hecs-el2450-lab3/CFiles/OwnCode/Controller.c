/*  When:
    - The Activate Simulatio is active.
    - The Automatic Control is active.
    Then:
    - This file is periodically called every h. TODO: Determine where is the actual sampling period stored/defined. 
    Input Variables:
    - x, y, theta: (in cm and degrees) the current pose of the robot as measured by the mocap system.
    - int x0, y0: the start position of the robot in cm.
    - int xg, yg: the goal position of the robot in cm.
    Output Variables:
    - left, right: the control input to the robot velocity of the left and right wheel. The control input is limited to the range [-800, 800]. Units?

    The purpose of the algorithm is first to rotate and then to move straight.
*/
// Task 6
theta_R = atan2(yg - y0, xg - x0)*180/M_PI; // Desired orientation of the robot in degrees.
// Calculate angle error with wrapping to [-180, 180]
angle_error = theta_R - theta;
if (angle_error > 180) angle_error -= 360;
else if (angle_error < -180) angle_error += 360;
omega = K_PSI_1 * angle_error;
// Convert angular speed to  left and right, given that v = 0.
left_0 = -omega/2.0;
right_0 = omega/2.0;

// Task 8
// calculate the velocity component vector
v_c[0] = cos(theta*(M_PI/180.0));
v_c[1] = sin(theta*(M_PI/180.0));
// Calculate the error in position
delta_0[0] = (x0-x) / 100.0; // Position error in the x direction in m.
delta_0[1] = (y0-y) / 100.0; // Position error in the y direction in m.
// Calculate the inner product of the velocity vector and the position error
d_0 = v_c[0]*delta_0[0] + v_c[1]*delta_0[1]; // [m]
// Calculate the desired translational velocity
v = K_OMEGA_1*d_0; // [1/s or rad/s]
v = v*180.0/M_PI; // Convert to 1°/s
left_1 = v;
right_1 = v;

// Task 6. Rotation control only.
// left = left_0;
// right = right_0;
// Task 8. Translation control only.
// left = left_1;
// right = right_1;
// Task 9. Combine the controllers.
task9_left = left_0 + left_1;
task9_right = right_0 + right_1;
left = (int) task9_left;
right = (int) task9_right;

// ----------------------
// Task 11 (Go-To-Goal Part I)
// ----------------------

delta_g[0] = xg - x;
delta_g[1] = yg - y;

theta_g = atan2(yg-y0, xg-x0);  // radians

v_g[0] = cos(theta_g);
v_g[1] = sin(theta_g);

d_g = v_g[0]*delta_g[0] + v_g[1]*delta_g[1];
v = K_OMEGA_2 * d_g;
v = v*180.0/M_PI; // Convert to 1°/s
right_0 = v;
left_0  = v;
// ----------------------
// Task 13 (Go-To-Goal Part II)
dp = p*sin(theta_g - theta*(M_PI/180.0));
omega = K_PSI_2 * (dp);
omega = omega*180.0/M_PI; // Convert to 1°/s
printf("The value of dp is %f\n", dp);
printf("The value of omega is %f\n", omega);
// Since v = 0
left_1 = -omega/2.0;
right_1 = omega/2.0;
left = (int) left_1 + (int) left_0;
right = (int) right_1 + (int) right_0;
// // Task 15. Both controls together.
task15_left = left_0 + left_1;
task15_right = right_0 + right_1;
// left = (int) (left_0 + left_1);
// right = (int) (right_0 + right_1);

// Task 17
pos_error = sqrt(delta_0[0]*delta_0[0] + delta_0[1]*delta_0[1]);
if (task17_state == 0) {
    // Check if we are close enough to (x0, y0) and oriented towards (xg, yg)
    // or if the rotor speed is below the minimum threshold
    if ((pos_error < pos_tol && fabs(angle_error) < ang_tol) || 
        (fabs(task9_left) < min_speed && fabs(task9_right) < min_speed)) {
        task17_state = 1;
        printf("Switching to Go-To-Goal (Task 13 & 15)\n");
    }
}

if (task17_state == 0) {
    left = (int) task9_left;
    right = (int) task9_right;
} else {
    left = (int) task15_left;
    right = (int) task15_right;
}

// 5. Motor Saturation (Clamping)
if (left > 800) left = 800;
else if (left < -800) left = -800;

if (right > 800) right = 800;
else if (right < -800) right = -800;

// Task 18