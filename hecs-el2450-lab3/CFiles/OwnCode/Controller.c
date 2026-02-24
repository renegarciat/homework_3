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
// Uncomment to do only a rotation control.
// Implement a controller that stabilize theta in theta_R.
// We can derive the new angular speed
// Derive theta_R
theta_R = atan2(yg, xg)*180/M_PI; // Desired orientation of the robot in degrees.
// Calculate angle error with wrapping to [-180, 180]
angle_error = theta_R - theta;
if (angle_error > 180) angle_error -= 360;
else if (angle_error < -180) angle_error += 360;
omega = K_PSI_1 * angle_error;
// Convert angular speed to  left and right, given that v = 0.
left_0 = -omega/2.0;
right_0 = omega/2.0;

// calculate the velocity vector
v_c[0] = cos(theta*(M_PI/180.0)); // Desired velocity in the x direction in m/s.
v_c[1] = sin(theta*(M_PI/180.0)); // Desired velocity in the y direction in m/s.
// Calculate the error in position
delta_0[0] = (xg-x) / 100.0; // Position error in the x direction in m.
delta_0[1] = (yg-y) / 100.0; // Position error in the y direction in m.
// Calculate the inner product of the velocity vector and the position error
d_0 = v_c[0]*delta_0[0] + v_c[1]*delta_0[1];
// Calculate the desired translational velocity
v = K_OMEGA_1*d_0; // [rad/s]
v = v*180.0/M_PI; // Convert to 1°/s
// left_1 = v;
// right_1 = v;

// Task 6. Rotation control only.
// left = left_0;
// right = right_0;
// Task 8. Translation control only.
// left = left_1;
// right = right_1;
// Task 9. Combine the controllers.
left = (int) (left_0 + left_1);
right = (int) (right_0 + right_1);