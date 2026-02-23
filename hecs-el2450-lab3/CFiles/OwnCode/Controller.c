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
// Task 6. Implement a controller that stabilize theta in theta_R.
//  We can derive the new angular speed
omega = K_PSI_1*(theta_R-theta);
// Convert angular speed to  left and right, given that v = 0.
left = -omega/2;
right = omega/2;

// Uncomment to do only a pose control.
// Task 7. Maintain the robot in the current position while rotating.
// calculate the velocity vector
v_c[0] = cos(theta*(M_PI/180.0)); // Desired velocity in the x direction in m/s.
v_c[1] = sin(theta*(M_PI/180.0)); // Desired velocity in the y direction in m/s.
// Calculate the error in position
delta_0[0] = (x0-x) / 100.0; // Position error in the x direction in m.
delta_0[1] = (y0-y) / 100.0; // Position error in the y direction in m.
// Calculate the inner product of the velocity vector and the position error
d_0 = v_c[0]*delta_0[0] + v_c[1]*delta_0[1];
// Calculate the desired translational velocity
v = K_OMEGA_1*d_0; // [m/s]
left = v*100; // We dont know the units of left and right, but assume cm/s.
right = v*100;
