/*  When:
    - The Activate Simulatio is active.
    - The Automatic Control is active.
    Then:
    - This file is periodically called every h.
    Input Variables:
    - x, y, theta: (in cm and degrees) the current pose of the robot as measured by the mocap system.
    - int x0, y0: the start position of the robot in cm.
    - int xg, yg: the goal position of the robot in cm.
    Output Variables:
    - left, right: the control input to the robot velocity of the left and right wheel. The control input is limited to the range [-800, 800]. Units?

    The purpose of the algorithm is first to rotate and then to move straight.
*/

// Triggering task 18 when position is set to (-1.25, 1.25) and then follow the prefix path to (0.25, 0.75) before switching to the suffix path to (0.25, 0.25) and back to (0.75, 0.75).
// ----------------------
if (task18_active == 1 && new_goal_to_be_set == 1)
{
    x0 = x;
    y0 = y;
    if ((fabs(x - prefix[0][0]) < pos_tol) && (fabs(y - prefix[0][1]) < pos_tol) && path_node_num == 0)
    {
        xg = prefix[path_node_num + 1][0];
        yg = prefix[path_node_num + 1][1];
    }
    else if (0 < path_node_num && path_node_num < 6)
    {
        xg = prefix[path_node_num + 1][0];
        yg = prefix[path_node_num + 1][1];
    }
    else if (path_node_num == 6)
    {
        xg = suffix[0][0];
        yg = suffix[0][1];
    }
    else if (path_node_num == 7)
    {
        xg = suffix[1][0];
        yg = suffix[1][1];
    }
    else if (path_node_num == 8)
    {
        xg = suffix[0][0];
        yg = suffix[0][1];
    }
    if (path_node_num >= 0 && path_node_num < 8)
    {
        path_node_num++;
    }
    else
    {
        path_node_num--;
    }
    printf("New goal\n");
    new_goal_to_be_set = 0; // Reset the flag after setting a new goal
}
// ----------------------

dx = xg - x0;
dy = yg - y0;
// Task 6
theta_R = atan2(dy, dx) * 180 / M_PI; // Desired orientation of the robot in degrees.
// Calculate angle error with wrapping to [-180, 180]
angle_error = theta_R - theta;
if (angle_error > 180)
    angle_error -= 360;
else if (angle_error < -180)
    angle_error += 360;
omega = K_PSI_1 * angle_error;
// Convert angular speed to  left and right, given that v = 0.
left_0 = -omega / 2.0;
right_0 = omega / 2.0;

// Task 8
// calculate the velocity component vector
v_c[0] = cos(theta * (M_PI / 180.0));
v_c[1] = sin(theta * (M_PI / 180.0));
// Calculate the error in position
delta_0[0] = (x0 - x) / 100.0; // Position error in the x direction in m.
delta_0[1] = (y0 - y) / 100.0; // Position error in the y direction in m.
// Calculate the inner product of the velocity vector and the position error
d_0 = v_c[0] * delta_0[0] + v_c[1] * delta_0[1]; // [m]
// Calculate the desired translational velocity
v = K_OMEGA_1 * d_0;  // [1/s or rad/s]
v = v * 180.0 / M_PI; // Convert to 1°/s
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

// ----------------------
// Task 11 (Go-To-Goal Part I)
// ----------------------

delta_g[0] = xg - x;
delta_g[1] = yg - y;

theta_g = atan2(dy, dx); // radians

v_g[0] = cos(theta_g);
v_g[1] = sin(theta_g);

d_g = v_g[0] * delta_g[0] + v_g[1] * delta_g[1];
v = K_OMEGA_2 * d_g;
v = v * 180.0 / M_PI; // Convert to 1°/s
right_0 = v;
left_0 = v;
// Task 13
//  1. Line direction

line_norm = sqrt(dx * dx + dy * dy);

if (line_norm < 1e-6)
{
    left = 0;
    right = 0;
}
else
{
    // 2. Perpendicular unit vector
    vg_perp_x = dy / line_norm;
    vg_perp_y = -dx / line_norm;

    // 3. Virtual point
    theta_rad = theta * M_PI / 180.0;

    // convert P_LOOKAHEAD (m) → cm
    xp = x + (P * 100.0) * cos(theta_rad);
    yp = y + (P * 100.0) * sin(theta_rad);

    // 4. Vector from start to virtual point
    vp_x = xp - x0;
    vp_y = yp - y0;

    // 5. Exact dp (cm)
    d_p = vg_perp_x * vp_x + vg_perp_y * vp_y;

    // convert cm → meters
    d_p = d_p / 100;

    // 6. Controller
    omega = K_PSI_2 * d_p * 180.0 / M_PI;
    v = 0;

    // 7. Wheel speeds
    right_1 = (int)(v + omega / 2.0);
    left_1 = (int)(v - omega / 2.0);
}
// ----------------------
// Task 14 (Simulation & Pole Invariance Test)
// ----------------------
// 2. Set your test lookahead distance (p) in meters.
// TODO: Run the simulation three times, changing this to 0.1, 0.5, and 2.0
// P = 0.5;
// x0 = -0.37;
// y0 = 0.23;
// dx = xg-x0;
// dy = yg-y0;
// 3. Maintain an invariant pole.
// We choose a constant C (e.g., 2.0) so the response time stays the same.
// invariant_pole = 0.5;
// K_PSI_2 = invariant_pole / P;
// printf("The controller gain K_PSI_2 is set to %f\n", K_PSI_2);
// line_norm = sqrt(dx*dx + dy*dy);

// if (line_norm < 1e-6) {
//      left = 0;
//      right = 0;
// } else {
//      // 2. Perpendicular unit vector
//      vg_perp_x =  dy / line_norm;
//      vg_perp_y = -dx / line_norm;

//      // 3. Virtual point
//      theta_rad = theta * M_PI / 180.0;

//      // convert P_LOOKAHEAD (m) → cm
//      xp = x + (P * 100.0) * cos(theta_rad);
//      yp = y + (P * 100.0) * sin(theta_rad);

//      // 4. Vector from start to virtual point
//      vp_x = xp - x0;
//      vp_y = yp - y0;

//      // 5. Exact dp (cm)
//      d_p = vg_perp_x * vp_x + vg_perp_y * vp_y;

//      // convert cm → meters
//      d_p = d_p / 100;

//      // 6. Controller
//      omega = K_PSI_2 * d_p *180.0/M_PI;
//      v = 0;

//      // 7. Wheel speeds
//      right_1 = (int)(v + omega / 2.0);
//      left_1  = (int)(v - omega / 2.0);
//     }
//     right = (int) right_1;
//     left = (int) left_1;

// Task 15. Both controls together.
task15_left = left_0 + left_1;
task15_right = right_0 + right_1;

// Task 17
pos_error = sqrt(delta_0[0] * delta_0[0] + delta_0[1] * delta_0[1]);
if (task17_state == 0)
{
    // Check if we are close enough to (x0, y0) and oriented towards (xg, yg)
    // or if the rotor speed is below the minimum threshold
    if ((pos_error < pos_tol && fabs(angle_error) < ang_tol) ||
        (fabs(task9_left) < min_speed && fabs(task9_right) < min_speed))
    {
        task17_state = 1;
        printf("Switching to Go-To-Goal (Task 13 & 15)\n");
    }
}

// Task 18 check if new goal should be set
// ------------------------------
if (task18_active == 1 && task17_state == 1)
{
    if (pos_error < pos_tol)
    {
        new_goal_to_be_set = 1; // Set the flag to indicate a new goal should be set
    }
}
// ------------------------------

if (task17_state == 0)
{
    left = (int)task9_left;
    right = (int)task9_right;
}
else
{
    left = (int)task15_left;
    right = (int)task15_right;
}

// 5. Motor Saturation (Clamping)
if (left > 800)
    left = 800;
else if (left < -800)
    left = -800;

if (right > 800)
    right = 800;
else if (right < -800)
    right = -800;
