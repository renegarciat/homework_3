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
if (target_reached == 1) {
    left = 0;
    right = 0;
} else {
    // 1. Define the current segment boundaries
    double current_xg = wp_x[wp_idx];
    double current_yg = wp_y[wp_idx];
    double current_x0 = (wp_idx == 0) ? x0 : wp_x[wp_idx - 1]; 
    double current_y0 = (wp_idx == 0) ? y0 : wp_y[wp_idx - 1];
    // 2. Override the global differences to use the current segment
    dx = current_xg - current_x0;
    dy = current_yg - current_y0;
    // 3. Calculate Global Errors for the State Machine
    theta_R = atan2(dy, dx)*180/M_PI; 
    angle_error = theta_R - theta;
    if (angle_error > 180) angle_error -= 360;
    else if (angle_error < -180) angle_error += 360;
    // Distance to the CURRENT waypoint target
    double dist_to_wp = sqrt(pow(current_xg - x, 2) + pow(current_yg - y, 2)) / 100.0;

    // State machine
    if (task17_state == 0) {
        // We are rotating, are we aligned yet?
        if (fabs(angle_error) < ang_tol) {
            task17_state = 1; // Switch to translation control
            printf("Aligned with waypoint %d. Switching to translation control.\n", wp_idx);
        }
    } else if (task17_state == 1) {
        // We are translating, have we reached the current waypoint?
        if (dist_to_wp < pos_tol) {
            if (wp_idx < wp_count - 1) {
                // Corner reached, move to next segment
                wp_idx++;
                task17_state = 0;
                printf("Reached waypoint %d. Moving to next segment.\n", wp_idx);
            } else {
                // Final target reached
                target_reached = 1;
                printf("Final target reached. Stopping robot.\n");
            }
        }
    }
    // Actuation
    if (target_reached == 1) {
        left = 0;
        right = 0;
    } else if (task17_state == 0) {
        // ROTATIONAL CONTROL
        theta_R = atan2(dy, dx)*180/M_PI; // Desired orientation of the robot in degrees.
        // Calculate angle error with wrapping to [-180, 180]
        angle_error = theta_R - theta;
        if (angle_error > 180) angle_error -= 360;
        else if (angle_error < -180) angle_error += 360;
        omega = K_PSI_1 * angle_error;
        v_c[0] = cos(theta*(M_PI/180.0));
        v_c[1] = sin(theta*(M_PI/180.0));
        // Calculate the error in position
        delta_0[0] = (current_x0-x) / 100.0; // Position error in the x direction in m.
        delta_0[1] = (y0-y) / 100.0; // Position error in the y direction in m.
        // Calculate the inner product of the velocity vector and the position error
        d_0 = v_c[0]*delta_0[0] + v_c[1]*delta_0[1]; // [m]
        // Calculate the desired translational velocity
        v = K_OMEGA_1*d_0; // [1/s or rad/s]
        v = v*180.0/M_PI; // Convert to 1°/s
        left = (int)(v - omega / 2.0);
        right = (int)(v + omega / 2.0);
    } else {
        // Translational control
        // TRANSLATIONAL CONTROL
    delta_g[0] = current_xg - x;
    delta_g[1] = current_yg - y;
    
    theta_g = atan2(dy, dx);  // radians
    
    v_g[0] = cos(theta_g);
    v_g[1] = sin(theta_g);
    
    d_g = v_g[0]*delta_g[0] + v_g[1]*delta_g[1];
    v = K_OMEGA_2_CM * d_g;
    v = v*180.0/M_PI; // Convert to 1°/s
    // 1. Line direction
    line_norm = sqrt(dx*dx + dy*dy);
    if (line_norm < 1e-6) {
         left = 0;
         right = 0;
    } else {
         // 2. Perpendicular unit vector
         vg_perp_x =  dy / line_norm;
         vg_perp_y = -dx / line_norm;
         // 3. Virtual point
         theta_rad = theta * M_PI / 180.0;
         // convert P_LOOKAHEAD (m) → cm
         xp = x + (P * 100.0) * cos(theta_rad);
         yp = y + (P * 100.0) * sin(theta_rad);
         // 4. Vector from start to virtual point
         vp_x = xp - current_x0;
         vp_y = yp - current_y0;
         // 5. Exact dp (cm)
         d_p = vg_perp_x * vp_x + vg_perp_y * vp_y;
         // convert cm → meters
         d_p = d_p / 100;
         // 6. Controller
         omega = K_PSI_2 * d_p *180.0/M_PI;
         // 7. Wheel speeds
        left = (int)(v - omega / 2.0);
        right = (int)(v + omega / 2.0);
        }
    }
}

// 5. Motor Saturation (Clamping)
if (left > 800) left = 800;
else if (left < -800) left = -800;

if (right > 800) right = 800;
else if (right < -800) right = -800;