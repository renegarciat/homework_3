// Reset state for Task 17 when a new goal is received
task17_state = 0;
wp_idx = 0;
target_reached = 0;

if (fabs(xg - x0) < float_tol && fabs(yg - y0) < float_tol) {
    target_reached = 1;
    wp_count = 0;
    printf("Robot is already at the target goal.\n");
} 
else if (fabs(x0 - xg) > float_tol && fabs(y0 - yg) > float_tol) {
    // Diagonal request: Create an L-shape path (2 segments)
    wp_x[0] = xg;  // Segment 1: Move along X
    wp_y[0] = y0;
    
    wp_x[1] = xg;  // Segment 2: Move along Y
    wp_y[1] = yg;
    
    wp_count = 2;
    printf("Diagonal Goal. Waypoints calculated: 2\n");
} else { 
    // Already sharing an axis: Straight line (1 segment)
    wp_x[0] = xg;
    wp_y[0] = yg;
    wp_count = 1;
    printf("Straight Goal. Waypoints calculated: 1\n");
}