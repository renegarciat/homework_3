%% Controller Response Visualization
% Parse and plot log_1.csv, log_2.csv, and log_3.csv
% Experiments with different initial orientation errors
clear; clc; close all;
%% Task 6
%% Load data from CSV files (skip header row)
data1 = dlmread('log_1.csv', ';', 1, 0);  % 90° initial error
data2 = dlmread('log_2.csv', ';', 1, 0);  % 180° initial error
data3 = dlmread('log_3.csv', ';', 1, 0);  % 45° initial error

%% Trim leading zeros: keep only one zero sample before motion starts
idx1 = find(data1(:,4) ~= 0, 1) - 1; data1 = data1(idx1:end, :);
idx2 = find(data2(:,4) ~= 0, 1) - 1; data2 = data2(idx2:end, :);
idx3 = find(data3(:,4) ~= 0, 1) - 1; data3 = data3(idx3:end, :);

%% Extract and normalize time (convert from microseconds to seconds, start from 0)
t1 = (data1(:,1) - data1(1,1)) / 1e6;
t2 = (data2(:,1) - data2(1,1)) / 1e6;
t3 = (data3(:,1) - data3(1,1)) / 1e6;

%% Extract orientation data and unwrap to remove ±180° discontinuities
% unwrap works in radians: convert deg->rad, unwrap, convert back to deg
theta1 = rad2deg(unwrap(deg2rad(data1(:,4))));
theta2 = rad2deg(unwrap(deg2rad(data2(:,4))));
theta3 = rad2deg(unwrap(deg2rad(data3(:,4))));

%% Plot Theta (orientation) response in separate windows
sp1 = 90;  sp2 = 180;  sp3 = 45;

datasets = { {t1, theta1, sp1, '90° initial error', 'b'}, ...
             {t2, theta2, sp2, '180° initial error', 'b'}, ...
             {t3, theta3, sp3, '45° initial error', 'b'} };

for i = 1:3
    t = datasets{i}{1};
    theta = datasets{i}{2};
    sp = datasets{i}{3};
    name = datasets{i}{4};
    color = datasets{i}{5};
    
    figure('Name', ['Controller Response - ' name]);
    
    % Subplot 1: Full Response
    subplot(2,1,1);
    hold on;
    plot(t, theta, [color '-'], 'LineWidth', 1.5, 'DisplayName', 'Response');
    yline(sp, [color '--'], 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
    xlabel('Time [s]');
    ylabel('\theta [deg]');
    title(['Full Response: ' name]);
    legend('Location', 'best');
    grid on;
    hold off;
    
    % Subplot 2: Zoomed-in Steady State
    subplot(2,1,2);
    hold on;
    plot(t, theta, [color '-'], 'LineWidth', 1.5, 'DisplayName', 'Response');
    yline(sp, [color '--'], 'LineWidth', 1.5, 'DisplayName', 'Setpoint');
    
    % Zoom to the last 20% of the time
    t_zoom_start = t(end) * 0.8;
    xlim([t_zoom_start, t(end)]);
    
    % Adjust Y-limits to center around setpoint and show steady state error
    zoom_idx = t >= t_zoom_start;
    theta_zoom = theta(zoom_idx);
    if ~isempty(theta_zoom)
        y_margin = max(0.5, max(abs(theta_zoom - sp)) * 1.5); % Dynamic margin
        ylim([sp - y_margin, sp + y_margin]);
        
        % Calculate steady state error (use the max. value of the last window)
        [max_err, max_idx] = max(abs(theta_zoom - sp));
        ss_error = theta_zoom(max_idx) - sp;
        
        % Add text box with steady state error
        text_str = sprintf('Steady-State Error: %.3f°', ss_error);
        
        % Get legend position to place the text box right above it
        lgd = legend('Location', 'best');
        drawnow; % Force update to get actual legend position
        lgd_pos = lgd.Position;
        
        % Place text box above the legend, shifted slightly left to avoid crossing boundaries
        annotation('textbox', [lgd_pos(1) - 0.07, lgd_pos(2) + lgd_pos(4) + 0.02, lgd_pos(3), 0.07], ...
            'String', text_str, 'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black');
    end
    if i==2
        % Easily visualize the ripple.
        xlim([3.425 5.503]);
        ylim([179.59 180.66]);
    
    end
    xlabel('Time [s]');
    ylabel('\theta [deg]');
    title('Steady-State Error');
    grid on;
    hold off;
end
%% Task 8
%% ========================================================================
%% Position Controller Response Visualization (Scenario 3)
%% ========================================================================

%% Load data from CSV files (skip header row)
data6 = dlmread('log_6.csv', ';', 1, 0);  % Scenario 3: Mixed Diagonal (-1, 1)

%% Extract and normalize time (convert from microseconds to seconds, start from 0)
t6 = (data6(:,1) - data6(1,1)) / 1e6;

%% Extract position data
x6 = data6(:,2); y6 = data6(:,3);

%% Calculate Error (Target X is -1, Target Y is 1)
error_x6 = -1 - x6;
error_y6 = 1 - y6;

%% Plot Position vs Time and Error
figure('Name', 'Position Control - Scenario 3');
% X and Y Position vs Time
subplot(2,1,1);
hold on;
plot(t6, x6, 'b-', 'LineWidth', 1.5, 'DisplayName', 'X Position');
plot(t6, y6, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Y Position');
yline(-1, 'k--', 'LineWidth', 1, 'DisplayName', 'Target X');
yline(1, 'r--', 'LineWidth', 1, 'DisplayName', 'Target Y');
xlabel('Time [s]');
ylabel('Position [m]');
title('Scenario 3: Position vs Time');
legend('Location', 'best');
grid on;
hold off;

% X Error vs Time (Zoomed-in Steady State)
subplot(2,1,2);
hold on;
plot(t6, x6, 'b-', 'LineWidth', 1.5, 'DisplayName', 'X Position');
yline(-1, 'k--', 'LineWidth', 1, 'DisplayName', 'Target X');

% Zoom to the last 20% of the time
t_zoom_start = t6(end) * 0.8;
xlim([t_zoom_start, t6(end)]);

% Adjust Y-limits to center around setpoint and show steady state error
zoom_idx = t6 >= t_zoom_start;
x_zoom = x6(zoom_idx);
sp_x = -1;
if ~isempty(x_zoom)
    y_margin = max(0.05, max(abs(x_zoom - sp_x)) * 1.5); % Dynamic margin
    ylim([sp_x - y_margin, sp_x + y_margin]);
    
    % Calculate steady state error (use the max. value of the last window)
    [max_err, max_idx] = max(abs(x_zoom - sp_x));
    ss_error = x_zoom(max_idx) - sp_x;
    
    % Add text box with steady state error
    text_str = sprintf('Steady-State Error: %.3f m', ss_error);
    
    % Get legend position to place the text box right above it
    lgd = legend('Location', 'best');
    drawnow; % Force update to get actual legend position
    lgd_pos = lgd.Position;
    
    % Place text box above the legend, shifted slightly left to avoid crossing boundaries
    annotation('textbox', [lgd_pos(1) - 0.07, lgd_pos(2) + lgd_pos(4) + 0.02, lgd_pos(3), 0.07], ...
        'String', text_str, 'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black');
end

xlabel('Time [s]');
ylabel('Position [m]');
title('Steady-State Error (Position)');
grid on;
hold off;

% Export the last figure to PDF
exportgraphics(gcf, 'task_8_ss-err_3.pdf', 'ContentType', 'vector');
%% Task 9
%% ========================================================================
%% Combined Position and Angle Controller Response Visualization (Scenario 1)
%% ========================================================================

%% Load data from CSV files (skip header row)
data7 = dlmread('log_7.csv', ';', 1, 0);  % Scenario 1: Combined Control

%% Extract and normalize time (convert from microseconds to seconds, start from 0)
t7 = (data7(:,1) - data7(1,1)) / 1e6;

%% Extract position and orientation data
x7 = data7(:,2); 
y7 = data7(:,3);
theta7 = rad2deg(unwrap(deg2rad(data7(:,4))));

%% Plot Position and Orientation vs Time
figure('Name', 'Combined Control - Scenario 1', 'Position', [100, 100, 800, 600]);

% Subplot 1: X and Y Position vs Time
subplot(2,1,1);
hold on;
plot(t7, x7, 'b-', 'LineWidth', 1.5, 'DisplayName', 'X Position');
plot(t7, y7, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Y Position');
yline(1, 'k--', 'LineWidth', 1, 'DisplayName', 'Target X & Y');
xlabel('Time [s]');
ylabel('Position [m]');
title('Scenario 1: Position vs Time');
legend('Location', 'best');
grid on;
hold off;

% Subplot 2: Orientation vs Time
subplot(2,1,2);
hold on;
plot(t7, theta7, 'g-', 'LineWidth', 1.5, 'DisplayName', '\theta');
yline(180, 'g--', 'LineWidth', 1, 'DisplayName', 'Target \theta');
xlabel('Time [s]');
ylabel('\theta [deg]');
title('Scenario 1: Orientation vs Time');
legend('Location', 'best');
grid on;
hold off;

exportgraphics(gcf, 'task_9_combined_pos_angle.pdf', 'ContentType', 'vector');
