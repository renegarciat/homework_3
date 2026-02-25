%% Controller Response Visualization
% Parse and plot log_1.csv, log_2.csv, and log_3.csv
% Experiments with different initial orientation errors
clear; clc; close all;

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
