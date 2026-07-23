clear; clc; close all;


% DT-Risk MPC Post-Processing & Figure Generation Script
% Synchronized with Manuscript: Paper 2_Majdi Hassan_V2.pdf

% Manuscript Parameters
d_safe = 0.250;        % Predefined Safety Distance Threshold (m)
risk_threshold = 1.0;  % Risk Threshold Boundary
r_robot = 0.105;       % TurtleBot3 Chassis Radius (m)
r_obs = 0.050;         % Obstacle Radius (m)

% Global Style Configuration for IEEE/Q1 Publication Standards
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultAxesFontSize', 12);
set(0, 'DefaultTextFontName', 'Arial');
set(0, 'DefaultTextFontSize', 12);

% Scenario 1: Obstacle-Free Environment (Fig. 5)
if exist('scenario1_results.mat', 'file')
    data1 = load('scenario1_results.mat');
    sim_obj1 = find_sim_data(data1, 'out1');
    
    [ref1, t1] = get_sim_signal(sim_obj1, data1, 'ref_out');
    [prop1, ~] = get_sim_signal(sim_obj1, data1, 'x_prop_out');
    
    ref1  = squeeze(ref1)';
    prop1 = squeeze(prop1)';
    t1    = t1(:);

    figure('Name', 'Fig 5: Obstacle-Free Environment', 'Color', 'w', 'Position', [100, 100, 650, 320]);
    plot(ref1(:,1), ref1(:,2), 'k--', 'LineWidth', 2); hold on;
    plot(prop1(:,1), prop1(:,2), 'b', 'LineWidth', 2.5);
    plot(prop1(1,1), prop1(1,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(prop1(end,1), prop1(end,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    
    grid on; axis equal;
    xlim([0, 7.2]); ylim([-0.3, 4.3]);
    xlabel('x (m)', 'FontSize', 13);
    ylabel('y (m)', 'FontSize', 13);
    legend({'Reference trajectory', 'Robot trajectory', 'Start', 'End'}, ...
           'Location', 'northeast', 'FontSize', 11);
    title('Reference trajectory and robot trajectory in obstacle-free environment', 'FontSize', 13, 'FontWeight', 'normal');
    
    saveas(gcf, 'Fig_5.png');
end


% Scenario 2: Static Obstacles Environment (Fig. 6)
if exist('scenario2_results.mat', 'file')
    data2 = load('scenario2_results.mat');
    sim_obj2 = find_sim_data(data2, 'out2');

    [ref2, ~]     = get_sim_signal(sim_obj2, data2, 'ref_out');
    [prop2, ~]    = get_sim_signal(sim_obj2, data2, 'x_prop_out');
    [obs2_raw, ~] = get_sim_signal(sim_obj2, data2, 'obs_out');

    ref2  = squeeze(ref2)';
    prop2 = squeeze(prop2)';

    % Extract obstacles
    obs2_1 = [3.5, 1.2];
    obs2_2 = [2.8, 0.8];
    obs2_3 = [5.0, 3.5];

    figure('Name', 'Fig 6: Static-Obstacle Environment', 'Color', 'w', 'Position', [100, 100, 700, 320]);
    plot(ref2(:,1), ref2(:,2), 'k--', 'LineWidth', 2); hold on;
    plot(prop2(:,1), prop2(:,2), 'b', 'LineWidth', 2.5);

    % Color matching exact manuscript style
    plot(obs2_1(1), obs2_1(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(obs2_2(1), obs2_2(2), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    plot(obs2_3(1), obs2_3(2), 'co', 'MarkerSize', 10, 'MarkerFaceColor', 'c');

    plot(prop2(1,1), prop2(1,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(prop2(end,1), prop2(end,2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

    grid on; axis equal;
    xlim([0, 7.2]); ylim([-0.3, 4.3]);
    xlabel('x (m)', 'FontSize', 13);
    ylabel('y (m)', 'FontSize', 13);
    legend({'Reference trajectory', 'Robot trajectory', ...
           'Static obstacle 1', 'Static obstacle 2', 'Static obstacle 3', ...
           'Start', 'End'}, 'Location', 'northeastoutside', 'FontSize', 11);
    title('Robot trajectory in static-obstacle environment', 'FontSize', 13, 'FontWeight', 'normal');
    
    saveas(gcf, 'Fig_6.png');
end

% Scenario 3: Dynamic Obstacles Environment (Fig. 7 & Fig. 8)
if exist('scenario3_results.mat', 'file')
    data3 = load('scenario3_results.mat');
    sim_obj3 = find_sim_data(data3, 'out3');

    [ref3, t3]     = get_sim_signal(sim_obj3, data3, 'ref_out');
    [conv3, ~]    = get_sim_signal(sim_obj3, data3, 'x_conv_out');
    [prop3, ~]    = get_sim_signal(sim_obj3, data3, 'x_prop_out');
    [risk3, ~]   = get_sim_signal(sim_obj3, data3, 'risk_out');
    [obs3_raw, ~] = get_sim_signal(sim_obj3, data3, 'obs_out');

    ref3  = squeeze(ref3)';
    conv3 = squeeze(conv3)';
    prop3 = squeeze(prop3)';
    risk3 = risk3(:);
    t3    = t3(:);

    obs3_list = extract_obstacles(obs3_raw);

 --
    % Figure 7: Navigation Trajectories in Dynamic-Obstacle Environment
    figure('Name', 'Fig 7: Dynamic-Obstacle Environment', 'Color', 'w', 'Position', [100, 100, 720, 330]);
    plot(ref3(:,1), ref3(:,2), 'k--', 'LineWidth', 2); hold on;
    plot(conv3(:,1), conv3(:,2), 'r:', 'LineWidth', 2.5);
    plot(prop3(:,1), prop3(:,2), 'b', 'LineWidth', 2.5);

    % Static Obstacle 1 (Yellow circle with black border)
    plot(3.5, 1.2, 'ko', 'MarkerSize', 11, 'LineWidth', 2, 'MarkerFaceColor', 'y');

    % Dynamic Obstacle Trajectories (Magenta & Cyan Dashed)
    if length(obs3_list) >= 2
        obs2 = obs3_list{2};
        plot(obs2(:,1), obs2(:,2), 'm--', 'LineWidth', 2.2);
    end
    if length(obs3_list) >= 3
        obs3 = obs3_list{3};
        plot(obs3(:,1), obs3(:,2), 'c--', 'LineWidth', 2.2);
    end

    plot(prop3(1,1), prop3(1,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(prop3(end,1), prop3(end,2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

    grid on; axis equal;
    xlim([0, 7.2]); ylim([-0.3, 4.3]);
    xlabel('x (m)', 'FontSize', 13);
    ylabel('y (m)', 'FontSize', 13);
    legend({'Reference trajectory', 'Conventional MPC', 'Proposed DT-Risk MPC', ...
           'Static obstacle 1', 'Dynamic obstacle 2', 'Dynamic obstacle 3', ...
           'Robot start', 'Robot end'}, 'Location', 'northeastoutside', 'FontSize', 11);
    title('Navigation trajectories in dynamic-obstacle environment', 'FontSize', 13, 'FontWeight', 'normal');
    
    saveas(gcf, 'Fig_7.png');

   
    % Figure 8: Future Obstacle Trajectory Prediction over Horizon
    k_show = round(length(t3) / 2);
    Ts = 0.1;
    Np = 20;

    figure('Name', 'Fig 8: Future Obstacle Prediction', 'Color', 'w', 'Position', [100, 100, 680, 360]); hold on;
    
    % Static Obstacle 1
    plot(3.5, 1.2, 'ko', 'MarkerSize', 11, 'LineWidth', 2, 'MarkerFaceColor', 'y');

    if length(obs3_list) >= 3
        obs2 = obs3_list{2};
        obs3 = obs3_list{3};

        % Actual full trajectories
        plot(obs2(:,1), obs2(:,2), 'm--', 'LineWidth', 2);
        plot(obs3(:,1), obs3(:,2), 'c--', 'LineWidth', 2);

        % Future Horizon Predictions
        for j = 2:3
            if j == 2, obsj = obs2; col = 'm'; else, obsj = obs3; col = 'c'; end

            xo  = obsj(k_show, 1);
            yo  = obsj(k_show, 2);
            vox = obsj(k_show, 3);
            voy = obsj(k_show, 4);

            pred_x = zeros(Np, 1);
            pred_y = zeros(Np, 1);
            for i = 1:Np
                pred_x(i) = xo + vox * i * Ts;
                pred_y(i) = yo + voy * i * Ts;
            end

            % Predicted trajectory dots & line matching Fig. 8
            plot(pred_x, pred_y, '.-', 'Color', col, 'LineWidth', 2.5, 'MarkerSize', 14);
        end
    end

    grid on; axis equal;
    xlim([2.0, 6.2]); ylim([0.8, 3.6]);
    xlabel('x (m)', 'FontSize', 13);
    ylabel('y (m)', 'FontSize', 13);
    legend({'Static obstacle 1', ...
           'Dynamic obstacle 2 actual trajectory', ...
           'Dynamic obstacle 3 actual trajectory', ...
           'Predicted dynamic obstacle 2', ...
           'Predicted dynamic obstacle 3'}, ...
           'Location', 'southeast', 'FontSize', 11);
    title('Future obstacle trajectory prediction over the MPC horizon', 'FontSize', 13, 'FontWeight', 'normal');
    
    saveas(gcf, 'Fig_8.png');
end


% Helper Functions
function obs_list = extract_obstacles(obs_raw)
    obs_list = {};
    sz = size(obs_raw);
    if length(sz) == 2
        obs_list{1} = obs_raw';
    elseif length(sz) == 3
        for i = 1:sz(2)
            obs_list{i} = squeeze(obs_raw(:, i, :))';
        end
    end
end

function sim_obj = find_sim_data(file_struct, target_name)
    sim_obj = [];
    if isfield(file_struct, target_name), sim_obj = file_struct.(target_name); return; end
    if isfield(file_struct, 'out'), sim_obj = file_struct.out; return; end
    if isfield(file_struct, 'ans'), sim_obj = file_struct.ans; return; end
    fields = fieldnames(file_struct);
    for k = 1:length(fields)
        if isa(file_struct.(fields{k}), 'Simulink.SimulationOutput') || isstruct(file_struct.(fields{k}))
            sim_obj = file_struct.(fields{k}); return;
        end
    end
end

function [val, time] = get_sim_signal(sim_obj, file_struct, name)
    val = []; time = [];
    if ~isempty(sim_obj)
        if isstruct(sim_obj) && isfield(sim_obj, name)
            [val, time] = parse_item(sim_obj.(name));
        elseif isa(sim_obj, 'Simulink.SimulationOutput')
            try [val, time] = parse_item(sim_obj.get(name)); catch; end
        end
    end
    if isempty(val) && isfield(file_struct, name)
        [val, time] = parse_item(file_struct.(name));
    end
end

function [val, time] = parse_item(item)
    val = []; time = [];
    if isempty(item), return; end
    if isa(item, 'timeseries')
        val = item.Data; time = item.Time;
    elseif isstruct(item) && isfield(item, 'signals')
        val = item.signals.values; time = item.time;
    elseif isprop(item, 'Values') || (isstruct(item) && isfield(item, 'Values'))
        val = item.Values.Data; time = item.Values.Time;
    elseif isnumeric(item) || islogical(item)
        val = item;
    end
end
