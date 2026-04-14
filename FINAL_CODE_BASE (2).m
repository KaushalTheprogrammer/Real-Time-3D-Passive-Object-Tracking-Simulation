function real_time_triangulation_simulation()
    % Real-time Triangulation with adaptive nonlinear refinement,
    % Kalman smoothing, and user-friendly file selection dialogs.

    % Select files
    %launchpad_file = selectCSVFile('Select launchpad.csv file (optional, press Cancel if none)');
    true_file      = selectCSVFile('Select true_target.csv file');
    eots1_file     = selectCSVFile('Select eots_station1.csv file');
    eots2_file     = selectCSVFile('Select eots_station2.csv file');
   % -------- Launchpad (robust read) --------
   launchpad_file = selectCSVFile('Select launchpad.csv file');
   T_launch = readtable(launchpad_file);

   % Convert table to numeric array (ignore headers)
   data = table2array(T_launch);

   % Remove rows with NaN (safety)
   data = data(all(~isnan(data),2), :);

   if size(data,2) < 3
        error('Launchpad file must have at least 3 columns (X Y Z)');
   end

   % Take FIRST row only (launchpad is fixed)
     launchpad_pos = data(1,1:3).';

   % ----handle duration safely ----
    if isduration(launchpad_pos)
        launchpad_pos = seconds(launchpad_pos);
    else
        launchpad_pos = double(launchpad_pos);
    end




   
    % Prompt user to select stations.csv file
    stations_file = selectCSVFile('Select stations.csv file (coordinates for Station 1 and 2)');
    % Read station coordinates from file
    T_stations = readtable(stations_file);
    % Ensure correct column names and extract positions
    if all(ismember({'X', 'Y', 'Z'}, T_stations.Properties.VariableNames))
        station1_pos = [T_stations.X(1); T_stations.Y(1); T_stations.Z(1)];
        station2_pos = [T_stations.X(2); T_stations.Y(2); T_stations.Z(2)];
    else
        error('The stations file must have columns named X, Y, and Z.');
    end

    % Display summary (original coordinates)
    disp('--- Station Coordinates from file (original frame) ---');
    fprintf('Station 1: [%.2f, %.2f, %.2f]\n', station1_pos);
    fprintf('Station 2: [%.2f, %.2f, %.2f]\n', station2_pos);

    % Read and prepare data (only valid rows, aligned length)
    [T_true, T1, T2, time_sim_ms] = prepareData(true_file, eots1_file, eots2_file);

    % ---- CENTER EVERYTHING AT FIRST TRUE TARGET POINT ----
    P0 = [T_true.Target_X(1); ...
          T_true.Target_Y(1); ...
          T_true.Target_Z(1)];

    % Shift ground truth so first point is (0,0,0)
    T_true.Target_X = T_true.Target_X - P0(1);
    T_true.Target_Y = T_true.Target_Y - P0(2);
    T_true.Target_Z = T_true.Target_Z - P0(3);

    % Shift station coordinates into same centered frame
    station1_pos = station1_pos - P0;
    station2_pos = station2_pos - P0;
    launchpad_pos = launchpad_pos - P0;


    % Parameters
    sim_dt_s = 0.02;              % 20ms simulation timestep
    nonlinear_threshold = 1e-2;   % threshold to trigger nonlinear refinement

    % Initialize results
    n = numel(time_sim_ms);
    P_est        = NaN(n,3);      % linear estimates
    P_nl         = NaN(n,3);      % nonlinear refined (raw)
    P_nl_smooth  = NaN(n,3);      % optional exponential smoothing of NL
    P_kalman     = NaN(n,6);      % Kalman state: [x y z vx vy vz]
    P_kalman_pos = NaN(n,3);      % Kalman position outputs

    % Kalman filter setup (constant velocity model)
    F = [1 0 0 sim_dt_s 0 0;
         0 1 0 0 sim_dt_s 0;
         0 0 1 0 0 sim_dt_s;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
    H = [eye(3), zeros(3,3)]; % measurement extracts position

    q_pos = 0.01; q_vel = 0.1;
    Q = diag([q_pos q_pos q_pos q_vel q_vel q_vel]);
    r_meas = 5.0;
    R = diag([r_meas r_meas r_meas]);

    xk = zeros(6,1);
    Pk = 1e3 * eye(6);

    % Show ready to track popup
    uiwait(warndlg('All files loaded. Ready to track target. Click OK to start.', 'Ready to Track'));

       % Setup figure
    %%%%%%%%%%%%%%%%% ===== REAL-TIME Close-Contact Visualization =====
    

    scr = get(0,'ScreenSize');   % [left bottom width height]

    figW = scr(3) * 0.25;       % 1/4th screen width
    figH = scr(4) * 0.25;       % 1/4th screen height

    figure('Name','Real-Time Target Tracking (Close-Contact View)', ...
       'Color','k', ...
       'Units','pixels', ...
       'Position',[50, scr(4)-figH-80, figW, figH], ...
       'NumberTitle','off', ...
       'Resize','off');
    ax = axes;
    hold(ax,'on'); grid(ax,'on'); view(ax,3);

    ax.Color     = 'k';
    ax.GridColor = [0.5 0.5 0.5];
    ax.XColor    = 'w';
    ax.YColor    = 'w';
    ax.ZColor    = 'w';
    ax.FontSize  = 14;
    ax.LineWidth = 1.4;

    xlabel('East (m)','FontWeight','bold');
    ylabel('North (m)','FontWeight','bold');
    zlabel('Altitude (m)','FontWeight','bold');

    title('Real-Time Close-Contact Target Tracking', ...
          'Color','w','FontSize',16,'FontWeight','bold');

    % Dominant ground truth
    hTrue = plot3(NaN,NaN,NaN,'g-','LineWidth',4);

    % Nonlinear estimate
    hNL   = plot3(NaN,NaN,NaN,'r-','LineWidth',2.5);

    % Current position marker
    hNow = scatter3(NaN,NaN,NaN,120,'y','filled');
    % hNow = plot3(NaN,NaN,NaN, ...
            % 'yx', ...              % yellow cross
            % 'MarkerSize',14, ...
            % 'LineWidth',3);


    legend({'Ground Truth','Estimated','Current Position'}, ...
           'TextColor','w','Location','northwest');

    axis vis3d;
     % [xmin xmax ymin ymax zmin zmax]

     % ===== Real-time deviation text (meters) =====
     hErrText = text(0,0,0,'', ...
        'Color','c', ...
        'FontSize',13, ...
        'FontWeight','bold', ...
        'BackgroundColor','k', ...
        'Margin',4);
     hRangeText = text(0,0,0,'', ...
         'Color','w', ...
         'FontSize',12, ...
         'FontWeight','bold', ...
         'BackgroundColor','k', ...
         'Margin',4);

     hAltText = text(0,0,0,'', ...
         'Color','w', ...
         'FontSize',12, ...
         'FontWeight','bold', ...
         'BackgroundColor','k', ...
         'Margin',4);




       % Exponential smoothing parameter for NL (keeps raw too)
       alpha = 0.15;

    % Main loop
    for k = 1:n
        % Unit vectors from sensors az/el inputs (degrees)
        u1 = azel_to_unit(T1.Azimuth_deg(k), T1.Elevation_deg(k));
        u2 = azel_to_unit(T2.Azimuth_deg(k), T2.Elevation_deg(k));

        % Linear estimate & residual norm
        [x_lin, res_norm] = linearSolve(station1_pos, station2_pos, u1, u2);
        P_est(k,:) = x_lin';

        % Nonlinear refine if residual above threshold
        if res_norm > nonlinear_threshold
            options = optimoptions('lsqnonlin', ...
                'Display','off', ...
                'MaxIter',200, ...
                'MaxFunctionEvaluations',2000, ...
                'StepTolerance',1e-8, ...
                'FunctionTolerance',1e-10);
            residual_func = @(x) nonlinearResiduals(x, station1_pos, station2_pos, u1, u2);
            try
                x_nl = lsqnonlin(residual_func, x_lin, [], [], options);
            catch ME
                warning('lsqnonlin failed at step %d: %s. Using linear estimate.', k, ME.message);
                x_nl = x_lin;
            end
            x_nl = x_nl(:);
            if ~isequal(size(x_nl), [3,1])
                warning('x_nl is not 3x1 at step %d; using linear estimate.', k);
                x_nl = x_lin;
            end
            P_nl(k,:) = x_nl';
        else
            P_nl(k,:) = x_lin';
        end

        % Exponential smooth of NL
        if k == 1
            P_nl_smooth(1,:) = P_nl(1,:);
        else
            P_nl_smooth(k,:) = alpha * P_nl(k,:) + (1-alpha) * P_nl_smooth(k-1,:);
        end

        % Kalman update (measurement = raw nonlinear)
        z = P_nl(k,:)'; % 3x1

        % Predict
        xk = F * xk;
        Pk = F * Pk * F' + Q;

        % Update
        S = H * Pk * H' + R;
        K = Pk * H' / S;
        xk = xk + K * (z - H * xk);
        Pk = (eye(6) - K * H) * Pk;

        % Store Kalman state and position
        P_kalman(k,:)     = xk';
        P_kalman_pos(k,:) = xk(1:3)';

        % Update plots
        if isvalid(hTrue)
            set(hTrue, 'XData', T_true.Target_X(1:k), ...
                       'YData', T_true.Target_Y(1:k), ...
                       'ZData', T_true.Target_Z(1:k));
        end
       
        if isvalid(hNL)
            set(hNL, 'XData', P_nl(1:k,1), ...
                     'YData', P_nl(1:k,2), ...
                     'ZData', P_nl(1:k,3));
        end
        % ===== Close-contact dynamic view (uses current sample only) =====
        cx = double(P_nl(k,1));
        cy = double(P_nl(k,2));
        cz = double(P_nl(k,3));

        tx = T_true.Target_X(k);
        ty = T_true.Target_Y(k);
        tz = T_true.Target_Z(k);

        err_meter = sqrt( (cx-tx)^2 + (cy-ty)^2 + (cz-tz)^2 );

        % -------- Launchpad-referenced metrics --------
        dx = cx - launchpad_pos(1);
        dy = cy - launchpad_pos(2);

       range_lp = sqrt(dx.^2 + dy.^2);


        altitude_lp = cz - launchpad_pos(3);



        zoom_xy = 300;   % meters
        zoom_z  = 200;

        xlim(ax,[cx-zoom_xy cx+zoom_xy]);
        ylim(ax,[cy-zoom_xy cy+zoom_xy]);
        zlim(ax,[max(0,cz-zoom_z) cz+zoom_z]);

        campos(ax,[cx-600 cy-600 cz+400]);
        camtarget(ax,[cx cy cz]);
        camup(ax,[0 0 1]);

        % Current position marker
        set(hNow,'XData',cx,'YData',cy,'ZData',cz);
        % ===== Smart stacked annotation (no overlap) =====
        base_offset = [20 20 15];   % anchor offset
        line_gap    = 18;           % vertical spacing (meters)

        % Deviation (top line)
        set(hErrText, ...
           'Position',[cx+base_offset(1), ...
                    cy+base_offset(2), ...
                    cz+base_offset(3)], ...
           'String',sprintf('Deviation : %.2f m', err_meter));

        % Range (middle line)
        set(hRangeText, ...
             'Position',[cx+base_offset(1), ...
                    cy+base_offset(2), ...
                    cz+base_offset(3) - line_gap], ...
             'String',sprintf('Range (LP) : %.1f m', range_lp));

        % Altitude (bottom line)
        set(hAltText, ...
             'Position',[cx+base_offset(1), ...
                    cy+base_offset(2), ...
                    cz+base_offset(3) - 2*line_gap], ...
             'String',sprintf('Altitude (LP) : %.1f m', altitude_lp));



        % ===== Color coding based on deviation =====
        if err_meter < 10
          hErrText.Color = [0 1 0];      % Green: very good
        elseif err_meter < 50
          hErrText.Color = [1 1 0];      % Yellow: moderate
        else
         hErrText.Color = [1 0 0];      % Red: large error
        end

        if altitude_lp < 0
            hAltText.Color = [1 0 0];   % Red: below launchpad
        else
            hAltText.Color = [0 1 1];   % Cyan: normal
        end


        % Ensure ground truth stays visible
        uistack(hTrue,'top');

        drawnow;
        pause(sim_dt_s);
    end

    % After loop: compute per-sample errors and RMSEs
    true_pos = [T_true.Target_X, T_true.Target_Y, T_true.Target_Z];

    % ==========================================================
% STEP 1: Horizontal and Vertical Error Computation
% ==========================================================
% Horizontal error = XY-plane error
horiz_err = sqrt( ...
    (P_nl(:,1) - true_pos(:,1)).^2 + ...
    (P_nl(:,2) - true_pos(:,2)).^2 );

% Vertical error = Z-axis error
vert_err = abs(P_nl(:,3) - true_pos(:,3));

% RMS values (used in plot titles)
rms_horiz = sqrt(mean(horiz_err.^2));
rms_vert  = sqrt(mean(vert_err.^2));

% ==========================================================
% STEP 2: Angular Separation Between Stations (degrees)
% ==========================================================
ang_sep = zeros(n,1);

for k = 1:n
    u1 = azel_to_unit(T1.Azimuth_deg(k), T1.Elevation_deg(k));
    u2 = azel_to_unit(T2.Azimuth_deg(k), T2.Elevation_deg(k));

    cosang = dot(u1, u2);
    cosang = min(max(cosang, -1), 1);   % numerical safety
    ang_sep(k) = acosd(cosang);
end



    % Fill missing if any (should be none now, but keep as safety)
    for c = 1:3
        P_est(:,c)        = fillmissing(P_est(:,c), 'nearest');
        P_nl(:,c)         = fillmissing(P_nl(:,c), 'nearest');
        P_nl_smooth(:,c)  = fillmissing(P_nl_smooth(:,c), 'nearest');
        P_kalman_pos(:,c) = fillmissing(P_kalman_pos(:,c), 'nearest');
    end

    % Per-sample Euclidean errors
    err_lin       = sqrt(sum((P_est        - true_pos).^2, 2));
    err_nl        = sqrt(sum((P_nl         - true_pos).^2, 2));
    err_nl_smooth = sqrt(sum((P_nl_smooth  - true_pos).^2, 2));
    err_kalman    = sqrt(sum((P_kalman_pos - true_pos).^2, 2));

    % RMSEs
    rmse_lin       = sqrt(mean(err_lin.^2));
    rmse_nl        = sqrt(mean(err_nl.^2));
    rmse_nl_smooth = sqrt(mean(err_nl_smooth.^2));
    rmse_kalman    = sqrt(mean(err_kalman.^2));

    fprintf('Linear RMSE: %.4f m\nNonlinear RMSE (raw): %.4f m\nNonlinear RMSE (smoothed): %.4f m\nKalman RMSE: %.4f m\n', ...
        rmse_lin, rmse_nl, rmse_nl_smooth, rmse_kalman);

    % Build output table with everything
    output_table = table(time_sim_ms, ...
        P_est(:,1), P_est(:,2), P_est(:,3), ...
        P_nl(:,1), P_nl(:,2), P_nl(:,3), ...
        P_nl_smooth(:,1), P_nl_smooth(:,2), P_nl_smooth(:,3), ...
        P_kalman_pos(:,1), P_kalman_pos(:,2), P_kalman_pos(:,3), ...
        err_lin, err_nl, err_nl_smooth, err_kalman, ...
        'VariableNames', {'Time_ms', ...
        'Lin_X','Lin_Y','Lin_Z', ...
        'NL_X','NL_Y','NL_Z', ...
        'NL_X_smooth','NL_Y_smooth','NL_Z_smooth', ...
        'KF_X','KF_Y','KF_Z', ...
        'Err_Lin','Err_NL','Err_NL_smooth','Err_KF'});

    % Ask the user if they want to save the predicted positions
    choice = questdlg('Do you want to save the predicted positions as a CSV file?', ...
        'Save Results', 'Yes', 'No', 'Yes');
    if strcmp(choice, 'Yes')
        [saveFile, savePath] = uiputfile('predicted_positions.csv', 'Save Predicted Positions As');
        if ~isequal(saveFile,0)
            writetable(output_table, fullfile(savePath, saveFile));
            disp(['Predicted positions saved to: ', fullfile(savePath, saveFile)]);
        else
            warning('File save cancelled. Results not written to disk.');
        end
    else
        disp('User chose not to save the predicted positions.');
    end

    % Optional: draw error plot automatically for quick inspection
    figure('Name','Per-sample Errors');
    plot(time_sim_ms, err_lin,       'b-', 'LineWidth', 1); hold on;
    plot(time_sim_ms, err_nl,        'r--', 'LineWidth', 1);
    plot(time_sim_ms, err_nl_smooth, 'm-', 'LineWidth', 1.2);
    plot(time_sim_ms, err_kalman,    'g-', 'LineWidth', 1.5);
    xlabel('Sample index'); ylabel('Euclidean error (m)');
    legend('Linear','NL raw','NL smoothed','Kalman','Location','best');
    grid on;
       

% --- UNIVERSAL PLOTTING STYLE ----
set(groot,'defaultFigureColor','w');         % White figure background
set(groot,'defaultAxesColor','w');           % White plot background

% Text colors (make everything dark)
set(groot,'defaultAxesXColor','k');
set(groot,'defaultAxesYColor','k');
set(groot,'defaultAxesZColor','k');
set(groot,'defaultTextColor','k');
set(groot,'defaultLegendTextColor','k');

% Optional visuals
set(groot,'defaultAxesGridColor',[0.4 0.4 0.4]);
set(groot,'defaultAxesLineWidth',1.2);
set(groot,'defaultLineLineWidth',2);

% Font sizes
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultLegendFontSize',12);
set(groot,'defaultTextFontSize',14);

% ---- Convert trajectories to km for plotting ----
true_km = true_pos / 1000;      % Ground truth trajectory (km)
nl_km   = P_nl    / 1000;       % Nonlinear refined prediction (km)

%%%%%%%%%%% Figure 1 2D Object Tracking 


figure('Name','2D Trajectory','Color','w'); clf;
hold on; grid on; axis equal;

plot(true_km(:,1), true_km(:,2), 'k-', 'LineWidth', 4);
plot(nl_km(:,1),   nl_km(:,2),   'r-', 'LineWidth', 2);

for i = 1:length(true_km)
    plot([true_km(i,1), nl_km(i,1)], ...
         [true_km(i,2), nl_km(i,2)], ...
         'Color',[0.2 0.2 1 0.35], 'LineWidth', 1.1);
end

scatter(station1_pos(1)/1000, station1_pos(2)/1000, 120, 'b', 'filled');
scatter(station2_pos(1)/1000, station2_pos(2)/1000, 120, 'm', 'filled');

xlabel('East (km)');
ylabel('North (km)');
title('2D Target Trajectory');

lg = legend({'Ground Truth','Predicted','Deviation','Stn 1','Stn 2'}, ...
            'Location','northwest');
beautify_legend(lg);


beautify_axes(gca);





%% ================= FIGURE 2 — HORIZONTAL ERROR =================
figure('Name','Horizontal Error','Color','w'); clf;

plot(time_sim_ms/1000, horiz_err, ...
     'Color',[0 0.45 0.9], 'LineWidth', 2.5);

grid on;
xlabel('Time (s)');
ylabel('Horizontal Error (m)');
title(sprintf('Horizontal Error (RMS = %.2f m)', rms_horiz));

beautify_axes(gca);




%% ================= FIGURE 3 — ERROR HISTOGRAM =================
figure('Name','Error Histogram','Color','w'); clf;

histogram(horiz_err, 40, ...
    'FaceColor',[0.25 0.45 0.9], ...
    'EdgeColor','k', ...
    'LineWidth',1.1);

xline(mean(horiz_err),'r-','LineWidth',2);

grid on;
xlabel('Horizontal Error (m)');
ylabel('Frequency');
title('Horizontal Error Distribution');

beautify_axes(gca);


%% ================= FIGURE 4 — 3D TRAJECTORY =================
figure('Name','3D Trajectory','Color','w'); clf;
hold on; grid on; view(3);

plot3(true_km(:,1), true_km(:,2), true_km(:,3), ...
      'k-', 'LineWidth', 3.5);

plot3(nl_km(:,1), nl_km(:,2), nl_km(:,3), ...
      'r-', 'LineWidth', 2.2);

scatter3(station1_pos(1)/1000, station1_pos(2)/1000, station1_pos(3)/1000, ...
         150, 'b', 'filled');

scatter3(station2_pos(1)/1000, station2_pos(2)/1000, station2_pos(3)/1000, ...
         150, 'm', 'filled');

xlabel('East (km)');
ylabel('North (km)');
zlabel('Up (km)');
title('3D Target Tracking');

axis vis3d; axis tight;
xlim([-3 3]); ylim([-3 3]); zlim([0 2]);

lg = legend({'Ground Truth','Predicted','Stn 1','Stn 2'});
beautify_legend(lg);


beautify_axes(gca);



%% ================= FIGURE 5 — ERROR vs ANGULAR SEPARATION =================
figure('Name','Error vs Geometry','Color','w'); clf;

scatter(ang_sep, horiz_err, 45, horiz_err, 'filled');
colormap jet; colorbar;

grid on;
xlabel('Angular Separation (deg)');
ylabel('Horizontal Error (m)');
title('Horizontal Error vs Sensor Geometry');
lg = legend('Horizontal Error');
beautify_legend(lg);


beautify_axes(gca);




%% ================= FIGURE 6 — ALTITUDE vs TIME =================
figure('Name','Altitude vs Time','Color','w'); clf;
hold on; grid on;

plot(time_sim_ms/1000, true_pos(:,3), 'k-', 'LineWidth', 3);
plot(time_sim_ms/1000, P_nl(:,3),     'r-', 'LineWidth', 2);

xlabel('Time (s)');
ylabel('Altitude (m)');
title(sprintf('Altitude (Vert RMS = %.2f m)', rms_vert));

lg = legend({'Ground Truth','Predicted'});
beautify_legend(lg);


beautify_axes(gca);




end

% ----------------------
% Helper Functions
% ----------------------
function file = selectCSVFile(prompt)
    [fileName, filePath] = uigetfile('*.csv', prompt);
    if isequal(fileName,0)
        error(['File selection cancelled or no file selected for: ', prompt]);
    end
    file = fullfile(filePath, fileName);
end

function [T_true_sync, T1_sync, T2_sync, time_sim_ms] = prepareData(true_file, eots1_file, eots2_file)
    % Load tables
    T_true = readtable(true_file);
    T1     = readtable(eots1_file);
    T2     = readtable(eots2_file);

    % Keep only rows without NaNs in useful columns (only valid data)
    T_true = rmmissing(T_true, 'DataVariables', {'Target_X','Target_Y','Target_Z'});
    T1     = rmmissing(T1,     'DataVariables', {'Azimuth_deg','Elevation_deg'});
    T2     = rmmissing(T2,     'DataVariables', {'Azimuth_deg','Elevation_deg'});

    % Force same length (common valid segment)
    n_min  = min([height(T_true), height(T1), height(T2)]);
    T_true = T_true(1:n_min, :);
    T1     = T1(1:n_min, :);
    T2     = T2(1:n_min, :);

    % Time vector for these valid samples (20 ms step)
    sim_dt_s    = 0.02;
    time_sim_ms = (0:n_min-1)' * 1000*sim_dt_s;

    % Return
    T_true_sync = T_true;
    T1_sync     = T1;
    T2_sync     = T2;
end

function u = azel_to_unit(az_deg, el_deg)
    az = deg2rad(az_deg); el = deg2rad(el_deg);
    u  = [cos(el)*cos(az); cos(el)*sin(az); sin(el)];
    u  = u / norm(u);
end

function [x, res_norm] = linearSolve(s1, s2, u1, u2)
    A1 = eye(3) - u1*u1';
    A2 = eye(3) - u2*u2';
    M  = A1 + A2;
    b  = A1*s1 + A2*s2;
    try
        R = chol(M);
        y = R' \ b;
        x = R  \ y;
    catch
        x = pinv(M)*b;
    end
    x = x(:);
    if ~isequal(size(x), [3,1])
        error('linearSolve: x is not 3x1. Actual size: %s', mat2str(size(x)));
    end
    res_norm = norm(M*x - b);
end

function r = nonlinearResiduals(P, s1, s2, u1, u2)
    P  = P(:);
    I3 = eye(3);
    A1 = I3 - (u1 * u1');   % projection onto plane perpendicular to u1
    A2 = I3 - (u2 * u2');
    d1 = P - s1;
    d2 = P - s2;
    r1_range = max(1e-3, norm(d1));
    r2_range = max(1e-3, norm(d2));
    w1 = 1 / (1 + r1_range);
    w2 = 1 / (1 + r2_range);
    res1 = A1 * d1;
    res2 = A2 * d2;
    r    = [w1 * res1; w2 * res2]; % 6x1 vector
end

function beautify_axes(ax)
    if nargin < 1
        ax = gca;
    end

    box(ax,'off');                     % remove dark frame
    ax.Color = 'w';

    ax.GridColor = [0.7 0.7 0.7];
    ax.GridAlpha = 0.4;

    ax.LineWidth = 1.2;
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.ZColor = 'k';

    ax.FontSize = 14;
end
function beautify_legend(lg)
    if nargin < 1 || ~isvalid(lg)
        lg = legend;
    end

    lg.Box        = 'on';      % clear box
    lg.Color      = 'w';       % white background
    lg.EdgeColor  = 'k';       % black border
    lg.FontSize   = 13;        % readable text
    lg.FontWeight = 'bold';    % clarity
    lg.ItemTokenSize = [30,18]; % bigger line/marker icons
end

