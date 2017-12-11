%% Script flags.
close all

reload_truth_files = false;
do_plotting = true;

% % % Matrix Indeces % % %
% Pose_With_Covariance_stamped matrix (extracted to matrix by msgs2mat.m)
msg_sec_idx = 1;    msg_nsec_idx = 2;   msg_x_idx = 3;  msg_y_idx = 4;  msg_z_idx = 5;
msg_qx_idx = 6;     msg_qy_idx = 7;     msg_qz_idx = 8; msg_qw_idx = 9;
% Ground Truth (Leica tracking system) ROS-Bag --> timeseries indeces.
leica_x_idx = 4;    leica_y_idx = 5;    leica_z_idx = 6;
% Axis indeces in Nx3 matrices.
pox_x_idx = 1;
pox_y_idx = 2;
pox_z_idx = 3;

%% Load Truth Data
% Calls a script that loads Leica data from rosbag and then parses the sample times from the truth data-set.
if reload_truth_files
    clear('pose_wcs_cell', 'leica_time_series')
    load_truth_trajectory; % Loades leica_time_series
end
% Parse truth message timeing.
truth_time_offset = 60; % The truth data isn't sampled regularly until a few seconds in ...
truth_start_epoch = leica_time_series.Time(truth_time_offset);
truth_end_epoch = leica_time_series.Time(end);
raw_truth_time = leica_time_series.Time(truth_time_offset:end);
truth_time = leica_time_series.Time(truth_time_offset:end) - leica_time_series.Time(truth_time_offset);

%% Load recorded mat files.
files_to_plot = {'ijrr17_pose_50', 'ijrr17_pose', 'ijrr17_pose_15', 'ijrr17_pose_7'};

for i = 1:length(files_to_plot)
    load(files_to_plot{i}); % Loads pose_wcs_cell
    pose_wcs_cell = pose_wcs_cell(:); % Force column form.
    num_msgs = size(pose_wcs_cell,1);
    % Extract estimated position with this anonymous function that parses Msg structure: Pose_with_covariance_stamped.
    pose_wcs_accessor = @(x) [pose_wcs_cell{x}.Header.Stamp.Sec, pose_wcs_cell{x}.Header.Stamp.Nsec, ...
                              pose_wcs_cell{x}.Pose.Pose.Position.X, pose_wcs_cell{x}.Pose.Pose.Position.Y, ...
                              pose_wcs_cell{x}.Pose.Pose.Position.Z, pose_wcs_cell{x}.Pose.Pose.Orientation.X, ...
                              pose_wcs_cell{x}.Pose.Pose.Orientation.Y, pose_wcs_cell{x}.Pose.Pose.Orientation.Z, ...
                              pose_wcs_cell{x}.Pose.Pose.Orientation.W,];
    % Extract Msg data.
    ijrr17_data = msgs2mat(pose_wcs_cell, pose_wcs_accessor, 9);

    % Subtract starting location so that both the truth and the recorded data start at the same place
    ijrr17_origin_loc_mat = kron(ones(num_msgs,1), ...
                                 [ijrr17_data(1,msg_x_idx), ijrr17_data(1,msg_y_idx), ijrr17_data(1,msg_z_idx)]);
    ijrr17_xyz = ijrr17_data(:,msg_x_idx:msg_z_idx) - ijrr17_origin_loc_mat;

    % The time samples of the truth data and the recorded data won't match up, so we need to do some interpolating.
    % Find closest index to the starting epoch of the truth time.
    ijrr17_raw_time = ijrr17_data(:, msg_sec_idx) + ijrr17_data(:, msg_nsec_idx)*1e-9;
    start_t_idx = find(ijrr17_raw_time >= truth_start_epoch, 1);
    % Determine which dataset needs to be interpolated.
    if ijrr17_raw_time(end) > truth_end_epoch
        % Not implemented yet.
        dbstop()
        a = 1;
    else
        % Find time indices in truth (longer in time, not necessarily in number of samples).
        [~, truth_end_t_idx] = min(abs(ijrr17_raw_time(end) - raw_truth_time));
        truth_time = leica_time_series.Time(truth_time_offset:truth_end_t_idx) - leica_time_series.Time(truth_time_offset);
        ijrr17_time = ijrr17_raw_time(start_t_idx:end) - ijrr17_raw_time(start_t_idx);
        % Trim truth data data.
        leica_pos_data = leica_time_series.Data(truth_time_offset:truth_end_t_idx, leica_x_idx:leica_z_idx);
        remaining_idxs = length(truth_time);
        leica_pos_data = leica_pos_data - kron(ones(remaining_idxs,1), leica_pos_data(1,:));
        % Interpolate (down-sample) estimate position data.
        ijrr17_xyz = ijrr17_xyz(start_t_idx:end,:);
        est_x = interp1(ijrr17_time, ijrr17_xyz(:,pox_x_idx), truth_time); % X
        est_y = interp1(ijrr17_time, ijrr17_xyz(:,pox_y_idx), truth_time); % Y
        est_z = interp1(ijrr17_time, ijrr17_xyz(:,pox_z_idx), truth_time); % Z
        % Also shift estimated position to have it's first point at (0,0,0).
        ijrr17_xyz = [est_x - est_x(1), est_y - est_y(1), est_z - est_z(1)];
    end
    
    % Apply rotation to align recorded initial heading with truth data set.
    theta = -.2;
    Rz = [cos(theta), -sin(theta), 0; ...
          sin(theta),  cos(theta), 0; ...
          0,           0, 1];
    ijrr17_xyz_corrected = (Rz*ijrr17_xyz')';

    % Plot
    if do_plotting
        if i==1
            % Plot Truth on first pass.
            traj_fig = figure();
            plot3(leica_pos_data(:,pox_x_idx),leica_pos_data(:,pox_y_idx),leica_pos_data(:,pox_z_idx));
            grid on
            hold on
            rms_fig = figure();
        end
        % Add recorded data to trajectory figure.
        figure(traj_fig)
        plot3(ijrr17_xyz_corrected(:,pox_x_idx), ijrr17_xyz_corrected(:,pox_y_idx), ijrr17_xyz_corrected(:,pox_z_idx))
        % Add recorded data to Error-magnitude figure.
        rms_fig = plot_traj_rms(leica_pos_data, ijrr17_xyz_corrected,truth_time, rms_fig);
    end
end 
%% Set legends and plot format.
if do_plotting
    % Traj. Fig.
    figure(traj_fig)
    Legend=cell(5,1);
    Legend{1}= 'Truth' ;
    Legend{2}= 'IJRR-17: 50-features';
    Legend{3}= 'IJRR-17: 25-features';
    Legend{4}= 'IJRR-17: 15-features';
    Legend{5}= 'IJRR-17: 7-features';
    legend(Legend);
    xlabel('X-pos (m)')
    ylabel('Y-pos (m)')
    zlabel('Z-pos (m)')
    % RMS Fig.
    figure(rms_fig)
    title('Error-Magnitude')
    Legend=cell(4,1);
    Legend{1}= 'IJRR-17: 50-features';
    Legend{2}= 'IJRR-17: 25-features';
    Legend{3}= 'IJRR-17: 15-features';
    Legend{4}= 'IJRR-17: 7-feature';
    legend(Legend,'Location','NorthWest');
    ylabel('Error (m)')
    xlabel('Time (sec)')
end