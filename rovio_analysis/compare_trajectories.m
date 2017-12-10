close all

reload_mat_files = false;
do_plotting = true;

% Script that loads Leica data from rosbag
if reload_mat_files
    clear('pose_wcs_cell', 'leica_time_series')
    load_truth_trajectory; % Loades leica_time_series
    load('ijrr17_pose'); % Loads pose_wcs_cell
end
% The truth data isn't sampled regularly until a few seconds in ...
truth_time_offset = 60;
truth_time_duration = leica_time_series.TimeInfo.End - leica_time_series.TimeInfo.Start;
truth_start_epoch = leica_time_series.Time(truth_time_offset);
truth_end_epoch = leica_time_series.Time(end);
raw_truth_time = leica_time_series.Time(truth_time_offset:end);
truth_time = leica_time_series.Time(truth_time_offset:end) - leica_time_series.Time(truth_time_offset);

num_msgs = size(pose_wcs_cell,1);

% Extract estimated position.
% Msg Accessor function.
pose_wcs_accessor = @(x) [pose_wcs_cell{x}.Header.Stamp.Sec, pose_wcs_cell{x}.Header.Stamp.Nsec, ...
                          pose_wcs_cell{x}.Pose.Pose.Position.X, ...
                          pose_wcs_cell{x}.Pose.Pose.Position.Y, pose_wcs_cell{x}.Pose.Pose.Position.Z, ...
                          pose_wcs_cell{x}.Pose.Pose.Orientation.X, pose_wcs_cell{x}.Pose.Pose.Orientation.Y, ...
                          pose_wcs_cell{x}.Pose.Pose.Orientation.Z, pose_wcs_cell{x}.Pose.Pose.Orientation.W,];
ijrr17_data = msgs2mat(pose_wcs_cell, pose_wcs_accessor, 9);


msg_sec_idx = 1;
msg_nsec_idx = 2;
msg_x_idx = 3;
msg_y_idx = 4;
msg_z_idx = 5;
msg_qx_idx = 6;
msg_qy_idx = 7;
msg_qz_idx = 8;
msg_qw_idx = 9;
pox_x_idx = 1;
pox_y_idx = 2;
pox_z_idx = 3;
leica_x_idx = 4;
leica_y_idx = 5;
leica_z_idx = 6;

ijrr17_origin_loc_mat = kron(ones(num_msgs,1), ...
                             [ijrr17_data(1,msg_x_idx), ijrr17_data(1,msg_y_idx), ijrr17_data(1,msg_z_idx)]);
ijrr17_xyz = ijrr17_data(:,msg_x_idx:msg_z_idx) - ijrr17_origin_loc_mat;

% Find closest index to the starting epoch of the truth time.
ijrr17_raw_time = ijrr17_data(:, msg_sec_idx) + ijrr17_data(:, msg_nsec_idx)*1e-9;
start_t_idx = find(ijrr17_raw_time >= truth_start_epoch, 1);
% Find final time value and trim data-sets.
if ijrr17_raw_time(end) > truth_end_epoch
    % Not implemented yet.
    dbstop()
    a = 1;
else
    [t_diff, truth_end_t_idx] = min(abs(ijrr17_raw_time(end) - raw_truth_time));
    truth_time = leica_time_series.Time(truth_time_offset:truth_end_t_idx) - leica_time_series.Time(truth_time_offset);
    remaining_idxs = length(truth_time);
    ijrr17_time = ijrr17_raw_time(start_t_idx:end) - ijrr17_raw_time(start_t_idx);
    % Trim irregularly sampled data.
    leica_pos_data = leica_time_series.Data(truth_time_offset:truth_end_t_idx, leica_x_idx:leica_z_idx);
    leica_pos_data = leica_pos_data - kron(ones(remaining_idxs,1), leica_pos_data(1,:));
    ijrr17_xyz = ijrr17_xyz(start_t_idx:end,:);
    % Interpolate (down-sample) estimate position data.
    est_x = interp1(ijrr17_time, ijrr17_xyz(:,pox_x_idx), truth_time); % X
    est_y = interp1(ijrr17_time, ijrr17_xyz(:,pox_y_idx), truth_time); % Y
    est_z = interp1(ijrr17_time, ijrr17_xyz(:,pox_z_idx), truth_time); % Z
    % Also shift estimated position to have it's first point at (0,0,0).
    ijrr17_xyz = [est_x - est_x(1), est_y - est_y(1), est_z - est_z(1)]; 
end


theta = -.2;
Rz = [cos(theta), -sin(theta), 0;...
      sin(theta),  cos(theta), 0;...
               0,           0, 1];
ijrr17_xyz_corrected = (Rz*ijrr17_xyz')';


if do_plotting
    % Plot Truth and Estimated position.
    traj_fig = figure();
    plot3(leica_pos_data(:,pox_x_idx),leica_pos_data(:,pox_y_idx),leica_pos_data(:,pox_z_idx));
    hold on
    figure(traj_fig)
    plot3(ijrr17_xyz_corrected(:,pox_x_idx), ijrr17_xyz_corrected(:,pox_y_idx), ijrr17_xyz_corrected(:,pox_z_idx))
    Legend=cell(2,1);
    Legend{1}=' Truth' ;
    Legend{2}=' IJRR-17';
    legend(Legend);
    xlabel('X-pos (m)')
    ylabel('Y-pos (m)')
    zlabel('Z-pos (m)')
    rms_fig = plot_traj_rms(leica_pos_data, ijrr17_xyz_corrected,truth_time);
    title('Error-Magnitude')
    Legend=cell(1,1);
    Legend{1}=' IJRR-17';
    legend(Legend);
    xlabel('Error (m)')
    ylabel('Time (sec)')
    
end
    