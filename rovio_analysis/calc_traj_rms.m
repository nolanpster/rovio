function rms = calc_traj_rms(traj_1, traj_2)
% Assume same number of points in matricies. (Nx3)
rms = sqrt(sum((traj_1 - traj_2).^2, 2));