function fig = plot_traj_rms(truth_traj, estimated_traj, time, fig)

if nargin < 4
    fig = figure();
else
    figure(fig);
    hold on
end

rms = calc_traj_rms(truth_traj, estimated_traj);
plot(time, rms);