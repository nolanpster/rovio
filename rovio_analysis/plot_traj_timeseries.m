function fig = plot_traj_timeseries(ts, fig)
% ts - timeseries of of trajectory points. Expects ts.Data columns to be [x-pos, y-pos, z-pos, ...] .
x_idx = 4;
y_idx = 5;
z_idx = 6;
if nargin > 1
    figure(fig);
else
    fig = figure();
end

% Show plot relative to origin (0,0,0).
x_start = ts(1,x_idx);
y_start = ts(1,y_idx);
z_start = ts(1,z_idx);

x_pos = ts(:,x_idx) - x_start;
y_pos = ts(:,y_idx) - y_start;
z_pos = ts(:,z_idx) - z_start;

plot3(x_pos, y_pos, z_pos)
