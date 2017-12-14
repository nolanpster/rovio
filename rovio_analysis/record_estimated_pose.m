matfile_name = 'ijrr17_pose';

rosshutdown
roscore_timeout = 10;
tic
while toc < roscore_timeout 
    try
        if ~robotics.ros.internal.Global.isNodeActive
            rosinit
            break
        end
    catch ME
        if toc < roscore_timeout
            continue
        else
            rethrow(ME)
        end 
    end
end

desiredRate = 45; %Hz
receive_timeout = 80;
overrunAction = 'slip';
rate = rosrate(desiredRate);
r.OverrunAction = overrunAction;
% disp('Hit any key to start subscribing.')
% pause
pose_wcs_cell = cell(0);
pose_sub = rossubscriber('/rovio/pose_with_covariance_stamped');
msg_idx = 1;
for receive_attempt = 1:desiredRate*receive_timeout
    % Receive a 'pose_with_covariance_stamped' message from ROVIO.
    pose_wcs_msg = [];
    try
        pose_wcs_msg = receive(pose_sub, 1/10);
    catch ME
        if strcmp(ME.identifier, 'robotics:ros:subscriber:WaitTimeout')
            continue
        else
            rethrow(ME)
        end
    end
    if ~isempty(pose_wcs_msg)
         % Might have to use a deep-copy
        pose_wcs_cell{msg_idx} = copy(pose_wcs_msg);
        msg_idx = msg_idx + 1;
    end
    waitfor(rate); 
    if mod(receive_attempt,100)==0
        fprintf('Total messages received: %i\n',msg_idx)
    end
end

rosshutdown
reply = input('Do you want to overwrite the recorded data file?? Y/N [Y]:','s');
if isempty(reply)
  reply = 'Y';
end
if STRCMP(upper(reply),'Y')
    save(matfile_name, 'pose_wcs_cell')
end