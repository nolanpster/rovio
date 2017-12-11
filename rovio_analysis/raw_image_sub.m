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
node = robotics.ros.Node('/test');
image0_sub = robotics.ros.Subscriber(node,'/cam0/image_raw','sensor_msgs/Image');
image1_sub = robotics.ros.Subscriber(node,'/cam1/image_raw','sensor_msgs/Image');
pause(1)
im0_msg = receive(image0_sub)
im1_msg = receive(image1_sub)


%pose_sub = rossubscriber('/rovio/pose_with_covariance_stamped');

%image0_sub = rossubscriber('/cam0/image_raw',1);
%image0_msg = receive(image0_sub, 1);

%image1_sub = rossubscriber('/cam1/image_raw',1);
%image1_msg = receive(image1_sub, 1);