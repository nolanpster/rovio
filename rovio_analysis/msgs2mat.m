function msg_mat = msgs2mat(msgs, accessor, num_cols)
% msgs - a cell array of ROS messages
% accessor - a function handle that returns 

num_msgs = size(msgs,1);
msg_mat = zeros(num_msgs,num_cols);
for msg_idx = 1:num_msgs
    msg_mat(msg_idx,:) = accessor(msg_idx);
end