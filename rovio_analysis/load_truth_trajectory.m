reload_all_data = false;
if reload_all_data
    easy01_dataset_bag = rosbag('/home/nolan/rovio_datasets/euroc/01_easy/dataset.bag');
    leica_bag = select(easy01_dataset_bag, 'Time', [easy01_dataset_bag.StartTime, easy01_dataset_bag.EndTime], ...
                       'Topic','/leica/position');
    all_leica_msgs = readMessages(leica_bag);
    leica_time_series = timeseries(leica_bag);
    save('easy01_truth', 'all_leica_msgs', 'leica_time_series')
else
    load('easy01_truth')
end
    