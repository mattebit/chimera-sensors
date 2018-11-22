% __________   _______        ______          _______________________
% ___  ____/   ___    |______ ___  /____      ___  __/__  __ \__  __/
% __  __/________  /| |_  __ `/_  /_  _ \     __  /  __  /_/ /_  /   
% _  /___/_____/  ___ |  /_/ /_  / /  __/     _  /   _  _, _/_  /    
% /_____/      /_/  |_|\__, / /_/  \___/      /_/    /_/ |_| /_/     
%                     /____/                                         
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

%% Load log txt file
dir log %display log files
file_str=input("file?: ",'s'); %choose log file
filename = strcat('log\',file_str,'.txt'); % path of the log file you want to load (.txt)

log = importdata(filename,'\t'); % import the data in a struct, data matrix will be in log

%%%%%%%%%%%%%%%%%%%
%log(log(:,3)>255 | log(:,4)>255 | log(:,5)>255 | log(:,6)>255 | log(:,7)>255 | log(:,8)>255 | log(:,9)>255 | log(:,10)>255,:) = []; % remove byte with value > 255
[~,idx] = sort(log(:,1)); % sort just the first column
log = log(idx,:); % log sorted by timestamp
%%%%%%%%%%%%%%%%%%%

log(any(isnan(log), 2), :) = []; % delete the rows with any NaN inside

%% Structure
% Data ->        Timestamp     ID       byte0     byte1     byte2     byte3     byte4     byte5     byte6     byte7
% Column Idx ->      1          2         3         4         5         6         7         8         9        10


%% Identifier
IDcode = log(:,2); % extract the column of messages IDs
time_stamps = log(:,1)/1000; % extract the column of time stamps

%% Save latitude from gps
% Structure of GPS messages -> ID = 208
% if byte0 = 3 -> byte1 = velocity unit?

tmp_idx = find(IDcode == 208); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 7); % find rows with selected ID and byte0 = 7
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 7

tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
stem(tmp_time_stamp);
% length(tmp_time_stamp)
% for i = 0:length(tmp_time_stamp)
%  delta(i)=tmp_time_stamp(i+1)-tmp_time_stamp(i);
% end
% stem(delta);
% Convert datas
GPS_longitude = [tmp_time_stamp, (comp2_16bit_unsigned(tmp_data(:,4),tmp_data(:,5)).*100000)+(comp2_16bit_unsigned(tmp_data(:,6),tmp_data(:,7)))]; % convert bytes and attach time stamp
GPS_longitude_orientation = [tmp_time_stamp, tmp_data(:,8)];
GPS_altitude = [tmp_time_stamp, comp2_16bit_unsigned(tmp_data(:,9),tmp_data(:,10))];

figure
stem(GPS_longitude(:,1),GPS_longitude(:,2))
xlabel('time (s)');ylabel('longitude');
grid on;
title('GPS longitude');

figure
stem(GPS_altitude(:,1),GPS_altitude(:,2))
xlabel('time (s)');ylabel('altitude');
grid on;
title('GPS altitude');

tmp_idx = find(IDcode == 208) % find rows with selected ID
tmp_data = log(tmp_idx,:) % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 8); % find rows with selected ID and byte0 = 8
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 8

tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s

GPS_latitude = [tmp_time_stamp, (comp2_16bit_unsigned(tmp_data(:,4),tmp_data(:,5)).*10000)+(comp2_16bit_unsigned(tmp_data(:,6),tmp_data(:,7)))]; % convert bytes and attach time stamp
GPS_latitude_orientation = [tmp_time_stamp, tmp_data(:,8)];
GPS_speed = [tmp_time_stamp, comp2_16bit_unsigned(tmp_data(:,9),tmp_data(:,10))];

figure
stem(GPS_latitude(:,1),GPS_latitude(:,2))
xlabel('time (s)');ylabel('latitude');
grid on;
title('GPS latitude');

figure
stem(GPS_speed(:,1),GPS_speed(:,2))
xlabel('time (s)');ylabel('speed');
grid on;
title('GPS speed');