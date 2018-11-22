% __________   _______        ______          _______________________
% ___  ____/   ___    |______ ___  /____      ___  __/__  __ \__  __/
% __  __/________  /| |_  __ `/_  /_  _ \     __  /  __  /_/ /_  /   
% _  /___/_____/  ___ |  /_/ /_  / /  __/     _  /   _  _, _/_  /    
% /_____/      /_/  |_|\__, / /_/  \___/      /_/    /_/ |_| /_/     
%                     /____/                                         
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

%% Load log txt file
dir log
file_str=input("file?: ",'s'); %choose log file
filename = strcat('log\',file_str,'.txt'); % path of the log file you want to load (.txt)

log = importdata(filename,'\t'); % import the data in a struct, data matrix will be in log

%%%%%%%%%%%%%%%%%%%
log(log(:,3)>255 | log(:,4)>255 | log(:,5)>255 | log(:,6)>255 | log(:,7)>255 | log(:,8)>255 | log(:,9)>255 | log(:,10)>255,:) = []; % remove byte with value > 255
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

% %% Is the motherfucking inverter on? 
% inverter_on = 1; % 1=yes, 2=fucking no
% % se provi a leggere dati degli inverter con gli inverter spenti ti da
% % errore
% 
% %% Save accumulator voltage and temperature
% % Structure of BMS messages -> ID = 170
% % if byte0 = 1 -> byte1,byte2,byte3 = voltage ; byte4,byte5 = temperature ; byte6, byte7 = max_temperature
% 
% tmp_idx = find(IDcode == 170); % find rows with selected ID
% tmp_data = log(tmp_idx,:); % select all data with the selected ID
% 
% tmp_idx = find(tmp_data(:,3) == 1); % find rows with selected ID and byte0 = 1, i.e. BMS sending voltage and temperature
% tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 1
% 
% tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% 
% % Convert accumulator voltage
% voltage = [tmp_time_stamp, hex2dec(strcat(dec2hex(tmp_data(:,4)),dec2hex(tmp_data(:,5)),dec2hex(tmp_data(:,6))))./10000 ]; % convert bytes in voltage and attach time stamp
% 
% % Convert accumulator temperature
% temperature_accumulator = [tmp_time_stamp, hex2dec(strcat(dec2hex(tmp_data(:,7)),dec2hex(tmp_data(:,8))))./100];
% 
% % Convert accumulator max temperature
% temperature_accumulator_max = [tmp_time_stamp, hex2dec(strcat(dec2hex(tmp_data(:,9)),dec2hex(tmp_data(:,10))))./100];
% 
% if 1
%     figure
%     plot(voltage(:,1),voltage(:,2))
%     xlabel('time (s)');ylabel('Voltage (V)');
%     grid
%     title('Accumulator Voltage');
% end
% 
% if 1
%     figure
%     plot(temperature_accumulator(:,1),temperature_accumulator(:,2),'DisplayName','Mean Temperature')
%     hold on
%     plot(temperature_accumulator_max(:,1),temperature_accumulator_max(:,2),'DisplayName','Peak Temperature')
%     hold off
%     xlabel('time (s)');ylabel('Temperature (°C)');
%     grid
%     title('Accumulator Temperature');
%     legend
% end
% 
% if inverter_on
% %     %% Save current requested to the right inverter
% %     % Structure of Inverter messages -> ID = 514
% %     % if byte0 = 144 -> byte1 = lsb ; byte2 = msb ; CA2 codex
% % 
% %     %tmp_idx = find(IDcode == 514); % find rows with selected ID
% %     tmp_idx = find(IDcode == 513); % find rows with selected ID
% %     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% % 
% %     tmp_idx = find(tmp_data(:,3) == 144); % find rows with selected ID and byte0 = 144, i.e. inverter sending requested current
% %     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 144
% % 
% %     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% % 
% %     % Convert inverter requested current
% %     current_inverter_right = [tmp_time_stamp, -comp2_16bit(tmp_data(:,5),tmp_data(:,4)).*424.4./(256*127*sqrt(2))]; % convert bytes in current and attach time stamp
% % 
% %     if 1
% %         figure
% %         plot(current_inverter_right(:,1),current_inverter_right(:,2))
% %         xlabel('time (s)');ylabel('Current (A)');
% %         grid
% %         title('Right Inverter Current');
% %     end
%     tmp_idx = find(log(:,2) == 513 & log(:,3) == 144); % find rows with selected ID
%     tmp_time_stamp = log(tmp_idx,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
%     current_inverter_right = [tmp_time_stamp,zeros(length(tmp_idx),1)];
% 
%     %% Save current requested to the left inverter
%     % Structure of Inverter messages -> ID = 513
%     % if byte0 = 144 -> byte1 = lsb ; byte2 = msb ; CA2 codex
% 
%     tmp_idx = find(IDcode == 513); % find rows with selected ID
%     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% 
%     tmp_idx = find(tmp_data(:,3) == 144); % find rows with selected ID and byte0 = 144, i.e. inverter sending requested current
%     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 144
% 
%     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% 
%     % Convert inverter requested current
%     current_inverter_left = [tmp_time_stamp, comp2_16bit(tmp_data(:,5),tmp_data(:,4)).*424.4./(256*127*sqrt(2))]; % convert bytes in current and attach time stamp
% 
%     if 1
%         figure
%         plot(current_inverter_left(:,1),current_inverter_left(:,2))
%         xlabel('time (s)');ylabel('Current (A)');
%         grid
%         title('Left Inverter Current');
%     end
% 
% %     if 1
% %         figure
% %         hold on
% %         plot(current_inverter_right(:,1),current_inverter_right(:,2))
% %         plot(current_inverter_left(:,1),current_inverter_left(:,2))
% %         hold off
% %         xlabel('time (s)');ylabel('Current (A)');
% %         grid
% %         title('Inverter Currents');
% %     end
% 
% %  %% Save voltage DCbus to the right inverter
% %     % Structure of Inverter messages -> ID = 386
% %     % if byte0 = 144 -> byte1 = lsb ; byte2 = msb ; CA2 codex
% % 
% %     tmp_idx = find(IDcode == 386); % find rows with selected ID
% %     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% % 
% %     tmp_idx = find(tmp_data(:,3) == 235); % find rows with selected ID and byte0 = 144, i.e. inverter sending requested current
% %     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 144
% % 
% %     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% % 
% %     % Convert inverter requested current
% %%%%%%%%%%%%%%%%% DA SISTEMARE LA CONVERSIONE !!!!!!!!!!!!!!!!
% %     voltage_dcbus_inverter_right = [tmp_time_stamp, -comp2_16bit(tmp_data(:,5),tmp_data(:,4)).*525./32767]; % convert bytes in current and attach time stamp
% % 
% %     if 1
% %         figure
% %         plot(voltage_dcbus_inverter_right(:,1),voltage_dcbus_inverter_right(:,2))
% %         xlabel('time (s)');ylabel('Voltage (V)');
% %         grid
% %         title('Right Inverter Voltage DCbus');
% %     end
%     tmp_idx = find(log(:,2) == 385 & log(:,3) == 235); % find rows with selected ID
%     tmp_time_stamp = log(tmp_idx,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
%     voltage_dcbus_inverter_right = [tmp_time_stamp,zeros(length(tmp_idx),1)];
% 
%  %% Save voltage DCbus to the left inverter
%     % Structure of Inverter messages -> ID = 385
%     % if byte0 = 144 -> byte1 = lsb ; byte2 = msb ; CA2 codex
% 
%     tmp_idx = find(IDcode == 385); % find rows with selected ID
%     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% 
%     tmp_idx = find(tmp_data(:,3) == 235); % find rows with selected ID and byte0 = 144, i.e. inverter sending requested current
%     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 144
% 
%     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% 
%     % Convert inverter requested current
%     %%%%%%%%%%%%%%%%% DA SISTEMARE LA CONVERSIONE !!!!!!!!!!!!!!!!
%     voltage_dcbus_inverter_left = [tmp_time_stamp, comp2_16bit(tmp_data(:,5),tmp_data(:,4)).*525./32767]; % convert bytes in current and attach time stamp
% 
%     if 1
%         figure
%         plot(voltage_dcbus_inverter_left(:,1),voltage_dcbus_inverter_left(:,2))
%         xlabel('time (s)');ylabel('Voltage (V)');
%         grid
%         title('Right Inverter Voltage DCbus');
%     end
% 
% %     if 1
% %         figure
% %         hold on
% %         plot(voltage_dcbus_inverter_right(:,1),voltage_dcbus_inverter_right(:,2))
% %         plot(voltage_dcbus_inverter_left(:,1),voltage_dcbus_inverter_left(:,2))
% %         hold off
% %         xlabel('time (s)');ylabel('Voltage (V)');
% %         grid
% %         title('Inverter Voltage DCbus');
% %     end    
% 
% %     %% Save resolver right inverter
% %     % Structure of Inverter messages -> ID = 386
% %     % if byte0 = 168 -> byte1 = lsb ; byte3 = msb ; CA2 codex
% % 
% %     tmp_idx = find(IDcode == 386); % find rows with selected ID
% %     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% % 
% %     tmp_idx = find(tmp_data(:,3) == 168); % find rows with selected ID and byte0 = 168, i.e. inverter sending requested current
% %     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 168
% % 
% %     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% % 
% %     % Convert resolver speed
% %     resolver_right = [tmp_time_stamp, -(comp2_16bit(tmp_data(:,5),tmp_data(:,4))).*6000./32767]; % convert bytes in current and attach time stamp
% % 
% %     if 1
% %         figure
% %         plot(resolver_right(:,1),resolver_right(:,2))
% %         xlabel('time (s)');ylabel('Motor speed (rpm)');
% %         grid
% %         title('Right Resolver Speed');
% %     end
%     tmp_idx = find(log(:,2) == 385 & log(:,3) == 168); % find rows with selected ID
%     tmp_time_stamp = log(tmp_idx,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
%     resolver_right = [tmp_time_stamp,zeros(length(tmp_idx),1)];
% 
%     %% Save resolver left inverter
%     % Structure of Inverter messages -> ID = 385
%     % if byte0 = 168 -> byte1 = lsb ; byte3 = msb ; CA2 codex
% 
%     tmp_idx = find(IDcode == 385); % find rows with selected ID
%     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% 
%     tmp_idx = find(tmp_data(:,3) == 168); % find rows with selected ID and byte0 = 168, i.e. inverter sending requested current
%     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 168
% 
%     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% 
%     % Convert resolver speed
%     resolver_left = [tmp_time_stamp, comp2_16bit(tmp_data(:,5),tmp_data(:,4)).*6000./32767]; % convert bytes in current and attach time stamp
% 
%     if 1
%         figure
%         plot(resolver_left(:,1),resolver_left(:,2))
%         xlabel('time (s)');ylabel('Motor speed (rpm)');
%         grid
%         title('Left Resolver Speed');
%     end
% 
%     % Plot both resolver speeds
%     if 1
%         figure
%         hold on
%         plot(resolver_right(:,1),resolver_right(:,2),'DisplayName','Right')
%         plot(resolver_left(:,1),resolver_left(:,2),'DisplayName','Left')
%         hold off
%         xlabel('time (s)');ylabel('Motor speed (rpm)');
%         grid
%         legend
%         title('Resolvers Speed');
%     end
%     
% %     % Approximated vehicle speed
% %         if 1
% %         figure
% %         hold on
% %         plot(resolver_right(:,1),resolver_right(:,2)./3.47/60*2*pi*0.2*3.6,'DisplayName','Right')
% %         plot(resolver_left(:,1),resolver_left(:,2)./3.47/60*2*pi*0.2*3.6,'DisplayName','Left')
% %         hold off
% %         xlabel('time (s)');ylabel('Vehicle speed (km/h)');
% %         grid
% %         legend
% %         title('Approximated vehicle speed from resolvers');
% %     end
%     
%     
% 
% %     %% Save IGBT temperature right inverter
% %     % Structure of Inverter messages -> ID = 386
% %     % if byte0 = 74 -> byte1 = lsb ; byte3 = msb ; CA2 codex
% % 
% %     tmp_idx = find(IDcode == 386); % find rows with selected ID
% %     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% % 
% %     tmp_idx = find(tmp_data(:,3) == 74); % find rows with selected ID and byte0 = 74, i.e. inverter sending requested current
% %     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 74
% % 
% %     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% % 
% %     % Convert resolver speed
% %     IGBT_temperature_right = [tmp_time_stamp, (comp2_16bit(tmp_data(:,5),tmp_data(:,4))-15797)./112.1182]; % convert bytes in current and attach time stamp
% % 
% %     if 1
% %         figure
% %         plot(IGBT_temperature_right(:,1),IGBT_temperature_right(:,2))
% %         xlabel('time (s)');ylabel('Temperature (°C)');
% %         grid
% %         title('IGBT Temperature Right Inverter');
% %     end
%     tmp_idx = find(log(:,2) == 385 & log(:,3) == 74); % find rows with selected ID
%     tmp_time_stamp = log(tmp_idx,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
%     IGBT_temperature_right = [tmp_time_stamp,zeros(length(tmp_idx),1)];
%     
%     %% Save IGBT temperature left inverter
%     % Structure of Inverter messages -> ID = 385
%     % if byte0 = 74 -> byte1 = lsb ; byte3 = msb ; CA2 codex
% 
%     tmp_idx = find(IDcode == 385); % find rows with selected ID
%     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% 
%     tmp_idx = find(tmp_data(:,3) == 74); % find rows with selected ID and byte0 = 74, i.e. inverter sending requested current
%     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 74
% 
%     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% 
%     % Convert resolver speed
%     IGBT_temperature_left = [tmp_time_stamp, (comp2_16bit(tmp_data(:,5),tmp_data(:,4))-15797)./112.1182]; % convert bytes in current and attach time stamp
% 
%     if 1
%         figure
%         plot(IGBT_temperature_left(:,1),IGBT_temperature_left(:,2))
%         xlabel('time (s)');ylabel('Temperature (°C)');
%         grid
%         title('IGBT Temperature Left Inverter');
%     end
% 
% %     if 1
% %         figure
% %         hold on
% %         plot(IGBT_temperature_right(:,1),IGBT_temperature_right(:,2),'DisplayName','Right Inverter')
% %         plot(IGBT_temperature_left(:,1),IGBT_temperature_left(:,2),'DisplayName','Left Inverter')
% %         hold off
% %         xlabel('time (s)');ylabel('Temperature (°C)');
% %         grid
% %         legend
% %         title('IGBT Temperatures');
% %     end
% 
% 
% %     %% Save right motor temperature
% %     % Structure of Inverter messages -> ID = 386
% %     % if byte0 = 73 -> byte1 = lsb ; byte3 = msb ; CA2 codex
% % 
% %     tmp_idx = find(IDcode == 386); % find rows with selected ID
% %     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% % 
% %     tmp_idx = find(tmp_data(:,3) == 73); % find rows with selected ID and byte0 = 73, i.e. inverter sending requested current
% %     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 73
% % 
% %     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% % 
% %     % Convert resolver speed
% %     motor_temperature_right = [tmp_time_stamp, (comp2_16bit(tmp_data(:,5),tmp_data(:,4))-9393.9)./55.1042]; % convert bytes in current and attach time stamp
% % 
% %     if 1
% %         figure
% %         plot(motor_temperature_right(:,1),motor_temperature_right(:,2))
% %         xlabel('time (s)');ylabel('Temperature (°C)');
% %         grid
% %         title('Right motor temperature');
% %     end
%     tmp_idx = find(log(:,2) == 385 & log(:,3) == 73); % find rows with selected ID
%     tmp_time_stamp = log(tmp_idx,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
%     motor_temperature_right = [tmp_time_stamp,zeros(length(tmp_idx),1)];
%     %% Save left motor temperature
%     % Structure of Inverter messages -> ID = 385
%     % if byte0 = 73 -> byte1 = lsb ; byte3 = msb ; CA2 codex
% 
%     tmp_idx = find(IDcode == 385); % find rows with selected ID
%     tmp_data = log(tmp_idx,:); % select all data with the selected ID
% 
%     tmp_idx = find(tmp_data(:,3) == 73); % find rows with selected ID and byte0 = 73, i.e. inverter sending requested current
%     tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 73
% 
%     tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s
% 
%     % Convert resolver speed
%     motor_temperature_left = [tmp_time_stamp, (comp2_16bit(tmp_data(:,5),tmp_data(:,4))-9393.9)./55.1042]; % convert bytes in current and attach time stamp
% 
%     if 1
%         figure
%         plot(motor_temperature_left(:,1),motor_temperature_left(:,2))
%         xlabel('time (s)');ylabel('Temperature (°C)');
%         grid
%         title('Left motor temperature');
%     end
% 
% %     if 1
% %         figure
% %         hold on
% %         plot(motor_temperature_right(:,1),motor_temperature_right(:,2),'DisplayName','Right')
% %         plot(motor_temperature_left(:,1),motor_temperature_left(:,2),'DisplayName','Left')
% %         hold off
% %         xlabel('time (s)');ylabel('Temperature (°C)');
% %         grid
% %         legend
% %         title('Motors temperature');
% %     end
%     
% end

%% SCS Pedals
% Structure of SCS pedal messages -> ID = 176
% if byte0 = 1 -> byte6 = thottle %

tmp_idx = find(IDcode == 176); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 2); % find rows with selected ID and byte0 = 1
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 1

tmp_time_stamp = tmp_data(:,1)./(1000*1000); % create a column of time stamps of the selected data and convert from ms to s

% Convert inverter requested current
SCS_pedal = [tmp_time_stamp, tmp_data(:,9)]; % convert bytes and attach time stamp

% SCS_pedal plotted in the next function

%% Save throttle and brake potentiometer
% Structure of throttle pot messages -> ID = 176
% if byte0 = 1 -> byte1 = thottle %

tmp_idx = find(IDcode == 176); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 1); % find rows with selected ID and byte0 = 1
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 1

tmp_time_stamp = tmp_data(:,1)./1000; % create a column of time stamps of the selected data and convert from ms to s

% Convert inverter requested current
throttle_perc = [tmp_time_stamp, tmp_data(:,4)]; % convert bytes and attach time stamp

% Structure of brake pot messages -> ID = 176
% if byte0 = 2 -> byte1 = brake %

tmp_idx = find(IDcode == 176); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 2); % find rows with selected ID and byte0 = 2
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 2

tmp_time_stamp = tmp_data(:,1)./(1000*1000); % create a column of time stamps of the selected data and convert from ms to s

% Convert inverter requested current
brake_perc = [tmp_time_stamp, tmp_data(:,4)]; % convert bytes and attach time stamp
% 
% [n w] =  buttord(0.95, 0.99,3,40);
% [b,a] = butter(n,w);
% prova = fft(tmp_data(:,4));
% 
% sign = filter(b,a,prova);
% aa = ifft(sign);
if 1
    figure
    hold on
    subplot(3,1,1);
        plot(throttle_perc(:,1),throttle_perc(:,2),'DisplayName', 'Throttle')
        xlabel('time (s)');ylabel('Percentage (%))');
        legend
        grid
        title('Throttle Percentage');
    subplot(3,1,2);
        plot(brake_perc(:,1),brake_perc(:,2),'DisplayName', 'Brake')
        xlabel('time (s)');ylabel('Percentage (%))');
        legend
        grid
        title('Brake Percentage');
    subplot(3,1,3);
        plot(SCS_pedal(:,1),SCS_pedal(:,2),'DisplayName', 'Throttle')
        xlabel('time (s)');ylabel('Logical (-))');
        legend
        grid
        title('SCS Pedal');
    hold off
end
%% Save steer potentiometer
% Structure of steer pot messages -> ID = 192
% if byte0 = 1 -> byte1 = steer %

tmp_idx = find(IDcode == 192); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 2); % find rows with selected ID and byte0 = 1
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 1

tmp_time_stamp = tmp_data(:,1)./(1000*1000); % create a column of time stamps of the selected data and convert from ms to s

% Convert inverter requested current
steer_perc = [tmp_time_stamp, tmp_data(:,4)]; % convert bytes and attach time stamp

right_steer_lim = -80;
left_steer_lim = 180;
range_steer = abs(right_steer_lim) + abs(left_steer_lim);
% steer_conversion = -100 + pot/range_steer
if 1
    figure
    plot(steer_perc(:,1),left_steer_lim - steer_perc(:,2).*range_steer./100)
    xlabel('time (s)');ylabel('Steer angle (deg))');
    grid
    title('Steer Angle');
end

% First order filter for accelerometers and gyros
fs_filt_acc = 250; % Hz
d = designfilt('lowpassfir', ...
    'PassbandFrequency',1*2*pi/fs_filt_acc,...
    'StopbandFrequency',1.2*2*pi/fs_filt_acc);

%% Save Accelerometer 
% Structure of accelerometer messages -> ID = 192
% if byte0 = 5 -> byte1 = acc X (m/s^2); byte2 = acc Y (m/s^2); byte3 = acc Z (m/s^2); 

tmp_idx = find(IDcode == 192); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 5); % find rows with selected ID and byte0 = 5
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 5

tmp_time_stamp = tmp_data(:,1)./(1000*1000); % create a column of time stamps of the selected data and convert from ms to s

% Convert inverter requested current
acc_x = [tmp_time_stamp, comp2_16bit(tmp_data(:,4),tmp_data(:,5))./100]; % convert bytes  and attach time stamp
acc_y = [tmp_time_stamp, comp2_16bit(tmp_data(:,6),tmp_data(:,7))./100]; % convert bytes  and attach time stamp
acc_z = [tmp_time_stamp, comp2_16bit(tmp_data(:,8),tmp_data(:,9))./100]; % convert bytes  and attach time stamp

% Convert inverter requested current FILTERED
% acc_x = [tmp_time_stamp, filtfilt(d,comp2_16bit(tmp_data(:,4),tmp_data(:,5))./100)]; % convert bytes  and attach time stamp
% acc_y = [tmp_time_stamp, filtfilt(d,comp2_16bit(tmp_data(:,6),tmp_data(:,7))./100)]; % convert bytes  and attach time stamp
% acc_z = [tmp_time_stamp, filtfilt(d,comp2_16bit(tmp_data(:,8),tmp_data(:,9))./100)]; % convert bytes  and attach time stamp

if 1
    figure
    subplot(3,1,1)
    plot(acc_x(:,1), acc_x(:,2))
    xlabel('time (s)');ylabel('AX (m/s^2))');
    grid
    title('Acceleration from IMU');
    
      subplot(3,1,2)
    plot(acc_y(:,1), acc_y(:,2))
    xlabel('time (s)');ylabel('AY (m/s^2))');
    grid
    
      subplot(3,1,3)
    plot(acc_z(:,1), acc_z(:,2))
    xlabel('time (s)');ylabel('AZ (m/s^2))');
    grid
end

%% Save Gyro 
% Structure of gyro messages -> ID = 192
% if byte0 = 4 -> byte1 = roll vel X (deg/s); byte2 = pitch vel Y (deg/s); byte3 = yaw vel Z (deg/s); 

tmp_idx = find(IDcode == 192); % find rows with selected ID
tmp_data = log(tmp_idx,:); % select all data with the selected ID

tmp_idx = find(tmp_data(:,3) == 4); % find rows with selected ID and byte0 = 4
tmp_data = tmp_data(tmp_idx,:); % select all data with the selected ID and byte0 = 4

tmp_time_stamp = tmp_data(:,1)./(1000*1000); % create a column of time stamps of the selected data and convert from ms to s

% Convert inverter requested current
phi_x = [tmp_time_stamp, comp2_16bit(tmp_data(:,4),tmp_data(:,5))./100]; % convert bytes  and attach time stamp
mu_y = [tmp_time_stamp, comp2_16bit(tmp_data(:,6),tmp_data(:,7))./100]; % convert bytes  and attach time stamp
omega_z = [tmp_time_stamp, comp2_16bit(tmp_data(:,8),tmp_data(:,9))./100]; % convert bytes  and attach time stamp


if 1
    figure
    subplot(3,1,1)
    plot(phi_x(:,1), phi_x(:,2))
    xlabel('time (s)');ylabel('phi X (deg/s))');
    grid
    title('Gyro velocities from IMU');
    
      subplot(3,1,2)
    plot(mu_y(:,1), mu_y(:,2))
    xlabel('time (s)');ylabel('mu Y (deg/s))');
    grid
    
      subplot(3,1,3)
    plot(omega_z(:,1), omega_z(:,2))
    xlabel('time (s)');ylabel('omega Z (deg/s))');
    grid
end


% %% Power (approximate)
% if inverter_on
%     % We need to resample and synchronize the signals
%     fs = 10; % frequency of resampled signal
%     % Convert the signals to timeseries (two columns, one sample times and one data)
%     voltage_ts = timeseries(voltage(:,2),voltage(:,1)); 
%     current_inverter_right_ts = timeseries(current_inverter_right(:,2),current_inverter_right(:,1));
%     current_inverter_left_ts = timeseries(current_inverter_left(:,2),current_inverter_left(:,1));
% 
%     [voltage_ts current_inverter_right_ts] = synchronize(voltage_ts, current_inverter_right_ts,'Uniform','Interval',1/fs);
%     [voltage_ts current_inverter_left_ts] = synchronize(voltage_ts, current_inverter_left_ts, 'Uniform','Interval',1/fs);
%    % [current_inverter_right_ts current_inverter_left_ts] = synchronize(current_inverter_right_ts, current_inverter_left_ts, 'Uniform','Interval',1/fs);
% 
%     if 1
%         figure
%         plot(voltage_ts.time,voltage_ts.data.*(current_inverter_right_ts.data + current_inverter_left_ts.data)/1000)
%         xlabel('time (s)');ylabel('Power (kW))');
%         grid
%         title('Total Power Pagot');
%     end
% end
% 
% %% Power at wheel (more accurate)
% if inverter_on
%     % We need to resample and synchronize the signals
%     fs = 10; % frequency of resampled signal
%     rho_m = 0.98; % motor efficency 
%     rho_g = 0.98; % gear efficency 
%     cphi = 0.85; % ~ cos phi for syncronous motor
%     % Convert the signals to timeseries (two columns, one sample times and one data)
%     voltage_dcbus_inverter_right_ts = timeseries(voltage_dcbus_inverter_right(:,2),voltage_dcbus_inverter_right(:,1)); 
%     voltage_dcbus_inverter_left_ts = timeseries(voltage_dcbus_inverter_left(:,2),voltage_dcbus_inverter_left(:,1)); 
%     current_inverter_right_ts = timeseries(current_inverter_right(:,2),current_inverter_right(:,1));
%     current_inverter_left_ts = timeseries(current_inverter_left(:,2),current_inverter_left(:,1));
% 
%     [voltage_dcbus_inverter_right_ts current_inverter_right_ts] = synchronize(voltage_dcbus_inverter_right_ts, current_inverter_right_ts,'Uniform','Interval',1/fs);
%     [voltage_dcbus_inverter_left_ts current_inverter_left_ts] = synchronize(voltage_dcbus_inverter_left_ts, current_inverter_left_ts, 'Uniform','Interval',1/fs);
%     %[current_inverter_right_ts current_inverter_left_ts] = synchronize(current_inverter_right_ts, current_inverter_left_ts, 'Uniform','Interval',1/fs);
% 
%     if 1
%         figure
%         plot(voltage_dcbus_inverter_right_ts.time,(sqrt(3).*voltage_dcbus_inverter_right_ts.data./sqrt(2).*(current_inverter_right_ts.data)+ ...
%                                        sqrt(3).*voltage_dcbus_inverter_left_ts.data./sqrt(2).*(current_inverter_left_ts.data))*cphi*rho_m*rho_g/1000)
%         xlabel('time (s)');ylabel('Power at wheel (kW))');
%         grid
%         title('Total Power at wheel');
%     end
% end