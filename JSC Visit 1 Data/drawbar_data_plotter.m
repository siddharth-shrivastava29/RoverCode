% imported data has 7 columns that are defined as below:
% [Status TCMID LEDID X(m) Y(m) Z(m) Timestamp(us)]

%% SETUP
% FIRST RUN THE PYTHON SCRIPT (drawbar-data-cleaner.py) TO CLEAN COMMENTS
% ENTER THE FILENAME IN THE LINE BELOW
filename = '7_12_19_02_wheel_3leggedmode_twoOnereuced_good_cleaned_rawdata.txt';
run('matlab_data_importer.m') % this runs the MATLAB generated importer, do not remove
%% THE PLOTS
figure
plot(cleanedrawdata(:,7),cleanedrawdata(:,4),'.')
xlabel('time (\mus)')
ylabel('X (m)')
title('Test raw drawbar data: X-displacement versus time')
figure
plot(cleanedrawdata(:,7),cleanedrawdata(:,5),'.')
xlabel('time (\mus)')
ylabel('Y (m)')
title('Test raw drawbar data: Y-displacement versus time')
figure
plot(cleanedrawdata(:,7),cleanedrawdata(:,6),'.')
xlabel('time (\mus)')
ylabel('Z (m)')
title('Test raw drawbar data: Z-displacement versus time')