%% DATA IMPORT

%% tabula rasa
clear all
close all
clc

%% load data
importdata('datarecording_1432488783.csv')

flight_data=ans.data;

%% save to appropriate variables

timestamp = flight_data(:,1);

panPosition = flight_data(:,2);
tiltPosition = flight_data(:,3);

panCtrlType = flight_data(:,4);
panSetValue = flight_data(:,5);

tiltCtrlType = flight_data(:,6);
tiltSetValue = flight_data(:,7);

local_lat = flight_data(:,8);
local_lon = flight_data(:,9);
local_elev = flight_data(:,10);

object_lat = flight_data(:,11);
object_lon = flight_data(:,12);
object_elev = flight_data(:,13);

object_x = flight_data(:,14);
object_y = flight_data(:,15);
object_z = flight_data(:,16);

estObject_x = flight_data(:,17);
estObject_y = flight_data(:,18);
estObject_z = flight_data(:,19);

estObject_lat = flight_data(:,20);
estObject_lon = flight_data(:,21);
estObject_elev = flight_data(:,22);

time_vector=(timestamp-timestamp(1))*10^(-6);


%% calculate desired antenna angles
%{
% KF estimator
p_antenna=[0 0 0];
position_true = [pNE.signals.values(1:end,2) pNE.signals.values(1:end,1) pD.signals.values(1:end)];

pan_true=zeros(length(position_true),1);
tilt_true=pan_true;
range=pan_true;

for i=1:length(position_true)
[pan_true(i),tilt_true(i),range(i)]=antenna_angle(position_true(i,:),p_antenna);
end

% linear estimator

position_lin = [pos_fwc_x(1:end,1) pos_fwc_y(1:end,1) pos_fwc_z(1:end,1)];

pan_lin=zeros(length(position_lin),1);
tilt_lin=pan_lin;

for i=1:length(position_lin)
[pan_lin(i),tilt_lin(i),~]=antenna_angle(position_lin(i,:),p_antenna);
end

time_lin=0:0.1:(((1/Ts_kf*stoptime)+1-v)*Ts_kf)-Ts_kf;

% hold angles

position_hold = [pos_hold_x(1:end,1) pos_hold_y(1:end,1) pos_hold_z(1:end,1)];

pan_hold=zeros(length(position_hold),1);
tilt_hold=pan_hold;

for i=1:length(position_hold)
[pan_hold(i),tilt_hold(i),~]=antenna_angle(position_hold(i,:),p_antenna);
end

time_hold=0:0.1:(((1/Ts_kf*stoptime)+1-v)*Ts_kf)-Ts_kf;

%}
%% plots


% flight path

figure('color','w')
title('Flight Path')
set(gcf,'units','normalized','outerposition',[0 0 1 1])
hold on; grid on; box on;axis equal;
 
h1=plot3(estObject_x,estObject_y,estObject_z,'-r');
h2=plot3(object_x,object_y,object_z,'-b');

xlabel('East(y) (m)')
ylabel('North(x) (m)')
zlabel('Up(z) (m)')
legend([h1,h2],'Antenna Estimation','Real Flight Path')

% antenna angles
%{
panPosition_deg=epos2deg(panPosition);
tiltPosition_deg=epos2deg(tiltPosition);

figure('color','w')
title('Antenna Angles')
%set(gcf,'units','normalized','outerposition',[0 0 1 1])

ax1=subplot(2,1,1);
hold on; grid on; box on;
h4=plot(pNE.time,pan_true*180/pi,'-b');
h3=plot(time_vector,panSetValue,'-r');
h7=plot(time_lin,pan_lin*180/pi,'color',[0 0.8 0]);
%h9=plot(time_hold,pan_hold*180/pi,'m');
ylim([40 160])
xlabel('Time (s)')
ylabel('Pan Angle [degree]')
legend([h3,h4,h7],'KFestPosition','truePosition','linestPosition')
%legend([h3,h4,h7,h9],'KFestPosition','truePosition','linestPosition','holdangle')

ax2=subplot(2,1,2);
hold on; grid on; box on;
h6=plot(pNE.time,tilt_true*180/pi,'-b');
h5=plot(time_vector,tiltSetValue,'-r');
h8=plot(time_lin,tilt_lin*180/pi,'color',[0 0.8 0]);
%h10=plot(time_hold,tilt_hold*180/pi,'m');

xlabel('Time (s)')
ylabel('Tilt Angle [degree]')
legend([h5,h6,h8],'estPosition','truePosition','linestPosition')
%legend([h5,h6,h8,h10],'estPosition','truePosition','linestPosition','holdangle')

linkaxes([ax1,ax2],'x')


%% error calculation

x_error=0:0.1:100;

y_pan_true=interp1(pNE.time,pan_true*180/pi,x_error);
y_pan_est=interp1(time_vector,panSetValue,x_error);

y_tilt_true=interp1(pNE.time,tilt_true*180/pi,x_error);
y_tilt_est=interp1(time_vector,tiltSetValue,x_error);

y_range=interp1(pNE.time,range,x_error);

y_pan_lin=interp1(time_lin,pan_lin*180/pi,x_error);
y_tilt_lin=interp1(time_lin,tilt_lin*180/pi,x_error);

y_pan_hold=interp1(time_hold,pan_hold*180/pi,x_error);
y_tilt_hold=interp1(time_hold,tilt_hold*180/pi,x_error);


figure('color','w')
title('Antenna Angles')
%set(gcf,'units','normalized','outerposition',[0 0 1 1])

ax1=subplot(3,1,1);
hold on; grid on; box on;
h4=plot(x_error,abs(y_pan_true-y_pan_est),'-b');
h7=plot(x_error,abs(y_pan_true-y_pan_lin),'color',[0 0.8 0]);
%h9=plot(x_error,abs(y_pan_true-y_pan_hold),'m');
ylim([0 5])
xlabel('Time (s)')
ylabel('pan error [degree]')
legend([h4,h7],'KF estimator','linear estimator')
%legend([h4,h7,h9],'KF estimator','linear estimator','hold angles')

ax2=subplot(3,1,2);
hold on; grid on; box on;
h5=plot(x_error,abs(y_tilt_true-y_tilt_est),'-b');
h8=plot(x_error,abs(y_tilt_true-y_tilt_lin),'color',[0 0.8 0]);
%h10=plot(x_error,abs(y_tilt_true-y_tilt_hold),'m');
ylim([0 5])
xlabel('Time (s)')
ylabel('Tilt error [degree]')
legend([h5,h8],'KF estimator','linear estimator')
%legend([h5,h8,h10],'KF estimator','linear estimator','hold angles')

ax3=subplot(3,1,3);
hold on; grid on; box on;
h6=plot(x_error,y_range,'r');
xlabel('Time (s)')
ylabel('Range [m]')
legend([h6],'Range')

linkaxes([ax1,ax2,ax3],'x')
%}




