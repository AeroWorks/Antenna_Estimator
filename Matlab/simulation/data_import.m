%% DATA IMPORT

%% tabula rasa
clear all
close all
clc

%% load data
importdata('datarecording_1432213897.csv')

load test6_120s_q0.025_wind4.mat

%% save to appropriate variables

timestamp = ans.data(:,1);

panPosition = ans.data(:,2);
tiltPosition = ans.data(:,3);

panCtrlType = ans.data(:,4);
panSetValue = ans.data(:,5);

tiltCtrlType = ans.data(:,6);
tiltSetValue = ans.data(:,7);

local_lat = ans.data(:,8);
local_lon = ans.data(:,9);
local_elev = ans.data(:,10);

object_lat = ans.data(:,11);
object_lon = ans.data(:,12);
object_elev = ans.data(:,13);

object_x = ans.data(:,14);
object_y = ans.data(:,15);
object_z = ans.data(:,16);

estObject_x = ans.data(:,17);
estObject_y = ans.data(:,18);
estObject_z = ans.data(:,19);

estObject_lat = ans.data(:,20);
estObject_lon = ans.data(:,21);
estObject_elev = ans.data(:,22);

time_vector=timestamp-timestamp(1);

%% adjust imported data to match simulation
v=1;

for i=2:length(panSetValue)
 if panSetValue(i)==panSetValue(i-1)
     v=v+1;
 end  
end

estObject_x=circshift(estObject_x,-v);
estObject_x=estObject_x(1:end-v);

estObject_y=circshift(estObject_y,-v);
estObject_y=estObject_y(1:end-v);

estObject_z=circshift(estObject_z,-v);
estObject_z=estObject_z(1:end-v);

panSetValue=circshift(panSetValue,-v);
panSetValue=panSetValue(1:end-v);

tiltSetValue=circshift(tiltSetValue,-v);
tiltSetValue=tiltSetValue(1:end-v);

time_vector=time_vector-time_vector(v);
time_vector=circshift(time_vector,-v+1);
time_vector=time_vector(1:end-v);


%% calculate forward prediction
pos_fwc_x =zeros((1/Ts_kf*stoptime)-v,1);
pos_fwc_y =zeros((1/Ts_kf*stoptime)-v,1);
pos_fwc_z =zeros((1/Ts_kf*stoptime)-v,1);
dt=Ts_kf;
u=1;
kf_time =zeros(1/Ts_kf*stoptime-v,1);

for i=2:(1/Ts_kf*stoptime)+1-v
   kf_time(i,1)=kf_time(i-1,1)+Ts_kf;
   
     if p_pdot.time(u)<kf_time(i)
        pos_fwc_x(i,1)=p_pdot.signals.values(u,3);
        pos_fwc_y(i,1)=p_pdot.signals.values(u,1);
        pos_fwc_z(i,1)=p_pdot.signals.values(u,5);

        u=u+1;
     else 
         pos_fwc_x(i,1)=pos_fwc_x(i-1,1)+dt*p_pdot.signals.values(u,4);

         pos_fwc_y(i,1)=pos_fwc_y(i-1,1)+dt*p_pdot.signals.values(u,2);

         pos_fwc_z(i,1)=pos_fwc_z(i-1,1)+dt*p_pdot.signals.values(u,6);

         u=u;
     end

end

%% calculate angle hold
pos_hold_x =zeros((1/Ts_kf*stoptime)-v,1);
pos_hold_y =zeros((1/Ts_kf*stoptime)-v,1);
pos_hold_z =zeros((1/Ts_kf*stoptime)-v,1);
u=1;
kf_time =zeros(1/Ts_kf*stoptime-v,1);

for i=2:(1/Ts_kf*stoptime)+1-v
   kf_time(i,1)=kf_time(i-1,1)+Ts_kf;
   
     if p_pdot.time(u)<kf_time(i)
        pos_hold_x(i,1)=p_pdot.signals.values(u,3);
        pos_hold_y(i,1)=p_pdot.signals.values(u,1);
        pos_hold_z(i,1)=p_pdot.signals.values(u,5);

        u=u+1;
     else 
         pos_hold_x(i,1)=pos_hold_x(i-1,1);

         pos_hold_y(i,1)=pos_hold_y(i-1,1);

         pos_hold_z(i,1)=pos_hold_z(i-1,1);

         u=u;
     end

end


%% calculate desired antenna angles

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


%% plots


% flight path

figure('color','w')
title('Flight Path')
%set(gcf,'units','normalized','outerposition',[0 0 1 1])
hold on; grid on; box on;axis equal;
 
h1=plot3(pos_fwc_x,pos_fwc_y,pos_fwc_z,'color',[0 0.8 0]);
h2=plot3(estObject_x,estObject_y,estObject_z,'-r');
h3=plot3(pNE.signals.values(:,2),pNE.signals.values(:,1),pD.signals.values,'-b');
%h4=plot3(pos_hold_x,pos_hold_y,pos_hold_z,'m');


xlabel('East(y) (m)')
ylabel('North(x) (m)')
zlabel('Up(z) (m)')
legend([h1,h2,h3],'Linear Forward Prediction','Antenna Estimation','Real Flight Path')

% antenna angles

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




