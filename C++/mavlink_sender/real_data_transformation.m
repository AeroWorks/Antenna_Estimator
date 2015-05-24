 %% DATA IMPORT

%% tabula rasa
clear all
close all
clc

%% load data
importdata('flight1.csv')
%importdata('flight2.csv')

flight_data=ans.data;

%% Import
for i=1:size(flight_data,2) %go through columns
    for j=1:size(flight_data,1) %go through row
        if isnan(flight_data(j,i))
            flight_data(j,i)=0;
        end
    end
end

timestamp=zeros(length(flight_data),1);

for i=1:length(flight_data)
timestamp(i,1) = flight_data(i,1)-(flight_data(1,1)-0.001);
end

% GLOBAL_POSITION_INT
GLOBAL_POSITION_INT_time_boot_ms = flight_data(:,2);

GLOBAL_POSITION_INT_lat = flight_data(:,3);
GLOBAL_POSITION_INT_lon = flight_data(:,4);
GLOBAL_POSITION_INT_alt = flight_data(:,5);

GLOBAL_POSITION_INT_relative_alt = flight_data(:,6);

GLOBAL_POSITION_INT_vx = flight_data(:,7);
GLOBAL_POSITION_INT_vy = flight_data(:,8);
GLOBAL_POSITION_INT_vz = flight_data(:,9);

GLOBAL_POSITION_INT_hdg = flight_data(:,10);

% ATTITUDE
ATTITUDE_time_boot_ms = flight_data(:,11);

ATTITUDE_roll = flight_data(:,12);
ATTITUDE_pitch = flight_data(:,13);
ATTITUDE_yaw = flight_data(:,14);

ATTITUDE_rollspeed = flight_data(:,15);
ATTITUDE_pitchspeed = flight_data(:,16);
ATTITUDE_yawspeed = flight_data(:,17);

% VFR_HUD
VFR_HUD_airspeed = flight_data(:,18);
VFR_HUD_groundspeed = flight_data(:,19);
VFR_HUD_heading = flight_data(:,20);
VFR_HUD_throttle = flight_data(:,21);
VFR_HUD_alt = flight_data(:,22);
VFR_HUD_climb = flight_data(:,23);

%% Rearange Data

% pair arrays and time vector to apropriate structs

GPS_pos_vel.time=timestamp;
GPS_pos_vel.data=[GLOBAL_POSITION_INT_lat GLOBAL_POSITION_INT_vx GLOBAL_POSITION_INT_lon GLOBAL_POSITION_INT_vy GLOBAL_POSITION_INT_alt GLOBAL_POSITION_INT_vz];

ATTITUDE_roll_angle.time=timestamp;
ATTITUDE_roll_angle.data=ATTITUDE_roll;

VFRHUD_airspeed.time=timestamp;
VFRHUD_airspeed.data=VFR_HUD_airspeed;

% delete rows if zero
rows_to_remove_GPS=any(GPS_pos_vel.data==0,2);
rows_to_remove_ATTITUDE=any(ATTITUDE_roll_angle.data==0,2);
rows_to_remove_VFRHUD=any(VFRHUD_airspeed.data==0,2);

GPS_pos_vel.time(rows_to_remove_GPS,:)=[];
GPS_pos_vel.data(rows_to_remove_GPS,:)=[];

ATTITUDE_roll_angle.time(rows_to_remove_ATTITUDE,:)=[];
ATTITUDE_roll_angle.data(rows_to_remove_ATTITUDE,:)=[];

VFRHUD_airspeed.time(rows_to_remove_ATTITUDE,:)=[];
VFRHUD_airspeed.data(rows_to_remove_ATTITUDE,:)=[];

Antenna.lat = 47.60284;
Antenna.lon = 8.53401;
Antenna.elev = 416;

[xNorth,yEast,zDown] = geodetic2ned(GPS_pos_vel.data(:,1)*10^(-7),GPS_pos_vel.data(:,3)*10^(-7),GPS_pos_vel.data(:,5)*10^(-3),Antenna.lat,Antenna.lon,Antenna.elev,wgs84Ellipsoid);

figure('color','w')
title('Flight Path')
set(gcf,'units','normalized','outerposition',[0 0 1 1])
hold on; grid on; box on;axis equal;
 
h1=plot3(xNorth,yEast,-zDown,'-r');

xlabel('East(y) (m)')
ylabel('North(x) (m)')
zlabel('Up(z) (m)')
legend([h1],'Real Flight Path')

%{.
%% Export

%[export_wgs84.lat, export_wgs84.lon, export_wgs84.h] = ...
%    ned2geodetic(p_pdot.signals.values(:,1),p_pdot.signals.values(:,3),-p_pdot.signals.values(:,5),Antenna.lat,Antenna.lon,Antenna.elev,wgs84Ellipsoid);

data = [GPS_pos_vel.time, ...
        ones(length(GPS_pos_vel.data(:,1)),1), ...
        zeros(length(GPS_pos_vel.data(:,1)),1), ...
        GPS_pos_vel.data(:,1)*10^(-7), ...
        GPS_pos_vel.data(:,3)*10^(-7), ...
        GPS_pos_vel.data(:,5)*10^(-3), ...
        GPS_pos_vel.data(:,2)*10^(-2),...
        GPS_pos_vel.data(:,4)*10^(-2),...
        GPS_pos_vel.data(:,6)*10^(-2),...
        zeros(length(GPS_pos_vel.data(:,1)),7)];

data = [data; 
       ATTITUDE_roll_angle.time, ...
       zeros(length(ATTITUDE_roll_angle.time),1), ...
       ones(length(ATTITUDE_roll_angle.time),1), ...
       zeros(length(ATTITUDE_roll_angle.time),6), ...
       ATTITUDE_roll_angle.data, ...
       zeros(length(ATTITUDE_roll_angle.time),5), ...
       VFRHUD_airspeed.data];

[~,isort] = sort(data(:,1));

data = data(isort,:);
   
fid = fopen('trajectory1.csv','w');
%fid = fopen('trajectory2.csv','w');
fprintf(fid,'%6.3f;%d;%d;%9.6f;%9.6f;%5.1f;%5.3f;%5.3f;%5.3f;%6.4f;%6.4f;%6.4f;%6.4f;%6.4f;%6.4f;%5.2f;\n',data');

fclose(fid);
%}