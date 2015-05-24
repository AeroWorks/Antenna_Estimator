%% ATC
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
clear
close all
clc

%% Run Aircraft Simulation - - - - - - - - - - - - - - - - - - - - - - - - 
run simulation_framework

%load sim_data.mat

%% Misc - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
% State vector Xhat:
% Xhat=[x v_x y v_y z v_z]

% x,y,z:        Position
% v_x,v_y,v_z:  Velocity
% omega:        Turn rate

% Noise parameters
q       = 0.005; %+0.15;     % Process noise covariance parameter
                %+0.0;  (still air)
                %+0.25;  default
                %+0.75;  windy
                
r_a       = m_p;        % (Position) measurement noise covariance parameter
r_b       = m_v;        % (Velocity) measurement noise covariance parameter
var_phi   = (2.5*pi/180)^2;

% Initialization
Xhat = p_pdot.signals.values(1,:)';   
P = diag([0.8^2,0.3^2,0.8^2,0.3^2,0.8^2,0.3^2]);
phi_old = phi.signals.values(1);
v_air_old = v_air.signals.values(1,:);

% Loop variables
Ts_kf = 0.1;                    % Kalman filter execution rate
u=2;                            % Index position measurement
v=2;                            % Index phi measurement

kf_time = (0:Ts_kf:stoptime)';
kf_time_old = 0.0;

pos_est = zeros(1/Ts_kf*stoptime+1,3);
vel_est = zeros(1/Ts_kf*stoptime+1,3);
omega_est = zeros(1/Ts_kf*stoptime+1,1);

pos_est(1,:) = [Xhat(1),Xhat(3),Xhat(5)];
vel_est(1,:) = [Xhat(2),Xhat(4),Xhat(6)];
omega_est(1,:) = tan(phi_old)*gravity/sqrt(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2);

% change time of phi and p_pdot measurement by a random number between a
% and b

a = 0;
b = 0.6;

for l=1:length(phi.time)
    phi.time(l) = phi.time(l) + (b-a).*rand(1,1) + a;
    v_air.time(l)= phi.time(l);
end

for l=1:length(p_pdot.time)
    p_pdot.time(l) = p_pdot.time(l) + (b-a).*rand(1,1) + a;
end

%% Kalman Filter
%cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
% kalman filter execution loop 
%cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc

for i=2:(1/Ts_kf*stoptime)+1  
%--------------------------------------------------------------------------   
% run high-level function with embedded kalman filter subfunctions 
%--------------------------------------------------------------------------
[Xhat,P,omega,phi_old,v_air_old,kf_time_old,v,u] = ...
    high_level...
    (Xhat,P,gravity,kf_time(i),kf_time_old,p_pdot.time(u),p_pdot.signals.values(u,:),...
    v_air.signals.values(v,:),v_air_old,phi.time(v),phi.signals.values(v),phi_old,r_a,r_b,var_phi,q,v,u);

%--------------------------------------------------------------------------   
% save data   
%--------------------------------------------------------------------------
pos_est(i-1,:) = [Xhat(1),Xhat(3),Xhat(5)];
vel_est(i-1,:) = [Xhat(2),Xhat(4),Xhat(6)];
omega_est(i-1) = omega;
end


v_wind = zeros(length(v_air.signals.values),1);
for i=1:length(v_air.signals.values)
v_wind(i,1) = abs(sqrt((vel_est(i,1))^2+(vel_est(i,2))^2+(vel_est(i,3))^2)-abs(v_air.signals.values(i)));
end

%% calculate forward prediction
pos_fwc_x =zeros((1/Ts_kf*stoptime),1);
pos_fwc_y =zeros((1/Ts_kf*stoptime),1);
pos_fwc_z =zeros((1/Ts_kf*stoptime),1);
dt=Ts_kf;
u=1;
time =zeros(1/Ts_kf*stoptime,1);

for i=2:(1/Ts_kf*stoptime)+1
   time(i,1)=time(i-1,1)+Ts_kf;
   
     if p_pdot.time(u)<time(i)
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

%% Plots
% kalman filter time vector

% Estimator

figure('color','w')
title('Position Estimate 3D')
set(gcf,'units','normalized','outerposition',[0 0 1 1])
hold on; grid on; box on;axis equal;
 
h3=plot3(wypts00(4,:),wypts00(3,:),h0*ones(1,size(wypts00,2)),'--ko','markersize',10);
plotcircle(wypts00(4,:),wypts00(3,:),h0,Racpt,'--m')
h5=plot3(pNE.signals.values(:,2),pNE.signals.values(:,1),pD.signals.values,'-b');
h6=plot3(pos_est(1:end-1,2),pos_est(1:end-1,1),pos_est(1:end-1,3),'-r');
h7=plot3(pos_fwc_x,pos_fwc_y,pos_fwc_z,'color',[0 0.8 0]);
han=plot3(p_antenna(:,2),p_antenna(:,1),p_antenna(:,3),'g*');
%view([-30,30])

xlabel('East(y) (m)')
ylabel('North(x) (m)')
zlabel('Down(z) (m)')
legend([h3,h5,h6,h7,han],'Track','Real Flight Path','KF Estimator','Forward Calculation','Antenna')

figure('color','w')
title('Turn Rate')
set(gcf,'units','normalized','outerposition',[0 0 1 1])

hold on; grid on; box on;
homega_est=plot(kf_time(1:end-1),omega_est(1:end-1)*180/pi,'-r');
homega_true=plot(omega_true.time,omega_true.signals.values*180/pi,'-b');
xlabel('Time (s)')
ylabel('Turn rate [degree/s]')

figure('color','w')
title('Absolute Velocity')
set(gcf,'units','normalized','outerposition',[0 0 1 1])

hold on; grid on; box on;
hv_est=plot(kf_time(1:end-1),sqrt(vel_est(1:end-1,1).^2+vel_est(1:end-1,2).^2+vel_est(1:end-1,3).^2),'-r');
hv_true=plot(pdotD.time,sqrt(pdot.signals.values(:,1).^2+pdot.signals.values(:,2).^2+pdotD.signals.values.^2),'-b');
xlabel('Time (s)')
ylabel('absolute velocity [m/s]')
%}

%% Error Calculation
%{.
error_North = zeros(length(pNE.signals.values),1);
error_East = zeros(length(pNE.signals.values),1);
error_Down = zeros(length(pNE.signals.values),1);
sum_error = zeros(length(pNE.signals.values),1);

for i=1:length(pNE.signals.values)-1
error_North(i,1)=abs(pNE.signals.values(i,2)-pos_est(i,2));

error_East(i,1)=abs(pNE.signals.values(i,1)-pos_est(i,1));

error_Down(i,1)=abs(pD.signals.values(i)-pos_est(i,3));

sum_error(i)=(error_North(i,1) + error_East(i,1) + error_Down(i,1));
end

error=sum(sum_error)/(stoptime);

%disp(error);
%}

%{
figure('color','w')
title('Absolute Error NED ')
set(gcf,'units','normalized','outerposition',[0 0 1 1])

ax4=subplot(4,1,1);
hold on; grid on; box on;
h14=plot(pNE.time(1:end-1),error_North(1:end-1,1),'-r');
axis([0,pNE.time(end),0,5])
xlabel('Time (s)')
ylabel('Absolute Error (North)')


ax5=subplot(4,1,2);
hold on; grid on; box on;
h15=plot(pNE.time(1:end-1),error_East(1:end-1,1),'-r');
xlabel('Time (s)')
ylabel('Absolute Error (East)')

ax6=subplot(4,1,3);
hold on; grid on; box on;
h16=plot(pNE.time(1:end-1),error_Down(1:end-1,1),'-r');
xlabel('Time (s)')
ylabel('Absolute Error (Down)')

linkaxes([ax4,ax5,ax6],'xy')

%}

figure('color','w')
title('Absolute Error NED ')
set(gcf,'units','normalized','outerposition',[0 0 1 1])

ax7=subplot(3,1,1);
hold on; grid on; box on;
h17=plot(pNE.time(1:end-1),sqrt(error_North(1:end-1,1).^2+error_East(1:end-1,1).^2+error_Down(1:end-1,1).^2),'-b');
xlabel('Time (s)')
ylabel('Abs. Error')

ax8=subplot(3,1,2);
hold on; grid on; box on;
h18=plot(v_wind,'-b');
xlabel('Time (s)')
ylabel('Abs. Windspeed in m/s')

ax11=subplot(3,1,3);
grid on; box on;
h18=plot(phi.signals.values,'-b');
xlabel('Time (s)')
ylabel('roll angle')

linkaxes([ax7,ax8,ax11],'x')


%% Antenna
%{
pan=zeros(length(pos_est),1);
tilt=zeros(length(pos_est),1);
range=zeros(length(pos_est),1);

for i=1:length(pos_est)
[pan(i,1),tilt(i,1),range(i,1)] = antenna_angle(pos_est(i,:),p_antenna);
end

figure('color','w')
title('Antenna Angles')
set(gcf,'units','normalized','outerposition',[0 0 1 1])

ax7=subplot(3,1,1);
hold on; grid on; box on;
h15=plot(kf_time(1:end-1),pan(1:end-1)*180/pi,'-r');
xlabel('Time (s)')
ylabel('Pan Angle [degree]')

ax8=subplot(3,1,2);
hold on; grid on; box on;
h16=plot(kf_time(1:end-1),tilt(1:end-1)*180/pi,'-r');
xlabel('Time (s)')
ylabel('Tilt Angle [degree]')

ax9=subplot(3,1,3);
hold on; grid on; box on;
h17=plot(kf_time(1:end-1),range(1:end-1),'-r');
xlabel('Time (s)')
ylabel('Range [m]')

linkaxes([ax7,ax8,ax9],'x')
%}

%% Realtime
%{
hline_old=line(0,0,0);
P1_antenna = zeros(length(pNE.time),3);
P2_antenna = zeros(length(pNE.time),3);
P1_est = zeros(length(pNE.time),3);
P2_est = zeros(length(pNE.time),3);
P1_true = zeros(length(pNE.time),3);
P2_true = zeros(length(pNE.time),3);

for i = 1:length(pNE.time)-1
P1_antenna(i,:) = [p_antenna(2),p_antenna(1),p_antenna(3)];

P2_antenna(i,:) = [range(i,1)*cos(tilt(i,1))*cos(pan(i,1))+p_antenna(2),...
                range(i,1)*cos(tilt(i,1))*sin(pan(i,1))+p_antenna(1),...
                range(i,1)*sin(tilt(i,1))+p_antenna(3)];
            
P2_est(i,:) = [pos_est(i,2),pos_est(i,1),pos_est(i,3)];
end

P1_est(1,:)=[pos_est(1,2),pos_est(1,1),pos_est(1,3)];
for i = 1:length(pos_est)-2
P1_est(i+1,:) = [pos_est(i,2),pos_est(i,1),pos_est(i,3)];
P2_est(i,:) = [pos_est(i,2),pos_est(i,1),pos_est(i,3)];
end

%P1_true(1,:)=[pNE.signals.values(1,2),pNE.signals.values(1,1),pD.signals.values(1)];
%for i = 1:length(pNE.signals.values)
%P1_true(i+1,:) = [pNE.signals.values(i,2),pNE.signals.values(i,1),pD.signals.values(i)];
%P2_true(i,:) = [pNE.signals.values(i,2),pNE.signals.values(i,1),pD.signals.values(i)];
%end

%% Show realtime plot

figure('color','w')
title('Realtime Data')
hold on; grid on; box on;axis equal;

h3=plot3(wypts00(4,:),wypts00(3,:),h0*ones(1,size(wypts00,2)),'--ko','markersize',10);
plotcircle(wypts00(4,:),wypts00(3,:),h0,Racpt,'--m')
h5=plot3(pNE.signals.values(:,2),pNE.signals.values(:,1),pD.signals.values,'-b');
han=plot3(p_antenna(:,2),p_antenna(:,1),p_antenna(:,3),'g*');
view([-30,30]) 

set(gcf,'units','normalized','outerposition',[0 0 1 1])

xlabel('East(y) (m)')
ylabel('North(x) (m)')
zlabel('Up(z) (m)')

for i = 1:length(pNE.time)

pts_antenna = [P1_antenna(i,:); P2_antenna(i,:)];
pts_est = [P1_est(i,:); P2_est(i,:)];
%pts_true = [P1_true(i,:); P2_true(i,:)];

hline_est = line(pts_est(:,1), pts_est(:,2), pts_est(:,3),'Color','r');
%hline_true = line(pts_true(:,1), pts_true(:,2), pts_true(:,3),'Color','b');
hline = line(pts_antenna(:,1), pts_antenna(:,2), pts_antenna(:,3),'Color','g');
delete(hline_old);
hline_old = hline;


legend([h3,h5,hline_est,han],'Track','Real Flight Path','Est Flight Path','Antenna');
%{    
['Antenna pan: '...
    ,num2str(round(pan(i,1)*180/pi)),' tilt: '...
    ,num2str(round(tilt(i,1)*180/pi)),' range: '...
    ,num2str(round(range(i,1)))])
%}
drawnow
end

%}
%% Display Command Window
disp('-------------------------------------------------------------------')
disp(' ')
disp(['Simulation time: ',num2str(stoptime)])
disp(' ')
disp('Measurement noise - - - - - - - - - - - - - - - - - - - - - - - - - ')
disp(['Variance position (m_p): ',num2str(m_p)])
disp(['Variance velocity (m_v): ',num2str(m_v)])
disp(['Variance roll-angle (m_phi): ',num2str(m_phi)])
disp(' ')
disp('Additional process noise - - - - - - - - - - - - - - - - - - - - - ')
disp(['Covariance parameter (q): ',num2str(q)])
disp(' ')
disp('Antenna - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ')
disp(['Position [North,East,Up]: ',num2str(p_antenna)])
disp(' ')
disp('KF - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ')
disp(['Execution rate [s]: ',num2str(Ts_kf)])
disp(' ')
disp('Error - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ')
disp(['Absolute Position Error: ',num2str(error)])
disp(' ')
disp('Wind - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - ')
disp(['Windspeed [m/s]: ',num2str(Vw)])
disp(' ')
disp('-------------------------------------------------------------------')








