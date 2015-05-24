%% Simulation: Framework & Parameter

load sim_buses.mat

%% Simulation - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
stoptime    = 120;	% sim stop time [s]
Tctrl       = 0.1;  % controller time step [s]
Ts          = 0.1;  % simout time-step [s]

% For Atmo model
b_uas       = 3.1;	% characteristic length [m]

%% Environment - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Vw = 1;             % wind speed [m/s]
xiw = 270*pi/180;   % wind direction [rad] (from north)
                    % 270: positive east direction
                    % 180: positive north direction
                    % 90: negative east direction
                    % 0: negative north direction
w = [Vw*cos(xiw);Vw*sin(xiw)];
gravity =   9.81;

% Trajectory - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
Racpt = 30; % waypoint acceptance radius [m]
wypts00=...
    [[1;0;-40;40;Racpt;0],[1;0;20;100;Racpt;0],[1;0;50;200;Racpt;0],...
    [1;0;-50;200;Racpt;0],[1;0;-80;0;Racpt;0]]; % curve

% [[1;0;-50;40;Racpt;0],[1;0;-50;150;Racpt;0],[1;0;-50;300;Racpt;0],...
% [1;0;-50;350;Racpt;0],[1;0;-50;400;Racpt;0]] % line

sizewypts00 = size(wypts00,2);
wypts0=struct([]);
for i=1:sizewypts00
    wypts0(1,i).active = wypts00(1,i);
    wypts0(1,i).wtype = wypts00(2,i);
    wypts0(1,i).p = wypts00(3:4,i);
    wypts0(1,i).param1 = wypts00(5,i);
    wypts0(1,i).param2 = wypts00(6,i);
end

%% UAS Parameters - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
VTd     = 10;           % desired airspeed [m/s]
phi_max = 30*pi/180;    % maximum roll angle [rad]

% UAS Initial Conditions
h0      = 1.5;%150;                          % height [m]
p0      = [2;2];                      % position [m]
xig0 	= 90*pi/180;                     % ground heading [rad]
xi0     = double(findxi(xig0,VTd,w));	% body heading [rad]
phi0    = 0;                            % roll angle [rad]

%% Down Sampling - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Ts_p_pdot = 2;    % update rate: every Ts*20 seconds [1/(Ts*20) Hz]
Ts_v_air =1;
Ts_phi = 1;       % Ts*2-Ts*11 result in error

%% Noise - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
% measurement noise variance
m_p     =(0.8)^2;
m_v     =(0.5)^2;
m_v_air =(0.1)^2;
m_phi   =(0.2*pi/180)^2; %0.25*pi/180;

% random measurement noise seeds
seed_p_pdot = [10;2;0.01;2311;132;453287];
             
seed_v_air = 33;

seed_phi    = 20143;

%% Antenna - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
p_antenna   =[0,0,0];     % position of antenna [north,east,down]

            % pan angle in degree
                            % 0 if pointing in east (y) direction
                            % grows counter clockwise
            % tilt angle in degree
                            % 0 in east (y) north (x) plane
                            % grows with height

%% Simulation
%cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
sim('aircraft_model.slx',stoptime);
%cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc





