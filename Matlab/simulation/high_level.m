function [Xhat_out,P,omega_out,phi_old_out,v_air_old_out,kf_time_old_out,v_out,u_out] = high_level(Xhat,P,gravity,kf_time,kf_time_old,p_pdot_time,p_pdot,v_air,v_air_old,phi_time,phi,phi_old,r_a,r_b,var_phi,q,v,u)
%--------------------------------------------------------------------------   
% check if position measurement is available    
%--------------------------------------------------------------------------
if p_pdot_time>=kf_time_old && p_pdot_time<kf_time
    
%--------------------------------------------------------------------------   
% check if phi measurement is available    
%--------------------------------------------------------------------------
    if phi_time>=kf_time_old && phi_time<kf_time %booth are available
        
        % (re-)order timestamps
        if p_pdot_time>phi_time
            timestamp_1=phi_time;
            timestamp_2=p_pdot_time;
            
                dt1 = timestamp_1 - kf_time_old; %calculate dt
   
            % predict with phi_old
            [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);  

                dt2 = timestamp_2 - timestamp_1; %calculate dt
        
            % predict with phi_new
            [Xhat2,~,P2]  = KF_predict(Xhat1,v_air,phi,P1,dt2,q,var_phi,gravity);  

            % udpate position
            [Xhat3,P3]  = KF_update_pos(Xhat2,p_pdot',P2,r_a,r_b);
   
                dt3 = kf_time - p_pdot_time; %calculate dt
            
            % predict with new position and new phi
            [Xhat_out,omega_out,P]  = KF_predict(Xhat3,v_air,phi,P3,dt3,q,var_phi,gravity);  
       
        elseif p_pdot_time<phi_time
            timestamp_1=p_pdot_time;
            timestamp_2=phi_time; 
        
                dt1 = timestamp_1 - kf_time_old; %calculate dt
        
            % predict
            [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);

            % udpate position
            [Xhat2,P2]  = KF_update_pos(Xhat1,p_pdot',P1,r_a,r_b);
   
                dt2 = timestamp_2 - timestamp_1; %calculate dt
        
            % predict with new position and old phi
            [Xhat3,~,P3]  = KF_predict(Xhat2,v_air_old,phi_old,P2,dt2,q,var_phi,gravity);  

   
                dt3 = kf_time - p_pdot_time; %calculate dt
            
            % predict with new position and new phi
            [Xhat_out,omega_out,P]  = KF_predict(Xhat3,v_air,phi,P3,dt3,q,var_phi,gravity);  
        
        elseif p_pdot_time==phi_time
            timestamp=p_pdot_time;
        
                dt1 = timestamp - kf_time_old; %calculate dt
            
            % predict
            [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);

            % udpate position
            [Xhat2,P2]  = KF_update_pos(Xhat1,p_pdot',P1,r_a,r_b);
   
                dt2 = kf_time - timestamp; %calculate dt
      
            % predict
            [Xhat_out,omega_out,P]  = KF_predict(Xhat2,v_air,phi,P2,dt2,q,var_phi,gravity);
         
        end
    
    u_out = u+1;
    v_out = v+1;
    phi_old_out = phi;
    v_air_old_out = v_air;
    
    else  % only position is available     
       
        dt1 = p_pdot_time - kf_time_old; %calculate dt
    
    % predict
    [Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    % udpate position
    [Xhat2,P2]  = KF_update_pos(Xhat1,p_pdot',P1,r_a,r_b);
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

        dt2 = kf_time - p_pdot_time; %calculate dt
      
    % predict
    [Xhat_out,omega_out,P]  = KF_predict(Xhat2,v_air_old,phi_old,P2,dt2,q,var_phi,gravity);

    u_out=u+1;
    v_out=v;
    phi_old_out = phi_old;
    v_air_old_out = v_air_old;
    
    end
    
%--------------------------------------------------------------------------   
% check if phi measurement is available but no position measurement    
%--------------------------------------------------------------------------
elseif phi_time>=kf_time_old && phi_time<kf_time

    dt1 = phi_time - kf_time_old; %calculate dt
    
% predict with old phi
[Xhat1,~,P1]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);  

    dt2 = kf_time - phi_time; %calculate dt
    
% predict with new phi
[Xhat_out,omega_out,P]  = KF_predict(Xhat1,v_air,phi,P1,dt2,q,var_phi,gravity);  

v_out = v+1;        
u_out = u;
phi_old_out = phi;
v_air_old_out = v_air;
    
%--------------------------------------------------------------------------   
% no measurement is available   
%--------------------------------------------------------------------------
else
    
    dt1 = kf_time - kf_time_old; %calculate dt

[Xhat_out,omega_out,P]  = KF_predict(Xhat,v_air_old,phi_old,P,dt1,q,var_phi,gravity);  

v_out = v;
u_out = u;
phi_old_out = phi_old;
v_air_old_out = v_air_old;
    
end

kf_time_old_out = kf_time;
end

%% Subfunctions

function [Xhat_new,omega_new,P_new]  = KF_predict(Xhat,v_air,phi,P,dt,q,var_phi,gravity)

omega = tan(phi)*gravity/sqrt(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2);

%{.
v_wind = abs(sqrt((Xhat(2))^2+(Xhat(4))^2+(Xhat(6))^2)-abs(v_air));

% only total airspeed! not vector!!
% scale with omega!

a=0.03; %0.07; %linear fit (LS)
b=0.08; %0.008; 

q_add = (a*sqrt(abs(v_wind))+b*abs(phi))^(1/9);
q_z=0.1+0.1*q_add;
q=q+q_add;

% TODO: just change the q which indecates the covariance in the wind direction
%}

%% 1. Compute F  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% (CT model in horizontal plane and CV model in vertical direction):
%{.
F_c = [0,1,0,0,0,0;...
    0,gravity*Xhat(2)*Xhat(4)*tan(phi)/(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)^(3/2),0,gravity*Xhat(4)^2*tan(phi)/(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)^(3/2)-gravity*tan(phi)/sqrt(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2),0,gravity*Xhat(4)*Xhat(6)*tan(phi)/(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)^(3/2);...
    0,0,0,1,0,0;...
    0,gravity*tan(phi)/sqrt(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)-gravity*Xhat(2)^2*tan(phi)/(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)^(3/2),0,-gravity*Xhat(2)*Xhat(4)*tan(phi)/(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)^(3/2),0,-gravity*Xhat(2)*Xhat(6)*tan(phi)/(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2)^(3/2);...
    0,0,0,0,0,1;...
    0,0,0,0,0,0];

% discrete approximation
F_d = eye(length(F_c))+F_c*dt+F_c^2*dt^2/2;
%}
%{
F_omega_large = [1 sin(omega*dt)/omega 0 -(1-cos(omega*dt))/omega 0 0;...
    0 cos(omega*dt) 0 -sin(omega*dt) 0 0;...
    0 (1-cos(omega*dt))/omega 1 sin(omega*dt)/omega 0 0;...
    0 sin(omega*dt) 0 cos(omega*dt) 0 0;...
    0 0 0 0 1 dt;...
    0 0 0 0 0 1 ];

F_omega_small = [1 dt 0 -omega*dt^2/2 0 0;...
    0 1-(omega*dt)^2/2 0 -omega*dt 0 0;...
    0 omega*dt^2/2 1 dt 0 0;...
    0 omega*dt 0 1-(omega*dt)^2/2 0 0;...
    0 0 0 0 1 dt;...
    0 0 0 0 0 1 ];

% Account for omega towards zero
if omega>0.001
    F_d=F_omega_large;
else
    F_d=F_omega_small;
end
%}

%% 2. Measurement covariance matrix Q
%{.
B = [0;...
    -4*gravity*Xhat(4)*cos(phi)^2/((cos(2*phi)+1)^2*sqrt(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2));...
    0;...
    4*gravity*Xhat(2)*cos(phi)^2/((cos(2*phi)+1)^2*sqrt(Xhat(2)^2+Xhat(4)^2+Xhat(6)^2));...
    0;...
    0];

Q_c = B*var_phi*B';

R = [-F_c,Q_c;zeros(length(F_c)),F_c'];

% discrete approximation
G = eye(length(R))+R*dt+1/2*R^2*dt^2;

F_dTQ_d = G(1:6,7:12);
F_dT = G(7:12,7:12);

Q_d = F_dT'*F_dTQ_d;

Q_additional = [dt^4/4*q dt^3/2*q 0 0 0 0;...
        dt^3/2*q dt^2*q^2 0 0 0 0;...
        0 0 dt^4/4*q dt^3/2*q 0 0;...
        0 0 dt^3/2*q dt^2*q^2 0 0;...
        0 0 0 0 dt^4*q_z dt^4/2*q_z^2;...
        0 0 0 0 dt^4/2*q_z^2 dt^4*q_z];

Q = Q_additional+Q_d;
    %}
%% 3.1 Predicted (a priori) state estimate - - - - - - - - - - - - - - - - -
%Xhat_new = F_d*Xhat;     
Xhat_new = Xhat+[Xhat(2)*dt;-omega*Xhat(4)*dt;Xhat(4)*dt;omega*Xhat(2)*dt;Xhat(6)*dt;0];
%% 3.2 Predicted (a priori) estimate covariance - - - - - - - - - - - - - -  
P_new = F_d*P*F_d' + Q;

%% 4. Calculate new turn rate omega_new - - - - - - - - - - - - - - - - - - - - - - 
omega_new = omega; %tan(phi)*gravity/sqrt(Xhat_new(2)^2+Xhat_new(4)^2+Xhat_new(6)^2);

end

function [Xhat_new,P_new]  = KF_update_pos(Xhat,Z,P,r_a,r_b)
%% 1. Calculate R and H - - - - - - - - - - - - - - - - - - - - - - - - - -
R = [(0.7*r_a)^2 0 0 0 0 0;...
        0 (0.4*r_b)^2 0 0 0 0;...
        0 0 (0.7*r_a)^2 0 0 0;...
        0 0 0 (0.4*r_b)^2 0 0;...
        0 0 0 0 (0.1*r_a)^2 0;...
        0 0 0 0 0 (0.2*r_b)^2];
    
H = [1 0 0 0 0 0;...
        0 1 0 0 0 0;...
        0 0 1 0 0 0;...
        0 0 0 1 0 0;...
        0 0 0 0 1 0;...
        0 0 0 0 0 1];
    
%% 2.1 Innovation or measurement residual - - - - - - - - - - - - - - - - -
Y = Z-H*Xhat;

%% 2.2 Innovation (or residual) covariance - - - - - - - - - - - - - - - - 
S = H*P*H'+R;

%% 2.3 Optimal Kalman gain - - - - - - - - - - - - - - - - - - - - - - - -
K = P*H'/S;

%% 2.4 Updated (a posteriori) state estimate - - - - - - - - - - - - - - - 
Xhat_new = Xhat + K*Y;

%% 2.5 Updated (a posteriori) estimate covariance - - - - - - - - - - - - -
P_new = (eye(length(K*H))-K*H)*P;

end









