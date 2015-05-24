function [pan_out,tilt,range] = antenna_angle(p_est,p_antenna)

persistent pan_old p

% position of antenna
x_ant=p_antenna(1);
y_ant=p_antenna(2);
z_ant=p_antenna(3);

% position with resp. to antenna
x_pos=p_est(1)-x_ant;
y_pos=p_est(2)-y_ant;
z_pos=p_est(3)-z_ant;

range = sqrt((x_pos)^2+(y_pos)^2+(z_pos)^2);
tilt = asin(z_pos/range);
pan = mod(atan(x_pos/y_pos),pi);

%disp(pan)

if isempty(pan_old)
pan_old=pan;
p=0;
end

if (pan-pan_old)<(-(90/180*pi))
    p=pi;     
end
if (pan-pan_old)>((90/180*pi))
    p=0;     
end

pan_out=pan+p;
pan_old=pan;

end