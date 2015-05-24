function phi_cmd = lateral_guidance(p,pdot,a,b)

ddr=50;     % look ahead distance
Kp=1;       % proportional gain

ab = b-a;
d = a+dot(p-a,ab)/sum(ab.^2)*ab;
db = b-d;
r = d+db/norm(db)*ddr;
L = r-p;

eta = atan2(L(2),L(1))-atan2(pdot(2),pdot(1));
if eta>pi, eta=eta-2*pi; end;
if eta<-pi, eta=eta+2*pi; end;

num = 2*sum(pdot.^2)*sin(eta)*Kp;
den = norm(L)*9.81;
phi_cmd = atan2(num,den);