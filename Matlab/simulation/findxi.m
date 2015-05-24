function [xi,Vgm] = findxi(xig,VT,w)

Vw=norm(w);
xiw = atan2(w(2),w(1));
gam = xig-xiw;
Vgm = Vw*cos(gam)+sqrt((VT^2+Vw^2*sin(gam)^2));
xi = atan2(Vgm*sin(xig)-w(2),Vgm*cos(xig)-w(1));

end