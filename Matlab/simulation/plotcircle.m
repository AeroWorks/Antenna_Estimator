function plotcircle(xpoints,ypoints,h0,R,linspec)

s=linspace(0,2*pi,500);
len=length(xpoints);

for i=1:len
    plot3(xpoints(i)+R*cos(s),ypoints(i)+R*sin(s),h0*ones(1,length(xpoints(i)+R*cos(s))),linspec)
end