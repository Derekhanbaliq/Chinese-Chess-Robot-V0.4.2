clear;
clc;
%based on LEFT arm system
%TBD variables:l1,l2,dx,dy,ang1,ang2,xpc,ypx,r,desiredspace.
l1 = 207; %length of 1st arm
l2 = 297; %length of 2rd arm
dx = 113;
dy = 77.5;
offsetd = [dx,dy]; 
%distance offset between the end of 1st arm and the original of the paper
%LEFT BOTTOM of the paper
%Suggestion:Please place the paper to make the left bottom node 
ang1 = 0;
ang2 = pi/2+asin(30/l2);
offseta = [ang1,ang2];
%angular offset of two arms:offseta(1) between the 1st arm and x axis.
%offseta(2) between 1&2 arms.
r = 28.567;  %Radius %%%%%%%%%%%%%%%%%%%%%%
xpc = 150; %%%%%%%%%%%%%%%%%%%%%%
ypc = 200+r; %%%%%%%%%%%%%%%%%%%%%%
Pc = [xpc,ypc]; %Center node
length = 2*pi*r;
desiredspace = 3; %%%%%%%%%%%%%%%%%%%%%%
step = floor(length/desiredspace);
%starts from the top node.
Coordinateprime(1,1) = xpc;
Coordinateprime(1,2) = ypc+r;
alphad = 2*pi/step;
for i=2:(step+1)
    alpha(i-1) = (i-1)*alphad;
    chord(i-1) = 2*r*sin(alpha(i-1)/2);
    alpha_chord_yaxis(i-1) = (pi-alpha(i-1))/2;
    Coordinateprime(i,1) = Coordinateprime(1,1)+chord(i-1).*sin(alpha_chord_yaxis(i-1));
    Coordinateprime(i,2) = Coordinateprime(1,2)-chord(i-1).*cos(alpha_chord_yaxis(i-1));
    if alpha(i-1) > 2*pi*(90-33.371)/360
        break;
    end
end
sizec=size(Coordinateprime);
S=sizec(1);
for i=1:S
    Coordinate(i,1)=Coordinateprime(S-i+1,1);
    Coordinate(i,2)=Coordinateprime(S-i+1,2);
end
for i=2:(4*S+1)
    alpha(i-1) = (i-1)*alphad;
    chord(i-1) = 2*r*sin(alpha(i-1)/2);
    alpha_chord_yaxis(i-1) = (pi-alpha(i-1))/2;
    Coordinate(i+S-1,1) = Coordinate(S,1)-chord(i-1).*sin(alpha_chord_yaxis(i-1));
    Coordinate(i+S-1,2) = Coordinate(S,2)-chord(i-1).*cos(alpha_chord_yaxis(i-1));
    if alpha(i-1) > pi
        break;
    end
end
sizec=size(Coordinate);
S=sizec(1);
Coordinate(S+1,1)=150;
Coordinate(S+1,2)=200;

%line
xp1 = 150;
yp1 = 200;
xp2 = 150+54.375;
yp2 = 200;
P1 = [xp1,yp1]; %Start node
P2 = [xp2,yp2]; %End node
length = sqrt((xp2-xp1)^2+(yp2-yp1)^2);
desiredspace = 3;
step = floor(length/desiredspace);
for i=1:(step+1)
    Coordinate(i+S+1,1) = xp1+(i-1)*(xp2-xp1)/step;
    Coordinate(i+S+1,2) = yp1+(i-1)*(yp2-yp1)/step;
end


sizec = size(Coordinate);
S=sizec(1);
xpc2 = 204.375;
ypc2 = 144.256;
r2 = 55.743;  %Radius
alphad = 2*pi/(step*3);
Coordinate(S+1,1) = xpc2;
Coordinate(S+1,2) = ypc2+r2;
for i=2:5*(step+1)
    alpha(i-1) = (i-1)*alphad;
    chord(i-1) = 2*r2*sin(alpha(i-1)/2);
    alpha_chord_yaxis(i-1) = (pi-alpha(i-1))/2;
    Coordinate(S+i,1) = Coordinate(S+1,1)+chord(i-1).*sin(alpha_chord_yaxis(i-1));
    Coordinate(S+i,2) = Coordinate(S+1,2)-chord(i-1).*cos(alpha_chord_yaxis(i-1));
    if alpha(i-1) > 1.5*pi-0.5824
        break;
    end
end

sizec = size(Coordinate);
S=sizec(1);

xcenter=(Coordinate(1,1)+Coordinate(S,1))/2;
ycenter=(Coordinate(1,2)+Coordinate(S,2))/2;

for i=1:S
    Coordinate(S+i,1)=2*xcenter-Coordinate(i,1);
    Coordinate(S+i,2)=2*ycenter-Coordinate(i,2);
end

sizec = size(Coordinate);
S=sizec(1);


%Positive angular value corresponds to anti-clockwise rotation.
D = zeros(S,1);
for i = 1:S
    D(i) = sqrt((dx-Coordinate(i,1))^2+(dy+Coordinate(i,2))^2);
    theta2defi(i) = -360/2/pi*(offseta(2)-acos((l1^2+l2^2-D(i)^2)/(2*l1*l2)));
    theta_between_Dl1(i) = acos((l1^2+D(i)^2-l2^2)/(2*D(i)*l1));
    theta_node_rod1rotationcenter(i) = atan2(dy+Coordinate(i,2),Coordinate(i,1)-dx);
    theta1defi(i) = -360/2/pi*(pi-theta_between_Dl1(i)-theta_node_rod1rotationcenter(i));
end


%read rot1&rot2
rot1(1) = theta1defi(1);
rot2(1) = theta2defi(1);
for i = 2:S
    rot1(i) = theta1defi(i)-theta1defi(i-1);
    rot2(i) = theta2defi(i)-theta2defi(i-1);
end
%returning command on Arduino.
plot(Coordinate(:,1),Coordinate(:,2));
axis equal;