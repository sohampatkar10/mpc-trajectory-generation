clear
close all
X = dlmread("data/states.txt");
U = dlmread("data/controls.txt");

x = X(:,1);
y = X(:,2);
z = X(:,3);

vx = X(:,4);
vy = X(:,5);
vz = X(:,6);

ax = X(:,7);
ay = X(:,8);
az = X(:,9);

jx = X(:,10);
jy = X(:,11);
jz = X(:,12);

ga = X(:,13);
wz = X(:,14);

q1 = X(:,15);
q2 = X(:,16);

sx = U(:,1);
sy = U(:,2);
sz = U(:,3);

wzd = U(:,4);
qd1 = U(:,5);
qd2 = U(:,6);

m = 6.7; xq = 1.1; yq = 1.1; zq = 0.2;
ml = 0.1; l1 = 0.255; l2 = 0.555;
J = diag([m*xq*xq, m*yq*yq + ml*l1*l1 + ml*l2*l2, m*zq*zq]);

for i=1:size(x,1)
    T = sqrt(ax(i)^2 + ay(i)^2 + (az(i)+9.81)*(az(i)+9.81));
    r(i) = (-ax(i)*sin(ga(i)) + ay(i)*cos(ga(i)))/T;
    p(i) = atan2(ax(i)*cos(ga(i)) + ay(i)*sin(ga(i)), az(i) + 9.81);

    Rz = [ax(i);ay(i);az(i)+9.81]/T;
    Ry = cross(Rz, [cos(ga(i));sin(ga(i));0])/norm(cross(Rz, [cos(ga(i));sin(ga(i));0]));
    Rx = cross(Ry, Rz);
    R = [Rx,Ry,Rz];
    a = rotm2eul(R);
    re(i) = a(3); pe(i) = a(2);
    
    Td = m*Rz'*[jx(i);jy(i);jz(i)];
    
    wx(i) = -Ry'*[jx(i);jy(i);jz(i)];
    wy(i) = Rx'*[jx(i);jy(i);jz(i)];
    
    wxd(i) = (-m*Ry'*[sx(i);sy(i);sz(i)] - wy(i)*wz(i)*T + 2*wx(i)*Td)/T;
    wyd(i) = (-m*Ry'*[sx(i);sy(i);sz(i)] - wx(i)*wz(i)*T - 2*wy(i)*Td)/T;
    
    tau = J*[wxd(i);wyd(i);wzd(i)] - cross(J*[wx(i);wy(i);wz(i)], [wx(i);wy(i);wz(i)]); 
    Tx(i) = tau(1); Ty(i) = tau(2); Tz = tau(3);
end

t = 0:(3/15):3;

figure
subplot(2,1,1)
plot(t,wx,'o')
subplot(2,1,2)
plot(t,wy,'o')
suptitle('angular velocities')

figure
subplot(2,1,1)
plot(t,r,'r')
subplot(2,1,2)
plot(t,p,'r')
suptitle('angular velocities')

% 
% figure
% subplot(3,1,1)
% plot(t,x)
% subplot(3,1,2)
% plot(t,y)
% subplot(3,1,3)
% plot(t,z)
% suptitle('Quad Position')
% 
% figure
% subplot(3,1,1)
% plot(t,vx)
% subplot(3,1,2)
% plot(t,vy)
% subplot(3,1,3)
% plot(t,vz)
% suptitle('Quad Velocity')
% 
% figure
% subplot(3,1,1)
% plot(t,ax)
% subplot(3,1,2)
% plot(t,ay)
% subplot(3,1,3)
% plot(t,az)
% suptitle('Quad Acceleration')

% figure
% subplot(2,1,1)
% plot(t,ga)
% subplot(2,1,2)
% plot(t,wz)
% suptitle('Yaw and Yaw rate')
% 
% figure
% subplot(2,1,1)
% plot(t,q1)
% subplot(2,1,2)
% plot(t,q2)
% suptitle('Joint Angles')
% 
% figure
% subplot(2,1,1)
% plot(t,qd1)
% subplot(2,1,2)
% plot(t,qd2)
% suptitle('Joint Velocities')
% 
% figure
% subplot(3,1,1)
% plot(t,sx,'o')
% subplot(3,1,2)
% plot(t,sy,'o')
% subplot(3,1,3)
% plot(t,sz,'o')
% suptitle('Quad Snap')
