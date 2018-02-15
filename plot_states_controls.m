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

t = 0:0.1:5;

figure
subplot(3,1,1)
plot(t,x)
subplot(3,1,2)
plot(t,y)
subplot(3,1,3)
plot(t,z)
suptitle('quad position')

figure
subplot(3,1,1)
plot(t,vx)
subplot(3,1,2)
plot(t,vy)
subplot(3,1,3)
plot(t,vz)
suptitle('quad velocity')

figure
subplot(3,1,1)
plot(t,ax)
subplot(3,1,2)
plot(t,ay)
subplot(3,1,3)
plot(t,az)
suptitle('quad acceleration')

figure
subplot(2,1,1)
plot(t,ga)
subplot(2,1,2)
plot(t,wz)
suptitle('yaw and yaw rate')

figure
subplot(2,1,1)
plot(t,q1)
subplot(2,1,2)
plot(t,q2)
suptitle('joint angles')

figure
subplot(2,1,1)
plot(t,qd1)
subplot(2,1,2)
plot(t,qd2)
suptitle('joint velocities')

