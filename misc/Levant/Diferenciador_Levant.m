clc
clear all
close all

data = dlmread ('datos2_yaw.txt','\t');

t =data(:,1);
yaw = data(:,2);
tau_yaw = data(:,3);


%% Levant primer orden
T = 0.02;
z0_z = 0;
z1_z = 0;
C = 150;

for i=1:length(t)-1
    v0_Z = z1_z(i) -   2*C^(1/3) * (abs( z0_z(i) - yaw(i) )^(2/3)) *  sign(z0_z(i) - yaw(i) );
    z0_z(i+1) = z0_z(i) + T*v0_Z;
    z1_z(i+1) = z1_z(i) + T*(-1.1*C*sign(z1_z(i) - v0_Z));
    z_est_1ro(i+1) = v0_Z;   
end    
euler1 = diff(yaw)/T;

plot(t(1:length(t)-1),z_est_1ro(1:length(t)-1)',t(1:length(t)-1),euler1)
xlabel('time')
legend('Levant_1ro','Euler')


%% Levant segundo orden
T = 0.02;
z0_z = 0;
z1_z = 0;
z2_z = 0;
C = 100;

for i=1:length(t)-1
    v0_Z = z1_z(i) -   2*C^(1/3) * (abs( z0_z(i) - yaw(i) )^(2/3)) *  sign(z0_z(i) - yaw(i) );
    v1_Z = z2_z(i) - 1.5*C^(1/2) * (abs( z1_z(i) - v0_Z  )^(1/2)) * sign(z1_z(i) - v0_Z);
    z0_z(i+1) = z0_z(i) + T*v0_Z;
    z1_z(i+1) = z1_z(i) + T*v1_Z;
    z2_z(i+1) = z2_z(i) + T*(-1.1*C*sign(z2_z(i) - v1_Z));
    z_est_2do(i+1) = v0_Z;   
end    
euler1 = diff(yaw)/T;

plot(t(1:length(t)-1),z_est_2do(1:length(t)-1)',t(1:length(t)-1),euler1)
xlabel('time')
legend('Levant_2do','Euler')

%% Levant Tercer Orden

z0_z = 0;
z1_z = 0;
z2_z = 0;
z3_z = 0;
C = 150;
for i=1:length(t)-1
   v0 = z1_z -   3*C^(1/4) * (abs( z0_z - yaw(i) )^(3/4)) *  sign(z0_z - yaw(i));
   v1 = z2_z -   2*C^(1/3) * (abs( z1_z - v0 )^(2/3)) *  sign(z1_z - v0);
   v2 = z3_z - 1.5*C^(1/2) * (abs( z2_z - v1 )^(1/2)) *  sign(z2_z - v1);
   z0_z = z0_z + T*v0;
   z1_z = z1_z + T*v1;
   z2_z = z2_z + T*v2;
   z3_z = z3_z + T*(-1.1*C*sign(z3_z - v2));   
   z_est_3ro(i+1) = v0;
end



plot(t(1:length(t)-1),z_est_3ro(1:length(t)-1)',t(1:length(t)-1),euler1)
xlabel('time')
legend('Levant_3ro','Euler')

%figure
%plot(t,yaw,'b',t,z_est2a,'r')
%plot(t,yaw,'b',t,z_est2a,'r',t,z_est2b,'g')

%%

plot(t(1:length(t)-1),z_est_1ro(1:length(t)-1)',t(1:length(t)-1),z_est_2do(1:length(t)-1)',t(1:length(t)-1),z_est_3ro(1:length(t)-1)',t(1:length(t)-1),euler1)
xlabel('time')
legend('Levant_1ro','Levant_2do','Levant_3ro','Euler')

