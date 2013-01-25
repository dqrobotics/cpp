clear all;
close all;
clear classes;
clc;

%% Definitions for DQ_kinematics
comau_DH_theta=  [0, -pi/2, pi/2, 0, 0, 0, pi];
comau_DH_d =     [-0.45, 0, 0, -0.64707, 0, -0.095, 0];
comau_DH_a =     [0, 0.150, 0.590, 0.13, 0, 0, 0];
comau_DH_alpha = [pi, pi/2, pi, -pi/2, -pi/2, pi/2, pi];
comau_dummy =    [0,0,0,0,0,0,1];

comau_DH_matrix = [comau_DH_theta;
    comau_DH_d;
    comau_DH_a;
    comau_DH_alpha;
    comau_dummy];

comau_kine = DQ_kinematics(comau_DH_matrix, 'modified');


%% Basic definitions for the simulation
initial_theta = [0,0,-pi/2,0,pi/2,0]';
desired_theta = [pi,pi/6,(-pi/2+pi/4),0,(pi/2+pi),0]';

xd = comau_kine.fkm(desired_theta);
xm = comau_kine.fkm(initial_theta);

error = vec8(xd-xm);
epsilon = 0.01;
%K = diag([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]);
K = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]);
%K = diag([0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]);
%K = diag([1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2]);
theta = initial_theta;
desired_vec(:,1) = vec8(xd);
manipulated_vec(:,1) = vec8(xm);
% figure;
% hold on;
% plot(comau_kine,theta);
% plot(xd,'scale',0.5);
% 
% axis equal;
% axis([-0.8,1.2,-0.8,0.8,-0.2,1.5]);
% view(27,34);
i=1;

while norm(error) > epsilon  
    tic
    xm = comau_kine.fkm(theta);
    time_fkm(i) = toc;
    tic
    J = comau_kine.jacobian(theta);
    time_jacobian(i) = toc;
    error = vec8(xd-xm);
    desired_vec(:,i) = vec8(xd);
    manipulated_vec(:,i) = vec8(xm);
    theta = theta + pinv(J)*K*error;
%     plot(comau_kine,theta);
    i = i+1;
end
%pause(10);
desired_theta = [(3*pi)/2,pi/6,(-pi/2+pi/4),0,(pi/2+pi),0]';
xd = comau_kine.fkm(desired_theta);
% plot(xd,'scale',0.5);
error = vec8(xd-xm);
while norm(error) > epsilon  
    tic
    xm = comau_kine.fkm(theta);
    time_fkm(i) = toc;
    tic
    J = comau_kine.jacobian(theta);
    time_jacobian(i) = toc;
    error = vec8(xd-xm);
    desired_vec(:,i) = vec8(xd);
    manipulated_vec(:,i) = vec8(xm);
    theta = theta + pinv(J)*K*error;
%     plot(comau_kine,theta);
    i = i+1;
end