%clc
clear all
close all
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

comau_kine_matlab = DQ_kinematics(comau_DH_matrix, 'modified');
%comau_kine_cpp = DQ_kin(comau_DH_matrix, 'modified');

%% Basic definitions for the simulation
desired_theta = [pi,pi/6,(-pi/2+pi/4),0,(pi/2+pi),0]';
% tic
% xd_matlab = comau_kine_matlab.fkm(desired_theta);
% toc
% tic
% xd_cpp = CMEX_raw_fkm(comau_DH_matrix, 'modified', desired_theta, 7);
% toc
% pause(2)
% tic
% J_matlab = comau_kine_matlab.jacobian(desired_theta);
% toc
% tic
% J_cpp = CMEX_jacobian(comau_DH_matrix, 'modified', desired_theta);
% toc
% pause(2)
tic
xm_matlab = comau_kine_matlab.fkm(desired_theta);
Jm_matlab = comau_kine_matlab.jacobian(desired_theta);
toc
tic
xm_cpp = CMEX_raw_fkm(comau_DH_matrix, 'modified', desired_theta, 7);
Jm_cpp = CMEX_jacobian(comau_DH_matrix, 'modified', desired_theta);
toc