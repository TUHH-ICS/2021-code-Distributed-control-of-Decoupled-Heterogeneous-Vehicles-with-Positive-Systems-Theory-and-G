%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script produces the results for Fig.3 in the above paper 
clear
clc
rng(1)
addpath('.\functions')
%% Initialize Information Flow dynamics
Information_flow_dynamics

%% Define the vehicle model and synthesize a local tracking controller
delta_t=0.1;
mass=[1;1;0.1;0.1;0.1];
b=[3;10;1;1;1];
gamma_q=zeros(N,1);
gamma_w=zeros(N,1);
G=cell(N,1);
for agent=1:N
    [A,Bu,Bw,C,D]=get_vehicle_ssdata(mass(agent),b(agent),delta_t);
    % Synthesis of Generalized H2 optimal controller(L2-Linf)
    tol=1e-9;
    rho=0.05;
    [gamma_q(agent),gamma_w(agent),F1,F2,F3]= get_2DOF_sf_gen_H2_controller(A,Bu,Bw,C,rho,tol);
    % Get closed loop matrices
    A_cl=A+Bu*F1;
    B_cl=[Bu*F2, Bu*F3, Bw];
    C_cl=C;
    D_cl=[D,0,0];
    G{agent,1}=ss(A_cl,B_cl,C_cl,D_cl,delta_t);
end
    
%% Simulate the system
disturbance_rejection_platooning

