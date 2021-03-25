%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function returns state-space data for a discrete-time model of a
% generic vehicle obtained by discretizing(ZOH) a continous time model
function [A,Bu,Bw,C,D]=get_vehicle_ssdata(m,b,delta_t)
%% System matrices
% Mass spring damper system
A_cont=[0 1; 0 -(b/m)];
B_cont=[0;(1/m)];
C_cont=[1 0];
D_cont=0;

%% Discretize by ZOH
sys_cont=ss(A_cont,B_cont,C_cont,D_cont);
sys_zoh=c2d(sys_cont,delta_t,'zoh'); 
[A,B,C,D]=ssdata(sys_zoh);

A=sys_zoh.A;
B=sys_zoh.B;
C=sys_zoh.C;
D=sys_zoh.D;

%% Input disturbance
Bu=B; % Input disturbance
Bw=B;
end