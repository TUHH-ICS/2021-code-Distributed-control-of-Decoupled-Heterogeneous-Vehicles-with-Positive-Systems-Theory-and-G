%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function synthesizes a controller that minimizes the generalized h2
% norm of the closed loop system.
function [gamma_q_analysis,gamma_w_analysis,F1,F2,F3]= get_2DOF_sf_gen_H2_controller(A,Bu,Bw,C,rho,tol)

% Build the generalized plant
Ao=[A,                   zeros(size(A,1),1);
    zeros(1,size(A,2)),  0];
Buo=[Bu; 
    0];

Co=[-C,1];
Co1=[-C,1;zeros(1,size(A,2)),0];

% get sizes
n=size(Ao,1);
m=size(Bw,2);
nu=size(Buo,2);
nz=size(Co,1);

% CVX for synthesis LMIs
cvx_begin sdp
cvx_solver sedumi

variable Q(n,n) symmetric
variable Y(nu,n)
variable F3(nu,1)
variable gamma_q_sqr
variable gamma_w_sqr

cvx_precision medium

LMI_1_11=Q;
LMI_1_21=zeros((1+m),n);
LMI_1_22=blkdiag(gamma_q_sqr,gamma_w_sqr*eye(m));
LMI_1_31=Ao*Q+Buo*Y;
LMI_1_32=[  Bu*F3,  Bw;...
            1,      0];
LMI_1_33=Q;




LMI_1=[ LMI_1_11,    LMI_1_21',  LMI_1_31';
        LMI_1_21,    LMI_1_22,   LMI_1_32';
        LMI_1_31,    LMI_1_32,   LMI_1_33]; 

LMI_2=[ Q,      (Co*Q)';
        Co*Q,     eye(nz)];

LMI_2_alt=[ Q,                   (Co1*Q+[0;rho]*Y)';
            Co1*Q+[0;rho]*Y,     eye(nz+1)];
    

minimize gamma_q_sqr+gamma_w_sqr
subject to:

Q>=tol*eye(n);
gamma_q_sqr>=tol;
gamma_w_sqr>=tol;

LMI_1>=tol*eye(size(LMI_1,1));
%LMI_2>=tol*eye(size(LMI_2,1));
LMI_2_alt>=tol*eye(size(LMI_2_alt,1));

cvx_end
status=cvx_status; 

%% Verify the LMIs while Debugging
% eig(Q)>tol
% eig(LMI_1)>tol
% eig(LMI_2)>tol

F=Y/Q;

%% Analysis of Generalized H2(L2-Linf) norm to get a better estimate
% Close the loop for the generalized plant
A_cl_gp=Ao+Buo*F;
Bq_cl_gp=[Bu*F3;1];
Bw_cl_gp=[Bw;0];
C_cl_gp=Co;

tol=1e-16;
[gamma_q_analysis]= gen_H2_analysis(A_cl_gp,Bq_cl_gp,C_cl_gp,tol);
[gamma_w_analysis]= gen_H2_analysis(A_cl_gp,Bw_cl_gp,C_cl_gp,tol);

%% Return optimal control gain matrices
F1=F(:,1:size(A,2));
F2=F(:,size(A,2)+1:end);
end