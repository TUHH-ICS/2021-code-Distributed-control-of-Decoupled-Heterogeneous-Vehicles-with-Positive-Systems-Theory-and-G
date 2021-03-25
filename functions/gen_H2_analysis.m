%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This funtion computes the generalized H2 norm of system
function [gamma_2_inf]= gen_H2_analysis(A,B,C,tol)

n=size(A,1);
m=size(B,2);
nz=size(C,1);

cvx_begin sdp
cvx_solver sedumi

variable K(n,n) symmetric
variable gamma_2_inf_sqr
cvx_precision medium

LMI_1=[ A'*K*A - K,          A'*K*B;
        B'*K*A,              B'*K*B-gamma_2_inf_sqr*eye(m,m)]; 

LMI_2=[ K,      C';
        C,      eye(nz)];

minimize gamma_2_inf_sqr
subject to:

K>=tol*eye(n);
gamma_2_inf_sqr>=tol

LMI_1<=-tol*eye(size(LMI_1,1));
LMI_2>=tol*eye(size(LMI_2,1));

cvx_end
status=cvx_status; 
gamma_2_inf=sqrt(gamma_2_inf_sqr);
%% Verify the LMIs while Debugging
% eig(K)>tol
% eig(LMI_1)<tol
% eig(LMI_2)>tol
end