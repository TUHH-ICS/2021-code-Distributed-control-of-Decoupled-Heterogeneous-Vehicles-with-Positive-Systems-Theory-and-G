%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script generates a network and synthesizes a diagonal weight matrix
%% Generate a Random Network Topology and synthesize weights for the first order protocol 
N=5;                     % number of agents. Please specify a perfect square in case of lattice topology
link_prob=0.6;           % For generating a random interconnection topology
K_lead=5;                % leader spring to origin
topo='rand';             % 'rand','star','line'
no_leaders=1;            % Set number of leaders in the group

% Randomly generate a Laplacian
[L,Deg]= init_info_flow_dynamics(N,link_prob,topo);
% Stabilize the first "no_leaders" agents to the origin    
I=eye(N);
L_lead=L;
for i=1:no_leaders
    L_lead = L_lead + K_lead*I(:,i)*I(i,:);
end
% Synthesize weights according to LMI
alpha_sqr=0.7;          % exponent for geometric convergece rate
tol=1e-6;               % tolerence for definiteness constraints
[P,E,status]= synthesis_exp_stability_LMI(L_lead,alpha_sqr,tol);
W=eye(N)-E*L_lead;
