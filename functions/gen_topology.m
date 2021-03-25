%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This function initializes a random interaction topology for the network
function A=gen_topology(n,link_prob,topo)
%% Random Topology
if topo == 'rand'
    d=zeros(n,1);
    while size(find(d),1)<n
        rand_mat=rand(n,n);
        rand_mat=rand_mat+rand_mat';
        A=rand_mat<2*link_prob;
        for i =1:n A(i,i)=0;end 
        d=A*ones(n,1);
    end
end

%% Star Topology
if topo=='star'
    A=zeros(n,n);
    A(:,1)=ones(n,1);
    A(1,:)=ones(n,1);
    A(1,1)=0;
end
%% Line Topology
if topo=='line' 
    A=zeros(n,n);
    A=full(gallery('tridiag',n,1,0,1));
end
%% Lattice Topology
if topo=='latt' 
    A=zeros(n,n);
    for i=1:n
        for j=1:n
            A(i,j)=(j==i+abs(sqrt(n)));
            
        end        
    end
    A=A+0.5*full(gallery('tridiag',n,1,0,1));
    A=(A+A');
end


%% Randomly generated

% A=[    0   0   0   1   1   0   1   1   0   0    1   1;
%        0   0   0   0   0   0   1   1   1   0    1   1;
%        0   0   0   1   1   1   1   0   0   1    1   1;
%        0   0   1   0   1   1   0   0   0   0    1   0;
%        1   0   1   1   0   1   0   1   0   1    0   1;
%        1   0   1   0   0   0   0   1   0   1    0   1;
%        0   1   0   0   0   1   0   1   1   1    0   0;
%        1   1   0   0   1   0   0   0   0   1    0   1;
%        1   1   1   1   0   0   0   0   0   1    0   1;
%        1   0   1   0   0   1   0   1   1   0    0   1;
%        1   0   1   0   1   0   0   1   0   0    0   0;
%        1   0   1   0   1   1   1   1   1   0    1   0 ]; 




end