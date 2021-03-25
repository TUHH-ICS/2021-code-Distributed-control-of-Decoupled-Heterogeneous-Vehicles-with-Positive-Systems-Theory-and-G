%---------------------------------------------------------------------------------------------------
% For Paper
% "Distributed  Control  of  Heterogeneous  Networks  of  Vehicles  with Positive  Systems  Theory  and  Generalized H2 Norm"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script simulates a formation forming scenario

% Simulate trajectories
time_steps=25;
beta=50;       % impulse disturbance amplitude

% Formation Reference
formation_ref=10*(-N/2+(1:N)');
formation_ref_std_form_control=kron(formation_ref,[1;0]);

% Initial conditions
p_ic=formation_ref+5*(-0.5+rand(N,1));
x_ic=kron(p_ic,[1;0]);

% Initialize variables to zero
n=size(A_cl,1);
m=size(B_cl,2);
r=size(C_cl,1);
x=zeros(N*n,time_steps);
y=zeros(N*r,time_steps);
p=zeros(N,time_steps);


% Set Initial conditions
x(:,1)=x_ic;
x_std_form_control(:,1)=x_ic;
p(:,1)=p_ic;
%% Time stepping
for i=2:time_steps+1    
    p(:,i)=W*(p(:,i-1))+E*L_lead*formation_ref;      
       
    for agent=1:N
        [A_cl,B_cl,C_cl,D_cl]=ssdata(G{agent});
        id_x=(agent-1)*n;
        id_y=(agent-1)*r;
        id_u=(agent-1)*m;
        
        % With disturbance
        B_stack_std_form_control=[Bu*[4 1.5] Bw];
        w_dist=0;
        if rand()<0.2
            w_dist=beta*(-1+2*rand());
        end
        %input_stack=[p_trans(agent,i-1);p_trans(agent,i);w_dist(agent,i-1)];
        input_stack=[p(agent,i-1);p(agent,i);w_dist];        
        [x(id_x+1:id_x+n,i),y(id_y+1:id_y+r,i-1)] = ss_time_increment(A_cl,B_cl,C_cl,x(id_x+1:id_x+n,i-1),input_stack);        
    end    
end
%% Bounds
C1=sqrt(condest(P)/(1-alpha_sqr)); 
zeta_bound=gamma_q*C1*norm(p_ic,2)+gamma_w*beta;

%% Plot
colors = { 'r', 'g', 'b','c','m','y'};
limits = @(x,j) [x-zeta_bound(j); x; x+zeta_bound(j)];
t=(1:(time_steps));

figure()
for i = 1:N
    plot(t, y(i,1:time_steps), 'Color', colors{i})
    leg{i}=['m=',num2str(mass(i)),', b=',num2str(b(i))];
    hold on
end
for i = 1:N    
    plot_shaded(t, limits(p(i,1:time_steps),i), colors{i})
    hold on
end
hold off

xlim([1 time_steps])
grid('on')
xlabel('time (k)')
ylabel('x(k)')
title('Formation Forming')
legend(leg)


