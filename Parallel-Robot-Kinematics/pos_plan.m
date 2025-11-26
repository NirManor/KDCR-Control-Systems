function x=pos_plan(t)
% The following function calculates the position of the end effector as a
% function of time.
% t is vector of the time

global xA yA phiA xB yB phiB T dt

% xA = -3; yA=-2; phiA=pi/4; 
% xB=-2; yB=0; phiB=0;

% Reshaping the t vector as a row vector
[rows,~]=size(t);
if rows>1
    t=t';
end

% preallocation
x=zeros(3,round(T/dt+1));

x=[xA+(xB-xA)*t/T; yA+(yB-yA)*t/T; phiA+(phiB-phiA)*t/T];
