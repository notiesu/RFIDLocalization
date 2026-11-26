function trajectory=generateAssignmentScenario(name)
%GENERATEASSIGNMENTSCENARIO Reproducible assignment paths, returned in memory.
% Exact geometry, duration and speed are design choices, not prescribed values.
if nargin<1, name='straight'; end
time=(0:0.1:20)'; speed=0.3; n=numel(time);
event=repmat("",n,1);
switch char(name)
    case 'straight'
        x=2+speed*time; y=2*ones(n,1);
        vx=speed*ones(n,1); vy=zeros(n,1);
    case 'turning'
        radius=speed*20/(pi/2); angle=speed*time/radius;
        x=2+radius*sin(angle); y=2+radius*(1-cos(angle));
        vx=speed*cos(angle); vy=speed*sin(angle);
    case 'abrupt_maneuver'
        % Continuous position; velocity changes east -> north at t=10 s.
        x=2+speed*min(time,10); y=2+speed*max(time-10,0);
        vx=speed*double(time<10); vy=speed*double(time>=10);
        event(round(10/0.1)+1)="abrupt_maneuver";
    otherwise
        error('controls:Scenario','Choose straight, turning or abrupt_maneuver.');
end
trajectory=table(time,x,y,vx,vy,event);
end
