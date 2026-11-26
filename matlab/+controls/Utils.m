classdef Utils
    % Value signals used by the six diagram subsystems.
    methods (Static)
        function v=coord(x,y)
            if nargin<1, x=0; end
            if nargin<2, y=0; end
            v=struct('x',x,'y',y);
        end
        function v=tagState(position,xdot,ydot)
            if nargin<1, position=controls.Utils.coord(); end
            if nargin<2, xdot=0; end
            if nargin<3, ydot=0; end
            v=struct('position',position,'xdot',xdot,'ydot',ydot);
        end
        function v=readerState(position,heading,turnSpeed,velocity)
            if nargin<1, position=controls.Utils.coord(); end
            if nargin<2, heading=0; end
            if nargin<3, turnSpeed=0; end
            if nargin<4, velocity=0; end
            v=struct('position',position,'heading',heading,'turnSpeed',turnSpeed,'velocity',velocity);
        end
        function v=measurement(Pr,isValid,uncertainty,bearing)
            if nargin<1, Pr=0; end
            if nargin<2, isValid=false; end
            if nargin<3, uncertainty=0; end
            if nargin<4, bearing=0; end
            v=struct('Pr',Pr,'isValid',logical(isValid),'uncertainty',uncertainty,'bearing',bearing);
        end
        function v=command(turnRate,velocity)
            if nargin<1, turnRate=0; end
            if nargin<2, velocity=0; end
            v=struct('turnRate',turnRate,'velocity',velocity);
        end
        function v=particle(state,weight)
            v=struct('state',state,'weight',weight);
        end
        function v=stateVector(s)
            v=[s.position.x;s.position.y;s.xdot;s.ydot];
        end
        function s=vectorState(v)
            s=controls.Utils.tagState(controls.Utils.coord(v(1),v(2)),v(3),v(4));
        end
        function v=wrapAngle(a), v=atan2(sin(a),cos(a)); end
        function positive(v,name,allowZero)
            if nargin<3, allowZero=false; end
            assert(isnumeric(v)&&isscalar(v)&&isfinite(v)&&(v>0||(allowZero&&v==0)),...
                'controls:Value','Invalid %s.',name);
        end
    end
end
