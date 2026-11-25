classdef ReaderPlant < handle
    properties (SetAccess=private)
        state
    end
    methods
        function obj=ReaderPlant(initialState)
            if nargin<1, initialState=controls.Utils.readerState(); end
            obj.state=initialState;
        end
        function state=getState(obj), state=obj.state; end
        function update(obj,command,dt)
            controls.Utils.positive(dt,'dt');
            midHeading=obj.state.heading+command.turnRate*dt/2;
            obj.state.position.x=obj.state.position.x+command.velocity*cos(midHeading)*dt;
            obj.state.position.y=obj.state.position.y+command.velocity*sin(midHeading)*dt;
            obj.state.heading=controls.Utils.wrapAngle(obj.state.heading+command.turnRate*dt);
            obj.state.velocity=command.velocity; obj.state.turnSpeed=command.turnRate;
        end
    end
end
