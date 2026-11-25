classdef ControllerConfig
    properties
        Kv=0.5
        Kw=2
        maxVelocity=2
        maxTurnRate=1.5
        desiredDistance=2
        uncertaintyScale=1
    end
    methods
        function obj=ControllerConfig(varargin)
            assert(mod(numel(varargin),2)==0,'controls:Options','Use name/value pairs.');
            for i=1:2:numel(varargin)
                name=char(varargin{i}); assert(isprop(obj,name),'controls:Options','Unknown controller option.');
                obj.(name)=varargin{i+1};
            end
            for name={'Kv','Kw','maxVelocity','maxTurnRate','desiredDistance'}
                controls.Utils.positive(obj.(name{1}),name{1},true);
            end
            controls.Utils.positive(obj.uncertaintyScale,'uncertaintyScale');
        end
    end
end
