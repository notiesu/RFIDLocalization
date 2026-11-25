classdef RFIDConfig
    properties
        referenceRSSI=-40
        referenceDistance=1
        pathLossExponent=2
        rssiStdDev=2
        bearingStdDev=0.1
        maxRange=10
        minRSSI=-80
        updateRate=10
        dropoutProbability=0.05
    end
    methods
        function obj=RFIDConfig(varargin)
            assert(mod(numel(varargin),2)==0,'controls:Options','Use name/value pairs.');
            for i=1:2:numel(varargin)
                name=char(varargin{i});
                assert(isprop(obj,name),'controls:Options','Unknown sensor option %s.',name);
                obj.(name)=varargin{i+1};
            end
            obj.validate();
        end
        function validate(obj)
            for name={'referenceDistance','pathLossExponent','maxRange','updateRate'}
                controls.Utils.positive(obj.(name{1}),name{1});
            end
            for name={'rssiStdDev','bearingStdDev'}
                controls.Utils.positive(obj.(name{1}),name{1},true);
            end
            assert(isscalar(obj.referenceRSSI)&&isfinite(obj.referenceRSSI)&&...
                isscalar(obj.minRSSI)&&isfinite(obj.minRSSI),'controls:Config','RSSI levels must be finite.');
            assert(isscalar(obj.dropoutProbability)&&isfinite(obj.dropoutProbability)&&...
                obj.dropoutProbability>=0&&obj.dropoutProbability<=1,'controls:Config','Dropout must be in [0,1].');
        end
    end
end
