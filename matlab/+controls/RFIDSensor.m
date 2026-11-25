classdef RFIDSensor < handle
    properties
        config
    end
    properties (Access=private)
        rng
    end
    methods
        function obj=RFIDSensor(config,seed)
            if nargin<2, seed=0; end
            config.validate(); obj.config=config;
            obj.rng=RandStream('mt19937ar','Seed',seed);
        end
        function measurement=measure(obj,trueTag,reader)
            c=obj.config; dx=trueTag.position.x-reader.position.x; dy=trueTag.position.y-reader.position.y;
            distance=hypot(dx,dy);
            measurement=controls.Utils.measurement(0,false,c.rssiStdDev,0);
            if distance>c.maxRange||rand(obj.rng)<c.dropoutProbability, return; end
            power=c.referenceRSSI-10*c.pathLossExponent*log10(max(distance,c.referenceDistance)/c.referenceDistance);
            power=power+c.rssiStdDev*randn(obj.rng);
            if power<c.minRSSI, return; end
            bearing=controls.Utils.wrapAngle(atan2(dy,dx)-reader.heading+c.bearingStdDev*randn(obj.rng));
            measurement=controls.Utils.measurement(power,true,c.rssiStdDev,bearing);
        end
    end
end
