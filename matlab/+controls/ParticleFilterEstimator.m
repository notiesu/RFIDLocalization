classdef ParticleFilterEstimator < handle
    properties
        config
        particleCount
        accelerationStdDev
        resamplingThreshold
        currentUncertainty=NaN
        particleField=struct('state',{},'weight',{})
    end
    properties (Dependent)
        initialized
        positionUncertainty
    end
    properties (Access=private)
        rng
    end
    methods
        function obj=ParticleFilterEstimator(config,varargin)
            p=inputParser;
            addParameter(p,'particleCount',500); addParameter(p,'accelerationStdDev',0.5);
            addParameter(p,'resamplingThreshold',0.5); addParameter(p,'seed',1);
            parse(p,varargin{:}); o=p.Results;
            assert(isscalar(o.particleCount)&&isfinite(o.particleCount)&&o.particleCount>=2&&fix(o.particleCount)==o.particleCount,...
                'controls:Config','particleCount must be an integer >=2.');
            controls.Utils.positive(o.accelerationStdDev,'accelerationStdDev',true);
            assert(isscalar(o.resamplingThreshold)&&o.resamplingThreshold>0&&o.resamplingThreshold<=1,...
                'controls:Config','Resampling threshold must be in (0,1].');
            config.validate(); obj.config=config; obj.particleCount=o.particleCount;
            obj.accelerationStdDev=o.accelerationStdDev; obj.resamplingThreshold=o.resamplingThreshold;
            obj.rng=RandStream('mt19937ar','Seed',o.seed);
        end
        function value=get.initialized(obj), value=~isempty(obj.particleField); end
        function value=get.positionUncertainty(obj)
            value=NaN; if ~obj.initialized, return; end
            estimate=obj.calculateStateEstimate(); variance=0;
            for i=1:numel(obj.particleField)
                p=obj.particleField(i); variance=variance+p.weight*((p.state.position.x-estimate.position.x)^2+...
                    (p.state.position.y-estimate.position.y)^2);
            end
            value=sqrt(variance);
        end
        function initializeFromMeasurement(obj,measurement,reader)
            c=obj.config; obj.particleField=repmat(controls.Utils.particle(controls.Utils.tagState(),1/obj.particleCount),obj.particleCount,1);
            for i=1:obj.particleCount
                rssi=measurement.Pr+measurement.uncertainty*randn(obj.rng);
                exponent=(c.referenceRSSI-rssi)/(10*c.pathLossExponent);
                distance=c.referenceDistance*10^min(exponent,log10(c.maxRange/c.referenceDistance));
                if distance<=c.referenceDistance, distance=min(c.referenceDistance,c.maxRange)*sqrt(rand(obj.rng)); end
                bearing=reader.heading+measurement.bearing+c.bearingStdDev*randn(obj.rng);
                obj.particleField(i).state=controls.Utils.tagState(controls.Utils.coord(...
                    reader.position.x+distance*cos(bearing),reader.position.y+distance*sin(bearing)),...
                    randn(obj.rng),randn(obj.rng));
            end
        end
        function state=getNextParticleState(obj,state,dt)
            controls.Utils.positive(dt,'dt');
            acceleration=obj.accelerationStdDev*randn(obj.rng,2,1);
            state.position.x=state.position.x+state.xdot*dt+0.5*acceleration(1)*dt^2;
            state.position.y=state.position.y+state.ydot*dt+0.5*acceleration(2)*dt^2;
            state.xdot=state.xdot+acceleration(1)*dt; state.ydot=state.ydot+acceleration(2)*dt;
        end
        function predictParticles(obj,dt)
            controls.Utils.positive(dt,'dt');
            for i=1:numel(obj.particleField), obj.particleField(i).state=obj.getNextParticleState(obj.particleField(i).state,dt); end
        end
        function updateWeights(obj,measurement,reader)
            if isempty(measurement)||~measurement.isValid||~obj.initialized, return; end
            c=obj.config; rssiSigma=max(measurement.uncertainty,1e-6); bearingSigma=max(c.bearingStdDev,1e-6);
            logWeights=-inf(numel(obj.particleField),1);
            for i=1:numel(obj.particleField)
                particle=obj.particleField(i); dx=particle.state.position.x-reader.position.x; dy=particle.state.position.y-reader.position.y;
                distance=max(hypot(dx,dy),c.referenceDistance);
                expectedRSSI=c.referenceRSSI-10*c.pathLossExponent*log10(distance/c.referenceDistance);
                expectedBearing=controls.Utils.wrapAngle(atan2(dy,dx)-reader.heading);
                rssiError=(measurement.Pr-expectedRSSI)/rssiSigma;
                bearingError=controls.Utils.wrapAngle(measurement.bearing-expectedBearing)/bearingSigma;
                if particle.weight>0, logWeights(i)=log(particle.weight)-0.5*(rssiError^2+bearingError^2); end
            end
            peak=max(logWeights); assert(isfinite(peak),'controls:Weights','No finite particle likelihood.');
            weights=exp(logWeights-peak); weights=weights/sum(weights);
            for i=1:numel(weights), obj.particleField(i).weight=weights(i); end
        end
        function estimate=calculateStateEstimate(obj)
            assert(obj.initialized,'controls:Uninitialized','Estimator needs a prior or valid RFID reading.');
            vector=zeros(4,1);
            for i=1:numel(obj.particleField), vector=vector+obj.particleField(i).weight*controls.Utils.stateVector(obj.particleField(i).state); end
            estimate=controls.Utils.vectorState(vector);
        end
        function ess=calculateEffectiveSampleSize(obj)
            ess=0; if obj.initialized, ess=1/sum([obj.particleField.weight].^2); end
        end
        function resampleParticles(obj)
            if ~obj.initialized, return; end
            n=numel(obj.particleField);
            offset=rand(obj.rng)/n;
            original=obj.particleField; index=1; cumulative=original(1).weight;
            for i=1:n
                target=offset+(i-1)/n;
                while target>cumulative&&index<n, index=index+1; cumulative=cumulative+original(index).weight; end
                obj.particleField(i)=controls.Utils.particle(original(index).state,1/n);
            end
        end
        function estimate=update(obj,measurement,reader,dt,predict)
            if nargin<5, predict=true; end
            controls.Utils.positive(dt,'dt');
            if ~obj.initialized
                if ~isempty(measurement)&&measurement.isValid, obj.initializeFromMeasurement(measurement,reader); end
            else
                if predict, obj.predictParticles(dt); end
                obj.updateWeights(measurement,reader);
            end
            estimate=[];
            if obj.initialized, estimate=obj.calculateStateEstimate(); end
            % Preserve reading-time spread before any resampling.
            obj.currentUncertainty=obj.positionUncertainty;
            if obj.initialized&&predict&&obj.calculateEffectiveSampleSize()<obj.resamplingThreshold*obj.particleCount
                obj.resampleParticles();
            end
        end
    end
end
