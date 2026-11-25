classdef DataLogger < handle
    properties
        entries={}
        maxVelocity
        maxTurnRate
        recoveryThreshold=0.5
        recoveryHoldTime=1
    end
    methods
        function obj=DataLogger(config)
            obj.maxVelocity=config.maxVelocity; obj.maxTurnRate=config.maxTurnRate;
        end
        function log(obj,entry)
            assert(isfinite(entry.time)&&(isempty(obj.entries)||entry.time>obj.entries{end}.time),...
                'controls:Time','Log times must increase.');
            obj.entries{end+1}=entry;
        end
        function values=availability(obj)
            values=false(numel(obj.entries),1); available=false;
            for i=1:numel(obj.entries)
                e=obj.entries{i}; if e.measurementDue, available=e.measurement.isValid; end
                values(i)=available;
            end
        end
        function events=recoveryStats(obj)
            events={}; dropoutStart=NaN;
            for i=1:numel(obj.entries)
                e=obj.entries{i};
                if ~isempty(e.event), events{end+1}=struct('event',e.event,'start',e.time,'end',e.time); end %#ok<AGROW>
                if e.measurementDue
                    if ~e.measurement.isValid&&isnan(dropoutStart), dropoutStart=e.time;
                    elseif e.measurement.isValid&&~isnan(dropoutStart)
                        events{end+1}=struct('event','dropout','start',dropoutStart,'end',e.time); dropoutStart=NaN; %#ok<AGROW>
                    end
                end
            end
            if ~isnan(dropoutStart), events{end+1}=struct('event','dropout','start',dropoutStart,'end',NaN); end
            if isempty(events), return; end
            starts=cellfun(@(e)e.start,events); [~,order]=sort(starts); events=events(order);
            available=obj.availability(); starts=starts(order);
            for j=1:numel(events)
                event=events{j}; event.recoveryTime=NaN; event.recoveredAt=NaN; event.status='not_recovered';
                if isnan(event.end), event.status='ongoing'; events{j}=event; continue; end
                future=starts(starts>event.start); nextEvent=Inf; if ~isempty(future), nextEvent=min(future); end
                streakStart=NaN;
                for i=1:numel(obj.entries)
                    e=obj.entries{i}; if e.time<event.end, continue; end
                    if e.time>=nextEvent, event.status='interrupted'; break; end
                    [error,~]=controls.DataLogger.errors(e);
                    if available(i)&&isfinite(error)&&error<=obj.recoveryThreshold
                        if isnan(streakStart), streakStart=e.time; end
                        if e.time-streakStart+1e-12>=obj.recoveryHoldTime
                            event.recoveryTime=e.time-event.end; event.recoveredAt=e.time; event.status='recovered'; break;
                        end
                    else, streakStart=NaN;
                    end
                end
                events{j}=event;
            end
        end
        function s=stats(obj)
            n=numel(obj.entries); time=zeros(n,1); error=nan(n,1); uncertainty=nan(n,1); separation=zeros(n,1);
            speed=false(n,1); turn=false(n,1);
            for i=1:n
                e=obj.entries{i}; time(i)=e.time; uncertainty(i)=e.uncertainty;
                [error(i),separation(i)]=controls.DataLogger.errors(e);
                speed(i)=obj.maxVelocity>0&&abs(e.command.velocity)>=obj.maxVelocity-1e-9;
                turn(i)=obj.maxTurnRate>0&&abs(e.command.turnRate)>=obj.maxTurnRate-1e-9;
            end
            dt=zeros(n,1); if n>1, dt=[diff(time);0]; end
            duration=sum(dt); available=obj.availability(); known=isfinite(uncertainty); errors=error(isfinite(error));
            s=struct('samples',n,'duration',duration,'positionRMSE',NaN,'maxError',NaN,...
                'validTimeRate',NaN,'meanPositionUncertainty',NaN,'maxPositionUncertainty',NaN,...
                'meanTrackingDistance',NaN,'maxTrackingDistance',NaN,'speedLimitUsage',NaN,'turnRateLimitUsage',NaN);
            if ~isempty(errors), s.positionRMSE=sqrt(mean(errors.^2)); s.maxError=max(errors); end
            if n>0, s.meanTrackingDistance=mean(separation); s.maxTrackingDistance=max(separation); end
            if any(known), s.maxPositionUncertainty=max(uncertainty(known)); end
            if sum(dt(known))>0, s.meanPositionUncertainty=sum(dt(known).*uncertainty(known))/sum(dt(known)); end
            if duration>0
                s.validTimeRate=100*sum(dt(available))/duration;
                if obj.maxVelocity>0, s.speedLimitUsage=100*sum(dt(speed))/duration; end
                if obj.maxTurnRate>0, s.turnRateLimitUsage=100*sum(dt(turn))/duration; end
            end
            s.recoveryEvents=obj.recoveryStats();
        end
        function exportCSV(obj,file)
            names={'time','true_x','true_y','est_x','est_y','reader_x','reader_y','heading','velocity','turnSpeed',...
                'rssi','valid','bearing','measurement_due','rfid_available','estimation_error','position_uncertainty',...
                'tracking_distance','command_velocity','command_turnRate','event'};
            rows=cell(numel(obj.entries),numel(names)); available=obj.availability();
            for i=1:numel(obj.entries)
                e=obj.entries{i}; estimate=[NaN NaN];
                if ~isempty(e.estimatedTag), estimate=[e.estimatedTag.position.x e.estimatedTag.position.y]; end
                [error,distance]=controls.DataLogger.errors(e);
                rows(i,:)=[num2cell([e.time e.trueTag.position.x e.trueTag.position.y estimate e.reader.position.x ...
                    e.reader.position.y e.reader.heading e.reader.velocity e.reader.turnSpeed e.measurement.Pr ...
                    double(e.measurement.isValid) e.measurement.bearing double(e.measurementDue) double(available(i)) ...
                    error e.uncertainty distance e.command.velocity e.command.turnRate]) {e.event}];
            end
            controls.Utils.ensureParent(file); writecell([names;rows],file);
        end
        function printStats(obj)
            s=obj.stats();
            fprintf('Position RMSE/max: %.3f / %.3f m | valid RFID time: %.1f%%\n',s.positionRMSE,s.maxError,s.validTimeRate);
            fprintf('Uncertainty mean/max: %.3f / %.3f m | separation mean/max: %.3f / %.3f m\n',...
                s.meanPositionUncertainty,s.maxPositionUncertainty,s.meanTrackingDistance,s.maxTrackingDistance);
            fprintf('Speed/turn limit usage: %.1f%% / %.1f%%\n',s.speedLimitUsage,s.turnRateLimitUsage);
            for i=1:numel(s.recoveryEvents)
                e=s.recoveryEvents{i}; fprintf('Recovery %s at %.2f s: %s (%.2f s)\n',e.event,e.start,e.status,e.recoveryTime);
            end
        end
    end
    methods (Static)
        function entry=entry(time,truth,estimate,reader,measurement,command,due,uncertainty,event)
            entry=struct('time',time,'trueTag',truth,'estimatedTag',estimate,'reader',reader,...
                'measurement',measurement,'command',command,'measurementDue',due,'uncertainty',uncertainty,'event',event);
        end
        function [error,distance]=errors(e)
            error=NaN;
            if ~isempty(e.estimatedTag), error=hypot(e.trueTag.position.x-e.estimatedTag.position.x,e.trueTag.position.y-e.estimatedTag.position.y); end
            distance=hypot(e.trueTag.position.x-e.reader.position.x,e.trueTag.position.y-e.reader.position.y);
        end
    end
end
