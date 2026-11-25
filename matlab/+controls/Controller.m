classdef Controller < handle
    properties
        config
    end
    methods
        function obj=Controller(config), obj.config=config; end
        function command=compute(obj,estimate,reader,uncertainty,available,mode)
            % 0 fixed reader, 1 baseline pursuit, 2 quality-aware standoff.
            command=controls.Utils.command();
            if isempty(estimate)||mode==0, return; end
            dx=estimate.position.x-reader.position.x; dy=estimate.position.y-reader.position.y;
            distance=hypot(dx,dy); if distance<=1e-12, return; end
            c=obj.config; error=controls.Utils.wrapAngle(atan2(dy,dx)-reader.heading);
            turnRate=max(-c.maxTurnRate,min(c.maxTurnRate,c.Kw*error));
            target=distance; gain=1;
            if mode==2
                target=max(0,distance-c.desiredDistance);
                if ~available||~isfinite(uncertainty), gain=0;
                else, gain=1/(1+uncertainty/c.uncertaintyScale); end
            end
            velocity=min(c.maxVelocity,c.Kv*target)*max(0,cos(error))*gain;
            command=controls.Utils.command(turnRate,velocity);
        end
    end
end
