classdef ScenarioGenerator
    methods (Static)
        function scenario=loadCSV(file)
            data=readtable(file,'VariableNamingRule','preserve','TextType','string');
            scenario=controls.ScenarioGenerator.fromTable(data);
        end

        function scenario=fromTable(data)
            assert(istable(data),'controls:Scenario','Supply a trajectory table or CSV filename.');
            fields={'time','x','y','vx','vy'};
            assert(all(ismember(fields,data.Properties.VariableNames)),'controls:CSV','Required columns: time,x,y,vx,vy.');
            values=data{:,fields}; n=size(values,1);
            assert(isnumeric(values)&&n>=2&&all(isfinite(values),'all'),'controls:CSV','Need at least two finite state rows.');
            time=values(:,1); dt=time(2)-time(1); expected=(0:n-1)'*dt;
            assert(time(1)==0&&dt>0&&all(diff(time)>0)&&all(abs(time-expected)<=max(1e-9,1e-7*abs(expected))),...
                'controls:CSV','Time must start at zero with uniform positive spacing.');
            states=repmat(controls.Utils.tagState(),n,1); events=repmat({''},n,1);
            for i=1:n, states(i)=controls.Utils.vectorState(values(i,2:5)); end
            if ismember('event',data.Properties.VariableNames)
                labels=string(data.event); labels(ismissing(labels)|labels=="NaN")=""; events=cellstr(labels);
            end
            scenario=struct('dt',dt,'states',states,'events',{events});
        end
    end
end
