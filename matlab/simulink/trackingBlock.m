function trackingBlock(block)
%TRACKINGBLOCK Normal-mode Level-2 adapter, one role per diagram subsystem.
% No code generation or operating-point restoration; class state is runtime-only.
setup(block);
end

function setup(block)
block.NumDialogPrms=3; role=char(block.DialogPrm(1).Data); config=settings(block,2);
block.DialogPrmsTunable={'Nontunable','Nontunable','Nontunable'};
switch role
    case 'scenario', inputs=[]; outputs=4;
    case 'sensor', inputs=[4 5]; outputs=6;
    case 'estimator', inputs=[6 5]; outputs=7;
    case 'controller', inputs=[7 5 1]; outputs=2;
    case 'plant', inputs=2; outputs=5;
    case 'logger', inputs=[4 7 5 6 2]; outputs=[];
    otherwise, error('controls:Role','Unknown block role.');
end
block.NumInputPorts=numel(inputs); block.NumOutputPorts=numel(outputs);
for i=1:numel(inputs)
    block.InputPort(i).Dimensions=inputs(i); block.InputPort(i).DatatypeID=0;
    block.InputPort(i).Complexity='Real'; block.InputPort(i).DirectFeedthrough=~strcmp(role,'plant');
end
for i=1:numel(outputs)
    block.OutputPort(i).Dimensions=outputs(i); block.OutputPort(i).DatatypeID=0; block.OutputPort(i).Complexity='Real';
end
block.SampleTimes=[config.dt 0];
if isprop(block,'OperatingPointCompliance'), block.OperatingPointCompliance='Disallow';
else, block.SimStateCompliance='DisallowSimState'; end
block.RegBlockMethod('Start',@start);
block.RegBlockMethod('PostPropagationSetup',@postSetup);
block.RegBlockMethod('Outputs',@outputsFcn);
block.RegBlockMethod('Update',@updateFcn);
block.RegBlockMethod('Terminate',@terminate);
end

function postSetup(block)
% Declare a discrete tick state so stateful handle-backed blocks are scheduled
% as discrete systems. Full state serialization is explicitly unsupported.
block.NumDworks=1; block.Dwork(1).Name='tick'; block.Dwork(1).Dimensions=1;
block.Dwork(1).DatatypeID=0; block.Dwork(1).Complexity='Real'; block.Dwork(1).UsedAsDiscState=true;
end

function start(block)
role=char(block.DialogPrm(1).Data); c=settings(block,2);
data=struct('lastTime',NaN,'cached',{{}},'component',[],'measurementIndex',0,'available',false);
block.Dwork(1).Data=0;
switch role
    case 'sensor', data.component=controls.RFIDSensor(c.sensorConfig,c.seed);
    case 'estimator'
        data.component=controls.ParticleFilterEstimator(c.sensorConfig,'particleCount',c.particles,'seed',c.seed+1);
    case 'controller', data.component=controls.Controller(c.controllerConfig);
    case 'plant', data.component=controls.ReaderPlant();
    case 'logger', data.component=controls.DataLogger(c.controllerConfig);
end
set_param(block.BlockHandle,'UserData',data,'UserDataPersistent','off');
end

function outputsFcn(block)
role=char(block.DialogPrm(1).Data); c=settings(block,2);
data=get_param(block.BlockHandle,'UserData'); time=block.CurrentTime;
if isequal(time,data.lastTime)
    for i=1:numel(data.cached), block.OutputPort(i).Data=data.cached{i}; end
    return;
end
out={};
switch role
    case 'scenario'
        index=round(time/c.dt)+1;
        assert(index>=1&&index<=numel(c.states),'controls:Time','Simulink exceeded the CSV trajectory.');
        out={controls.Utils.stateVector(c.states(index))};
    case 'plant'
        state=data.component.getState(); out={[state.position.x;state.position.y;state.heading;state.velocity;state.turnSpeed]};
    case 'sensor'
        due=time+1e-12>=data.measurementIndex/c.sensorConfig.updateRate;
        m=controls.Utils.measurement(0,false,c.sensorConfig.rssiStdDev,0);
        if due
            tag=controls.Utils.vectorState(block.InputPort(1).Data); reader=unpackReader(block.InputPort(2).Data);
            m=data.component.measure(tag,reader); data.available=m.isValid;
            data.measurementIndex=floor((time+1e-12)*c.sensorConfig.updateRate)+1;
        end
        out={[m.Pr;double(m.isValid);m.uncertainty;m.bearing;double(due);double(data.available)]};
    case 'estimator'
        v=block.InputPort(1).Data; m=[];
        if v(5)~=0, m=controls.Utils.measurement(v(1),v(2)~=0,v(3),v(4)); end
        reader=unpackReader(block.InputPort(2).Data);
        estimate=data.component.update(m,reader,c.dt,round(time/c.dt)~=0);
        signal=[nan(4,1);0;NaN;v(6)];
        if ~isempty(estimate), signal=[controls.Utils.stateVector(estimate);1;data.component.currentUncertainty;v(6)]; end
        out={signal};
    case 'controller'
        vector=block.InputPort(1).Data; mode=block.InputPort(3).Data;
        assert(ismember(mode,[0 1 2]),'controls:Controller','Invalid controller selection.');
        command=controls.Utils.command();
        if vector(5)~=0&&mode~=0
            estimate=controls.Utils.vectorState(vector(1:4)); reader=unpackReader(block.InputPort(2).Data);
            command=data.component.compute(estimate,reader,vector(6),vector(7)~=0,mode);
        end
        out={[command.turnRate;command.velocity]};
    case 'logger'
        tag=controls.Utils.vectorState(block.InputPort(1).Data); v=block.InputPort(2).Data; estimate=[]; uncertainty=v(6);
        if v(5)~=0, estimate=controls.Utils.vectorState(v(1:4)); end
        reader=unpackReader(block.InputPort(3).Data); v=block.InputPort(4).Data;
        m=controls.Utils.measurement(v(1),v(2)~=0,v(3),v(4)); due=v(5)~=0;
        commandVector=block.InputPort(5).Data; command=controls.Utils.command(commandVector(1),commandVector(2));
        events=settings(block,3);
        event=events{round(time/c.dt)+1};
        data.component.log(controls.DataLogger.entry(time,tag,estimate,reader,m,command,due,uncertainty,event));
end
for i=1:numel(out), block.OutputPort(i).Data=out{i}; end
data.cached=out; data.lastTime=time; set_param(block.BlockHandle,'UserData',data);
end

function updateFcn(block)
block.Dwork(1).Data=block.Dwork(1).Data+1;
if ~strcmp(char(block.DialogPrm(1).Data),'plant'), return; end
c=settings(block,2);
if round(block.CurrentTime/c.dt)<round(c.stopTime/c.dt)
    data=get_param(block.BlockHandle,'UserData'); command=block.InputPort(1).Data;
    data.component.update(controls.Utils.command(command(1),command(2)),c.dt);
end
end

function terminate(block)
role=char(block.DialogPrm(1).Data); c=settings(block,2);
data=get_param(block.BlockHandle,'UserData');
if strcmp(role,'logger')&&isstruct(data)&&isa(data.component,'controls.DataLogger')
    logger=data.component; assignin('base','trackingLog',logger);
    completed=numel(logger.entries)==round(c.stopTime/c.dt)+1;
    if ~isempty(c.output)&&completed
        logger.exportCSV(c.output);
    end

end
end

function value=settings(block,index)
% Dialog parameters are variable-name strings, never custom MATLAB objects.
% Values live in this model's workspace, not the caller's/base workspace.
workspace=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
value=evalin(workspace,char(block.DialogPrm(index).Data));
end

function reader=unpackReader(v)
reader=controls.Utils.readerState(controls.Utils.coord(v(1),v(2)),v(3),v(5),v(4));
end

