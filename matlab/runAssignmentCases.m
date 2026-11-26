function [results,logs]=runAssignmentCases()
% Assignment analyses: 3 paths x fixed/baseline/final, plus 2 non-nominal cases.
setupControls(); cases={'straight','turning','abrupt_maneuver'};
modes={'fixed','baseline','final'}; rows=cell(0,16); logs=struct();
for i=1:numel(cases)
    for mode=0:2
        name=[cases{i} '_' modes{mode+1}]; sensor=controls.RFIDConfig();
        config=main(generateAssignmentScenario(cases{i}),...
            'controllerMode',mode,'sensorConfig',sensor);
        sim('rfid_tracking'); logger=evalin('base','trackingLog');
        logs.(name)=logger;
        rows(end+1,:)=summaryRow(name,logger,config.requirements); %#ok<AGROW>
    end
end
for condition={'increased_rssi_noise','increased_dropout'}
    name=condition{1}; sensor=controls.RFIDConfig();
    if strcmp(name,'increased_rssi_noise'), sensor.rssiStdDev=4;
    else, sensor.dropoutProbability=0.2; end
    config=main(generateAssignmentScenario('abrupt_maneuver'),...
        'controllerMode',2,'sensorConfig',sensor);
    sim('rfid_tracking'); logger=evalin('base','trackingLog');
    logs.(name)=logger;
    rows(end+1,:)=summaryRow(name,logger,config.requirements); %#ok<AGROW>
end
names={'Case','PositionRMSE','MaxError','ValidTimePercent','MeanUncertainty','MaxUncertainty',...
    'MeanSeparation','MaxSeparation','SpeedLimitTimePercent','TurnLimitTimePercent','MaxRecoveryTime',...
    'AccuracyMet','AvailabilityMet','ActuatorLimitsMet','RecoveryMet','UnrecoveredEvents'};
results=cell2table(rows,'VariableNames',names);
end

function row=summaryRow(name,logger,requirements)
s=logger.stats(); recovery=[]; incomplete=0;
for i=1:numel(s.recoveryEvents)
    event=s.recoveryEvents{i};
    if strcmp(event.status,'recovered'), recovery(end+1)=event.recoveryTime; %#ok<AGROW>
    else, incomplete=incomplete+1; end
end
maxRecovery=NaN; if ~isempty(recovery), maxRecovery=max(recovery); end
recoveryMet="n/a";
if ~isempty(s.recoveryEvents)
    recoveryMet="no"; if incomplete==0&&maxRecovery<=requirements.recoveryTime, recoveryMet="yes"; end
end
limitsMet=all(cellfun(@(e)abs(e.reader.velocity)<=logger.maxVelocity+1e-9&&...
    abs(e.reader.turnSpeed)<=logger.maxTurnRate+1e-9,logger.entries));
% Provisional targets declared in initControls, not fitted to these outcomes.
row={name,s.positionRMSE,s.maxError,s.validTimeRate,s.meanPositionUncertainty,s.maxPositionUncertainty,...
    s.meanTrackingDistance,s.maxTrackingDistance,s.speedLimitUsage,s.turnRateLimitUsage,maxRecovery,...
    s.positionRMSE<=requirements.positionRMSE,s.validTimeRate>=requirements.validTimeRate,limitsMet,recoveryMet,incomplete};
end
