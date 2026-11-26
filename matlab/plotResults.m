function figureHandles=plotResults(logger)
% One figure per assignment plot; use MATLAB's figure tools to save or export.
n=numel(logger.entries); assert(n>0,'controls:Empty','No simulation data.');
time=zeros(n,1); truth=zeros(n,2); estimate=nan(n,2); reader=zeros(n,2);
error=nan(n,1); uncertainty=nan(n,1); rssi=nan(n,1); available=logger.availability();
velocity=zeros(n,2); turn=zeros(n,2);
for i=1:n
    e=logger.entries{i}; time(i)=e.time;
    truth(i,:)=[e.trueTag.position.x e.trueTag.position.y]; reader(i,:)=[e.reader.position.x e.reader.position.y];
    if ~isempty(e.estimatedTag), estimate(i,:)=[e.estimatedTag.position.x e.estimatedTag.position.y]; end
    [error(i),~]=controls.DataLogger.errors(e); uncertainty(i)=e.uncertainty;
    if e.measurementDue&&e.measurement.isValid, rssi(i)=e.measurement.Pr; end
    velocity(i,:)=[e.reader.velocity e.command.velocity]; turn(i,:)=[e.reader.turnSpeed e.command.turnRate];
end
names={'trajectories','localization','rssi','valid_reads','speed','turn_rate'};
titles={'Trajectories and workspace','Localization quality','Valid RSSI measurements',...
    'Valid RFID reads','Reader speed','Reader turn rate'};
figureHandles=gobjects(numel(names),1);
for i=1:numel(names)
    figureHandles(i)=figure('Name',titles{i},'NumberTitle','off','Color','white',...
        'Position',[100 100 900 600]);
    switch names{i}
        case 'trajectories'
            plot(truth(:,1),truth(:,2),estimate(:,1),estimate(:,2),reader(:,1),reader(:,2));
            legend('True tag','Estimate','Reader'); hold on;
            rectangle('Position',[0 0 10 10],'LineStyle',':');
            axis equal; xlabel('x (m)'); ylabel('y (m)');
        case 'localization'
            plot(time,[error uncertainty]); legend('Position error','Estimated RMS spread');
            xlabel('Time (s)'); ylabel('Distance (m)');
        case 'rssi'
            plot(time,rssi,'.'); xlabel('Time (s)'); ylabel('RSSI (dBm)');
        case 'valid_reads'
            stairs(time,double(available)); ylim([-0.1 1.1]); yticks([0 1]);
            yticklabels({'Invalid','Valid'}); xlabel('Time (s)'); ylabel('Latest scheduled RFID read');
        case 'speed'
            plot(time,velocity); yline(logger.maxVelocity,':'); legend('Reader','Command','Limit');
            xlabel('Time (s)'); ylabel('Speed (m/s)');
        case 'turn_rate'
            plot(time,turn); yline(logger.maxTurnRate,':'); yline(-logger.maxTurnRate,':');
            legend('Reader','Command','Upper limit','Lower limit');
            xlabel('Time (s)'); ylabel('Turn rate (rad/s)');
    end
    grid on; title(titles{i}); set(gca,'FontSize',12);
end
end
