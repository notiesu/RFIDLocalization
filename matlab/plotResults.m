function figureHandle=plotResults(logger,filename)
% Figures required by the assignment; no animation or particle-history export.
if nargin<2, filename=''; end
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
figureHandle=figure('Color','white');
if ~isempty(filename), set(figureHandle,'Visible','off'); end
layout=tiledlayout(figureHandle,3,2); layout.TileSpacing='compact';
nexttile; plot(truth(:,1),truth(:,2),estimate(:,1),estimate(:,2),reader(:,1),reader(:,2));
legend('True tag','Estimate','Reader'); hold on; rectangle('Position',[0 0 10 10],'LineStyle',':');
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)'); title('Trajectories and workspace');
nexttile; plot(time,[error uncertainty]); legend('Position error','Estimated RMS spread');
xlabel('Time (s)'); ylabel('Distance (m)'); grid on; title('Localization quality');
nexttile; plot(time,rssi,'.'); xlabel('Time (s)'); ylabel('RSSI (dBm)'); grid on;
nexttile; stairs(time,double(available)); ylim([-0.1 1.1]); xlabel('Time (s)'); ylabel('RFID available'); grid on;
nexttile; plot(time,velocity); yline(logger.maxVelocity,':'); legend('Reader','Command','Limit');
xlabel('Time (s)'); ylabel('Speed (m/s)'); grid on;
nexttile; plot(time,turn); yline(logger.maxTurnRate,':'); yline(-logger.maxTurnRate,':'); legend('Reader','Command');
xlabel('Time (s)'); ylabel('Turn rate (rad/s)'); grid on;
if ~isempty(filename)
    controls.Utils.ensureParent(filename); exportgraphics(figureHandle,filename,'Resolution',160); close(figureHandle);
end
end
