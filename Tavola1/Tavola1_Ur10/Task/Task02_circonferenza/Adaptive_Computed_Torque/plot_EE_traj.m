%% PLOT TRAIETTORIA EE
posEE = [];
for i=1:length(out.posEE.signals.values)
    posEE(:,i) = out.posEE.signals.values(:,1,i);
end
plot3(posEE(1,:), posEE(2,:), posEE(3,:))
