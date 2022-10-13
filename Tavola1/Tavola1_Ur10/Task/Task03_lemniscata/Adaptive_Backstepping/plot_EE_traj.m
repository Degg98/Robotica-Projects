%% PLOT TRAIETTORIA EE
posEE = [];
for i=1:length(out.pos_EE.signals.values)
    posEE(:,i) = out.pos_EE.signals.values(:,1,i);
end
plot3(posEE(1,:), posEE(2,:), posEE(3,:))
