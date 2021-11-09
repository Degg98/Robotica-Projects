x = out.q_simout.Data(1,:);
y = out.q_simout.Data(2,:);
th = out.q_simout.Data(3,:);

plot(x,y, "r-", "LineWidth", 3);
grid on;
title("Unicycle Position");
xlabel("x");
ylabel("y");
