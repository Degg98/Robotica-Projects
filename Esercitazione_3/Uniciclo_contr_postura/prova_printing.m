x = out.q_simout.Data(1,:);
y = out.q_simout.Data(2,:);
th = out.q_simout.Data(3,:);

plot(x(1),y(1), "bo-", "LineWidth", 2);
lx = [x(1) x(1)+0.2*cos(th(1))];
ly = [y(1) y(1)+0.2*sin(th(1))];
line(lx,ly);
hold on
for v = 2:720
    plot(x(v),y(v), "ro-", "LineWidth", 1);
    lx = [x(v) x(v)+0.2*cos(th(v))];
    ly = [y(v) y(v)+0.2*sin(th(v))];
    line(lx,ly);
    grid on;
    title("Unicycle Position");
    xlabel("x");
    ylabel("y");
    pause(0.00000000002);
end
plot(x(721),y(721), "go-", "LineWidth", 2);
hold off;