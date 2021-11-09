x = out.q_simout.Data(1,:);
y = out.q_simout.Data(2,:);
th = out.q_simout.Data(3,:);

radius = 0.4;

plot(x(1),y(1), "bo-", "LineWidth", 2, "MarkerSize", 18);
c = circle_plot(x(1), y(1), radius);
lx = [x(1) x(1)+0.6*cos(th(1))];
ly = [y(1) y(1)+0.6*sin(th(1))];
l = line(lx,ly, "linewidth", 3);
l1 = line(lx,ly, "linewidth", 3);
hold on;
r = animatedline(x(1),y(1),"Color",'r','LineWidth',3);
%hold off;
for v = 2:721
    addpoints(r,x(v-1),y(v-1));
    drawnow;
    c.Position(1) = x(v)- radius;
    c.Position(2) = y(v) - radius;
    l.XData = [x(v) x(v)+0.6*cos(th(v))];
    l.YData = [y(v) y(v)+0.6*sin(th(v))];
    drawnow;
    grid on;
    title("Unicycle Position");
    xlabel("x");
    ylabel("y");  
    pause(0.0000002);
end
plot(x(721),y(721), "go-", "LineWidth", 2, "MarkerSize", 18);
hold off;