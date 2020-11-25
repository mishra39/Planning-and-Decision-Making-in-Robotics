gamma = 25;
delta = 1;
armDOF = 5;
graph_size = 1:1:50000;
rad = [];
for i=1:50000
    radius = ((gamma/delta)*(log(i)/ i))^(1/armDOF);
    rad = [rad;radius];
end
plot(rad)