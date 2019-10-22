init; 

t_sim = 5;
N_min = 2;
N_max = 20;
n = 100;
N_values = round(linspace(N_min, N_max, n));
x_values = zeros(1,n);

for i = 1:n
    x = load(strcat('xSol_',num2str(N_values(i)),'.txt'));
    x_values(i) = x(end,3);
end

plotI(N_values,x_values,'-o');
xlimI(N_values);
ylimI(x_values);
xlabelI('$N$');
ylabelI('$x_3(t_f)$');
cropPlotI;
axisComma;
printI('xN');

N_show = 8;
x_show = load(strcat('xSol_',num2str(N_show),'.txt'));
u_show = load(strcat('uSol_',num2str(N_show),'.txt'));
dt = t_sim/N_show;
time_u = linspace(0,t_sim,N_show-1);
time_x = linspace(0,t_sim,N_show);

figure;
plotI(time_u, u_show,'o'); hold on;
plotIz(time_u, u_show,{'-','b'});
xlabelI('$t$ ($s$)');
ylabelI('$u$');
cropPlotI;
axisComma;
printI('u');

figure;
plotI(time_x, x_show, '-o'); 
xlabelI('$t$ ($s$)');
ylabelI('$x$');
cropPlotI;
axisComma;
printI('x');