init;

u = load('uSol.txt');
x = load('xSol.txt');
tU = (1:length(u))*0.1;
tX = (0:length(x)-1)*0.1;

figure; 
plotI(tU,u,'-');
xlimI(tU);
ylimI(u);
titleI('Ação de controle');
xlabelI('$t$ ($s$)');
ylabelI('$u$');
cropPlotI;
printI('u');

figure; 
plotI(tX,x,'-');
xlimI(tX);
ylimI(x);
titleI('Estados');
xlabelI('$t$ ($s$)');
ylabelI('$x$');

lgd = {''};
for i = 1:size(x,2)
    lgd(i) = {['$x_', num2str(i),'$']};
end

legendI(lgd,2);
cropPlotI;
printI('x');