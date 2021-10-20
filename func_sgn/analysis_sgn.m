% analysis_sgn
% clear all;
% close all;
%% task 1
% a1 = 1;
% a2 = 0.5;
% a3 = 0.3;
% x = -3:0.1:3;
% y1 = tanh(x/a1);
% y2 = tanh(x/a2);
% y3 = tanh(x/a3);
% 
% figure(1)
% plot(x,y1,'-k','LineWidth',2); 
% hold on;
% plot(x,y2,'-.r','LineWidth',2);
% plot(x,y3,'-.b','LineWidth',2);
% grid on;
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,450]);
% set(gca,'FontSize',13);
% xlabel('x ','FontSize',13);
% ylabel('y','FontSize',13);
% h1=legend('y-a=1','y-a=0.5','y-a=0.3','Location','SouthEast');
% set(h1,'box','off');

%% task 2
% k = 0.5;
% a = 0.3;
% x = -2:0.1:2;
% n = length(x);
% y1 = zeros(1,n);
% y2 = zeros(1,n);
% y3 = zeros(1,n);
% 
% for i = 1:n
%     y1(1,i) = sign(x(i));
%     
%     if x(i) > k
%         y2(1,i) = 1;
%     elseif x(i) < -k
%         y2(1,i) = -1;
%     else
%         y2(1,i) = x(i)/k;
%     end
%     
%     y3(1,i) = tanh(x(i)/a);
% end
% 
% 
% figure(1)
% plot(x,y1,'-k','LineWidth',2); 
% hold on;
% plot(x,y2,'-.r','LineWidth',2);
% plot(x,y3,'-.b','LineWidth',2);
% grid on;
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,450]);
% set(gca,'FontSize',13);
% xlabel('x ','FontSize',13);
% ylabel('y','FontSize',13);
% h1=legend('sign','sat','tanh','Location','SouthEast');
% set(h1,'box','off');

%% task 3

a = 1.5;
FcL0 = 2;
lambda = 3;                      
zeta = 1.2;
BiasL = zeta/lambda;

FcU0 = 6;
eta = 2;
xi = 0.8;
BiasU = xi/eta;

t = 0:0.1:3;

e = 0.5; %%5,1,0.5

PPFunc_L = -tanh(e/a)*( (FcL0 - BiasL)*exp(-lambda*t) + BiasL );
PPFunc_U = tanh(e/a)*( (FcU0 - BiasU)*exp(-eta*t) + BiasU );

figure(1)
plot(t,PPFunc_L,':b','LineWidth',2); 
hold on;
plot(t,PPFunc_U,':r','LineWidth',2);
% plot(x,y3,'-.b','LineWidth',2);
grid on;
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,450]);
set(gca,'FontSize',13);
xlabel('t (sec) ','FontSize',13);
ylabel('e','FontSize',13);
h1=legend('L_{e0=5}','U_{e0=5}','L_{e0=1}','U_{e0=1}','L_{e0=0.5}','U_{e0=0.5}','Location','NorthEast');
set(h1,'box','off');
%% task4
% a = 0.6;
% FcL0 = 3;
% lambda = 3;
% zeta = 0.9;
% BiasL = zeta/lambda;
% 
% FcU0 = 10;
% eta = 2;
% xi = 0.6;
% BiasU = xi/eta;
% 
% t = 0:0.1:3;
% n = length(t);
% e = -2:0.1:2;
% m = length(e);
% 
% PPFunc_L = zeros(n,m);
% PPFunc_U = zeros(n,m);
% for i=1:n  
%     for j= 1:m
%         PPFunc_L(i,j) = -tanh(e(j)/a)*( (FcL0 - BiasL)*exp(-lambda*t(i)) + BiasL );
%         PPFunc_U(i,j) = tanh(e(j)/a)*( (FcU0 - BiasU)*exp(-eta*t(i)) + BiasU );
%     end    
% end
% 
% figure(1)
% mesh(e,t,PPFunc_L);
% grid on;
% hold on;
% mesh(e,t,PPFunc_U);
% xlabel('e','FontSize',13);
% ylabel('t','FontSize',13);
% zlabel('h','FontSize',13);
% saveas(gcf,'LDratio.png');
% saveas(gcf,'LDratio.fig');

% figure(1)
% plot(t,PPFunc_L,'--k','LineWidth',2); 
% hold on;
% plot(t,PPFunc_U,'--r','LineWidth',2);
% % plot(x,y3,'-.b','LineWidth',2);
% grid on;
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,450]);
% set(gca,'FontSize',13);
% xlabel('t (sec) ','FontSize',13);
% ylabel('e','FontSize',13);
% h1=legend('PPFunc_L','PPFunc_U','Location','SouthEast');
% set(h1,'box','off');