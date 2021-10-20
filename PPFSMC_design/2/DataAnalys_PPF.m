clear all;
close all;
load PPFData.mat;
My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

%  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量
T = PPFDataRec(:,1); 
Eror_alp = PPFDataRec(:,2);  
ErorV = PPFDataRec(:,3); 
Td1 = PPFDataRec(:,4); Tc1 = PPFDataRec(:,5); 
dErorV = PPFDataRec(:,6);  
rhot = PPFDataRec(:,7);  xt = PPFDataRec(:,8);  
flag_t = PPFDataRec(:,9);   flag_e = PPFDataRec(:,10); %构成欧拉角矢量 EulerAng = [theta ,psi, phi];
t_cvg = PPFDataRec(:,11); 
L = PPFDataRec(:,12);  
q = PPFDataRec(:,13);  

x = length(L);
U = zeros(x,1);
for i=1:x
    if L(i)==1
        U(i)=0.5;
    else
        U(i)=1;
    end
end


figure(1);
plot(T,Eror_alp*Rad2Deg,'-r','LineWidth',2); 
hold on
grid on
plot(T,0.5*rhot*Rad2Deg,'-.b','LineWidth',2);
plot(T,-U.*rhot*Rad2Deg,'-.b','LineWidth',2);

 
% figure(2);
% plot(T,flag_t,'-r','LineWidth',2);  
% figure(3);
% plot(T,flag_e,'-r','LineWidth',2); 
% figure(4);
% plot(T,t_cvg,'-r','LineWidth',2); 
figure(5);
% plot(T,Td1,'-r','LineWidth',2); 
 plot(T,ErorV,'-b','LineWidth',2);    
% figure(6);
% plot(T,xt,'-r','LineWidth',2);  
% % 
figure(7);
plot(T,L,'-r','LineWidth',2);
% % 
% % figure(8);
% % plot(T,dErorV,'-r','LineWidth',2); 
% 
% figure(9);
% plot(T,q*Rad2Deg,'-r','LineWidth',2); 

%   
%  figure(10);
%  plot(T,Dlt_alp,'-r','LineWidth',2); 
%  figure(11);
%  plot(T,Dlt_bet,'-r','LineWidth',2);  
%  figure(12);
%  plot(T,Dlt_mu,'-r','LineWidth',2);