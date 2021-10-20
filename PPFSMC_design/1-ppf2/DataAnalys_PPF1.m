clear all;
close all;
load PPFData_ppfn2.mat;
My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

%  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% PPFData = [ErorV, Td1, Tc1, dErorV, PPF_L, PPF_U, gn, flag_t, flag_e, t_cvg ];%1*10的向量
T = PPFDataRec(:,1); 
Eror_alp = PPFDataRec(:,2);  
ErorV = PPFDataRec(:,3); 
Td1 = PPFDataRec(:,4); Tc1 = PPFDataRec(:,5); 
dErorV = PPFDataRec(:,6);  
PPF_L = PPFDataRec(:,7);  PPF_U = PPFDataRec(:,8);  
gn = PPFDataRec(:,9);   flag_t = PPFDataRec(:,10); %构成欧拉角矢量 EulerAng = [theta ,psi, phi];
flag_e = PPFDataRec(:,11); 
ErorV0 = PPFDataRec(:,12);  
q = PPFDataRec(:,13);  

figure(1);
plot(T,Eror_alp*Rad2Deg,'-r','LineWidth',2); 
hold on
grid on
plot(T,PPF_L*Rad2Deg,'-.k','LineWidth',2);
plot(T,PPF_U*Rad2Deg,'-.k','LineWidth',2);
 
figure(2);
 plot(T,ErorV,'-b','LineWidth',2);    
 

figure(6);
plot(T,gn,'-r','LineWidth',2);  

% figure(7);
% plot(T,Td1,'-r','LineWidth',2); 

figure(3);
plot(T,flag_t,'-r','LineWidth',2);  
figure(4);
plot(T,flag_e,'-r','LineWidth',2); 
% figure(5);
% plot(T,t_cvg,'-r','LineWidth',2); 
% % 
figure(8);
plot(T,ErorV0*Rad2Deg,'-r','LineWidth',2); 
% 
figure(9);
plot(T,q*Rad2Deg,'-r','LineWidth',2); 
