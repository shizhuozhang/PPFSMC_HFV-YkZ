clear all;
close all;

My_PI = 3.14159;
Rad2Deg = 1800 / My_PI; 

%% odr1
% load SMCData3_odr1.mat
% load PPFData_odr1.mat;
% 
% %  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% % PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量
% T = PPFDataRec(:,1); 
% Eror_alp = PPFDataRec(:,2);  
% ErorV = PPFDataRec(:,3); rhot = PPFDataRec(:,7); 
% 
% % SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% % Ero_alp = SMCDataRec3(:,2);   
% sigma_alp= SMCDataRec3(:,8);   
% q = SMCDataRec3(:,14);   Ero_qw = SMCDataRec3(:,17);   sigma_q= SMCDataRec3(:,23); 
% Dlt_e = SMCDataRec3(:,29);
% 
% figure(1);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,900,400]);
% 
% subplot(1,2,1)
% plot(T,Eror_alp*Rad2Deg,'-r','LineWidth',2); 
% hold on
% grid on
% plot(T,-1*rhot*Rad2Deg,'-.k','LineWidth',2);
% plot(T,0.5*rhot*Rad2Deg,'-.k','LineWidth',2);
% axis([0 10 -6 3])
% xlabel('Time (s)','FontSize',13);
% ylabel('Error-alpha (deg)','FontSize',13);
% % title('Error','FontSize',13);
% set(gca,'FontSize',13);
% 
% % subplot(1,2,2)
% % plot(T,Dlt_e,'-r','LineWidth',2); 
% % axis([0 10 -20 20])
% % xlabel('Time (s)','FontSize',13);
% % ylabel('Delta-e (deg)','FontSize',13);
% % title('Dlt-e','FontSize',13);
% % set(gca,'FontSize',13);
% %% odr2
% load SMCData3_odr2.mat
% load PPFData_odr2.mat;
% 
% %  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% % PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量
% T = PPFDataRec(:,1); 
% Eror_alp = PPFDataRec(:,2);  
% ErorV = PPFDataRec(:,3); rhot = PPFDataRec(:,7); 
% 
% % SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% % Ero_alp = SMCDataRec3(:,2);   
% sigma_alp= SMCDataRec3(:,8);   
% q = SMCDataRec3(:,14);   Ero_qw = SMCDataRec3(:,17);   sigma_q= SMCDataRec3(:,23); 
% Dlt_e = SMCDataRec3(:,29);
% 
% % figure(1);
% % set(gcf,'windowstyle','normal');
% % set(gcf,'position',[550,100,1000,400]);
% 
% subplot(1,2,2)
% plot(T,Eror_alp*Rad2Deg,'-r','LineWidth',2); 
% hold on
% grid on
% plot(T,-1*rhot*Rad2Deg,'-.k','LineWidth',2);
% plot(T,0.5*rhot*Rad2Deg,'-.k','LineWidth',2);
% axis([0 8 -6 6])
% xlabel('Time (s)','FontSize',13);
% ylabel('Error-alpha (deg)','FontSize',13);
% % title('Error','FontSize',13);
% set(gca,'FontSize',13);

% subplot(1,2,2)
% plot(T,Dlt_e,'-r','LineWidth',2); 
% axis([0 10 -20.2 20.2])
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e','FontSize',13);
% set(gca,'FontSize',13);

%% nov1
load SMCData3_nov1-1.mat
load PPFData_nov1-1.mat;

%  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量
T = PPFDataRec(:,1); 
Eror_alp = PPFDataRec(:,2);  
ErorV = PPFDataRec(:,3); 
PPF_L = PPFDataRec(:,7);  PPF_U = PPFDataRec(:,8);  

% SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% Ero_alp = SMCDataRec3(:,2);   
sigma_alp= SMCDataRec3(:,8);   
q = SMCDataRec3(:,14);   Ero_qw = SMCDataRec3(:,17);   sigma_q= SMCDataRec3(:,23); 
Dlt_e = SMCDataRec3(:,29);

figure(1);
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,900,400]);

subplot(1,2,1)
plot(T,Eror_alp*Rad2Deg,'-r','LineWidth',2); 
hold on
grid on
plot(T,PPF_L*Rad2Deg,'-.k','LineWidth',2);
plot(T,PPF_U*Rad2Deg,'-.k','LineWidth',2);
axis([0 10 -4 3])
xlabel('Time (s)','FontSize',13);
ylabel('Error-alpha (deg)','FontSize',13);
% title('Error','FontSize',13);
set(gca,'FontSize',13);
% 
% subplot(1,2,2)
% plot(T,Dlt_e,'-r','LineWidth',2); 
% axis([0 10 -20.2 20.2])
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e','FontSize',13);
% set(gca,'FontSize',13);

%% nov1
load SMCData3_nov2.mat
load PPFData_nov2.mat;

%  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量
T = PPFDataRec(:,1); 
Eror_alp = PPFDataRec(:,2);  
ErorV = PPFDataRec(:,3); 
PPF_L = PPFDataRec(:,7);  PPF_U = PPFDataRec(:,8);  

% SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% Ero_alp = SMCDataRec3(:,2);   
sigma_alp= SMCDataRec3(:,8);   
q = SMCDataRec3(:,14);   Ero_qw = SMCDataRec3(:,17);   sigma_q= SMCDataRec3(:,23); 
Dlt_e = SMCDataRec3(:,29);

% figure(1);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,1000,400]);

subplot(1,2,2)
plot(T,Eror_alp*Rad2Deg,'-r','LineWidth',2); 
hold on
grid on
plot(T,PPF_L*Rad2Deg,'-.k','LineWidth',2);
plot(T,PPF_U*Rad2Deg,'-.k','LineWidth',2);
axis([0 10 -4 5])
xlabel('Time (s)','FontSize',13);
ylabel('Error-alpha (deg)','FontSize',13);
% title('Error','FontSize',13);
set(gca,'FontSize',13);
% 
% subplot(1,2,2)
% plot(T,Dlt_e,'-r','LineWidth',2); 
% axis([0 10 -20.2 20.2])
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e','FontSize',13);
% set(gca,'FontSize',13);