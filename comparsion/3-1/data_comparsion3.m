clear all;
close all;

My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

load PPFData.mat;  %%预设性能数据
T = PPFDataRec(:,1); 
% Eror_alp_PPF = PPFDataRec(:,2);  
ErorV = PPFDataRec(:,3); 
dErorV = PPFDataRec(:,6);  
PPF_L = PPFDataRec(:,7);  PPF_U = PPFDataRec(:,8);   
load SMCData3.mat
Eror_alp_PPF = SMCDataRec3(:,2);  sigma_alp_PPF= SMCDataRec3(:,8);  q_PPF = SMCDataRec3(:,14);  
Eror_qw_PPF = SMCDataRec3(:,17);   sigma_q_PPF= SMCDataRec3(:,23); Dlt_e_PPF = SMCDataRec3(:,29);

load SMCData2.mat  %%单环滑模数据
% SMCData1= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% SMCData2= [Eror_ang',dEror_ang',dComAng', ddComAng', sigma',fw',Dstb', Delta];
Ero_alp_SG = SMCDataRec2(:,2);   Ero_bta_SG = SMCDataRec2(:,3);  Ero_mu_SG = SMCDataRec2(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
sigma_alp_SG = SMCDataRec2(:,14);   sigma_bta_SG = SMCDataRec2(:,15);   sigma_mu_SG = SMCDataRec2(:,16); %构成角速率矢量 Omega = [q ,r, p];
Dlt_e_SG = SMCDataRec2(:,23);  Dlt_a_SG = SMCDataRec2(:,24);    Dlt_r_SG = SMCDataRec2(:,25); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];

load CtrlData_1_0_1.mat;
q_CNC = CtrlData(1:10000,5);  Dlt_e_CNC = CtrlData(1:10000,11);  
Ero_alp_CNC = CtrlData(1:10000,14);  Ero_qw_CNC = CtrlData(1:10000,17);  Dlt_alp_CNC = CtrlData(1:10000,23);

% load FltData2_CNC.mat  
% alpha_CNC = FltData2(:,12);   q_CNC = FltData2(:,15); Dlt_e_CNC = FltData2(:,21);  
% Cmd_alp = FltData2(:,18);
% Ero_alp_CNC = FltData2(:,24);  Ero_qw_CNC = FltData2(:,27); 

%% 不同方法的俯仰通道误差曲线对比
figure(1);
plot(T,Eror_alp_PPF*Rad2Deg,'-r','LineWidth',2); 
hold on
grid on
plot(T,Ero_alp_SG/2*Rad2Deg,':m','LineWidth',2); 
plot(T,Ero_alp_CNC*Rad2Deg,'--b','LineWidth',2); 
plot(T,PPF_L*Rad2Deg,'-.k','LineWidth',2);
plot(T,PPF_U*Rad2Deg,'-.k','LineWidth',2);

xlabel('Time (s)','FontSize',13);
ylabel('Error_{\alpha} (deg)','FontSize',13);
axis([0 50 -0.5 0.8])
h1=legend('NPPSMC','SSMC','CNC','Location','NorthEast');
set(h1,'box','off');
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,650,450]);
set(gca,'FontSize',13);
magnify   %%局部放大镜

figure(2);
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,400]);
plot(T,Dlt_e_PPF,'-r','LineWidth',2); 
axis([0 50 -20 5])
grid on
xlabel('Time (s)','FontSize',13);
ylabel('Delta-e (deg)','FontSize',13);
set(gca,'FontSize',13);


% figure(2);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,1000,400]);
% 
% subplot(1,3,1)
% plot(T,Dlt_e_PPF,'-r','LineWidth',2); 
% axis([0 50 -20 5])
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e-NPPSMC','FontSize',13);
% set(gca,'FontSize',13);
% subplot(1,3,2)
% plot(T,Dlt_e_SG,':m','LineWidth',2); 
% axis([0 50 -20 5])
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e-SSMC','FontSize',13);
% set(gca,'FontSize',13);
% 
% subplot(1,3,3)
% plot(T,Dlt_e_CNC,'--b','LineWidth',2); 
% axis([0 50 -20 5])
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('Delta-e (deg)','FontSize',13);
% title('Dlt-e-CNC','FontSize',13);
% set(gca,'FontSize',13);


% figure(3);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,ErorV,'-.r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('ErrorV (deg)','FontSize',13);
% axis([0 50 -1.5 0.5])
% 
% figure(4);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,Cmd_alp*Rad2Deg,'-.r','LineWidth',2);
% hold on
% grid on
% plot(T,(Cmd_alp + 2*Eror_alp_PPF)*Rad2Deg,':b','LineWidth',2); 
% h1=legend('\alpha_{cmd}','\alpha_{act}','Location','NorthEast');
% set(h1,'box','off');
% xlabel('Time (s)','FontSize',13);
% ylabel('alpha (deg)','FontSize',13);
% axis([0 50 0 20])
% magnify   %%局部放大镜
