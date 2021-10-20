clear all;
close all;

My_PI = 3.14159;
Rad2Deg = 10*180 / My_PI; 

load PPFData.mat;  %%预设性能数据
%  Buf3 = [(Time + h_step), Eror_ang_1(1),PPFData];
% PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量
T = PPFDataRec(:,1); 
% Eror_alp_PPF = PPFDataRec(:,2);  
ErorV = PPFDataRec(:,3); 
Td1 = PPFDataRec(:,4); Tc1 = PPFDataRec(:,5); 
dErorV = PPFDataRec(:,6);  
rhot = PPFDataRec(:,7);  xt = PPFDataRec(:,8);  
flag_t = PPFDataRec(:,9);   flag_e = PPFDataRec(:,10); %构成欧拉角矢量 EulerAng = [theta ,psi, phi];
t_cvg = PPFDataRec(:,11); 
L = PPFDataRec(:,12);  
% q_PPF = PPFDataRec(:,13); 

load SMCData3.mat
Eror_alp_PPF = SMCDataRec3(:,2);  sigma_alp_PPF= SMCDataRec3(:,8);  q_PPF = SMCDataRec3(:,14);  
Ero_qw_PPF = SMCDataRec3(:,17);   sigma_q_PPF= SMCDataRec3(:,23); Dlt_e_PPF = SMCDataRec3(:,29);

load SMCData2.mat  %%单环滑模数据
% SMCData1= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% SMCData2= [Eror_ang',dEror_ang',dComAng', ddComAng', sigma',fw',Dstb', Delta];
Ero_alp_SG = SMCDataRec2(:,2);   Ero_bta_SG = SMCDataRec2(:,3);  Ero_mu_SG = SMCDataRec2(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
sigma_alp_SG = SMCDataRec2(:,14);   sigma_bta_SG = SMCDataRec2(:,15);   sigma_mu_SG = SMCDataRec2(:,16); %构成角速率矢量 Omega = [q ,r, p];
Dlt_e_SG = SMCDataRec2(:,23);  Dlt_a_SG = SMCDataRec2(:,24);    Dlt_r_SG = SMCDataRec2(:,25); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];


load SMCData1.mat   %%双环滑模数据
Ero_alp_DB = SMCDataRec1(:,2);   Ero_bta_DB = SMCDataRec1(:,3);  Ero_mu_DB = SMCDataRec1(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
sigma_alp_DB= SMCDataRec1(:,8);   sigma_bta_DB= SMCDataRec1(:,9);sigma_mu_DB= SMCDataRec1(:,10);
q_DB = SMCDataRec1(:,14);   r_DB = SMCDataRec1(:,15);   p_DB = SMCDataRec1(:,16); %构成角速率矢量 Omega = [q ,r, p];
Ero_qw_DB = SMCDataRec1(:,17);   Ero_rw_DB = SMCDataRec1(:,18);  Ero_pw_DB = SMCDataRec1(:,19);  %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
sigma_q_DB= SMCDataRec1(:,23);   sigma_r_DB= SMCDataRec1(:,24);sigma_p_DB= SMCDataRec1(:,25);
Dlt_e_DB = SMCDataRec1(:,29);  Dlt_a_DB = SMCDataRec1(:,30);    Dlt_r_DB = SMCDataRec1(:,31); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];

% load SMCData1_gain2.mat  %%外环增益反馈数据
% % SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% Ero_alp_GA = SMCDataRec1(:,2);   Ero_bta_GA = SMCDataRec1(:,3);  Ero_mu_GA = SMCDataRec1(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
% sigma_alp_GA= SMCDataRec1(:,8);   sigma_bta_GA= SMCDataRec1(:,9);sigma_mu_GA= SMCDataRec1(:,10);
% q_GA = SMCDataRec1(:,14);   r_GA = SMCDataRec1(:,15);   p_GA = SMCDataRec1(:,16); %构成角速率矢量 Omega = [q ,r, p];
% 
% Ero_qw_GA = SMCDataRec1(:,17);   Ero_rw_GA = SMCDataRec1(:,18);  Ero_pw_GA = SMCDataRec1(:,19);  %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
% sigma_q_GA= SMCDataRec1(:,23);   sigma_r_GA= SMCDataRec1(:,24);sigma_p_GA= SMCDataRec1(:,25);
% Dlt_e_GA = SMCDataRec1(:,29);  Dlt_a_GA = SMCDataRec1(:,30);    Dlt_r_GA = SMCDataRec1(:,31); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];

%% 不同方法的俯仰通道误差曲线对比
figure(1);
plot(T,Eror_alp_PPF*Rad2Deg,'-r','LineWidth',2); 
hold on
grid on
plot(T,Ero_alp_SG*Rad2Deg,':m','LineWidth',2); 
plot(T,Ero_alp_DB*Rad2Deg,'--b','LineWidth',2); 
%plot(T,Ero_alp_GA*Rad2Deg,'--m','LineWidth',2);

plot(T,0.5*rhot*Rad2Deg,'-.k','LineWidth',2);
plot(T,-1*rhot*Rad2Deg,'-.k','LineWidth',2);
xlabel('Time (s)','FontSize',13);
ylabel('Error_{\alpha} (deg)','FontSize',13);
axis([0 10 -3 1.5])
h1=legend('PPSMC','SSMC','DSMC','Location','SouthEast');
set(h1,'box','off');
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,800,500]);
set(gca,'FontSize',13);

figure(2);
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,1000,400]);

subplot(1,3,1)
plot(T,Dlt_e_PPF,'-r','LineWidth',2); 
axis([0 10 -20 20])
xlabel('Time (s)','FontSize',13);
ylabel('Delta-e (deg)','FontSize',13);
title('Dlt-e-PPSMC','FontSize',13);
set(gca,'FontSize',13);
subplot(1,3,2)
plot(T,Dlt_e_SG,':m','LineWidth',2); 
axis([0 10 -20 20])
xlabel('Time (s)','FontSize',13);
ylabel('Delta-e (deg)','FontSize',13);
title('Dlt-e-SSMC','FontSize',13);
set(gca,'FontSize',13);

subplot(1,3,3)
plot(T,Dlt_e_DB,'--b','LineWidth',2); 
axis([0 10 -20 20])
xlabel('Time (s)','FontSize',13);
ylabel('Delta-e (deg)','FontSize',13);
title('Dlt-e-DSMC','FontSize',13);
set(gca,'FontSize',13);

figure(3);
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,600,400]);
set(gca,'FontSize',13);
plot(T,ErorV,'-.r','LineWidth',2);
grid on
xlabel('Time (s)','FontSize',13);
ylabel('ErrorV (deg)','FontSize',13);
axis([0 10 -2 1])

% figure(4);
% set(gcf,'windowstyle','normal');
% set(gcf,'position',[550,100,600,400]);
% set(gca,'FontSize',13);
% plot(T,sigma_alp_PPF,'-.r','LineWidth',2);
% grid on
% xlabel('Time (s)','FontSize',13);
% ylabel('sigma-alp (deg)','FontSize',13);
% axis([0 10 -2 1])
