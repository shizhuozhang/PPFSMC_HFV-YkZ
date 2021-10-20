clear all;
close all;
load SMCData3.mat
My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

% SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% SMCDataRec3=[];
T = SMCDataRec3(:,1); 
Ero_alp = SMCDataRec3(:,2);   Ero_bta = SMCDataRec3(:,3);  Ero_mu = SMCDataRec3(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
sumEro_alp = SMCDataRec3(:,5);   sumEro_bta = SMCDataRec3(:,6);  sumEro_mu = SMCDataRec3(:,7);   
sigma_alp= SMCDataRec3(:,8);   sigma_bta= SMCDataRec3(:,9);sigma_mu= SMCDataRec3(:,10);
dCmd_alp = SMCDataRec3(:,11);  dCmd_bet = SMCDataRec3(:,12);    dCmd_mu = SMCDataRec3(:,13); %构成指令矢量 CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
q = SMCDataRec3(:,14);   r = SMCDataRec3(:,15);   p = SMCDataRec3(:,16); %构成角速率矢量 Omega = [q ,r, p];

Ero_qw = SMCDataRec3(:,17);   Ero_rw = SMCDataRec3(:,18);  Ero_pw = SMCDataRec3(:,19);  %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
sumEro_qw = SMCDataRec3(:,20);   sumEro_rw = SMCDataRec3(:,21);  sumEro_pw = SMCDataRec3(:,22);
sigma_q= SMCDataRec3(:,23);   sigma_r= SMCDataRec3(:,24);sigma_p= SMCDataRec3(:,25);
dComOmg_q = SMCDataRec3(:,26);   dComOmg_r = SMCDataRec3(:,27);   dComOmg_p = SMCDataRec3(:,28); 
Dlt_e = SMCDataRec3(:,29);  Dlt_a = SMCDataRec3(:,30);    Dlt_r = SMCDataRec3(:,31); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];

%% 分析
 
 figure(1);
 plot(T,Ero_alp*Rad2Deg,'-r','LineWidth',2); 
 hold on
 grid on
 plot(T,sumEro_alp,'-b','LineWidth',2);
 
 figure(2);
 plot(T,sigma_alp,'-r','LineWidth',2); 
 figure(3);
 plot(T,dCmd_alp,'-r','LineWidth',2); 
%  figure(4);
%  plot(T,dst_a,'-r','LineWidth',2); 
 
 figure(5);
 plot(T,q*Rad2Deg,'-r','LineWidth',2); 
 hold on
 grid on
 plot(T,dComOmg_q,'-b','LineWidth',2); 
 
 figure(6);
 plot(T,Ero_qw,'-r','LineWidth',2); 
 hold on
 grid on
 plot(T,sumEro_qw,'-b','LineWidth',2); 
 
 figure(7);
 plot(T,sigma_q,'-r','LineWidth',2); 
 figure(8);
 plot(T,Dlt_e,'-r','LineWidth',2); 