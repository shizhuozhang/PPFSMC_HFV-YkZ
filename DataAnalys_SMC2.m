clear all;
close all;
load SMCData2.mat
My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

% SMCData1= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% SMCData2= [Eror_ang',dEror_ang',dComAng', ddComAng', sigma',fw',Dstb', Delta];
T = SMCDataRec2(:,1); 
Ero_alp = SMCDataRec2(:,2);   Ero_bta = SMCDataRec2(:,3);  Ero_mu = SMCDataRec2(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
dEro_alp = SMCDataRec2(:,5);   dEro_bta = SMCDataRec2(:,6);  dEro_mu = SMCDataRec2(:,7);
dCmd_alp = SMCDataRec2(:,8);  dCmd_bet = SMCDataRec2(:,9);    dCmd_mu = SMCDataRec2(:,10); 
ddCmd_alp = SMCDataRec2(:,11);  ddCmd_bet = SMCDataRec2(:,12);    ddCmd_mu = SMCDataRec2(:,13); 
sigma_alp = SMCDataRec2(:,14);   sigma_bta = SMCDataRec2(:,15);   sigma_mu = SMCDataRec2(:,16); %构成角速率矢量 Omega = [q ,r, p];
fw_alp= SMCDataRec2(:,17);   fw_bta= SMCDataRec2(:,18); fw_mu= SMCDataRec2(:,19);
Dstb_alp = SMCDataRec2(:,20);   Dstb_bet = SMCDataRec2(:,21);  Dstb_mu = SMCDataRec2(:,22);
Dlt_e = SMCDataRec2(:,23);  Dlt_a = SMCDataRec2(:,24);    Dlt_r = SMCDataRec2(:,25); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];


%% 分析
 
 figure(1);
 plot(T,Ero_alp*Rad2Deg,'-r','LineWidth',2); 
 hold on
 grid on
 plot(T,dEro_alp,'-b','LineWidth',2);
 
 figure(2);
 plot(T,dCmd_alp,'-r','LineWidth',2); 
 hold on
 grid on
 plot(T,ddCmd_alp,'-b','LineWidth',2); 
 
 figure(3);
 plot(T,sigma_alp,'-r','LineWidth',2); 
%  figure(4);
%  plot(T,fw_alp,'-r','LineWidth',2); 
%  figure(5);
%  plot(T,Dstb_alp,'-r','LineWidth',2); 
 figure(6);
 plot(T,Dlt_e,'-r','LineWidth',2); 
 
% 
%  figure(1);
%  plot(T,Ero_bta*Rad2Deg,'-r','LineWidth',2); 
%  figure(2);
%  plot(T,sumEro_bta*Rad2Deg,'-r','LineWidth',2); 
%  figure(3);
%  plot(T,sigma_bta,'-r','LineWidth',2); 
%  figure(4);
%  plot(T,dCmd_bet,'-r','LineWidth',2); 
%  figure(5);
%  plot(T,r*Rad2Deg,'-r','LineWidth',2); 
%  figure(6);
%  plot(T,Ero_rw,'-r','LineWidth',2); 
%  figure(7);
%  plot(T,sumEro_rw,'-r','LineWidth',2); 
%  figure(8);
%  plot(T,sigma_r,'-r','LineWidth',2); 
%  figure(9);
%  plot(T,dComOmg_r,'-r','LineWidth',2); 
%  figure(10);
%  plot(T,Dlt_r,'-r','LineWidth',2); 
 
%  figure(1);
%  plot(T,Ero_mu*Rad2Deg,'-r','LineWidth',2); 
%  figure(2);
%  plot(T,sumEro_mu*Rad2Deg,'-r','LineWidth',2); 
%  figure(3);
%  plot(T,sigma_mu,'-r','LineWidth',2); 
%  figure(4);
%  plot(T,dCmd_mu,'-r','LineWidth',2); 
%  figure(5);
%  plot(T,p*Rad2Deg,'-r','LineWidth',2); 
%  figure(6);
%  plot(T,Ero_pw,'-r','LineWidth',2); 
%  figure(7);
%  plot(T,sumEro_pw,'-r','LineWidth',2); 
%  figure(8);
%  plot(T,sigma_p,'-r','LineWidth',2); 
%  figure(9);
%  plot(T,dComOmg_p,'-r','LineWidth',2); 
%  figure(10);
%  plot(T,Dlt_a,'-r','LineWidth',2); 
