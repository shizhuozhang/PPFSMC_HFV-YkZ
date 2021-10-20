clear all;
close all;
load SMCData1.mat
My_PI = 3.14159;
Rad2Deg = 180 / My_PI; 

% SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];

T = SMCDataRec1(:,1); 
Ero_alp = SMCDataRec1(:,2);   Ero_bta = SMCDataRec1(:,3);  Ero_mu = SMCDataRec1(:,4);   %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
sumEro_alp = SMCDataRec1(:,5);   sumEro_bta = SMCDataRec1(:,6);  sumEro_mu = SMCDataRec1(:,7);   
sigma_alp= SMCDataRec1(:,8);   sigma_bta= SMCDataRec1(:,9);sigma_mu= SMCDataRec1(:,10);
dCmd_alp = SMCDataRec1(:,11);  dCmd_bet = SMCDataRec1(:,12);    dCmd_mu = SMCDataRec1(:,13); %构成指令矢量 CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
q = SMCDataRec1(:,14);   r = SMCDataRec1(:,15);   p = SMCDataRec1(:,16); %构成角速率矢量 Omega = [q ,r, p];

Ero_qw = SMCDataRec1(:,17);   Ero_rw = SMCDataRec1(:,18);  Ero_pw = SMCDataRec1(:,19);  %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
sumEro_qw = SMCDataRec1(:,20);   sumEro_rw = SMCDataRec1(:,21);  sumEro_pw = SMCDataRec1(:,22);
sigma_q= SMCDataRec1(:,23);   sigma_r= SMCDataRec1(:,24);sigma_p= SMCDataRec1(:,25);
dComOmg_q = SMCDataRec1(:,26);   dComOmg_r = SMCDataRec1(:,27);   dComOmg_p = SMCDataRec1(:,28); 
Dlt_e = SMCDataRec1(:,29);  Dlt_a = SMCDataRec1(:,30);    Dlt_r = SMCDataRec1(:,31); %构成实际舵偏矢量 DeltaA = [Dlt_a ,Dlt_e, Dlt_r];
dst_a = SMCDataRec1(:,32);  dst_b = SMCDataRec1(:,33);    dst_m = SMCDataRec1(:,34);

%% 分析
 
 figure(1);
 plot(T,Ero_alp*Rad2Deg,'-r','LineWidth',2); 
 hold on
 grid on
 plot(T,sumEro_alp,'-b','LineWidth',2);
%  
%  figure(2);
%  plot(T,sigma_alp,'-r','LineWidth',2); 
%  figure(3);
%  plot(T,dCmd_alp,'-r','LineWidth',2); 
% %  figure(4);
% %  plot(T,dst_a,'-r','LineWidth',2); 
%  
%  figure(5);
%  plot(T,q*Rad2Deg,'-r','LineWidth',2); 
%  hold on
%  grid on
%  plot(T,dComOmg_q,'-b','LineWidth',2); 
%  
%  figure(6);
%  plot(T,Ero_qw,'-r','LineWidth',2); 
%  hold on
%  grid on
%  plot(T,sumEro_qw,'-b','LineWidth',2); 
%  
%  figure(7);
%  plot(T,sigma_q,'-r','LineWidth',2); 
 figure(8);
 plot(T,Dlt_e,'-r','LineWidth',2); 
 

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
