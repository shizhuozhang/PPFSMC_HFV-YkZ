%%��ͨ��ģ������
%˫�����ƣ����룺ָ��ǣ�ʵ�ʽǣ����ٶ�
%������Ƕȿ��������ٶȿ������Ǽ��ٶȣ���Ч��ƫ
% alpha = FltSdt_F(4);   beta = FltSdt_F(5);   mu = FltSdt_F(6); %����������ʸ�� AeroAng = [alpha, beta, mu];
% q = FltSdt_F(7);   r = FltSdt_F(8);   p = FltSdt_F(9); %���ɽ�����ʸ�� Omega = [q ,r, p];
%Cmd_alp = CtrlSdt_B(1);  Cmd_bet = CtrlSdt_B(2);    Cmd_mu = CtrlSdt_B(3); %����ָ��ʸ�� CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
%Ero_pit = CtrlSdt_P(1);  Ero_yaw = CtrlSdt_P(2);    Ero_rol = CtrlSdt_P(3); %���ɿ������ʸ�� Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
%Ero_qw = CtrlSdt_P(4);  Ero_rw = CtrlSdt_P(5);    Ero_pw = CtrlSdt_P(6); %���ɽ��ٶȿ������ʸ�� Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
%Acc_pit = CtrlSdt_P(7);  Acc_yaw = CtrlSdt_P(8);    Acc_rol = CtrlSdt_P(9); %���ɽǼ��ٶ�ʸ�� Accelrt = [Acc_pit ,Acc_yaw, Acc_rol];
%Dlt_alp = CtrlSdt_P(10);  Dlt_bet = CtrlSdt_P(11);    Dlt_mu = CtrlSdt_P(12); %���ɵ�Ч��ƫʸ�� DeltaE = [Dlt_alp ,Dlt_bet, Dlt_mu];
% PPFData = [flag_t, flag_e, t_cvg, rhot, L, U, xt, ErorV, dErorV];
function [ErorA_ang, Eror_omg, Delta, SMCData, PPFData1] = Ctrller_PPFSMC(AfnPara, CmdAng, AeroAng, Omega)

global step;     %%��¼���沽��,�����������
global h_step;   %%���沽��
global Rad2Deg;  %%����ת��Ϊ��
%�¶����ȫ�ֱ�����Ϊ�˼�¼�м�ֵ���������
global eV_alpSum; %�Ƕȿ�������ۼ� ��ͨ������
global CmdAng_lst;  %��¼�ϴνǶȣ�������ȡ�仯��
global EOmgSum; %���ٶȿ�������ۼ� ��ͨ������
global Cmdomg_lst;  %��¼�ϴν��ٶȣ�������ȡ�仯��
global ErorA_lst;
global eV_alp_lst;

if step==1 
    CmdAng_lst = zeros(3,1); %��ʼ������
    EOmgSum = zeros(3,1);
    Cmdomg_lst = zeros(3,1); %��ʼ������
    ErorA_lst = zeros(3,1); %��ʼ������
    eV_alpSum = 0;   
    eV_alp_lst = 0; %��ʼ������
end

%ģ�Ͳ�����ֵ
%%%% AfnPara = [gs,gf, ff, dst_F, dst_Mx];%%3*(3+3+1+1+1)=3*9
gs = AfnPara(:,1:3);
gf = AfnPara(:,4:6);
ff = AfnPara(:,7);
dst_F = AfnPara(:,8);
dst_M = AfnPara(:,9);

%% �����ڵĲ���
Dstb_alp = dst_F(1);%���Ƶĸ��Ž�
Int_limt1 = 10; %�����޷���С
c_alp = 0.01;  %�Ƕȿ��ƵĻ��ֻ�ģ��������
k1_alp = 0.3; k2_alp = 1;  %�����ɲ���

% c_alp = 0.008;  %�Ƕȿ��ƵĻ��ֻ�ģ��������
% k1_alp = 0.1; k2_alp = 1.2;  %�����ɲ���

Dstb_est_O = dst_M ;%���Ƶĸ��Ž�
Int_limt2 = 5; %�����޷���С
c_omg = [1; 1; 1];  %���ٶȿ��ƵĻ��ֻ�ģ�������� ���ò�����20
k1_omg = [0.5; 0.5; 0.5];  %�����ɲ��� ���ò�����10
k2_omg = [1.5; 1.5; 1.5];      %���ò�����5


% PpfPara = [rho0,rho_inf,Mx,lambda];
% PpfPara_alp =[0.3/Rad2Deg, 0.02/Rad2Deg, 0.3, 1];
% PpfPara_alp =[0.3/Rad2Deg, 0.02/Rad2Deg, 0.5, 1];
% PpfPara_alp =[0.4/Rad2Deg, 0.03/Rad2Deg, 0.6, 1];
%PpfPara_alp =[0.6/Rad2Deg, 0.05/Rad2Deg, 0.6, 1];
 PpfPara_alp =[2/Rad2Deg, 0.15/Rad2Deg, 0.6, 1];
%% ֱ��ʹ��ʸ��������ͨ��
                    %%%%%%%%%%% �⻷���Ƕȿ���,�����漰���任����ͨ���ֿ�д�ȽϷ���%%%%%%%%%%%
ErorA_ang = (AeroAng' - CmdAng');  %�Ƕȿ�����ע�����ﶨ����� ʵ�ʽǶȼ�����ָ��� 
dErorA = (ErorA_ang - ErorA_lst)./h_step;
dComAng = (CmdAng' - CmdAng_lst)./h_step; %ע������ʹ��������
% dComalp = dComAng(1);

%Ԥ�����ܵ����任  ֻ������ͨ��    PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10������
% [PPFData1] = TansError1(PpfPara_alp, ErorA_ang(1), dErorA(1), eV_alp_lst);
%[PPFData1] = TansError_N1(PpfPara_alp, ErorA_ang(1), dErorA(1), eV_alp_lst);
 [PPFData1] = TansError_N2(PpfPara_alp, ErorA_ang(1), dErorA(1), eV_alp_lst);
eV_alp = PPFData1(1);    Td1 = PPFData1(2);    Tc1 = PPFData1(3);  

ErorA_lst = ErorA_ang;
CmdAng_lst = CmdAng'; %��¼ָ���
eV_alp_lst = eV_alp;
 
eV_alpSum = eV_alpSum + eV_alp; %�����ֲ��޷�
if abs(eV_alpSum) > Int_limt1
    eV_alpSum = sign(eV_alpSum)*Int_limt1;
end

sigma_eValp = eV_alp + c_alp*eV_alpSum; %%�Ƕȿ��ƵĻ�ģ����    ע������ʹ��������

eV_angSum = [eV_alpSum;0;0];
sigma_ang = [sigma_eValp;0;0];

%������  
 Omg_p = (Td1*dComAng(1) - c_alp*eV_alp - Tc1 - Td1*Dstb_alp - k1_alp*tanh(sigma_eValp/0.8) - k2_alp*sigma_eValp) /Td1/gs(1,1) ; % ��������   
%  Omg_p = -0.001*Td1*eV_alp;

gain = [-0.8;0.8;-0.8];%��Ծ����ʱ���ϵ��ҪСһЩ
Omg_v1 = gain .* ErorA_ang;
Omg_v(1,1) = Omg_p; 
Omg_v(2,1) = Omg_v1(2,1);  Omg_v(3,1) = Omg_v1(3,1); %%����ͨ���û�ģ����������ͨ��ֱ�����淴������

                    %%%%%%%%%%% �ڻ����ٶȿ��� ʹ��ʸ������%%%%%%%%%%%
CmdOmg = Omg_v; %%�⻷������ڻ�����                    
Eror_omg = (Omega' - CmdOmg); %���ٶȿ�����ע�����ﶨ����� ʵ�ʼ�����  
EOmgSum = EOmgSum + Eror_omg; %�����ֲ��޷�

if abs(EOmgSum(1)) > Int_limt2
    EOmgSum(1) = sign(EOmgSum(1))*Int_limt2;
end
if abs(EOmgSum(2)) > Int_limt2
    EOmgSum(2) = sign(EOmgSum(2))*Int_limt2;
end
if abs(EOmgSum(3)) > Int_limt2
    EOmgSum(3) = sign(EOmgSum(3))*Int_limt2;
end  

sigma_omg = Eror_omg + c_omg.*EOmgSum;   %%���ٶȿ��ƵĻ�ģ����    
dComOmg = (CmdOmg - Cmdomg_lst)./h_step;  %

%������
Uc =  gf\ (dComOmg - ff - c_omg.*Eror_omg - Dstb_est_O - k1_omg.*tanh(sigma_omg/0.8)- k2_omg.*sigma_omg);% �⻷�����ɣ���������������

Cmdomg_lst = CmdOmg; %��¼ָ���
%% �õ����յĶ�ƫ  
if abs(Uc(1)) > 20
    Uc(1) = sign(Uc(1))*20;
end
if abs(Uc(2)) > 20
    Uc(2) = sign(Uc(2))*20;
end
if abs(Uc(3)) > 20
    Uc(3) = sign(Uc(3))*20;
end  
Delta = Uc'; %%ת��������

SMCData= [ErorA_ang', eV_angSum', sigma_ang', dComAng',Omg_v',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc'];
% SMCData= [sigma_ang', sigma_omg'];
end 
