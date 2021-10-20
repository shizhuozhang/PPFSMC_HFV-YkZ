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
function [Eror_ang, Delta,SMCData] = Ctrller_SMC_Sg(AfnPara, CmdAng, AeroAng, Omega)

global step;     %%��¼���沽��,�����������
global h_step;   %%���沽��
%�¶����ȫ�ֱ�����Ϊ�˼�¼�м�ֵ���������
global CmdAng_lst;  %��¼�ϴνǶȣ�������ȡ�仯��
global dCmdAng_lst;  %��¼�ϴν��ٶȣ�������ȡ�仯��

if step==1
    CmdAng_lst = zeros(3,1); %��ʼ������
    dCmdAng_lst = zeros(3,1); %��ʼ������
end

%ģ�Ͳ�����ֵ
%%%% AfnPara = [gs,gf, ff, dst_F, dst_Mx];%%3*(3+3+1+1+1)=3*9
gs = AfnPara(:,1:3);
gf = AfnPara(:,4:6);
ff = AfnPara(:,7);
dst_F = AfnPara(:,8);
dst_M = AfnPara(:,9);
dgs = AfnPara(:,10:12);


%% �����ڵĲ���
Dstb_est = [0;0;0];
% c = [5; 5; 5];  %���ٶȿ��ƵĻ��ֻ�ģ�������� ���ò�����20
% k1 = [3; 2; 3];  %�����ɲ��� ���ò�����10
% k2 = [15; 10; 15];      %���ò�����
c = [2; 2; 2];  %���ٶȿ��ƵĻ��ֻ�ģ�������� ���ò�����20
k1 = [1.5; 1.5; 1.5];  %�����ɲ��� ���ò�����10
k2 = [3; 3; 3];      %���ò�����
%% ֱ��ʹ��ʸ��������ͨ��
gx = gs*gf;
fw = gs*ff + dgs*(Omega') + (gs*(Omega')).*c;
Dstb = c.*dst_F + gs*(dst_M + Dstb_est) ;%���Ƶĸ��Ž�

dComAng = (CmdAng' - CmdAng_lst)./h_step; %ע������ʹ��������
ddComAng = (dComAng - dCmdAng_lst)./h_step; %ע������ʹ��������

Eror_ang = (AeroAng' - CmdAng'); 
dEror_ang = gs*(Omega') + dst_F - dComAng;
sigma = c.*Eror_ang + dEror_ang;

%������
Uc =  gx\ (c.*dComAng + ddComAng- fw - Dstb - k1.*tanh(sigma/0.8)- k2.*sigma);% �⻷�����ɣ���������������
%Uc = gf \(dComOmg - ff - c_omg*Eror_omg - Dstb_est_O - k1_omg*sign(sigma_omg)); % �⻷�����ɣ���������������

CmdAng_lst = CmdAng'; %��¼ָ���
dCmdAng_lst = dComAng;
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

SMCData= [Eror_ang',dEror_ang',dComAng', ddComAng', sigma',fw',Dstb', Delta];
% SMCData= [sigma_ang', sigma_omg'];
end 
