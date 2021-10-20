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
function [Eror_ang, Eror_omg, Delta,SMCData] = Ctrller_SMC_Db(AfnPara, CmdAng, AeroAng, Omega)

global step;     %%��¼���沽��,�����������
global h_step;   %%���沽��
%�¶����ȫ�ֱ�����Ϊ�˼�¼�м�ֵ���������
global EAngSum; %�Ƕȿ�������ۼ� ��ͨ������
global EOmgSum; %���ٶȿ�������ۼ� ��ͨ������
global CmdAng_lst;  %��¼�ϴνǶȣ�������ȡ�仯��
global Cmdomg_lst;  %��¼�ϴν��ٶȣ�������ȡ�仯��

if step==1
    EAngSum = zeros(3,1);
    EOmgSum = zeros(3,1);
    CmdAng_lst = zeros(3,1); %��ʼ������
    Cmdomg_lst = zeros(3,1); %��ʼ������
end

%ģ�Ͳ�����ֵ
%%%% AfnPara = [gs,gf, ff, dst_F, dst_Mx];%%3*(3+3+1+1+1)=3*9
 gs = AfnPara(:,1:3);
gf = AfnPara(:,4:6);
ff = AfnPara(:,7);
 dst_F = AfnPara(:,8);
dst_M = AfnPara(:,9);
% dgs = AfnPara(:,10:12);

%% �����ڵĲ���
Dstb_est_A = dst_F;%���Ƶĸ��Ž�
Int_limt1 = 3; %�����޷���С
c_ang = [0.2; 0.2; 0.2];  %�Ƕȿ��ƵĻ��ֻ�ģ��������
k1_ang = [0.1; 0.1; 0.1];  %�����ɲ���
k2_ang = [0.3; 0.3; 0.3];

% c_ang = [1; 1; 1];  %�Ƕȿ��ƵĻ��ֻ�ģ��������
% k1_ang = [0.1; 0.1; 0.1];  %�����ɲ���
% k2_ang = [0.5; 0.5; 0.5];

Dstb_est_O = dst_M ;%���Ƶĸ��Ž�
Int_limt2 = 5; %�����޷���С
c_omg = [1; 1; 1];  %���ٶȿ��ƵĻ��ֻ�ģ�������� ���ò�����20
k1_omg = [0.5; 0.5; 0.5];  %�����ɲ��� ���ò�����10
k2_omg = [1.5; 1.5; 1.5];      %���ò�����5


%% ֱ��ʹ��ʸ��������ͨ��
                    %%%%%%%%%%% �⻷���Ƕȿ���%%%%%%%%%%%
Eror_ang = (AeroAng' - CmdAng');  %�Ƕȿ�����ע�����ﶨ����� ʵ�ʽǶȼ�����ָ���  
EAngSum = EAngSum + Eror_ang; %�����ֲ��޷�

if abs(EAngSum(1)) > Int_limt1
    EAngSum(1) = sign(EAngSum(1))*Int_limt1;
end
if abs(EAngSum(2)) > Int_limt1
    EAngSum(2) = sign(EAngSum(2))*Int_limt1;
end
if abs(EAngSum(3)) > Int_limt1
    EAngSum(3) = sign(EAngSum(3))*Int_limt1;
end

sigma_ang = Eror_ang + c_ang.*EAngSum; %%�Ƕȿ��ƵĻ�ģ����    ע������ʹ��������
dComAng = (CmdAng' - CmdAng_lst)./h_step; %ע������ʹ��������
CmdAng_lst = CmdAng'; %��¼ָ���

%������  
 Omg_v = gs \( dComAng - c_ang.*Eror_ang - k1_ang.*tanh(sigma_ang/0.8) - k2_ang.*sigma_ang - Dstb_est_A); % �⻷�����ɣ���������������    

%%%�����⻷���Խ���Ϊ���Կ��ƣ���˿�ֱ��������ı����������п��ƣ��ڻ���Ȼ�ǻ�ģ����
%%%%��ֱ�����⻷���棬�����ڻ��ȶ�
% sigma_ang = Omg_v;
% Eror_ang = (AeroAng' - CmdAng');  %�Ƕȿ�����ע�����ﶨ����� ʵ�ʽǶȼ�����ָ��� 
gain = [-40;40;-40];%��Ծ����ʱ���ϵ��ҪСһЩ
% gain = [-2;2;-2];%����ָ��ʱ���ϵ�����Դ�һЩ
Omg_v1 = gain .* Eror_ang;
% Omg_v(2,1) = Omg_v1(2,1);  Omg_v(3,1) = Omg_v1(3,1); %%���Ը���ͨ���û�ģ����������ͨ��ֱ�����淴�����ƣ��Ƿ����
                    %%%%%%%%%%% �ڻ����ٶȿ��� %%%%%%%%%%%
%  CmdOmg = Omg_v; %%�⻷������ڻ�����                    
CmdOmg = Omg_v1; %%�⻷������ڻ�����

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
%Uc = gf \(dComOmg - ff - c_omg*Eror_omg - Dstb_est_O - k1_omg*sign(sigma_omg)); % �⻷�����ɣ���������������

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

SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc',Dstb_est_A'];
% SMCData= [sigma_ang', sigma_omg'];
end 
