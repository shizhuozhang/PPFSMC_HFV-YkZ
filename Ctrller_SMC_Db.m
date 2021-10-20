%%普通滑模控制器
%双环控制，输入：指令角，实际角，角速度
%输出：角度控制误差，角速度控制误差，角加速度，等效舵偏
% alpha = FltSdt_F(4);   beta = FltSdt_F(5);   mu = FltSdt_F(6); %构成气动角矢量 AeroAng = [alpha, beta, mu];
% q = FltSdt_F(7);   r = FltSdt_F(8);   p = FltSdt_F(9); %构成角速率矢量 Omega = [q ,r, p];
%Cmd_alp = CtrlSdt_B(1);  Cmd_bet = CtrlSdt_B(2);    Cmd_mu = CtrlSdt_B(3); %构成指令矢量 CmdAng = [Cmd_alp ,Cmd_bet, Cmd_mu];
%Ero_pit = CtrlSdt_P(1);  Ero_yaw = CtrlSdt_P(2);    Ero_rol = CtrlSdt_P(3); %构成控制误差矢量 Eror_ang = [Ero_pit ,Ero_yaw, Ero_rol];
%Ero_qw = CtrlSdt_P(4);  Ero_rw = CtrlSdt_P(5);    Ero_pw = CtrlSdt_P(6); %构成角速度控制误差矢量 Eror_omg = [Ero_qw ,Ero_rw, Ero_pw];
%Acc_pit = CtrlSdt_P(7);  Acc_yaw = CtrlSdt_P(8);    Acc_rol = CtrlSdt_P(9); %构成角加速度矢量 Accelrt = [Acc_pit ,Acc_yaw, Acc_rol];
%Dlt_alp = CtrlSdt_P(10);  Dlt_bet = CtrlSdt_P(11);    Dlt_mu = CtrlSdt_P(12); %构成等效舵偏矢量 DeltaE = [Dlt_alp ,Dlt_bet, Dlt_mu];
function [Eror_ang, Eror_omg, Delta,SMCData] = Ctrller_SMC_Db(AfnPara, CmdAng, AeroAng, Omega)

global step;     %%记录仿真步数,用于增益调度
global h_step;   %%仿真步长
%新定义的全局变量，为了记录中间值而不被清除
global EAngSum; %角度控制误差累计 三通道向量
global EOmgSum; %角速度控制误差累计 三通道向量
global CmdAng_lst;  %记录上次角度，用于求取变化率
global Cmdomg_lst;  %记录上次角速度，用于求取变化率

if step==1
    EAngSum = zeros(3,1);
    EOmgSum = zeros(3,1);
    CmdAng_lst = zeros(3,1); %初始化参数
    Cmdomg_lst = zeros(3,1); %初始化参数
end

%模型参数赋值
%%%% AfnPara = [gs,gf, ff, dst_F, dst_Mx];%%3*(3+3+1+1+1)=3*9
 gs = AfnPara(:,1:3);
gf = AfnPara(:,4:6);
ff = AfnPara(:,7);
 dst_F = AfnPara(:,8);
dst_M = AfnPara(:,9);
% dgs = AfnPara(:,10:12);

%% 带调节的参数
Dstb_est_A = dst_F;%估计的干扰界
Int_limt1 = 3; %积分限幅大小
c_ang = [0.2; 0.2; 0.2];  %角度控制的积分滑模变量参数
k1_ang = [0.1; 0.1; 0.1];  %趋近律参数
k2_ang = [0.3; 0.3; 0.3];

% c_ang = [1; 1; 1];  %角度控制的积分滑模变量参数
% k1_ang = [0.1; 0.1; 0.1];  %趋近律参数
% k2_ang = [0.5; 0.5; 0.5];

Dstb_est_O = dst_M ;%估计的干扰界
Int_limt2 = 5; %积分限幅大小
c_omg = [1; 1; 1];  %角速度控制的积分滑模变量参数 可用参数：20
k1_omg = [0.5; 0.5; 0.5];  %趋近律参数 可用参数：10
k2_omg = [1.5; 1.5; 1.5];      %可用参数：5


%% 直接使用矢量控制三通道
                    %%%%%%%%%%% 外环，角度控制%%%%%%%%%%%
Eror_ang = (AeroAng' - CmdAng');  %角度控制误差，注意这里定义的是 实际角度减期望指令角  
EAngSum = EAngSum + Eror_ang; %误差积分并限幅

if abs(EAngSum(1)) > Int_limt1
    EAngSum(1) = sign(EAngSum(1))*Int_limt1;
end
if abs(EAngSum(2)) > Int_limt1
    EAngSum(2) = sign(EAngSum(2))*Int_limt1;
end
if abs(EAngSum(3)) > Int_limt1
    EAngSum(3) = sign(EAngSum(3))*Int_limt1;
end

sigma_ang = Eror_ang + c_ang.*EAngSum; %%角度控制的滑模变量    注意这里使用列向量
dComAng = (CmdAng' - CmdAng_lst)./h_step; %注意这里使用列向量
CmdAng_lst = CmdAng'; %记录指令角

%控制律  
 Omg_v = gs \( dComAng - c_ang.*Eror_ang - k1_ang.*tanh(sigma_ang/0.8) - k2_ang.*sigma_ang - Dstb_est_A); % 外环控制律，产生角速率期望    

%%%由于外环可以近似为线性控制，因此可直接用下面的比例反馈进行控制，内环依然是滑模控制
%%%%先直接用外环增益，调节内环稳定
% sigma_ang = Omg_v;
% Eror_ang = (AeroAng' - CmdAng');  %角度控制误差，注意这里定义的是 实际角度减期望指令角 
gain = [-40;40;-40];%阶跃测试时这个系数要小一些
% gain = [-2;2;-2];%跟踪指令时这个系数可以大一些
Omg_v1 = gain .* Eror_ang;
% Omg_v(2,1) = Omg_v1(2,1);  Omg_v(3,1) = Omg_v1(3,1); %%测试俯仰通道用滑模，其他两个通道直接增益反馈控制，是否可行
                    %%%%%%%%%%% 内环角速度控制 %%%%%%%%%%%
%  CmdOmg = Omg_v; %%外环输出做内环输入                    
CmdOmg = Omg_v1; %%外环输出做内环输入

Eror_omg = (Omega' - CmdOmg); %角速度控制误差，注意这里定义的是 实际减期望  
EOmgSum = EOmgSum + Eror_omg; %误差积分并限幅

if abs(EOmgSum(1)) > Int_limt2
    EOmgSum(1) = sign(EOmgSum(1))*Int_limt2;
end
if abs(EOmgSum(2)) > Int_limt2
    EOmgSum(2) = sign(EOmgSum(2))*Int_limt2;
end
if abs(EOmgSum(3)) > Int_limt2
    EOmgSum(3) = sign(EOmgSum(3))*Int_limt2;
end  

sigma_omg = Eror_omg + c_omg.*EOmgSum;   %%角速度控制的滑模变量    
dComOmg = (CmdOmg - Cmdomg_lst)./h_step;  %

%控制律
Uc =  gf\ (dComOmg - ff - c_omg.*Eror_omg - Dstb_est_O - k1_omg.*tanh(sigma_omg/0.8)- k2_omg.*sigma_omg);% 外环控制律，产生角速率期望
%Uc = gf \(dComOmg - ff - c_omg*Eror_omg - Dstb_est_O - k1_omg*sign(sigma_omg)); % 外环控制律，产生角速率期望

Cmdomg_lst = CmdOmg; %记录指令角
%% 得到最终的舵偏  
if abs(Uc(1)) > 20
    Uc(1) = sign(Uc(1))*20;
end
if abs(Uc(2)) > 20
    Uc(2) = sign(Uc(2))*20;
end
if abs(Uc(3)) > 20
    Uc(3) = sign(Uc(3))*20;
end  
Delta = Uc'; %%转成行向量

SMCData= [Eror_ang', EAngSum', sigma_ang', dComAng',Omg_v1',Eror_omg', EOmgSum', sigma_omg',dComOmg',Uc',Dstb_est_A'];
% SMCData= [sigma_ang', sigma_omg'];
end 
