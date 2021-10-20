%%仿射非线性模型参数计算
function AfnPara = AfnNonlnrPara(DeltaA, Env_para, FltSdt_S, FltSdt_F) 

global m0;   %%再入飞行质量
global Jx;   %%绕X轴转动惯量 
global Jy;   %%绕X轴转动惯量 
global Jz;   %%绕X轴转动惯量 
global Xcg;  %%质心到参考力矩中心距离
global S;    %%翼展参考面积
global b;    %%翼展
global c;    %%平均气动弦长
global Uncertn_flag; %是否引入外部风干扰（随机干扰）
global Uncertn_para;
global Coef_aero;  %%气动模型计算的转换系数 %本应该用弧度到度的转换系数，但由于有问题，暂时用这个调试
V2R = -1 * [1,0,-1; 1,0,1; 0,1,0]; %%由等效的舵指令转化为实际的舵偏 乘-1是因为，等效舵偏的定义为正的舵偏产生正的姿态角
%% 参数赋值
%不确定参数
xm  = Uncertn_para(1); xJx = Uncertn_para(2);  xJy = Uncertn_para(3); xJz = Uncertn_para(4);
xRho = Uncertn_para(5); 
xCD = Uncertn_para(6);  xCY = Uncertn_para(7);  xCL = Uncertn_para(8);  
xCl = Uncertn_para(9);  xCm = Uncertn_para(10); xCn = Uncertn_para(11);

%舵偏参数
LE = DeltaA(1); RE = DeltaA(2); RUD = DeltaA(3);%%注意舵偏顺序 e,a,r ，和下面的气动函数有点不一致

%飞行状态参数
%X_loc = FltSdt_S(1);   Y_loc = FltSdt_S(2);  Height = FltSdt_S(3);
Veloc = FltSdt_S(4);   gamma = FltSdt_S(5); %  chi = FltSdt_S(6);

alpha = FltSdt_F(4);   ALPHA = Coef_aero * alpha; 
beta = FltSdt_F(5);    mu = FltSdt_F(6);
q = FltSdt_F(7);       r = FltSdt_F(8);    p = FltSdt_F(9);

%大气环境参数
g = Env_para(1);  Rho = Env_para(2);  Ma = Env_para(3);   

[CD_Coe, CY_Coe, CL_Coe, Clw_Coe, Cmw_Coe, Cnw_Coe] = AeroCofict4afn(Ma,ALPHA,beta,RE,LE,RUD,p,q,r,Veloc); %%针对仿射非线性的计算

%气动极限不确定性
if Uncertn_flag == 1
    m0 = m0*(1 + xm); %质量不确定性
    Jx = Jx*(1 + xJx); Jy = Jy*(1 + xJy); Jz = Jz*(1 + xJz);%转动惯量不确定性，
    Rho = Rho*(1 + xRho);%空气密度不确定性
    CD_Coe = CD_Coe*(1 + xCD); CY_Coe = CY_Coe*(1 + xCY); CL_Coe = CL_Coe*(1 + xCL);%气动参数不确定性
    Clw_Coe = Clw_Coe*(1 + xCl); Cmw_Coe = Cmw_Coe*(1 + xCm); Cnw_Coe = Cnw_Coe*(1 + xCn);
end

CD_Dt0 = CD_Coe(1);   CD_alp = CD_Coe(2);   CD_dDte = CD_Coe(3);  CD_dDta = CD_Coe(4);  CD_dDtr = CD_Coe(5);
CY_Dt0 = CY_Coe(1);   CY_bta = CY_Coe(2);   CY_dDte = CY_Coe(3);  CY_dDta = CY_Coe(4);  CY_dDtr = CY_Coe(5); 
CL_Dt0 = CL_Coe(1);   CL_alp = CL_Coe(2);   CL_dDte = CL_Coe(3);  CL_dDta = CL_Coe(4);
Cl_Dt0 = Clw_Coe(1); C_lw_aero = Clw_Coe(2);Cl_dDte = Clw_Coe(3); Cl_dDta = Clw_Coe(4); Cl_dDtr = Clw_Coe(5);
Cm_Dt0 = Cmw_Coe(1); C_mw_aero = Clw_Coe(2);Cm_dDte = Cmw_Coe(3); Cm_dDta = Cmw_Coe(4); Cm_dDtr = Cmw_Coe(5);  
Cn_Dt0 = Cnw_Coe(1); C_nw_aero = Clw_Coe(2);Cn_dDte = Cnw_Coe(3); Cn_dDta = Cnw_Coe(4); Cn_dDtr = Cnw_Coe(5);  

q_bar = (Veloc^2)*Rho/2; %%动压的计算


%% 参数计算
%第一个系数矩阵
F_alp = m0*g*cos(gamma)*cos(mu) - CL_alp*q_bar*S;
F_bta = m0*g*cos(gamma)*sin(mu) + CY_bta*q_bar*S;
F_mu = (CY_bta*cos(mu) + CL_alp*sin(mu))*q_bar*S*tan(gamma) + (CL_alp*q_bar*S - m0*g*cos(gamma)*cos(mu))*tan(beta);
f_alp = F_alp/m0/Veloc/cos(beta);
f_bta = F_bta/m0/Veloc;
f_mu =  F_mu/m0/Veloc;
fs = [f_alp; f_bta; f_mu];

%第二个系数矩阵
M_aerox = C_lw_aero*b*q_bar*S; %除去舵力矩的力矩系数
M_aeroy = (C_mw_aero*c + Xcg*(CD_alp*sin(alpha) + CL_alp*cos(alpha)))*q_bar*S;
M_aeroz = (C_nw_aero*b - Xcg*CY_bta)*q_bar*S;
f_p = (Jy - Jz)*q*r/Jx + M_aerox/Jx;
f_q = (Jz - Jx)*p*r/Jy + M_aeroy/Jy;
f_r = (Jx - Jy)*p*q/Jz + M_aeroz/Jz;
ff = [f_p; f_q; f_r];

%其它系数矩阵
% B_dlt = (1/m0/Veloc)*[0, 0, 1/cos(beta);  0, 1, 0;  0, tan(gamma)*cos(mu), tan(beta)+tan(gamma)*sin(mu)];
B_dlt = (1/m0/Veloc)*[1/cos(beta); tan(gamma)*cos(mu); tan(beta)+tan(gamma)*sin(mu)]; %%应该是矩阵，这里略去01值，写成向量形式，使用时应注意
J_inv = [1/Jx; 1/Jy; 1/Jz];  %%应该是对角阵，这里略去0值，写成向量形式，使用时应注意
gs = [ -cos(alpha)*tan(beta), 1, -sin(alpha)*tan(beta);
           sin(alpha),        0,    -cos(alpha);
        cos(alpha)/cos(beta), 0,   sin(alpha)/cos(beta)];

% 舵的力系数矩阵   
galp_De = - CL_dDte;
galp_Da = - CL_dDta;
galp_Dr = 0;
gbta_De = CY_dDte;
gbta_Da = CY_dDta;
gbta_Dr = CY_dDtr;
gmu_De = CY_dDte*tan(gamma)*cos(mu) + CL_dDte*(tan(gamma)*sin(mu) + tan(beta));
gmu_Da = CY_dDta*tan(gamma)*cos(mu) + CL_dDta*(tan(gamma)*sin(mu) + tan(beta));
gmu_Dr = CY_dDtr*tan(gamma)*cos(mu);    
gs_dlt = q_bar*S*[galp_De, galp_Da, galp_Dr;  gbta_De, gbta_Da, gbta_Dr;  gmu_De, gmu_Da, gmu_Dr];
gs_dltE = gs_dlt*V2R; %转化为等效舵

%舵的力矩系数矩阵
gp_De = b*Cl_dDte; 
gp_Da = b*Cl_dDta;
gp_Dr = b*Cl_dDtr;
gq_De = c*Cm_dDte + Xcg*(CD_dDte*sin(alpha) + CL_dDte*cos(alpha));
gq_Da = c*Cm_dDta + Xcg*(CD_dDta*sin(alpha) + CL_dDta*cos(alpha));
gq_Dr = c*Cm_dDtr + Xcg*CD_dDtr*sin(alpha);
gr_De = b*Cn_dDte - Xcg*CY_dDte;
gr_Da = b*Cn_dDta - Xcg*CY_dDta;
gr_Dr = b*Cn_dDtr - Xcg*CY_dDtr;
gf_dlt = q_bar*S*[gp_De, gp_Da, gp_Dr;  gq_De, gq_Da, gq_Dr;  gr_De, gr_Da, gr_Dr];
gf_dltE = gf_dlt*V2R; %转化为等效舵

%空舵的力系数矩阵
FDt0_alp = -CL_Dt0*q_bar*S;
FDt0_bta = CY_Dt0*q_bar*S;
FDt0_mu = (CY_Dt0*tan(gamma)*cos(mu) + CL_Dt0*(tan(gamma)*sin(mu) + tan(beta)))*q_bar*S;
F_dlt0 = [FDt0_alp; FDt0_bta; FDt0_mu];

%空舵的力矩系数矩阵
MDt0_x = b*Cl_Dt0*q_bar*S;
MDt0_y = (c*Cm_Dt0 + Xcg*(CD_Dt0*sin(alpha) + CL_Dt0*cos(alpha)))*q_bar*S;
MDt0_z = (b*Cn_Dt0 - Xcg*CY_Dt0)*q_bar*S;
M_dlt0 = [MDt0_x; MDt0_y; MDt0_z];

%% 参数整合

% AfnPara = [B_dlt, gs_dlt, fs, gs, J_inv, gf_dlt, ff, F_dlt0, M_dlt0]; %3*(1+3+1+3+1+3+1+1+1) = 3*15  %%使用实际舵
AfnPara = [B_dlt, gs_dltE, fs, gs, J_inv, gf_dltE, ff, F_dlt0, M_dlt0]; %3*(1+3+1+3+1+3+1+1+1) = 3*15  %%使用等效舵
end