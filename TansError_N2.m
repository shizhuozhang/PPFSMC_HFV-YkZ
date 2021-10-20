%改进的预设性能形式
%常规的预设性能形式
%设置性能指标，然后按照指标将实际误差转化为虚拟误差，并求解虚拟误差的导数
%输入：某种实际误差,实际误差的导数和上次的虚拟误差,这里使用的是标量的形式，不能用向量
%输出：转化后的虚拟误差和虚拟误差的导数
function [PPFData] = TansError_N2(PpfPara,ErorA, dErorA, ErorV_lst)

global h_step;   %%仿真步长
global step;     %%记录仿真步数
global t_cvg; %收敛时间
global flag_t;%时间清零标志位 1表示已清零，0表示未清零
% global Rad2Deg;  %%弧度转换为度
global flag_e; %初始误差正负标志位
global flag_x; %初始误差正负标志位
global t_hold;
global ErorA0;

a = 0.002;  %缩放系数

% PpfPara = [rho0,rho_inf,Mx,lambda];
rho0 = PpfPara(1);  %初始最大误差 12度
rho_inf = PpfPara(2);  %稳态误差界 0.3度
Mx = PpfPara(3); %限定最大超调
lambda = PpfPara(4); %收敛速率，应在2以上

if step==1
    t_cvg = 0; %初始化参数 
    t_hold =0;
    flag_t = 1; %初始化参数,收敛时间已清零
    flag_x = 1;
     ErorA0 = ErorA;
%     flag_e = sign(ErorA); %初始化误差参数 
end

%% 收敛时间策略
rho_limit = Mx*rho_inf ;

if (abs(ErorA) > 1.1*rho_limit)&&(flag_t== 0)
    t_cvg = 0;
    t_hold = 0;
    flag_t = 1;  
    flag_x = 1;
    ErorA0 = ErorA;
else
    t_cvg = t_cvg + h_step;%时间累加，未清零
    if abs(ErorA) <= rho_limit
        t_hold = t_hold +1;
        if t_hold > 200 %||step<200
            flag_t = 0;  %误差已经收敛到界内，并持续1秒，允许时间清零
        end
    else
        t_hold = 0;
    end
end 

flag_e = sign(ErorA0); %初始化误差参数
if (abs(ErorA) >= abs(ErorA0)) &&(flag_x == 1)
    ErorA0 = ErorA; %不断更新初始误差，找到最大的误差
else
    flag_x = 0;
end
r = tanh(ErorA0 / a);
%% 性能函数
rhot = (rho0 - rho_inf)*exp(-lambda * t_cvg) + rho_inf;
drhot = (rho_inf - rho0)*lambda*exp( -lambda * t_cvg);

g1t = Mx * rhot;   dg1t = Mx * drhot; 
g2t = rhot;     dg2t = drhot;
h1t = - r* g1t;
h2t = r* g2t;

PPF_L = min(h1t,h2t);  
PPF_U = max(h1t,h2t);  

%% 误差变换
kt = (ErorA - PPF_L) /(PPF_U - PPF_L);%%%对于误差超出界限的情况，下面会出现负值。
gn = (1-kt) / kt;
if gn >= 3
    gn = 3;
% elseif gn <= 0.01
%     gn = 0.01; %%变换后的误差
end

if gn <= 0
    ErorV = ErorV_lst;
else
    ErorV = log( 1/gn ); %%变换后的误差
end

g0 = g1t + g2t;  dg0 = dg1t + dg2t; 
gp = g2t*dg1t - g1t*dg2t;

Td1 = flag_e * gn /r /g0;
Tc1 = (flag_e * gn /g0 /g0) *(gp - (dg0 *ErorA /r));
dErorV = Td1*dErorA + Tc1; %利用推导的解析式求导

PPFData = [ErorV, Td1, Tc1, dErorV, PPF_L, PPF_U, gn, flag_t, flag_e, ErorA0 ];%1*10的向量

end