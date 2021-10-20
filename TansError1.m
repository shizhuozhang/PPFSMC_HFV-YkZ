%常规的预设性能形式
%设置性能指标，然后按照指标将实际误差转化为虚拟误差，并求解虚拟误差的导数
%输入：某种实际误差,实际误差的导数和上次的虚拟误差,这里使用的是标量的形式，不能用向量
%输出：转化后的虚拟误差和虚拟误差的导数
function [PPFData] = TansError1(PpfPara,ErorA, dErorA, ErorV_lst)

global h_step;   %%仿真步长
global step;     %%记录仿真步数
global t_cvg; %收敛时间
global flag_t;%时间清零标志位 1表示已清零，0表示未清零
% global Rad2Deg;  %%弧度转换为度
global flag_e; %初始误差正负标志位
global t_hold;

if step==1
    t_cvg = 0; %初始化参数 
    t_hold =0;
    flag_t = 1; %初始化参数,收敛时间已清零
    flag_e = -1; %初始化误差参数 
end

%% 性能函数
% PpfPara = [rho0,rho_inf,Mx,lambda];
rho0 = PpfPara(1);  %初始最大误差 12度
rho_inf = PpfPara(2);  %稳态误差界 0.3度
Mx = PpfPara(3); %限定最大超调
lambda = PpfPara(4); %收敛速率，应在2以上

rhot = (rho0 - rho_inf)*exp(-lambda * t_cvg) + rho_inf;

% % % 收敛时间策略
% if (abs(ErorA) > rho_inf)&&(flag_t== 0)
%     t_cvg = 0;
%     flag_t = 1;  
%     flag_e = sign(ErorA); %初始化误差参数
% else
%     t_cvg = t_cvg + h_step;%时间累加，未清零
%     if abs(ErorA) <= rho_inf
%         t_hold = t_hold +1;
%         if t_hold > 400 %||step<200
%             flag_t = 0;  %误差已经收敛到界内，并持续1秒，允许时间清零
%         end
%     else
%         t_hold = 0;
%     end
% end
  t_cvg = step*h_step;   
%误差约束的界函数
if flag_e >=0
    L = Mx ;  
    U = 1;
else
    L = 1 ;  
    U = Mx;
end

% PPF_L = -L* rhot;  
% PPF_U = U*rhot;

%% 误差变换
xt = ErorA /rhot;%%%对于误差超出界限的情况，下面会出现负值。
drhot = (rho_inf - rho0)*lambda*exp( -lambda * t_cvg);

yt = (U*xt + L*U)/(L*U - L*xt);
if yt<0
    ErorV = ErorV_lst;
else
    ErorV = log(yt)/2; %%变换后的误差
end


Td1 = ((1/(xt + L))-(1/(xt - U)))/2/rhot;
Tc1 = -Td1* ErorA* drhot/ rhot;
dErorV = Td1*dErorA + Tc1; %利用推导的解析式求导

%利用上次的虚拟误差，差分求导
% dErorV = (ErorV - ErorV_lst)/h_step;

PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10的向量

end