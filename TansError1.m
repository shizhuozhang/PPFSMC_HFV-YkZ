%�����Ԥ��������ʽ
%��������ָ�꣬Ȼ����ָ�꽫ʵ�����ת��Ϊ������������������ĵ���
%���룺ĳ��ʵ�����,ʵ�����ĵ������ϴε��������,����ʹ�õ��Ǳ�������ʽ������������
%�����ת��������������������ĵ���
function [PPFData] = TansError1(PpfPara,ErorA, dErorA, ErorV_lst)

global h_step;   %%���沽��
global step;     %%��¼���沽��
global t_cvg; %����ʱ��
global flag_t;%ʱ�������־λ 1��ʾ�����㣬0��ʾδ����
% global Rad2Deg;  %%����ת��Ϊ��
global flag_e; %��ʼ���������־λ
global t_hold;

if step==1
    t_cvg = 0; %��ʼ������ 
    t_hold =0;
    flag_t = 1; %��ʼ������,����ʱ��������
    flag_e = -1; %��ʼ�������� 
end

%% ���ܺ���
% PpfPara = [rho0,rho_inf,Mx,lambda];
rho0 = PpfPara(1);  %��ʼ������ 12��
rho_inf = PpfPara(2);  %��̬���� 0.3��
Mx = PpfPara(3); %�޶���󳬵�
lambda = PpfPara(4); %�������ʣ�Ӧ��2����

rhot = (rho0 - rho_inf)*exp(-lambda * t_cvg) + rho_inf;

% % % ����ʱ�����
% if (abs(ErorA) > rho_inf)&&(flag_t== 0)
%     t_cvg = 0;
%     flag_t = 1;  
%     flag_e = sign(ErorA); %��ʼ��������
% else
%     t_cvg = t_cvg + h_step;%ʱ���ۼӣ�δ����
%     if abs(ErorA) <= rho_inf
%         t_hold = t_hold +1;
%         if t_hold > 400 %||step<200
%             flag_t = 0;  %����Ѿ����������ڣ�������1�룬����ʱ������
%         end
%     else
%         t_hold = 0;
%     end
% end
  t_cvg = step*h_step;   
%���Լ���Ľ纯��
if flag_e >=0
    L = Mx ;  
    U = 1;
else
    L = 1 ;  
    U = Mx;
end

% PPF_L = -L* rhot;  
% PPF_U = U*rhot;

%% ���任
xt = ErorA /rhot;%%%�����������޵�������������ָ�ֵ��
drhot = (rho_inf - rho0)*lambda*exp( -lambda * t_cvg);

yt = (U*xt + L*U)/(L*U - L*xt);
if yt<0
    ErorV = ErorV_lst;
else
    ErorV = log(yt)/2; %%�任������
end


Td1 = ((1/(xt + L))-(1/(xt - U)))/2/rhot;
Tc1 = -Td1* ErorA* drhot/ rhot;
dErorV = Td1*dErorA + Tc1; %�����Ƶ��Ľ���ʽ��

%�����ϴε������������
% dErorV = (ErorV - ErorV_lst)/h_step;

PPFData = [ErorV, Td1, Tc1, dErorV, rhot, xt, flag_t, flag_e, t_cvg, L];%1*10������

end