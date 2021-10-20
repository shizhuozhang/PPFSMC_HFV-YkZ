%�Ľ���Ԥ��������ʽ
%�����Ԥ��������ʽ
%��������ָ�꣬Ȼ����ָ�꽫ʵ�����ת��Ϊ������������������ĵ���
%���룺ĳ��ʵ�����,ʵ�����ĵ������ϴε��������,����ʹ�õ��Ǳ�������ʽ������������
%�����ת��������������������ĵ���
function [PPFData] = TansError_N2(PpfPara,ErorA, dErorA, ErorV_lst)

global h_step;   %%���沽��
global step;     %%��¼���沽��
global t_cvg; %����ʱ��
global flag_t;%ʱ�������־λ 1��ʾ�����㣬0��ʾδ����
% global Rad2Deg;  %%����ת��Ϊ��
global flag_e; %��ʼ���������־λ
global flag_x; %��ʼ���������־λ
global t_hold;
global ErorA0;

a = 0.002;  %����ϵ��

% PpfPara = [rho0,rho_inf,Mx,lambda];
rho0 = PpfPara(1);  %��ʼ������ 12��
rho_inf = PpfPara(2);  %��̬���� 0.3��
Mx = PpfPara(3); %�޶���󳬵�
lambda = PpfPara(4); %�������ʣ�Ӧ��2����

if step==1
    t_cvg = 0; %��ʼ������ 
    t_hold =0;
    flag_t = 1; %��ʼ������,����ʱ��������
    flag_x = 1;
     ErorA0 = ErorA;
%     flag_e = sign(ErorA); %��ʼ�������� 
end

%% ����ʱ�����
rho_limit = Mx*rho_inf ;

if (abs(ErorA) > 1.1*rho_limit)&&(flag_t== 0)
    t_cvg = 0;
    t_hold = 0;
    flag_t = 1;  
    flag_x = 1;
    ErorA0 = ErorA;
else
    t_cvg = t_cvg + h_step;%ʱ���ۼӣ�δ����
    if abs(ErorA) <= rho_limit
        t_hold = t_hold +1;
        if t_hold > 200 %||step<200
            flag_t = 0;  %����Ѿ����������ڣ�������1�룬����ʱ������
        end
    else
        t_hold = 0;
    end
end 

flag_e = sign(ErorA0); %��ʼ��������
if (abs(ErorA) >= abs(ErorA0)) &&(flag_x == 1)
    ErorA0 = ErorA; %���ϸ��³�ʼ���ҵ��������
else
    flag_x = 0;
end
r = tanh(ErorA0 / a);
%% ���ܺ���
rhot = (rho0 - rho_inf)*exp(-lambda * t_cvg) + rho_inf;
drhot = (rho_inf - rho0)*lambda*exp( -lambda * t_cvg);

g1t = Mx * rhot;   dg1t = Mx * drhot; 
g2t = rhot;     dg2t = drhot;
h1t = - r* g1t;
h2t = r* g2t;

PPF_L = min(h1t,h2t);  
PPF_U = max(h1t,h2t);  

%% ���任
kt = (ErorA - PPF_L) /(PPF_U - PPF_L);%%%�����������޵�������������ָ�ֵ��
gn = (1-kt) / kt;
if gn >= 3
    gn = 3;
% elseif gn <= 0.01
%     gn = 0.01; %%�任������
end

if gn <= 0
    ErorV = ErorV_lst;
else
    ErorV = log( 1/gn ); %%�任������
end

g0 = g1t + g2t;  dg0 = dg1t + dg2t; 
gp = g2t*dg1t - g1t*dg2t;

Td1 = flag_e * gn /r /g0;
Tc1 = (flag_e * gn /g0 /g0) *(gp - (dg0 *ErorA /r));
dErorV = Td1*dErorA + Tc1; %�����Ƶ��Ľ���ʽ��

PPFData = [ErorV, Td1, Tc1, dErorV, PPF_L, PPF_U, gn, flag_t, flag_e, ErorA0 ];%1*10������

end