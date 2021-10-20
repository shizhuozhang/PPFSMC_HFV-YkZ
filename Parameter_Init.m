%%��ʼ���������
%����������������κ�������
%�������ʼ�ķ���״̬�����Ͳ�ȷ�����趨ֵ
function [FltSdt_S, FltSdt_F] = Parameter_Init(Ft_p)

global ctrl_case; %%ѡ��������Σ���Ϊȫ�������ƺ͵������
global Rad2Deg;  %%����ת��Ϊ��
global Uncertn_flag; %�Ƿ������ⲿ����ţ�������ţ�
global Uncertn_para;
global Bias_case; %��ƫ���
global Bias_amp;  %��ƫ���޷�ֵ%��ƫ�����趨�����0.3
%% ��ȷ�����趨����������������ƫ���������������������ƫ���������趨30%��ƫ��
%��ȷ������˳��;    xm0; xJx; xJy; xJz; xRho; xCD; xCY; xCL; xCl; xCm; xCn ; 
Uncertn_para = zeros(1,11); %��ʼֵ����Ϊ0
if Uncertn_flag == 1  
    %����������ƫ����Ϊ�ṹ�������������������֣������������������ַ�Ϊ��������ͨ�������о���
    BCas = Limit_bias(4,Bias_amp/100);
    
    xRho = BCas(Bias_case,1); xm  = BCas(Bias_case,2);  %%���е���ƫ��������ѹ������
    xJy = BCas(Bias_case,3);  xCm = BCas(Bias_case,4);% �Ը�����ز���������ƫ xRho;xCL; xCm;
    xJx = BCas(Bias_case,3);  xCl = BCas(Bias_case,4);% �Թ�ת�����������������ƫ xRho; xCl;
%     xJz = BCas(Bias_case,3);  xCn = BCas(Bias_case,4);  % ��ƫ�������������������ƫ xRho; xCY; xCn ;
%     xCD = BCas(Bias_case,3);  xCL = BCas(Bias_case,4);  xCY = BCas(Bias_case,3);   
    xJz = 0;  xCn = 0;  % ��ƫ�������������������ƫ xRho; xCY; xCn ;
    xCD = 0;  xCL = 0;  xCY = 0;  
    
    Uncertn_para(1) = xm;  Uncertn_para(2) = xJx;   Uncertn_para(3) = xJy;  Uncertn_para(4) = xJz;
    Uncertn_para(5) = xRho; Uncertn_para(6) = xCD; Uncertn_para(7) = xCY;  Uncertn_para(8) = xCL;
    Uncertn_para(9) = xCl; Uncertn_para(10) = xCm;  Uncertn_para(11) = xCn;
else     
    Uncertn_para = zeros(1,11); 
end
%%�����趨 ��Χ0-0.2  ���ʺͲ�ȷ���Բ�� ��Ӧ����ʱ��ģ�������ﲻ����ʼ���������ļ��㺯�������

%% 100sȫ��������
if ctrl_case==1  %���˸߶ȡ��ٶ���������ȫ������
    X_loc = 0;    Y_loc = 0;    Height = 33000;
    Veloc = 4590;    gamma = 0/Rad2Deg;    chi = 0/Rad2Deg;
    alpha = 0/Rad2Deg;    beta = 0/Rad2Deg;    mu = 0/Rad2Deg;
    q = 0;    r = 0;    p = 0;   
    
%% �������   ���ݱ�Ƶ����������㴦�ľ���״̬�趨
else  
    switch Ft_p
        case 1
            X_loc = 0;    Y_loc = 0;    Height = 33000;
            Veloc = 4590;    gamma = 0;    chi = 0;
            alpha = 0;    beta = 0;    mu = 0;
            q = 0;    r = 0;    p = 0;  
         case 2
            X_loc = 0;    Y_loc = 0;    Height = 33000;
            Veloc = 4590;    gamma = 0;    chi = 0;
            alpha = 2/Rad2Deg;    beta = 0;    mu = 0/Rad2Deg;
            q = 0;    r = 0;    p = 0;  
        case 3  
            X_loc = 0;    Y_loc = 0;    Height = 325130;
            Veloc = 4559;    gamma = -0.98/Rad2Deg;    chi = 0/Rad2Deg;
            alpha = 14/Rad2Deg;    beta = 0/Rad2Deg;    mu = -2/Rad2Deg;
            q = 0;    r = 0;    p = 0;  
    end 
end

%ͨ�����ι�ϵ��ŷ����̬��
AeroAng = [alpha, beta, mu];
Trajct = [Veloc, gamma, chi];
EulerAng = GometrRelat(1, AeroAng ,Trajct);%���ŷ����
theta = EulerAng(1); psi = EulerAng(2); phi = EulerAng(3);

FltSdt_S(1) = X_loc;   FltSdt_S(2) = Y_loc;   FltSdt_S(3) = Height;
FltSdt_S(4) = Veloc;   FltSdt_S(5) = gamma;   FltSdt_S(6) = chi;

FltSdt_F(1) = theta;   FltSdt_F(2) = psi;   FltSdt_F(3) = phi;
FltSdt_F(4) = alpha;   FltSdt_F(5) = beta;   FltSdt_F(6) = mu;
FltSdt_F(7) = q;   FltSdt_F(8) = r;   FltSdt_F(9) = p;
end
