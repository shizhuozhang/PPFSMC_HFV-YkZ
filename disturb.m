clear all;
close all;

Time = zeros(1,1000);
d1 = zeros(1,1000);
d2 = zeros(1,1000);
d3 = zeros(1,1000);
d4 = zeros(1,1000);
d5 = zeros(1,1000);
for i=1:1:1000
    Time(i) = 0.1*i;
    d1(i) = 0.1*sin(0.5*Time(i));
    d2(i) = 0.1*sin(1*Time(i));  
    d3(i) = 0.1*sin(1.5*Time(i));
    d4(i) = 0.1*sin(2*Time(i));
end
d5 = d1+ d2+ d3+ d4;
figure(1)
plot(Time,d5,'-r','LineWidth',1.5);
grid on
xlabel('Time (s)','FontSize',13);
ylabel('Disturbance Value ','FontSize',13);
set(gcf,'windowstyle','normal');
set(gcf,'position',[550,100,800,500]);
set(gca,'FontSize',13);