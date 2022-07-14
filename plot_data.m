%% Plots comparison between admittance and variable control using DQ log mapping

%% Test 1: Environment stiffness = 10000 N/m
load("Data/impedance_control.mat");

%%MOTIVATION

f1 = figure();
f1.Renderer = 'painters';
grid on
hold on
plot(tt,sres_f.xref(4,:),'r--','LineWidth',3);
hold on, grid on
plot(tt,sres.x(4,:),'b','LineWidth',2);
hold on, grid on
plot(tt,sres_f.x(4,:),'g','LineWidth',2);
ylabel('$z/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('des','var','const','Interpreter', 'latex', 'FontSize', 10)

f2 = figure();
f2.Renderer = 'painters';
grid on
hold on
plot(tt,sres.fext(3,:),'b','LineWidth',2);
hold on, grid on
plot(tt,sres_f.fext(3,:),'g','LineWidth',2);
ylabel('$F/\mathrm{N}$', 'Interpreter', 'latex', 'FontSize', 12)
legend('var','const','Interpreter', 'latex', 'FontSize', 10)


f3 = figure();
f3.Renderer = 'painters';
grid on
hold on
plot(tt(1:301),k_data(:,3),'b','LineWidth',2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% grid on
% hold on
% plot(tt,sres_f.xref(2,:),'r--','LineWidth',3);
% hold on, grid on
% plot(tt,sres.x(2,:),'b','LineWidth',2);
% hold on, grid on
% plot(tt,sres_f.x(2,:),'g','LineWidth',2);
% ylabel('$x/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
% legend('des','var','const','Interpreter', 'latex', 'FontSize', 10)
% 
% 
% grid on
% hold on
% plot(tt,sres_f.xref(3,:),'r--','LineWidth',3);
% hold on, grid on
% plot(tt,sres.x(3,:),'b','LineWidth',2);
% hold on, grid on
% plot(tt,sres_f.x(3,:),'g','LineWidth',2);
% ylabel('$y/\mathrm{m}$', 'Interpreter', 'latex', 'FontSize', 12)
% legend('des','var','const','Interpreter', 'latex', 'FontSize', 10)

