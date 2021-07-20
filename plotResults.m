%% PLOT SIMULATION RESULTS

linewidth = 6;
fontsize = 30;
lgdFontsize = 40;

t = linspace(0, T, nSamples)';

%% Error plot
figure
hold on
grid on
plot(t,abs(repmat(q_ref(1),length(t),1) - xHistory(:,1)));
plot(t,abs(repmat(q_ref(2),length(t),1) - xHistory(:,2)));
xlabel('[s]')
ylabel('[rad]')
lgd = legend('$|e_1|$', '$|e_2|$', 'Interpreter', 'latex', 'Location', 'best');
title('Links position error')
set(findall(gcf,'type','line'),'linewidth',linewidth);
set(gca,'FontSize',fontsize);  
lgd.FontSize = lgdFontsize;

%% Torque plot
figure
hold on
grid on
plot(t,uHistory(:,1))
plot(t,uHistory(:,2))
xlabel('[s]')
ylabel('[Nm]')
lgd = legend('$\tau_1$', '$\tau_2$', 'Interpreter', 'latex', 'Location', 'best');
title('Controlled torque')
set(findall(gcf,'type','line'),'linewidth',linewidth); 
set(gca,'FontSize',fontsize);
lgd.FontSize = lgdFontsize;

%% GP error plot
% figure
% hold on
% grid on
% t = linspace(0,10, size(eHistory, 1));
% plot(t,eHistory(:,1));
% xlabel('Re-training step')
% lgd = legend('error', 'Location', 'best');
% title('Norm of prediction error')
% set(findall(gcf,'type','line'),'linewidth',linewidth); 
% set(gca,'FontSize',fontsize);
% lgd.FontSize = lgdFontsize;