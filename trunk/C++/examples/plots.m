subplot 241
plot(manipulated_vec(1,:),'y','linewidth',2);
hold on
plot(desired_vec(1,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(1,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 242
plot(manipulated_vec(2,:),'y','linewidth',2);
hold on
plot(desired_vec(2,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(2,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 243
plot(manipulated_vec(3,:),'y','linewidth',2);
hold on
plot(desired_vec(3,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(3,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 244
plot(manipulated_vec(4,:),'y','linewidth',2);
hold on
plot(desired_vec(4,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(4,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 245
plot(manipulated_vec(5,:),'y','linewidth',2);
hold on
plot(desired_vec(5,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(5,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 246
plot(manipulated_vec(6,:),'y','linewidth',2);
hold on
plot(desired_vec(6,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(6,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 247
plot(manipulated_vec(7,:),'y','linewidth',2);
hold on
plot(desired_vec(7,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(7,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

subplot 248
plot(manipulated_vec(8,:),'y','linewidth',2);
hold on
plot(desired_vec(8,:), 'b', 'linewidth', 2);
plot(manipulated_vec2(8,:),'--k','linewidth',2);
set(gca,'XLim',[1 length(desired_vec(1,:))], 'XTick', [length(desired_vec(1,:))/2, length(desired_vec(1,:))]);

ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'Simulação: DQ Robotics Matlab e biblioteca C++ com K = 0.6','HorizontalAlignment','center','VerticalAlignment', 'top');
%text(0.5, 1,'\bf Simulação: Biblioteca C++ com K =
%0.05','HorizontalAlignment','center','VerticalAlignment', 'top');