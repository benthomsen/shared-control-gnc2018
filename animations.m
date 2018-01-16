% ARS = AnomalyResponseSim('act',0); % for actuator dynamics
ARS = AnomalyResponseSim('del',0); % for time delay dynamics

% shorthand notation - just sloppy modification to use this script with new
% sim class format
SP  = ARS.simPars;
PSP = ARS.postSimPars;
CS  = ARS.controlSim;

ts    = SP.ts;
ts_rm = SP.ts_rm;
tfin  = SP.tfin;

i_tsrm = PSP.i_tsrm;
tsim   = PSP.tsim;

%% Figure 1: Command and Output
close all;

figure('Position',[100, 100, 640, 240])
filename = 'phi_plot.gif';


plot(tsim, PSP.r_deg, 'LineWidth', 1); hold on; grid on;
plot(tsim, PSP.x_deg(1,:), 'LineWidth', 1);
ylim([-2 25])
xlim([0 tfin])
line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
ylabel('$\phi$ (deg)', 'interpreter', 'latex')
xlabel('t (s)')
title('Stability Axis Bank Angle', 'interpreter', 'latex')
legend('Command (r)', 'Bank angle (\phi)')

set(gcf,'color','w'); % set figure background to white
drawnow;

F(length(tsim)) = struct('cdata',[],'colormap',[]);

F(1) = getframe(1); % save scatter plot as a frame for animation

%     % save animation to .gif file
im = frame2im(F(1));
[A,map] = rgb2ind(im,256); 
imwrite(A,map,filename,'gif','LoopCount',3,'DelayTime',0.05);

for i = 1:500:length(tsim)
    hold off;

    plot(tsim, PSP.r_deg, 'LineWidth', 1); hold on; grid on;
    plot(tsim(1:i), PSP.x_deg(1,1:i), 'LineWidth', 1);
    ylim([-2 25])
    xlim([0 tfin])
    line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
    line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
    ylabel('$\phi$ (deg)', 'interpreter', 'latex')
    xlabel('t (s)')
    title('Stability Axis Bank Angle', 'interpreter', 'latex')
    legend('Command (r)', 'Bank angle (\phi)')
    
    set(gcf,'color','w'); % set figure background to white
    drawnow;

    F(i) = getframe(1); % save scatter plot as a frame for animation
    
%     % save animation to .gif file
    im = frame2im(F(i));
    [A,map] = rgb2ind(im,256); 
    imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
end

%% Figure 2: Adaptive Parameters
close all;

figure('Position',[100, 100, 640, 240])
filename = 'theta_plot.gif';


% second figure: adaptive feedback gains for 3 phases
plot(tsim, PSP.th1, 'LineWidth', 1); hold on; grid on;
plot(tsim, PSP.th2, 'LineWidth', 1); 
plot(tsim, PSP.q, 'LineWidth', 1);
plot(tsim(i_tsrm+1:end), PSP.th3, 'LineWidth', 1);
ylim([-40 40])
xlim([0 tfin])
line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
ylabel('$\theta_i$', 'interpreter', 'latex')
xlabel('t (s)')
title('Adaptive Parameter Values', 'interpreter', 'latex')
legend('\theta_1', '\theta_2', 'q', '\theta_3')

set(gcf,'color','w'); % set figure background to white
drawnow;

F(length(tsim)) = struct('cdata',[],'colormap',[]);

F(1) = getframe(1); % save scatter plot as a frame for animation

%     % save animation to .gif file
im = frame2im(F(1));
[A,map] = rgb2ind(im,256); 
imwrite(A,map,filename,'gif','LoopCount',3,'DelayTime',0.05);


for i = 1:500:length(tsim)
    hold off;

    plot(tsim(1:i), PSP.th1(1:i), 'LineWidth', 1); hold on; grid on;
    plot(tsim(1:i), PSP.th2(1:i), 'LineWidth', 1); 
    plot(tsim(1:i), PSP.q(1:i), 'LineWidth', 1);
    ylim([-40 40])
    xlim([0 tfin])
    if (i > i_tsrm)
        plot(tsim(i_tsrm+1:i), PSP.th3(1:length(tsim(i_tsrm+1:i))), 'LineWidth', 1);
        line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
        line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
        legend('\theta_1', '\theta_2', 'q', '\theta_3')
    else
        line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
        line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
        legend('\theta_1', '\theta_2', 'q')
    end
    ylabel('$\theta_i$', 'interpreter', 'latex')
    xlabel('t (s)')
    title('Adaptive Parameter Values', 'interpreter', 'latex')
    
    set(gcf,'color','w'); % set figure background to white
    drawnow;

    F(i) = getframe(1); % save scatter plot as a frame for animation
    
%     % save animation to .gif file
    im = frame2im(F(i));
    [A,map] = rgb2ind(im,256); 
    imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
end

%% Figure 3: Error between x(1:2) and xm(1:2) - norm
close all;

figure('Position',[100, 100, 640, 240])
filename = 'e_plot.gif';


% third figure: error between desired response and actual - norm
plot(tsim, PSP.norm_e, 'LineWidth', 1); hold on; grid on;
ylim([0 0.4])
xlim([0 tfin])
line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
ylabel('$\|e\|_2$', 'interpreter', 'latex')
xlabel('t (s)')
title('Magnitude of Error ($e = \left[e_\phi, \quad e_p\right]^T)$', 'interpreter', 'latex')

set(gcf,'color','w'); % set figure background to white
drawnow;

F(length(tsim)) = struct('cdata',[],'colormap',[]);

F(1) = getframe(1); % save scatter plot as a frame for animation

%     % save animation to .gif file
im = frame2im(F(1));
[A,map] = rgb2ind(im,256); 
imwrite(A,map,filename,'gif','LoopCount',3,'DelayTime',0.05);

for i = 1:500:length(tsim)
    hold off;

    plot(tsim(1:i), PSP.norm_e(1:i), 'LineWidth', 1); hold on; grid on;
    ylim([0 0.4])
    xlim([0 tfin])
    line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
    line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
    ylabel('$\|e\|_2$', 'interpreter', 'latex')
    xlabel('t (s)')
    title('Magnitude of Error ($e = \left[e_\phi, \quad e_p\right]^T)$', 'interpreter', 'latex')
    
    set(gcf,'color','w'); % set figure background to white
    drawnow;

    F(i) = getframe(1); % save scatter plot as a frame for animation
    
%     % save animation to .gif file
    im = frame2im(F(i));
    [A,map] = rgb2ind(im,256); 
    imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
end