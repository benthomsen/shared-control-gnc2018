% Made by Ben Thomsen in January 2018 to go along with paper:
% Benjamin Thomsen, Anuradha M. Annaswamy, and Eugene Lavretsky. "Shared
% Control Between Human and Adaptive Autopilots", 2018 AIAA Guidance,
% Navigation, and Control Conference, AIAA SciTech Forum, (AIAA 2018-1574) 
% 
% https://doi.org/10.2514/6.2018-1574
% 
% simMode argument can be either 'act' for actuator dynamics or 'del' for
% time delay
% 
% Use this from the MATLAB command prompt like:
% ARS = AnomalyResponseSim('act', 1)
% or
% ARS = AnomalyResponseSim('del', 1)
%

classdef AnomalyResponseSim < handle
    properties
        simPars
        postSimPars
        controlSim
        xmSim
    end
    
    methods
        function anom = AnomalyResponseSim(simMode, genPlots)
            if nargin == 0
                simMode = 'act';
                genPlots = 1;
            end
            
            % ****** Parameters ****** 
            SP.tfin  = 180;   % simulation length (sec)
            SP.ts    = 30;    % plant dynamics switch time (anomaly)
            SP.ts_rm = 90;    % controller switching time  (anomaly response)
            SP.dT    = 0.005; % fixed time step

            % 2D reference model dynamics
            bm = 8;
            am0 = 8;
            am1 = 6;
            SP.Am = [0 1; -am0 -am1];
            SP.Bm = [0; bm];

            % 3D reference model dynamics
            bm_3 = 32;
            SP.Am_3d = [0 1 0; 0 0 1; -32 -32 -10];
            SP.Bm_3d = [0; 0; bm_3];

            % Commands
            r_in = [0; 0.15; 0.30];
            SP.r_timed = timeseries(repmat(r_in, SP.tfin/(5*length(r_in))+1, 1), 0:5:(SP.tfin+10));

            % Plant dynamics
            bp1 = 0.318; % DC plant gain before switch
            ppole_1 = 0;    % first plant pole (pure integration for bank angle output)
            ppole_2 = 1.10; % second plant pole
            SP.Ap = [0 1; -ppole_1*ppole_2 -ppole_1-ppole_2];
            SP.Bp = [0; bp1];
            if contains(simMode, 'del')
                SP.tau = 0.2;   % time delay added
            elseif contains(simMode, 'act')
                bp2 = 1.8 * bp1; % DC plant gain after switch
                ppole_3 = 1.8;
                SP.Bp_3 = [0; 0; bp2];
                SP.Ap_3 = [0 1 0; 0 0 1; ...
                        -(ppole_1*ppole_2*ppole_3) -(ppole_1*ppole_2 + ppole_1*ppole_3 + ppole_2*ppole_3) -(ppole_1 + ppole_2 + ppole_3)];
            else
                error('Invalid simMode selected')
            end

            % ****** Adaptive Parameters ****** 

            gamma = 10;     % learning rate
            SP.gamma      = gamma;
            SP.gamma_3d_q = gamma; % learning rate on q with 3D controller
            if contains(simMode, 'del')
                SP.gamma_3d_th = gamma * [1 0 0; 0 1 0; 0 0 0.01]; % learning rate on 3D feedback
                SP.Lm    = (- SP.Am - gamma*eye(2));  % feedback gains on error for reference model
                alpha = 0.91;                   % coefficient to scale Lm_3d by
                SP.Lm_3d = (-SP.Am_3d - SP.gamma_3d_th) * alpha; % higher-dimensional CRM feedback gain
            elseif contains(simMode, 'act')
                SP.gamma_3d_th = gamma*eye(3);
                SP.Lm = (- SP.Am -gamma*eye(2));   % feedback gains on error for reference model
                SP.Lm_3d = -SP.Am_3d -gamma*eye(3);
            end

            SP.P    = lyap((SP.Am+SP.Lm)', gamma*eye(2));      % lyapunov equation solution with Q = gamma*eye(2)
            SP.P_3d = lyap((SP.Am_3d+SP.Lm_3d)', SP.gamma_3d_th); % lyapunov equation solution with Q = gamma_3d_th

            % initial conditions - here set to true values for sims in paper
            scale = 1.0; % scale all ICs compared to matching cond. values
            bp1_s0 = bp1 * scale;
            ppole_1_s0 = ppole_1 * scale;
            ppole_2_s0 = ppole_2 * scale;

            SP.q_0       = bm/bp1_s0;
            SP.theta_0_1 = (ppole_1_s0*ppole_2_s0 - am0)/bp1_s0; % (ap0-am0)/bp1, but ap0 is 0
            SP.theta_0_2 = ((ppole_1_s0 + ppole_2_s0) - am1)/bp1_s0;
            SP.Theta_0   = [SP.theta_0_1, SP.theta_0_2];
            
            anom.simPars = SP;
            
            anom.runSim(simMode);
            if genPlots
                anom.plotSim(simMode);
            end
            
        end
    end
    
    methods (Access = private)
        function runSim(anom, simMode)
                        
            names = fieldnames(anom.simPars);
            for i=1:length(names)
                str = sprintf('%s = anom.simPars.%s;', names{i}, names{i});
                eval(str);
            end

            % ****** Simulink ****** 

            options = simset('SrcWorkspace','current');
            
            tsimstart = tic;
            if contains(simMode, 'del')
                disp('Starting simulation of anomalous time delay');
                anom.controlSim = sim('AnomalyResponseDelay',[],options);
            elseif contains(simMode, 'act')
                disp('Starting simulation of first-order actuator dynamics')
                anom.controlSim = sim('AnomalyResponseActuators',[],options);
            end
            tsimfin = toc(tsimstart);
            disp(['Simulation finished after ', num2str(tsimfin), ' sec']);

            anom.xmSim = sim('OpenLoopRefModel',[],options);
            
            % ****** Post Processing ******
            % indices of relevant points in returned data
            PSP.i_ts   = ceil(ts/dT)+1;
            PSP.i_tsrm = ceil(ts_rm/dT)+1;
            i_tsrm = PSP.i_tsrm; % just for use here
            PSP.tsim   = anom.controlSim.tout;
            PSP.openLoopResponse = squeeze(anom.xmSim.xm_full_ol);
            PSP.x_deg = squeeze(anom.controlSim.x)*180/pi;
            PSP.r_deg = anom.controlSim.r*180/pi;
            
            PSP.th1 = [anom.controlSim.Theta(1:i_tsrm,1); anom.controlSim.Theta_3d(i_tsrm+1:end,1)];
            PSP.th2 = [anom.controlSim.Theta(1:i_tsrm,2); anom.controlSim.Theta_3d(i_tsrm+1:end,2)];
            PSP.th3 = anom.controlSim.Theta_3d(i_tsrm+1:end,3);
            PSP.q   = [anom.controlSim.q(1:i_tsrm); anom.controlSim.q_3d(i_tsrm+1:end)];
            PSP.th3_aug = [zeros(i_tsrm,1); PSP.th3];

            norm_e = zeros(1, length(PSP.tsim));
            norm_e(1:i_tsrm) = (squeeze(anom.controlSim.e(1,:,1:i_tsrm)).^2 + squeeze(anom.controlSim.e(2,:,1:i_tsrm)).^2).^(0.5);
            norm_e(i_tsrm+1:end) = (squeeze(anom.controlSim.e_3d(1,:,i_tsrm+1:end)).^2 + squeeze(anom.controlSim.e_3d(2,:,i_tsrm+1:end)).^2).^(0.5);
            PSP.norm_e = norm_e;
            
            anom.postSimPars = PSP;
        end
        
        function plotSim(anom, simMode)
            
            SP = anom.simPars;
            ts    = SP.ts;
            ts_rm = SP.ts_rm;
            tfin  = SP.tfin;
            
            PSP = anom.postSimPars;
            i_tsrm = PSP.i_tsrm;
            tsim   = PSP.tsim;
            norm_e = PSP.norm_e;
            
            set(0,'defaultAxesFontSize', 14);

            % Figure 1: Command and Output
            figure('Position',[100, 100, 720, 240])
            plot(tsim, PSP.r_deg, 'LineWidth', 1); hold on; grid on;
            plot(tsim, PSP.x_deg(1,:), 'LineWidth', 1);
            ylim([-2 25])
            line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            ylabel('$\phi$ (deg)', 'interpreter', 'latex')
            xlabel('Time (s)')
            title('Stability Axis Bank Angle', 'interpreter', 'latex')
            legend('Command (r)', 'Bank angle (\phi)')


            % Figure 2: Adaptive Parameters
            figure('Position',[100, 100, 720, 240])
            plot(tsim, PSP.th1, 'LineWidth', 1); hold on; grid on;
            plot(tsim, PSP.th2, 'LineWidth', 1); plot(tsim(i_tsrm+1:end), PSP.th3, 'LineWidth', 1);
            plot(tsim, PSP.q, 'LineWidth', 1);
            % ylim([-1 1])
            line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            ylabel('$\theta_i$', 'interpreter', 'latex')
            xlabel('Time (s)')
            title('Adaptive Parameter Values', 'interpreter', 'latex')
            legend('\theta_1', '\theta_2', '\theta_3', 'q')


            % Figure 3: Error between x(1:2) and xm(1:2) - norm
            figure('Position',[100, 100, 720, 240])
            plot(tsim, norm_e, 'LineWidth', 1); hold on; grid on;
            ylim([0 0.4])
            line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            ylabel('$\|e\|$ ', 'interpreter', 'latex')
            xlabel('Time (s)')
            title('Magnitude of Error ($e = \left[e_\phi, \quad e_p\right]^T)$', 'interpreter', 'latex')

            % % Figure 4: Control signal
            % figure('Position',[100, 100, 720, 240])
            % plot(tsim, anom.controlSim.u, 'LineWidth', 1); hold on; grid on;
            % % ylim([0 0.4])
            % line([ts ts],ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            % line([ts_rm ts_rm], ylim,'Color',[0 0 0],'LineStyle','-.', 'LineWidth', 1);
            % ylabel('$u$ ', 'interpreter', 'latex')
            % xlabel('t (s)')
            % title('Control Input', 'interpreter', 'latex')
            
            % Figure 5: Step response
            % last 5 seconds of stage 2
            i21 = ceil((ts_rm-5)/SP.dT)+1;
            i22 = i_tsrm;
            theta1_bar_t2 = mean(PSP.th1(i21:i22));
            theta2_bar_t2 = mean(PSP.th2(i21:i22));
            qbar_t2       = mean(PSP.q(i21:i22));
            % last 5 seconds of stage 3
            i31 = ceil((tfin-5)/SP.dT)+1;
            theta1_bar_t3 = mean(PSP.th1(i31:end));
            theta2_bar_t3 = mean(PSP.th2(i31:end));
            theta3_bar_t3 = mean(PSP.th3_aug(i31:end));
            qbar_t3       = mean(PSP.q(i31:end));

            s = tf('s');
            rm2 = 8/(s^2+6*s+8);
            rm3 = 32/((s^2+6*s+8)*(s+4));
            figure('Position',[100, 100, 720, 280]);
            if contains(simMode, 'del')
                tau = SP.tau;
                Y_cl_trans_1 = (0.318*qbar_t2)/(s^2 + (1.98 - 0.318*theta2_bar_t2 * exp(-tau*s))*s  - 0.318*theta1_bar_t2 * exp(-tau*s)); % using thetas, q from end of adjustment phase
                Y_cl_3 = (0.318*qbar_t3)/((1 - 0.318*theta3_bar_t3 * exp(-tau*s))*s^2 + (1.98 - 0.318*theta2_bar_t3 * exp(-tau*s))*s  - 0.318*theta1_bar_t3 * exp(-tau*s)); % using thetas, q from end of adjustment phase
                step(rm2, Y_cl_trans_1, Y_cl_3, rm3, 8); 
                title('Unit Step Response: Closed-Loop Behavior with Sensor Delay', 'interpreter', 'latex'); 
            elseif contains(simMode, 'act')
                Y_cl_trans_1 = (0.5724*qbar_t2)/(s^3 + 2.90*s^2 + (1.98 - 0.5724*theta2_bar_t2)*s - 0.5724*theta1_bar_t2); % using thetas, q from end of adjustment phase
                Y_cl_3 = (0.5724*qbar_t3)/(s^3 + (2.90 - 0.5724*theta3_bar_t3)*s^2 + (1.98 - 0.5724*theta2_bar_t3)*s - 0.5724*theta1_bar_t3); % using thetas, q from end of adjustment phase
                step(rm2, Y_cl_trans_1, Y_cl_3, rm3, 8); 
                title('Unit Step Response: Closed-Loop Behavior with Anomalous Actuator Dynamics', 'interpreter', 'latex'); 
            end
            grid on; 
            legend('Second-Order Reference Model', 'Post-Anomaly', 'Post-Correction', 'Third-Order Reference Model', 'location', 'southeast'); 
            
        end
    end
        
end