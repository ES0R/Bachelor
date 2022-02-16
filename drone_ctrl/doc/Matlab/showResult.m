% Show display and/or save to file
% closed loop assumes Lead in return path
%
% [C, Cd] = showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
%
% plt = set to 1 to enable plot (else 0 for silent)
% fil = add results to file with name 'fn'
% fn = filename where result is added
% sys system to control
% kp = proportional gain
% ti = PI time constant [sec] (0 if none)
% Ni = integrator design number
% td = Lead time constant [sec]
% al = Lead alpha (1 if none)
% gm = desired phase margin
% tle= title string
% returns:
% C = Kp * Ci   - controller in forward path
% Cd =          - lead term in return path
%
function [C, Cd] = showResult(plt, fil, fn, name, sys, w, kp, ti, Ni, td, al, gm, tle)
    % build controller
    C = kp;
    if Ni > 0 % I-term
        C = C * tf([ti 1],[ti 0]);
    end
    Cd = 1;
    if al < 1 % Lead term
        Cd = tf([td 1],[al*td 1]);
    end
    Gol = sys*C*Cd; % open loop
    % phase margin and crossover frq
    [Gm,Pm,Wcg,Wcp] = margin(Gol); 
    Gcl = minreal(sys*C/(1+Gol)); % closed loop
    if plt
        ww = logspace(log10(Wcp)-2, log10(Wcp)+0.5, 500);
        [Ms,Ps,Wo] = bode(sys,ww);
        [Mo,Po] = bode(Gol, Wo);
        [Mc,Pc] = bode(Gcl,Wo);
        bw = bandwidth(Gcl);
        figure
        subplot(2,2,1)
        hold off
        % bode magnitude
        semilogx(Wo, 20*log10(squeeze(Ms)))
        hold on
        semilogx(Wo, 20*log10(squeeze(Mo)))
        semilogx(Wo, 20*log10(squeeze(Mc)))
        grid on
        ylabel('Magnitude');
        title(tle,'Color','red');
        txt = {['Crossover = ' num2str(Wcp) ' rad/s'],...
               ['Bandwidth = ' num2str(bw) ' rad/s']};
        annotation('textbox',[0.15,0.3,0.3,0.4], ...
            'String',txt,'EdgeColor','none','FontSize',9)
        % phase
        subplot(2,2,3)
        hold off
        semilogx(Wo, squeeze(Ps))
        hold on
        semilogx(Wo, squeeze(Po))
        semilogx(Wo, squeeze(Pc))
        grid on
        ylabel('Phase (deg)');
        xlabel('Frequency (rad/s)')
        legend('system','open loop','closed loop','Location','south west');
        % step responce
        subplot(2,2,2)
        hold off
        step(Gcl)
        grid on
        si = stepinfo(Gcl);
        txt = {['RiseTime = ' num2str(si.RiseTime) ' s'],...
               ['Overshoot= ' num2str(si.Overshoot)], ...
               ['PeakTime = ' num2str(si.PeakTime) ' s']};
        annotation('textbox',[0.65,0.35,0.3,0.4], ...
            'String',txt,'EdgeColor','none','FontSize',9)
        subplot(2,2,4)
        hold off
        nyquist(Gol)
        axis([-6,1,-3,3])
        grid on
        fig = gcf;    %or one particular figure whose handle you already know, or 0 to affect all figures
        set( findall(fig, '-property', 'fontsize'), 'fontsize', 8)
    end
    if fil
        fileID=fopen(fn,'a');
        fprintf(fileID, '#\n');
        fprintf(fileID, '# %s\n', tle);
        fprintf(fileID, '[%s]\n', name);
        fprintf(fileID, '# system transfer function (matlab style)\n');
        [num,den] = tfdata(sys);
        fprintf(fileID, 'num');
        for i = 1:size(num{1},2)
            fprintf(fileID, ' %g', num{1}(i));
        end
        fprintf(fileID, '\nden');
        for i = 1:size(den{1},2)
            fprintf(fileID, ' %g', den{1}(i));
        end
        fprintf(fileID, '\n');
        fprintf(fileID, '# crossover frequency (rad/s)\n');
        fprintf(fileID, 'crossover %g (%g)\n', Wcp, w);
        fprintf(fileID, 'kp %g\n', kp);
        if Ni > 0
          fprintf(fileID, '# Has I-term\n');
          fprintf(fileID, 'tau_i %g\n', ti);
          fprintf(fileID, 'Ni %g\n', Ni);
        end
        if (al < 1)
          fprintf(fileID, '# Has Lead-term\n');
          fprintf(fileID, 'tau_d %g\n', td);
          fprintf(fileID, 'alpha %g\n', al);
        end
        fprintf(fileID, '# phase margin (deg)\n');
        fprintf(fileID, 'gamma_m %g (%g)\n', Pm, gm);
        fprintf(fileID, '#\n');
        fclose(fileID);
    end
end