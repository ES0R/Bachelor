% save controller to file with open fileID
%
% [] = saveControllerToFile(fileID, name, sys, w, kp, ti, Ni, td, al, gm)
% fileID = ID of open file
% name = controller ID (string)
% sys  = transfer function of system (info only)
% w    = crossover frequency [rad/s] (info only)
% kp   = Kp (proportional gain)
% ti   = PI time constant [sec]
% Ni   = PI zero position (info only)
% td   = Lead time constant [sec]
% al   = Lead alpha
% gm   = phase margin in design [degrees] (info only)
%
function [] = saveControllerToFile(fileID, name, sys, w, kp, ti, Ni, td, al, gm)
    fprintf(fileID, '[%s]\n', name);
    fprintf(fileID, '# transfer function (matlab style)\n');
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
    fprintf(fileID, 'crossover %g\n', w);
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
    fprintf(fileID, 'gamma_m %g\n', gm);
    fprintf(fileID, '#\n');
end