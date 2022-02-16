% function to evaluate P-Lead controller parameters 
% based on system, alpha and desired phasemargin
%
% [wc, Kp, td, ok] = solvepd(G1, gm, al, w)
%            G1 is system to design for
%            gm is desired phase margin
%            al is desired alpha
%            w is optional frequency vector (e.g. w = logspace(-1,4,2000))
% result
%            ok is cound of valid solutions
%            wc is crossing frequency
%            Kp is proportional gain
%            td is time constant for Lead
% 
% NB! do not work for systems with delay
%
function [wc, Kp, taud, ok] = findpd(G1, gm, al, w, N0)
    % find phase to look for
    phiM = asin((1-al)/(1+al)) * 180/pi;
    PG = -180 + gm - phiM;
    % test if frequency range is specified
    if nargin < 4
        % else use Bode plot default
        [M,P0,W]=bode(G1);
    else
        [M,P0,W]=bode(G1, w);
    end
    if nargin < 5
        N = 0;
        % assume lowest frequency has
        % a phase less than +180 degree
        N = 0;
        while P0(1) - N * 360 > 180
            N = N + 1;
        end
    else
        N = N0;
    end
    % make phase vector with start
    % frequenct less than +180
    P = P0 - N * 360;
    % find phase angle at lowest frequency
    % for finding number of phase crossings
    [m1,p1] = bode(G1,W(1));
    vs = p1 - PG - N * 360;
    n = 0;
    % find number of phase crossings
    for i = 2:size(W,1)
       v = P(i) - PG;
       if v*vs < 0
           n = n+1;
           % interpolate (linear) to potential crossing frequency
           wcs(n) = W(i)-(v/(v - vs)*(W(i) - W(i-1)));
       end
       vs = v;
    end
    % design lead for all crossings
    for j = 1:n
        td(j) = 1/(sqrt(al)*wcs(j));
        Cd = tf([td(j) 1],[td(j)*al 1]);
        [mj,pj] = bode(G1*Cd,wcs(j));
        kp(j) = 1/mj;
        Gol(j) = kp(j)*Cd*G1;
    end
    % test if solutions are stable.
    % use the stable solution with highest frequency
    nOK = 0; % index with OK solution
    ok = 0;  % count of ok solutions
    for k = 1:n
        % make closed loop
        Gcl(k) = feedback(Gol(k),1);
        % test for stability
        try
        if (isstable(Gcl(k)))
            nOK = k;
            ok = ok + 1;
        end
        catch ME
            sprintf('Is stable test failed, probably not OK')
        end
    end
    % return found parameters.
    if ok > 0
        Kp = kp(nOK);
        taud = td(nOK);
        wc = wcs(nOK);
    else
        % no solution
        Kp = 1;
        taud = 0;
        wc = 1;
    end
    sprintf('Found %d valid solution(s) out of %d phase crossing(s)', ok, n)
end
