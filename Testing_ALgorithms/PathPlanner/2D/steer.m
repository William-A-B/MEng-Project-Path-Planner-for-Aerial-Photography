function A = steer(qr, qn, val, eps)
   qnew = [0 0];

   % Steer towards qn with maximum step size of eps
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist(qr,qn);
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
   end   
   A = [qnew(1), qnew(2)];
end

% function A = steer(qr, qn, val, eps)
%    % qn is the nearest existing node
%    % qr is the target (next planned point)
%    % val is the distance between them
%    % eps is the max step size
%    % mode determines horizontal or vertical movement restriction
% 
%    mode = 'vertical'
% 
%    qnew = [0 0];
% 
%    % Move only in X or Y, depending on predefined path
%    if strcmp(mode, 'horizontal')  % Enforce movement along X first
%        if abs(qr(1) - qn(1)) >= eps
%            qnew(1) = qn(1) + sign(qr(1) - qn(1)) * eps; % Step in X direction
%            qnew(2) = qn(2); % Keep Y the same
%        else
%            qnew(1) = qr(1);
%            qnew(2) = qr(2);
%        end
%    else  % 'vertical' mode, move along Y
%        if abs(qr(2) - qn(2)) >= eps
%            qnew(1) = qn(1); % Keep X the same
%            qnew(2) = qn(2) + sign(qr(2) - qn(2)) * eps; % Step in Y direction
%        else
%            qnew(1) = qr(1);
%            qnew(2) = qr(2);
%        end
%    end
% 
%    A = [qnew(1), qnew(2)];
% end
% 
% function A = steer(qr, qn, val, eps)
%    qnew = [0 0];
% 
%    % Determine whether to move in X or Y direction
%    if qn(1) == qr(1)  % If X is aligned, move in Y
%        if abs(qr(2) - qn(2)) >= eps
%            qnew = [qn(1), qn(2) + sign(qr(2) - qn(2)) * eps];
%        else
%            qnew = qr;  % Stop at target
%        end
%    else  % If Y is aligned, move in X
%        if abs(qr(1) - qn(1)) >= eps
%            qnew = [qn(1) + sign(qr(1) - qn(1)) * eps, qn(2)];
%        else
%            qnew = qr;
%        end
%    end
% 
%    A = qnew;
% end