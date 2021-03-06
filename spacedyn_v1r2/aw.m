%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   version 1.2 // Sep. 13, 2001, Last modification by H.Hamano
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Aw( w0 ) returns a 3x3 transformation representing a 
%	rotation about the vector w0.
%
%
%	See also CX, CY, CZ.

function E0 = aw( w0 )

global d_time


if ( norm(w0)==0 )
   E0 = eye(3);
   
else
   th = norm(w0) * d_time;
   w = w0 ./ norm(w0);
   
   E0 =[ cos(th)+w(1)^2*(1-cos(th)) ...
         w(1)*w(2)*(1-cos(th))-w(3)*sin(th) ...
         w(3)*w(1)*(1-cos(th))+w(2)*sin(th);
         
         w(1)*w(2)*(1-cos(th))+w(3)*sin(th) ...
         cos(th)+w(2)^2*(1-cos(th)) ...
         w(3)*w(2)*(1-cos(th))-w(1)*sin(th);
         
         w(3)*w(1)*(1-cos(th))-w(2)*sin(th) ...
         w(3)*w(2)*(1-cos(th))+w(1)*sin(th) ...
         cos(th)+w(3)^2*(1-cos(th)) ];
   
end

%%%EO
