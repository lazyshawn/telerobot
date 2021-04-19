%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The SpaceDyn, a MATLAB toolbox for Space and Mobile Robots.
% (C)1998 The Space Robotics Lab. directed by Kazuya Yoshida,
% Tohoku University, Japan.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   version 1 // Oct 4, 1999, Last modification by K.Yoshida
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% I_DYN		Caluclation of Forward Dynamics
%
%		Test File of inverce dynamics
%
%	'06 1/15 H.Nakanishi
%

function Force = i_dyn(R0, A0, v0, w0, vd0, wd0, q, qd, qdd, Fe, Te)

global BB S0 SS SE J_type Qi Qe
global c0 cc ce
global m0 m mass inertia0 inertia
global num_q Ez Gravity


% Calculation of coordinate transfromation matrices
AA = calc_aa( A0, q );

% Calculation of position vectors
RR = calc_pos( R0, A0, AA, q );

% Calculation of inertia matrices, HH
HH = calc_hh( R0, RR, A0, AA );

% Calculation of velocty dependent term, Force0
% This is obtained by the RNE inverse dynamics computation with
% the accerelations and external forces zero.
qdd0 = zeros(num_q,1);
acc0 = zeros(3,1);
fe0  = zeros(3,num_q);

Force0 = r_ne( R0, RR, A0, AA, v0, w0, acc0, acc0, q, qd, qdd0, fe0, fe0 );

% Force = forces on the generalized coordinate.
% Force_ex = forces on the end points.
Force = zeros(6+num_q,1);
Force_ex = zeros(6+num_q,1);

% % F0, T0 are forces on the centroid of the 0-th body.
% Force(1:3) = F0;
% Force(4:6) = T0;

% If Multi body system, tau is a joint torque.
% if ( num_q ~= 0 )
%    Force(7:num_q+6) = tau;
% end

% Calculate external forces

% If single body system, no external forces.
if num_q == 0
   % Note that the body 0 cannot have an endpoint.
   Fx   = zeros(3,1);
   Tx   = zeros(3,1);
   taux = [];
   
% Multi body system
else
   Fx    = zeros(3,1);
   Tx    = zeros(3,1);
   taux  = zeros(num_q,1);
   
   E_3 = eye(3,3);
   O_3 = zeros(3,3);
   num_e = 1;
   
   for i = 1 : num_q
      
      if SE(i)==1
         joints = j_num(num_e);
         tmp = calc_je(RR, AA, q, joints);
         JJ_tx_i = tmp(1:3,:);
         JJ_rx_i = tmp(4:6,:);
         
         num_e = num_e + 1;
         
         A_I_i = AA(:,i*3-2:i*3);
         Re0i = RR(:,i) - R0 + A_I_i*ce(:,i);
         
         Me_i = [         E_3      O_3;
                  tilde(Re0i)      E_3;
                     JJ_tx_i'  JJ_rx_i' ];
         F_ex(:,i) = Me_i * [ Fe(:,i) ; Te(:,i) ];
         
      end
      
   end
   
   for i = 1 : num_q
      
      Fx   = Fx   + F_ex(1:3,i);
      Tx   = Tx   + F_ex(4:6,i);
      taux = taux + F_ex(7:6+num_q,i);
      
   end
   
end

Force_ex(1:3) = Fx;
Force_ex(4:6) = Tx;
Force_ex(7:6+num_q) = taux;

% Calculation of the acclelation
%a_Force = Force - Force0 + Force_ex;

Acc(1:3) = vd0;
Acc(4:6) = wd0;
Acc(7:6+num_q) = qdd;

a_Force = HH * Acc';

Force = a_Force + Force0 - Force_ex;

% Acc = HH\a_Force;
% Acc = inv(HH)*a_Force;
% 
% vd0 = Acc(1:3);
% wd0 = Acc(4:6);
% qdd = Acc(7:6+num_q);

% if num_q == 0
%    qdd=[];
% end


%%%EOF
