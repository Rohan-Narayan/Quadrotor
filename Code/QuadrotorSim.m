%TEST CODE

clear all
clc

deg2rad = @(x) x*(pi/180)

phi = deg2rad( 43 );
theta = deg2rad( 46 );
psi = deg2rad( 21 );
q_x =  [ cos( phi/2 ) sin( phi/2 )*[1 0 0] ]';
q_y =  [ cos( theta/2 ) sin( theta/2 )*[0 1 0] ]';
q_z =  [ cos( psi/2 ) sin( psi/2 )*[0 0 1] ]';

% test the quat2RotMatrix
R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]
quat2RotMatrix( q_x ) 

R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]
quat2RotMatrix( q_y )


R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1 ]
quat2RotMatrix( q_z )

% test the quatMultiplication
R_zyx = R_z* R_y * R_x
q_zyx = quatMultiplication( quatMultiplication( q_z, q_y ), q_x );
quat2RotMatrix( q_zyx ) 


% test rotMatrix2Quat
q_x_rec = rotMatrix2Quat(R_x);
norm(q_x - q_x_rec)

q_y_rec = rotMatrix2Quat(R_y);
norm(q_y - q_y_rec)

q_z_rec = rotMatrix2Quat(R_z);
norm(q_z - q_z_rec)

q_zyx_rec = rotMatrix2Quat(R_zyx);
norm(q_zyx - q_zyx_rec)


%FUNCTIONS
function [q] = eulerAngles2Quat (eulerAngles)
%euler_angles = [ roll pitch yaw ] - transpose
%q = quaternion = [ qw qx qy qz ] - transpose

q = zeros(4,1);

r = eulerAngles(1);
p = eulerAngles(2);
y = eulerAngles(3);
r2 = r/2.0;
p2 = p/2.0;
y2 = y/2.0;

q(1) = cos(r2)*cos(p2)*cos(y2) + sin(r2)*sin(p2)*sin(y2);
q(2) = sin(r2)*cos(p2)*cos(y2) - cos(r2)*sin(p2)*sin(y2);
q(3) = cos(r2)*sin(p2)*cos(y2) + sin(r2)*cos(p2)*sin(y2);
q(4) = cos(r2)*cos(p2)*sin(y2) - sin(r2)*sin(p2)*cos(y2);

end

function [R] = eulerAngles2RotMatrix (eulerAngles)
%euler_angles = [ roll pitch yaw ] - transpose
%R = Rotation Matrix = Rz(yaw)*Ry(pitch)*Rx(roll)

roll = eulerAngles(1);
pitch = eulerAngles(2);
yaw = eulerAngles(3);

R = [ cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
    sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
    -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)];

end

function [qNext] = integrateQuaternionDiscrete (qCurrent, omega, dt)
%q = quaternion = [ qw qx qy qz ] - transpose
%omega = constant body rate = [omega_x omega_y omega_z] - transpose
%dt = time step (integrated over)

omegaNorm = norm(omega);

if (omegaNorm > 1e-4)
    [ ~, Q_bar ] = quatQ([ 0; omega]);
    Lambda = 1/2*Q_bar;
    qNext = (eye(4)*cos(omegaNorm*dt/2) + 2/omegaNorm*Lambda*sin(omegaNorm*dt/2))*qCurrent;
else
    qNext = qCurrent;
end

end

function [ qSkew ] = omega2QuatSkew( omega )
% Converts rates to a quaternion skew.
% q = quaternion = [ qw qx qy qz ] - transpose    
%omega = onnstant body rate = [omega_x omega_y omega_z] - transpose

qSkew = [0       , -omega(1), -omega(2), -omega(3);...
        omega(1),  0       ,  omega(3), -omega(2);...
        omega(2), -omega(3),  0       ,  omega(1);...
        omega(3),  omega(2), -omega(1),  0        ];
end

function [ alpha, vector ] = quat2AngleAxis( q )
%QUAT2ANGLEAXIS q = [qw qx qy qz]'
    alpha = 2*acos( q(1) );
    vector = q(2:4)/sin(alpha/2);

end

function [eulerAngles] = quat2EulerAngles(q)
% euler_angles = [roll pitch yaw]'
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

eulerAngles = zeros(3,1);

eulerAngles(1) = atan2(2*q(1)*q(2) + 2*q(3)*q(4), q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4));
eulerAngles(2) = -asin(2*q(2)*q(4) - 2*q(1)*q(3));
eulerAngles(3) = atan2(2*q(1)*q(4) + 2*q(2)*q(3), q(1)*q(1) + q(2)*q(2) - q(3)*q(3) - q(4)*q(4));

end

function [R] = quat2RotMatrix(q)
% Transforms a quaternion q into a rotation matrix.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

[ Q, Q_bar ]= quatQ(q);

%
% R_tmp = [ 1 0; 0 R(q) ] = Q_bar(q)'*Q(q)
%

R_tmp = Q_bar'*Q;
R = R_tmp(2:4,2:4);
end

function [ omega ] = quat2omega( q2, q1, dt )
%QUAT2OMEGA computes the angular rates omega (in world frame) for a transition from q2 to q1
% within dt. 

    q_dot = (q2-q1)/dt;
    q2_bar = [q2(1);-q2(2:4)];

    tmp = 2*quatMultiplication( q_dot, q2_bar ); % in world system
%     B = 2*quatMultiplication( q2_bar, q_dot ); % in Body system 
    
    omega = tmp(2:4);
end

function [q_inv] = quatInverse(q)
% Takes the inverse of a quaternion.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

q_inv = [q(1); -q(2:4)];

end

function [mult] = quatMultiplication(q, p)
% Multiplies the two quaternions q*p in the indicated order.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.
[ Q, ~ ]= quatQ(q); 
  
mult = Q*p;

end

function [ Q, Q_bar ] = quatQ( q )

Q = [ q(1) -q(2) -q(3) -q(4);
      q(2)  q(1) -q(4)  q(3);
      q(3)  q(4)  q(1) -q(2);
      q(4) -q(3)  q(2)  q(1) ];

Q_bar = [ q(1) -q(2) -q(3) -q(4);
          q(2)  q(1)  q(4) -q(3);
          q(3) -q(4)  q(1)  q(2);
          q(4)  q(3) -q(2)  q(1) ];

end

function [ bodyrates ] = quaternionDeltaToBodyRates( q2, q1, dt )
%Computes the angular velocity in body 
% coordinate system for a rotation from q1 to q2 within the time dt

    q_e = quatMultiplication(quatInverse(q1), q2);
    if (q_e(1) < 0)
        q_e = -q_e;
    end

    a = sinc(acos(q_e(1)) / pi);

    alpha = [q_e(2) / a, q_e(3) / a, q_e(4) / a]';

    bodyrates = 2.0 * alpha / dt;
end

function [ bodyrates ] = quaternionRatesToBodyRates( q2, q1, dt )
%Computes the angular velocity in body 
%coordinate system for a rotation from q1 to q2 within the time dt


	% Note, that the space of unit quaternion S(3) double covers the space
	% of physical attitudes SO(3) therefore q = -q.
	% I want the minimal q_dot, therefore I need to check which is
	% better q or -q (they both represent the same attitude in space)

	if( norm(-q2 -q1) < norm(q2 -q1) )
		q2 = -q2;
    end
    
	q_dot = ( q2 - q1 ) /dt;

	% [ o W_omega ]' = 2q_dot * q_bar in world frame
	% [ o B_omega ]' = 2q_bar * q_dot in body frame

    bodyrates = 2*(  quatMultiplication( quatInverse(q2), q_dot ));
    bodyrates = bodyrates(2:4);       
end

function [euler_angles] = rotMatrix2EulerAngles(R)
% Computes the euler angles from a rotation matrix using the y-y-x convention.
% R = Rz(yaw)*Ry(pitch)*Rx(roll)
% euler_angles = [roll pitch yaw]'

euler_angles = zeros(3,1);

euler_angles(1) = atan2(R(3, 2), R(3, 3));
euler_angles(2) = -asin(R(3, 1));
euler_angles(3) = atan2(R(2, 1), R(1, 1));

end

function q = rotMatrix2Quat(M)

i = 1; j = 2; k = 3;
if M(2,2) > M(1,1)
    i = 2; j = 3; k = 1;
end

if M(3,3) > M(i,i)
    i = 3; j = 1; k = 2;
end

t = M(i,i) -(M(j,j) + M(k,k)) + 1;
q = [1 0 0 0]';
if (t > 1e-10)
    q(1) = M(k,j) - M(j,k);
    q(i+1) = t;
    q(j+1) = M(i,j) + M(j,i);
    q(k+1) = M(k,i) + M(i,k);
    q = q * 0.5 / sqrt(t);
end

q = q/norm(q);

end

function [v_rot] = rotateVectorByQuat(v,q)
% Rotates a vector v by a quaternion q.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.
% The vector v is defined as v = [vx vy vz]'

[ Q, Q_bar ]= quatQ(q);

v_rot_hom = Q_bar'*Q*[0; v];

v_rot = v_rot_hom(2:4);

end

function [ M ] = skew( v )
%Computes the skew symetric matrix M from a vector v
M = [ 0  -v(3)  v(2);
 v(3)   0 -v(1);
-v(2)  v(1)   0];

end

function [ q ] = slerp( q0, q1, alpha )
%Slerp = Spherical Linear Interpolation
%computes the interpolation between two quaternions

omega = acos( q0'*q1 );

q = sin( (1-alpha)*omega )/sin( omega )*q0 + sin( (alpha)*omega )/sin( omega )*q1; 

end

function [ v ] = unskew( M )
%Returns the vector v from a skew symetric matrix M
%  0  -w3  w2 
%  w3   0 -w1
% -w2  w1   0
    v = [ M(3,2);M(1,3);M(2,1)  ];
    if ( norm( v+[ M(2,3); M(3,1); M(1,2) ] ) > 0.1 )
       fprintf( 'Matrix not skew symmetric\n' )
        v = []
    end

end

