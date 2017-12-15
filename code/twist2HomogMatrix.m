function H = twist2HomogMatrix(twist)

% twist2HomogMatrix Convert twist coordinates to 4x4 homogeneous matrix
%
% Input:
% -twist(6,1): twist coordinates. Stack linear and angular parts [v;w]
% Output:
% -H(4,4): Euclidean transformation matrix (rigid body motion)

v = twist(1:3); % linear part
w = twist(4:6); % angular part

se_matrix = [Cross2Matrix(w), v; 0 0 0 0]; % Lie algebra matrix
H = expm(se_matrix);

end

function M = Cross2Matrix(x)

M = [0,-x(3),x(2); x(3),0,-x(1);-x(2),x(1),0];

end