function A = dh_matrix(alpha_i, a_i, d_i, theta_i)
% DH_MATRIX(alpha_i, a_i, d_i, theta_i)  Computes the DH matrix

ci = cosd(theta_i * 180 / pi);
si = sind(theta_i * 180 / pi);
cai = cosd(alpha_i * 180 / pi);
sai = sind(alpha_i * 180 / pi);

A(1, :) = [ci,  -cai*si,    sai*si,     a_i*ci  ];
A(2, :) = [si,  cai*ci,     -sai*ci,    a_i*si  ];
A(3, :) = [0,   -sai,       cai,        d_i     ];
A(4, :) = [0,   0,          0,          1       ];

end