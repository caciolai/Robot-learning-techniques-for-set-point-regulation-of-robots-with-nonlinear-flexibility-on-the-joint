function G = g(in1)
%G
%    G = G(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    24-Apr-2021 11:33:28

q1 = in1(1,:);
q2 = in1(2,:);
t2 = q1+q2;
t3 = cos(t2);
t4 = t3.*3.92266;
t5 = -t4;
G = [t5-cos(q1).*3.92266;t5];
