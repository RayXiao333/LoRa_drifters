

T = 0.05;
beta = 0.005;
X_INS=zeros(5,1);
Y_GPS = zeros(3,1)
Bias_pre = zeros(3,1);
P_0 = [10,10,10,10,90*3.14/180,5,5,25*3.14/180];
H = [zeros(3,2) eye(3) zeros(3)];
P = diag(P_0);
R=diag([1,1,3.14/180]);
Q=diag([0.1,0.1,0.1*3.14/180]);

%input:
U = [0; 0; 0];

%Loop: 
C = [cos(X_INS(5,1)), sin(X_INS(5,1)); cos(X_INS(5,1)), -sin(X_INS(5,1))];
A_E=[ [zeros(2);eye(2);zeros(4,2)] zeros(8,3) [C; zeros(6,2)] [zeros(4,1);1;zeros(3,1)] ];
B_E = [[C;zeros(6,2)] [zeros(4,1);1;zeros(3,1)] ];

U_k = U - Bias_pre;
X_INS = [X_INS(1)+ T*U_k(1); X_INS(2)+ T*U_k(2); 0.5*T*X_INS(1)+X_INS(3); 0.5*T*X_INS(2)+X_INS(4); X_INS(5)+T*U_k(3) ]
P = A_E*P*transpose(A_E) + B_E*Q*transpose(B_E) + P_0*beta;

Y_E = X_INS([3:5])-Y_GPS;
G_k = P*transpose(H) * inv(H*P*transpose(H)+R);
P=(eye(8)-G_k*H)*P;
X_E_Pre=[zeros(5,1);Bias_pre] + G_k*Y_E;
Bias_pre = X_E_Pre([6:8]);


