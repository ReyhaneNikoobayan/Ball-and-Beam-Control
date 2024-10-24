
s=tf('s');
theta_v=0.0274/(0.003228*s^2 + 0.003508*s);
R_theta=0.36/(s^2 +0.24*s);
gain=0.2;
G=theta_v*R_theta*gain*0.5;

Q=1/(G*((s+1)^4));
K=Q/(1-Q*G);

