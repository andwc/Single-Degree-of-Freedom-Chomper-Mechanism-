load('mydata.mat');
rho=D_dot_x_history;
phi=linspace(0,2*pi,length(rho));
figure(1)
polarplot(phi,rho);
