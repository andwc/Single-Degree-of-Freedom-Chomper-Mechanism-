function [phi, rho] = cartesianToPolar(x, y)
    rho = sqrt(x.^2 + y.^2);
    phi = atan2(y, x);
    % 注意：atan2返回的角度范围是[-pi, pi]。如果需要[0, 2*pi]范围，
    % 可以根据具体需求调整phi值。
    % 如果希望phi在[0, 2*pi]范围内，可以使用如下代码：
    phi = mod(phi, 2*pi);
end
clear;
img = imread("凸轮2.png");
img_gray = rgb2gray(img);
binary = imbinarize(img_gray); 
[B, ~] = bwboundaries(binary);
contour_points = B{2}; 
y=contour_points(:,2);
x=contour_points(:,1);
x=(x-693);
y=(y-343);
[phi,rho]=cartesianToPolar(x,y);
phi=mod(phi-pi/2,pi*2);
[phi2, idx] = sort(phi); 
rho2 = rho(idx);
beishu = 1.613/rho2(1);
rho2 = rho2*beishu;

img = imread("凸轮1.png");
img_gray = rgb2gray(img);
binary = imbinarize(img_gray); 
[B, ~] = bwboundaries(binary);
contour_points = B{2}; 
y=contour_points(:,2);
x=contour_points(:,1);
beishu=min(sqrt(x.^2+y.^2));
x=(x-226)/beishu;
y=(y-1323)/beishu;
[phi,rho]=cartesianToPolar(x,y);
phi=mod(phi-pi/2,pi*2);
[phi1, idx] = sort(phi); 
rho1 = rho(idx);
beishu = 3.775/rho1(1);
rho1 = rho1*beishu;

img = imread("凸轮3.png");
img_gray = rgb2gray(img);
binary = imbinarize(img_gray); 
[B, ~] = bwboundaries(binary);
contour_points = B{2}; 
y=contour_points(:,2);
x=contour_points(:,1);
beishu=min(sqrt(x.^2+y.^2));
x=(x-132)/beishu;
y=(y-326)/beishu;
[phi,rho]=cartesianToPolar(x,y);
phi=mod(phi-pi/2,pi*2);
[phi3, idx] = sort(phi); 
rho3 = rho(idx);
beishu = 4.675/rho3(1);
rho3 = rho3*beishu;


t_phi = [phi1./6;phi2./6+pi/3;phi2./6+2*pi/3;phi2./6+pi;phi2./6+4*pi/3;phi3./6+5*pi/3;phi1(1)];
t_rho = [rho1;rho2;rho2;rho2;rho2;rho3;rho1(1)];

figure(1);
polarplot(-t_phi,t_rho);


print('下颚凸轮拟合高分辨率.png', '-dpng', '-r600');