clear;
clc;
%{ 
—————————————————单自由度大嘴花动作全过程仿真——————————————————
本机构由  翻斗花园第一突击队  设计  
matlab文件由  陈俊逸  完成
————————————————————————————————————————————————————————————
颜色区分：
    红色为机架（即电机系）
    紫色为茎
    蓝色为上颚
    绿色为下颚
————————————————————————————————————————————————————————————
渲染说明：
    初始文件因只有最终的外部结构部分，如果想看细节部分，取消代码最下方渲染的注释即可。
（297行开始）
————————————————————————————————————————————————————————————
标号规则：

点的标号没有下划线后缀的是茎的
_dot是上颚的
_ddot是下颚的
后缀为2的是世界坐标系下的
U点是电机位置

如果不清楚点的对应关系，可将文件最后的注释代码选中（382行开始），ctrl+t一键取消注释
注意：这会大量损失流畅度，造成卡顿。
————————————————————————————————————————————————————————————
茎上的齿轮为完整齿轮
传动凸轮的齿轮均为非完整齿轮

！特别说明：
在仿真的时候算凸轮角度的时候写了很多if else
    有一些判断是初始条件优化后的结果，但传动比恒定，实际是可行的。
    有一些是非完整齿轮的结果。

%}


%极坐标转换函数
function [phi, rho] = cartesianToPolar(x, y)
    rho = sqrt(x.^2 + y.^2);
    phi = atan2(y, x);
    phi = mod(phi, 2*pi);
end
%画滑块用的函数
function [huakuai_x,huakuai_y] = huakuai(x,y,phi,h_rho,h_phi)
    h1=[x+h_rho*cos(h_phi+phi),y+h_rho*sin(h_phi+phi)];
    h2=[x+h_rho*cos(pi-h_phi+phi),y+h_rho*sin(pi-h_phi+phi)];
    h4=[x+h_rho*cos(-h_phi+phi),y+h_rho*sin(-h_phi+phi)];
    h3=[x+h_rho*cos(pi+h_phi+phi),y+h_rho*sin(pi+h_phi+phi)];
    huakuai_x=[h1(1),h2(1),h3(1),h4(1),h1(1)];
    huakuai_y=[h1(2),h2(2),h3(2),h4(2),h1(2)];
end
%将凸轮图片转换为坐标点（参数需要额外开一个文件手动调试）
function [p,r] = img2dot(file_name,px,py,l,alpha)
    img = imread(file_name);
    img_gray = rgb2gray(img);
    binary = imbinarize(img_gray);
    [B, ~] = bwboundaries(binary);
    contour_points = B{2};
    y=contour_points(:,2);
    x=contour_points(:,1);
    x=(x-px);
    y=(y-py);
    [phi,rho]=cartesianToPolar(x,y);
    phi=mod(phi-pi/2,pi*2);
    [p, idx] = sort(phi);
    p=p+alpha;
    r = rho(idx);
    beishu = l/r(1);
    r = r*beishu;
end


%% 参数设置
%茎
CD=24;
AB=22;
ACC=18;
AE=38-ACC;
EG=20;
GI=102;
%上颚
CD_dot=15;
AC_dot=10;
img = imread('上颚轮廓.png');
img_gray = rgb2gray(img);
binary = imbinarize(img_gray);
[B, ~] = bwboundaries(binary);
contour_points = B{2};
sy=-contour_points(:,1);
sx=contour_points(:,2);

sx=(sx-300);
sy=(sy+820);
sx=sx/900*190;
sy=sy/900*190;

theta =- pi / 12; 
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
sx = sx(:); 
sy = sy(:); 
points = [sx, sy]; 
rotated_points = (R * points')'; 
sx= rotated_points(:, 1);
sy= rotated_points(:, 2);

%下颚
OC_ddot=50;
img = imread('下颚轮廓.png');
img_gray = rgb2gray(img);
binary = imbinarize(img_gray);
[B, ~] = bwboundaries(binary);
contour_points = B{2};
xy=-contour_points(:,1);
xx=contour_points(:,2);

xx=(xx-150);
xy=(xy+370);
xx=xx/900*280;
xy=xy/900*280;

theta = -pi / 30; 
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
xx = xx(:); 
xy = xy(:); 
points = [xx, xy]; 
rotated_points = (R * points')'; 
xx= rotated_points(:, 1);
xy= rotated_points(:, 2);

%滑块
huakuai_rho=3;
huakuai_phi=atan(1/2);

%周期
T = 2.1*8;

%固定点坐标
U=[-20,10];
O = [0, 0];
A = [-51.6,-8];
B = [A(1)+AB,A(2)];
O_dot=[0,20];
A_dot=[O_dot(1)+30,O_dot(2)];
O_ddot=[O_dot(1)+3,O_dot(2)-10];
A_ddot=[O_ddot(1)+27,O_ddot(2)-5];

%转速
w_jing=2*pi/6.3;
w_shang=2*pi/4.2;
w_xia=2*pi*6/12.6;
w=2*pi/T;

%齿轮
rA=10;
rE1=10;
rE2=12;
rG=8;
zhuansubi=rE1/rG;
l3=sqrt((U(1)-O_ddot(1))^2+(U(2)-O_ddot(2))^2);
rU3=l3*(w_xia/(w+w_xia));
rO_ddot=l3*(w/(w+w_xia));
l2=sqrt((U(1)-O_dot(1))^2+(U(2)-O_dot(2))^2);
rU2=l2*(w_shang/(w+w_shang));
rO_dot=l2*(w/(w+w_shang));
l1=sqrt((U(1)-O(1))^2+(U(2)-O(2))^2);
rU1=l1*(w_jing/(w+w_jing));
rO=l1*(w/(w+w_jing));

%凸轮
%茎
[phi_jing,rho_jing]=img2dot('茎凸轮.png',665.5,688.5,24.5,-pi*13/12);
rho_jing=rho_jing-10;
%上颚
[phi_shang,rho_shang]=img2dot("上凸轮.png",703,755,10.325,-pi/4);
phi_shang=mod(phi_shang-pi*0,2*pi);
%下颚
[phi_xia,rho_xia]=img2dot("下凸轮.png",1007,409,2.883,-pi/12);


%% 创建幕布

figure(1);
cla;
hold on;
axis equal;
xlim([-100,400]);
ylim([-100,400]);
grid on;
title('机构动态仿真');
xlabel('X坐标');
ylabel('Y坐标');

%仿真时间
Time=30;%此处可以更改仿真的"时间"
tick=1/60;%此处可以更改"帧率"

%% 开始仿真
for i = 3:tick:T
    cla;
    %% 计算部分

    % 凸轮计算
    t = mod(i,T);
    %——————茎——————
    if t<6.3
        theta_jing_i=mod(t*w_jing,2*pi);
    else
        theta_jing_i=0;
    end
    [~, idx] = min(abs(mod(phi_jing + pi-theta_jing_i,2*pi))); 
    rho_jing_i = rho_jing(idx); 
    %——————上颚——————
    t1=4.95;
    t2=6.3;
    t3=15.3;
    if t<t2-t1
        theta_shang_i=mod((t1-t2+t)*w_shang,2*pi);
    elseif t<t1
        theta_shang_i=0;
    elseif t<t2
        theta_shang_i=(t-t1)*w_shang;
    elseif t<t3
        theta_shang_i=(t2-t1)*w_shang;
    else
        theta_shang_i=mod((t2-t1)*w_shang+(t-t3)*w_shang,2*pi);
    end
    [~, idx] = min(abs(mod(phi_shang - theta_shang_i,2*pi)));
    rho_shang_i = rho_shang(idx);
    %——————下颚——————
    theta_xia_gun_0=-pi*0.3;
    if t<1
        theta_xia_gan_i = mod(theta_xia_gun_0+(11.6+t)*w_xia,2*pi);
        theta_xia_tulun_i = mod((11.6+t)*w_xia*5/6,2*pi);
    elseif t<5.2
        theta_xia_gan_i=theta_xia_gun_0;
        theta_xia_tulun_i=0;
    else
        theta_xia_gan_i = mod(theta_xia_gun_0+(t-5.2)*w_xia,2*pi);
        theta_xia_tulun_i = mod((t-5.2)*w_xia*5/6,2*pi);
    end
    [~, idx] = min(abs(mod(phi_xia+theta_xia_gan_i-theta_xia_tulun_i,2*pi)));
    rho_xia_i = rho_xia(idx);

    % 动点计算
    % ——————茎——————
    D=[-rho_jing_i,0];
    C=[D(1)-CD,0];

    CAB=atan2(C(2)-A(2),C(1)-A(1));
    CC=[A(1)+ACC*cos(CAB),A(2)+ACC*sin(CAB)];
    E=[A(1)-AE*cos(CAB),A(2)-AE*sin(CAB)];
    F=[E(1)+AB,E(2)];
    G=[E(1),E(2)-EG];
    H=[F(1),F(2)-EG];

    AEF=CAB;
    HGI=zhuansubi*AEF;
    I=[G(1)+GI*cos(HGI),G(2)-GI*sin(HGI)];
    J=[I(1)+AB,I(2)];
    
    %——————上颚——————
    D_dot = [O_dot(1)+rho_shang_i,O_dot(2)];
    AD_dot=A_dot(1)-D_dot(1);
    CAD_dot = acos((AD_dot^2+AC_dot^2-CD_dot^2)/(2*AD_dot*AC_dot));
    C_dot = [A_dot(1)+AC_dot*cos(pi-CAD_dot),A_dot(2)+AC_dot*sin(pi-CAD_dot)];

    %——————下颚——————
    D_ddot=[O_ddot(1)+rho_xia_i*cos(-theta_xia_gan_i),O_ddot(2)+rho_xia_i*sin(-theta_xia_gan_i)];
    C_ddot=[O_ddot(1)+OC_ddot*cos(-theta_xia_gan_i),O_ddot(2)+OC_ddot*sin(-theta_xia_gan_i)];



    % 转换坐标系
    O2=[O(1)-I(1),O(2)-I(2)];
    A2=[A(1)-I(1),A(2)-I(2)];
    B2=[B(1)-I(1),B(2)-I(2)];
    C2=[C(1)-I(1),C(2)-I(2)];
    CC2=[CC(1)-I(1),CC(2)-I(2)];
    D2=[D(1)-I(1),D(2)-I(2)];
    E2=[E(1)-I(1),E(2)-I(2)];
    F2=[F(1)-I(1),F(2)-I(2)];
    G2=[G(1)-I(1),G(2)-I(2)];
    H2=[H(1)-I(1),H(2)-I(2)];
    I2=[0,0];
    J2=[J(1)-I(1),J(2)-I(2)];
    U2=[U(1)-I(1),U(2)-I(2)];
    O_dot2=[O_dot(1)-I(1),O_dot(2)-I(2)];
    A_dot2=[A_dot(1)-I(1),A_dot(2)-I(2)];
    C_dot2=[C_dot(1)-I(1),C_dot(2)-I(2)];
    D_dot2=[D_dot(1)-I(1),D_dot(2)-I(2)];
    O_ddot2=[O_ddot(1)-I(1),O_ddot(2)-I(2)];
    A_ddot2=[A_ddot(1)-I(1),A_ddot(2)-I(2)];
    C_ddot2=[C_ddot(1)-I(1),C_ddot(2)-I(2)];
    D_ddot2=[D_ddot(1)-I(1),D_ddot(2)-I(2)];
    

    %% 画图
    % ——————机架——————
    jijia_fill_x=[A2(1),O_dot2(1),A_dot2(1),A_ddot2(1),O2(1),B2(1)];
    jijia_fill_y=[A2(2),O_dot2(2),A_dot2(2),A_ddot2(2),O2(2),B2(2)];
    fill(jijia_fill_x, jijia_fill_y,'r', ... 
     'FaceAlpha', 0.3, ...      
     'EdgeColor', 'r', ...        
     'LineWidth', 1);         
    % ——————茎——————
    %凸轮
    x_jing =O2(1) + rho_jing .* cos((phi_jing-theta_jing_i));
    y_jing =O2(2) + rho_jing .* sin((phi_jing-theta_jing_i));
    plot(x_jing, y_jing, '-','Color', [0.5, 0, 0.7], 'LineWidth', 1);    
    %涂色
    jing_fill_x=[A2(1),E2(1),G2(1),I2(1),J2(1),H2(1),F2(1),B2(1)];
    jing_fill_y=[A2(2),E2(2),G2(2),I2(2),J2(2),H2(2),F2(2),B2(2)];
    fill(jing_fill_x, jing_fill_y,[0.5, 0, 0.7], ...
     'FaceAlpha', 0.3, ... 
     'EdgeColor', [0.5, 0, 0.7], ...
     'LineWidth', 1.5);  
    %连杆
    % plot([C2(1), D2(1)], [C2(2), D2(2)], '-','Color', [0.5, 0, 0.7], 'LineWidth', 1);
    % plot([A2(1), CC2(1)], [A2(2), CC2(2)], '-', 'Color', [0.5, 0, 0.7],  'LineWidth', 1);
    %滚子  
    % plot(D2(1), D2(2), 'o','Color', [0.5, 0, 0.7], 'MarkerSize', 3, 'MarkerFaceColor', [0.5,0,0.7]);
    %滑块
    % [huakuai1_x,huakuai1_y]=huakuai(C2(1),C2(2),CAB,huakuai_rho,huakuai_phi);
    % plot(huakuai1_x,huakuai1_y,'-','Color',[0.5,0,0.7]);
    %齿轮
    % rectangle('Position', [A2(1)-rA,A2(2)-rA, 2 * rA, 2 * rA],'Curvature', [1, 1], 'EdgeColor', [0.5,0,0.7], 'LineWidth', 1);
    % rectangle('Position', [E2(1)-rE1,E2(2)-rE1, 2 * rE1, 2 * rE1],'Curvature', [1, 1], 'EdgeColor', [0.5,0,0.7], 'LineWidth', 1);
    % rectangle('Position', [E2(1)-rE2,E2(2)-rE2, 2 * rE2, 2 * rE2],'Curvature', [1, 1], 'EdgeColor', [0.5,0,0.7], 'LineWidth', 1);
    % rectangle('Position', [G2(1)-rG,G2(2)-rG,2*rG,2*rG],'Curvature', [1, 1], 'EdgeColor', [0.5,0,0.7], 'LineWidth', 1);
    % rectangle('Position', [U2(1)-rU1,U2(2)-rU1,2*rU1,2*rU1],'Curvature', [1, 1], 'EdgeColor', [0.5,0,0.7], 'LineWidth', 1);
    % rectangle('Position', [O2(1)-rO,O2(2)-rO,2*rO,2*rO],'Curvature', [1, 1], 'EdgeColor', [0.5,0,0.7], 'LineWidth', 1);

    % ——————上颚——————
    %凸轮
    x_shang =O_dot2(1) + rho_shang .* cos((phi_shang-theta_shang_i));
    y_shang =O_dot2(2) + rho_shang .* sin((phi_shang-theta_shang_i));
    plot(x_shang, y_shang, 'b-', 'LineWidth', 1); 
    %涂色
    theta_shang_fill = pi/2-CAD_dot;
    sx_dot=sx.*cos(theta_shang_fill)-sy.*sin(theta_shang_fill)+A_dot2(1);
    sy_dot=sy.*cos(theta_shang_fill)+sx.*sin(theta_shang_fill)+A_dot2(2);
    fill(sx_dot,sy_dot ,'b', ... 
     'FaceAlpha', 0.3, ...
     'EdgeColor', 'b', ...
     'LineWidth', 1);
    %连杆
    % plot([D_dot2(1), C_dot2(1)], [D_dot2(2), C_dot2(2)], 'b-', 'LineWidth', 1);
    % plot([C_dot2(1), A_dot2(1)], [C_dot2(2), A_dot2(2)], 'b-', 'LineWidth', 1);
    % plot([A_dot2(1), D_dot2(1)], [A_dot2(2), D_dot2(2)], 'b-', 'LineWidth', 1);
    %滚子
    % plot(D_dot2(1), D_dot2(2), 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
    %齿轮
    % rectangle('Position', [U2(1)-rU2,U2(2)-rU2,2*rU2,2*rU2],'Curvature', [1, 1], 'EdgeColor','b', 'LineWidth', 1);
    % rectangle('Position', [O_dot2(1)-rO_dot,O_dot2(2)-rO_dot,2*rO_dot,2*rO_dot],'Curvature', [1, 1], 'EdgeColor', 'b', 'LineWidth', 1);
    % 
    % ——————下颚—————— 
    %凸轮
    x_xia =O_ddot2(1) + rho_xia .* cos((phi_xia-theta_xia_tulun_i));
    y_xia =O_ddot2(2) + rho_xia .* sin((phi_xia-theta_xia_tulun_i));
    plot(x_xia, y_xia, 'g-', 'LineWidth', 1); 
    %涂色
    theta_xia_fill = -atan2(A_ddot2(1)-D_ddot2(1),A_ddot2(2)-D_ddot2(2))+pi/2;
    sx_ddot=xx.*cos(theta_xia_fill)-xy.*sin(theta_xia_fill)+D_ddot2(1);
    sy_ddot=xy.*cos(theta_xia_fill)+xx.*sin(theta_xia_fill)+D_ddot2(2);
    fill(sx_ddot,sy_ddot ,'g', ... 
     'FaceAlpha', 0.3, ...
     'EdgeColor', 'g', ...
     'LineWidth', 1);
    % %连杆
    % plot([D_ddot2(1), O_ddot2(1)], [D_ddot2(2), O_ddot2(2)], 'g-', 'LineWidth', 1);
    % plot([D_ddot2(1), A_ddot2(1)], [D_ddot2(2), A_ddot2(2)], 'g-', 'LineWidth', 1);
    % plot([O_ddot2(1), C_ddot2(1)], [O_ddot2(2), C_ddot2(2)], 'g-', 'LineWidth', 1);
    % %滚子
    % plot(D_ddot2(1), D_ddot2(2), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    %齿轮
    % rectangle('Position', [U2(1)-rU3,U2(2)-rU3,2*rU3,2*rU3],'Curvature', [1, 1], 'EdgeColor','g', 'LineWidth', 1);
    % rectangle('Position', [O_ddot2(1)-rO_ddot,O_ddot2(2)-rO_ddot,2*rO_ddot,2*rO_ddot],'Curvature', [1, 1], 'EdgeColor', 'g', 'LineWidth', 1);
    % %滑块
    % [huakuai3_x,huakuai3_y]=huakuai(A_ddot2(1),A_ddot2(2),theta_xia_fill,huakuai_rho,huakuai_phi);
    % plot(huakuai3_x,huakuai3_y,'g-');
    
    
    drawnow;
end
