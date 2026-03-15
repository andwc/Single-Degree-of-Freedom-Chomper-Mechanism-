clear;
clc;
%{ 
—————————————————单自由度大嘴花动作全过程仿真——————————————————
本机构设计由  翻斗花园第一突击队  设计  
matlab文件由  陈俊逸  完成
————————————————————————————————————————————————————————————
颜色区分：
    红色为机架（即电机系）
    紫色为茎
    蓝色为上颚
    绿色为下颚
————————————————————————————————————————————————————————————
点的标号规则：

点的标号没有下划线后缀的是茎的
_dot是上颚的
_ddot是下颚的
后缀为2的是世界坐标系下的
U点是电机位置

如果不清楚点的对应关系，可将文件最后的注释代码选中，ctrl+t一键取消注释
注意：这会大量损失流畅度
————————————————————————————————————————————————————————————
茎上的齿轮为完整齿轮
传动凸轮的齿轮均为非完整齿轮

！特别说明：
在仿真的时候算凸轮角度的时候写了很多if else
    有一些判断是初始条件优化后的结果，但传动比恒定，实际是可行的。
    有一些是非完整齿轮的结果。

%}
function [phi, rho] = cartesianToPolar(x, y)
    rho = sqrt(x.^2 + y.^2);
    phi = atan2(y, x);
    phi = mod(phi, 2*pi);
end

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
ACC=18.733;
AE=38.19-ACC;
EG=20.001;
GI=102.623;
%上颚
CD_dot=15;
AC_dot=10;
sx=[0,0,43.168,134.967,209.078,275.058];
sy=[0,10,91.645,98.261,66.554,0];
%下颚
OC_ddot=50;
xx=[0,10.133,302.691,262.439,170.655,63.763];
xy=[0,1.448,1.448,-72.28,-102.451,-88.895];
%滑块
huakuai_rho=3;
huakuai_phi=atan(1/2);

%周期
T = 16.971;

%固定点坐标
U=[-20,10];
O = [0, 0];
A = [-61.6+10,-8];
B = [A(1)+AB,A(2)];
O_dot=[0,20];
A_dot=[O_dot(1)+30,O_dot(2)];
O_ddot=[O_dot(1)+3,O_dot(2)-10];
A_ddot=[O_ddot(1)+27,O_ddot(2)-4.5];

%转速
w_jing=2*pi/6.471;
w_shang=2*pi/4.2;
w_xia=2*pi*6/12.6;
w=2*pi/T;

%齿轮
rA=9.729;
rE1=9.729;
rE2=11.429;
rG=8.572;
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
% figure(1);
% clf;
% hold on;
% axis equal;
% grid on;
% title('机构动态仿真');
% xlabel('X坐标');
% ylabel('Y坐标');

%仿真时间
Time=30;
tick=1/30;

total_time=0:tick:T;
canshu1=zeros(1,length(total_time));
canshu2=zeros(1,length(total_time));
canshu3=zeros(1,length(total_time));
canshu4=zeros(1,length(total_time));
canshu5=zeros(1,length(total_time));
counter = 1;
%% 开始仿真
for i = 0:tick:T
    t = mod(i,T);
    %——————茎——————
    if t<6.471
        theta_jing_i=mod(t*w_jing,2*pi);
    else
        theta_jing_i=0;
    end
    [~, idx] = min(abs(mod(phi_jing + pi-theta_jing_i,2*pi))); 
    rho_jing_i = rho_jing(idx); 
    %——————上颚——————
    if t<1.35
        theta_shang_i=mod((-1.35+t)*w_shang,2*pi);
    elseif t<5.121
        theta_shang_i=0;
    elseif t<6.471
        theta_shang_i=(t-5.121)*w_shang;
    elseif t<15.471
        theta_shang_i=(6.471-5.121)*w_shang;
    else
        theta_shang_i=mod((6.471-5.121)*w_shang+(t-15.471)*w_shang,2*pi);
    end
    [~, idx] = min(abs(mod(phi_shang - theta_shang_i,2*pi)));
    rho_shang_i = rho_shang(idx);
    %——————下颚——————
    theta_xia_gun_0=-pi*0.3;
    if t<1
        theta_xia_gan_i = mod(theta_xia_gun_0+(11.6+t)*w_xia,2*pi);
        theta_xia_tulun_i = mod((11.6+t)*w_xia*5/6,2*pi);
    elseif t<5.371
        theta_xia_gan_i=theta_xia_gun_0;
        theta_xia_tulun_i=0;
    else
        theta_xia_gan_i = mod(theta_xia_gun_0+(t-5.371)*w_xia,2*pi);
        theta_xia_tulun_i = mod((t-5.371)*w_xia*5/6,2*pi);
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
    
    canshu1(counter)=CAB*(180/pi);
    canshu5(counter)=(pi-HGI)*(180/pi);
    canshu2(counter)=norm(D_dot-O_dot)*10;
    % canshu3(counter)=mod(-theta_xia_gan_i*(180/pi),360);
    % canshu4(counter)=norm(D_ddot-O_ddot)*10;
    canshu3(counter)=norm(D_ddot-O_ddot)*10*cos(-theta_xia_gan_i);
    canshu4(counter)=norm(D_ddot-O_ddot)*10*sin(-theta_xia_gan_i);
    counter = counter + 1;
    drawnow;
end


% data_to_save1 = [total_time', canshu1']; 
% writematrix(data_to_save1, 'canshu1.csv');
% 
% data_to_save5 = [total_time', canshu5']; 
% writematrix(data_to_save5, 'canshu5.csv');
% data_to_save2 = [total_time', canshu2']; 
% writematrix(data_to_save2, 'canshu2.csv');
data_to_save3 = [total_time', canshu3']; 
writematrix(data_to_save3, 'canshu3.csv');
data_to_save4 = [total_time', canshu4',]; 
writematrix(data_to_save4, 'canshu4.csv');

disp('ok')