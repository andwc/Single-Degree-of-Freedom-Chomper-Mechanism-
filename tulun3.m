% 读取Excel文件中的凸轮数据
filename = '嘴调试.xlsx';
sheet = 'Sheet1';

% 读取数据，跳过前几行标题
data = readmatrix(filename, 'Sheet', sheet, 'Range', 'F2:G422'); % 根据实际数据范围调整

% 提取角度φ(第1列)和半径ρ(第2列)
% rho = data(:, 2); 
% part1 = linspace(0, 5*pi/3, 321);
% part2 = linspace(5*pi/3, 2*pi, 102);
% phi = [part1(1:end-1), part2(1:end-1)];
% phi=phi';
% 
% rho=[rho;rho(1)];
% phi=[phi;phi(1)];
rho1 = data(1:64, 2);
rho2=data(321:421,2);
part1 = linspace(0, pi, 65);
part2 = linspace(pi, 2*pi, 102);
phi = [part1(1:end-1), part2(1:end-1)];
phi=phi';
rho=[rho1;rho2];
rho=[rho;rho(1)];
phi=[phi;phi(1)];

% 使用样条插值生成更多点以使曲线更平滑
% numPoints = 1000; % 设定希望生成的插值点数
% phi_fine = linspace(min(phi), max(phi), numPoints);
% rho_fine = interp1(phi, rho, phi_fine, 'spline');

% 将极坐标转换为笛卡尔坐标
% x_fine = rho_fine .* cos(phi_fine);
% y_fine = rho_fine .* sin(phi_fine);
x = rho .* cos(phi);
y = rho .* sin(phi);


% 绘制凸轮轮廓
figure('Color', 'white', 'Position', [100, 100, 800, 600]);
plot(x, y, 'b-', 'LineWidth', 1);
hold on;

% % 添加基圆(可选) - 假设最小半径为基圆半径
% base_radius = min(rho);
% theta = linspace(0, 2*pi, 100);
% plot(base_radius*cos(theta), base_radius*sin(theta), 'r--', 'LineWidth', 1.5);

% 添加旋转中心
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% 设置图形属性
axis equal; 
grid on;
box on;
title('凸轮轮廓图', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X坐标', 'FontSize', 12);
ylabel('Y坐标', 'FontSize', 12);

% 设置坐标轴范围，提供适当边距
xlim_range = max(abs([min(x), max(x)])) * 1.2;
ylim_range = max(abs([min(y), max(y)])) * 1.2;
xlim([-xlim_range, xlim_range]);
ylim([-ylim_range, ylim_range]);
% 保存图像
exportgraphics(gcf, '上颚凸轮拟合.png', 'Resolution', 600);