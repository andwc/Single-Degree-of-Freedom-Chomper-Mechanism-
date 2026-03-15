% 读取Excel文件中的凸轮数据
filename = '副本副本茎调试.xlsx';
sheet = 'Sheet1';

% 读取数据，跳过前几行标题
data = readmatrix(filename, 'Sheet', sheet, 'Range', 'N2:O69'); % 根据实际数据范围调整

% 提取角度φ(第1列)和半径ρ(第2列)
phi = data(:, 1);  % N列 - 角度
rho = 20+data(:, 2);  % O列 - 半径

% 筛选掉NaN值
validIdx = ~isnan(phi) & ~isnan(rho);
phi = phi(validIdx);
rho = rho(validIdx);

% 确认角度单位 - 根据数据看起来是以弧度为单位
% 如果数据是以度为单位，请取消下面这行注释
% phi = deg2rad(phi);

% 使用样条插值生成更多点以使曲线更平滑
numPoints = 1000; % 设定希望生成的插值点数
phi_fine = linspace(min(phi), max(phi), numPoints);
rho_fine = interp1(phi, rho, phi_fine, 'spline');

% 将极坐标转换为笛卡尔坐标
x_fine = rho_fine .* cos(phi_fine);
y_fine = rho_fine .* sin(phi_fine);

% 绘制凸轮轮廓
figure('Color', 'white', 'Position', [100, 100, 800, 600]);
plot(x_fine, y_fine, 'b-', 'LineWidth', 2);
hold on;

% 添加基圆(可选) - 假设最小半径为基圆半径
base_radius = min(rho);
theta = linspace(0, 2*pi, 100);
plot(base_radius*cos(theta), base_radius*sin(theta), 'r--', 'LineWidth', 1.5);

% 添加旋转中心
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% 设置图形属性
axis equal;  % 确保纵横比相等，使圆形看起来是圆的
grid on;
box on;
title('凸轮轮廓图', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X坐标', 'FontSize', 12);
ylabel('Y坐标', 'FontSize', 12);
legend('凸轮轮廓', '基圆', '旋转中心', 'Location', 'best');

% 设置坐标轴范围，提供适当边距
xlim_range = max(abs([min(x_fine), max(x_fine)])) * 1.2;
ylim_range = max(abs([min(y_fine), max(y_fine)])) * 1.2;
xlim([-xlim_range, xlim_range]);
ylim([-ylim_range, ylim_range]);
% 保存图像
%exportgraphics(gcf, '凸轮轮廓图.png', 'Resolution', 300);


disp('凸轮轮廓绘制完成!');