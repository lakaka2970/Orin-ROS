%% ============================================================
%  雷达信号处理脚本 (MATLAB版)
%  对应Python版本，解析合成数据并执行距离-多普勒FFT
% ============================================================

clear; clc; close all;

%% ======================== 物理常数 ==========================
C0 = 299792458.0;   % 光速 m/s

%% ======================== 用户配置 ==========================
% 必须与数据生成时的参数完全一致！
config.input_file = 'synthetic_data_fixed1.bin';
config.num_frames = 1;
config.frame_to_analyze = 0;   % MATLAB索引从1开始，但这里0表示第一帧，后面会+1

% 波形参数
config.freq_start_Hz = 77e9;
config.freq_slope_MHzPus = 1.0;     % MHz/us
config.sample_rate_kHz = 5000.0;    % kHz
config.nof_adc_sample_per_chirp = 2048;
config.nof_chirp = 512;
config.time_idle_us = 5.0;
config.time_ramp_end_us = 2.0;

% 天线配置
config.num_rx = 16;
config.is_complex = false;   % 16T16R为实信号

% FFT配置
config.range_fft_size = 2048;
config.velocity_fft_size = 512;
config.use_hanning_window = true;

% 绘图配置
config.plot_rx_channel = 1;         % 1-based (对应Python的0)
config.plot_num_chirps = 3;
config.dynamic_range_db = 60;
config.show_grid = true;

%% ======================== 数据解析 ==========================
fprintf('正在解析数据...\n');
adc_data = parse_raw_radar_data(config);

%% ======================== 提取指定帧 ==========================
frame_idx = config.frame_to_analyze + 1;  % MATLAB 1-based
adc_frame = adc_data(:,:,:,frame_idx);    % [num_rx, Ns, Nc]

%% ======================== 计算各阶段 ==========================
fprintf('正在计算各阶段数据...\n');
stages = compute_all_stages(adc_frame, config);

%% ======================== 输出系统参数 ==========================
fprintf('\n📊 雷达系统参数:\n');
fprintf('  距离分辨率: %.2f m\n', stages.range_res);
fprintf('  最大探测距离: %.2f m\n', stages.max_range);
fprintf('  速度分辨率: %.2f m/s\n', stages.velocity_res);
fprintf('  最大不模糊速度: ±%.2f m/s\n', stages.max_velocity);

%% ======================== 绘图 ==========================
fprintf('正在生成图表...\n');
plot_raw_adc(stages, config);
plot_range_fft(stages, config);
plot_doppler_slice(stages, config);
plot_range_doppler_map(stages, config);

fprintf('完成！\n');

%% ======================== 函数定义 ==========================
function adc_data = parse_raw_radar_data(config)
% 解析二进制雷达数据（适配 data_generate_fixed.py 的 C 顺序布局）
% 输入: config 结构体，需包含字段：
%   num_rx, nof_adc_sample_per_chirp, nof_chirp, num_frames
% 返回: [num_rx, Ns, Nc, num_frames] double 数组

num_rx = config.num_rx;
Ns = config.nof_adc_sample_per_chirp;
Nc = config.nof_chirp;
nf = config.num_frames;

% 打开文件
fid = fopen(config.input_file, 'rb');
if fid == -1
    error('无法打开文件: %s', config.input_file);
end
raw_int16 = fread(fid, inf, 'int16');
fclose(fid);

% 校验大小
expected_total = num_rx * Ns * Nc * nf;
if length(raw_int16) ~= expected_total
    error('文件大小不匹配！预期 int16 数量: %d, 实际: %d', ...
          expected_total, length(raw_int16));
end

% 关键修正：按 (rx, chirp, samp) 顺序重塑，因为变化最快的是 rx，其次 chirp，最后 samp
data_4d = reshape(raw_int16, num_rx, Nc, Ns, nf);   % (rx, chirp, samp, frame)
adc_data = permute(data_4d, [1, 3, 2, 4]);          % (rx, samp, chirp, frame)
adc_data = double(adc_data);

fprintf('✅ 数据解析完成\n');
fprintf('  形状: [%d, %d, %d, %d] (通道, 采样点, chirp, 帧)\n', size(adc_data));
end


function stages = compute_all_stages(adc_frame, config)
% adc_frame: [num_rx, Ns, Nc]
% 返回结构体包含各阶段数据和坐标轴

[num_rx, Ns, Nc] = size(adc_frame);
N_r = config.range_fft_size;
N_v = config.velocity_fft_size;

% 1. 原始数据
stages.raw_data = adc_frame;


if config.use_hanning_window
    % 生成距离窗（长度 Ns）
    range_win = hann(Ns, 'periodic');   % Ns x 1
    % 扩展为 [1, Ns, 1] 以便广播
    range_win_3d = reshape(range_win, 1, Ns, 1);
    % 先只乘距离窗
    adc_windowed_range = adc_frame .* range_win_3d;
else
    adc_windowed_range = adc_frame;
end

% 4. 距离FFT (沿采样点维，即第2维)
range_fft_full = fft(adc_windowed_range, N_r, 2);    % [num_rx, N_r, Nc]
figure;mesh(squeeze(db(range_fft_full(1, :, :))));
% 取正频率部分（第1到 N_r/2）
range_fft = range_fft_full(:, 1:N_r/2, :);     % [num_rx, N_r/2, Nc]
stages.range_fft = range_fft;
if config.use_hanning_window
    % 生成多普勒窗（长度 Nc）
    doppler_win = hann(Nc, 'periodic');   % Nc x 1
    % 扩展为 [1, 1, Nc] 以便广播到 [num_rx, N_r/2, Nc]
    doppler_win_3d = reshape(doppler_win, 1, 1, Nc);
    % 在距离FFT结果上乘多普勒窗（沿第3维）
    range_fft_windowed = range_fft .* doppler_win_3d;
else
    range_fft_windowed = range_fft;
end
% 5. 多普勒FFT (沿chirp维，即第3维)
doppler_fft = fft(range_fft_windowed, N_v, 3);           % [num_rx, N_r/2, N_v]
stages.doppler_fft = doppler_fft;

% 6. 多通道积累（非相干积累）
rd_power = sum(abs(doppler_fft).^2, 1); % [1, N_r/2, N_v]
rd_power = squeeze(rd_power);                   % [N_r/2, N_v]
rd_map_db = 10*log10(rd_power + 1e-12);
rd_map_db = rd_map_db - max(rd_map_db(:));
stages.rd_map_db = rd_map_db;
tx_ddma_idx = [0, 1, 2, 3, 4, 5, 6, 7,8,9,10,11,12,13,14,15]*config.nof_chirp / 32
% -------------------------------------------------------------------------
% VCH NCI 虚拟通道非相干积累
% 对应 Python 向量化版本
% rx_nci      = [n_chirps, n_range_bins]  (Doppler, Range)
% vch_nci     = [n_range_bins, n_chirps]  (Range, Doppler)
% -------------------------------------------------------------------------
n_tx = length(tx_ddma_idx);
tx_ddma = int64(tx_ddma_idx);  % 确保整数索引

% 生成正确索引（向量化，无错）
doppler_indices = int64((0:config.nof_chirp-1)');  % 列向量 [n_chirps, 1]
vch_nci = zeros(N_r/2, config.nof_chirp, 'single');

for tx_i = 1:n_tx
    % 计算当前 tx 对应的所有偏移后多普勒索引（列向量）
    db_idx = mod(doppler_indices + tx_ddma(tx_i), config.nof_chirp);
    db_idx(db_idx == 0) = config.nof_chirp;  % MATLAB 索引从 1 开始！
    
    % 累加（维度完全匹配）
    vch_nci = vch_nci + abs(rd_power(:, db_idx));
end

% 7. 计算坐标轴
fs = config.sample_rate_kHz * 1e3;               % Hz
slope = config.freq_slope_MHzPus * 1e12;         % Hz/s
range_res = 299792458 * fs / (2 * slope * N_r);          % 米
max_range = range_res * (N_r/2);
range_axis = linspace(0, max_range, N_r/2);
stages.range_axis = range_axis;
stages.range_res = range_res;
stages.max_range = max_range;

lambda0 = 299792458 / config.freq_start_Hz;
pri = (config.time_idle_us + config.time_ramp_end_us) * 1e-6;  % s
max_velocity = lambda0 / (4 * pri);                 % m/s
velocity_axis = linspace(-max_velocity, max_velocity, N_v);
stages.velocity_axis = velocity_axis;
stages.velocity_res = velocity_axis(2) - velocity_axis(1);
stages.max_velocity = max_velocity;

% 时间轴（单个chirp内，微秒）
stages.time_axis_us = (0:Ns-1) / fs * 1e6;
end

function plot_raw_adc(stages, config)
rx = config.plot_rx_channel;
num_chirps = min(config.plot_num_chirps, size(stages.raw_data,3));

figure('Name', '原始ADC时域波形', 'Position', [100, 100, 1000, 500]);
hold on;
for chirp = 1:num_chirps
    plot(stages.time_axis_us, squeeze(stages.raw_data(rx, :, chirp)), ...
        'LineWidth', 1, 'DisplayName', sprintf('Chirp %d', chirp-1));
end
xlabel('时间 (\mus)');
ylabel('ADC采样值 (int16)');
title(sprintf('原始ADC时域波形 (通道 %d, 前%d个chirp)', rx-1, num_chirps));
legend('Location', 'best');
grid on;
if config.show_grid
    grid on;
else
    grid off;
end
end

function plot_range_fft(stages, config)
rx = config.plot_rx_channel;
range_fft_single = squeeze(stages.range_fft(rx, :, :));  % [N_r/2, Nc]
% 平均功率
range_spectrum = mean(abs(range_fft_single).^2, 2);
range_spectrum_db = 10*log10(range_spectrum + 1e-12);
range_spectrum_db = range_spectrum_db - max(range_spectrum_db);

figure('Name', '距离FFT谱', 'Position', [100, 100, 1000, 500]);
plot(stages.range_axis, range_spectrum_db, 'LineWidth', 1.5);
xlabel('距离 (m)');
ylabel('幅度 (dB)');
title(sprintf('距离FFT谱 (通道 %d, 所有chirp平均)', rx-1));
ylim([-config.dynamic_range_db, 5]);
grid on;
hold on;
% 标注目标位置（示例）
targets = [50.0, 30.0];
for i = 1:length(targets)
    xline(targets(i), 'r--', sprintf('%.1fm 目标', targets(i)), 'LabelOrientation', 'horizontal');
end
legend('距离谱', 'Location', 'best');
end

function plot_doppler_slice(stages, config)
rd_map_db = stages.rd_map_db;
figure;mesh(rd_map_db);
xlabel('速度 (m/s)');
ylabel('幅度 (dB)');
title('目标距离门的多普勒谱切片');
grid on;
legend('Location', 'best');
end

function plot_range_doppler_map(stages, config)
rd_map_db = stages.rd_map_db;
range_axis = stages.range_axis;
velocity_axis = stages.velocity_axis;

figure('Name', '距离-速度热力图', 'Position', [100, 100, 1200, 800]);
imagesc(velocity_axis, range_axis, rd_map_db);
colormap('jet');
caxis([-config.dynamic_range_db, 0]);
colorbar;
xlabel('速度 (m/s)');
ylabel('距离 (m)');
title('距离-速度二维频谱');
axis xy;
grid on;
if config.show_grid
    set(gca, 'GridColor', 'white', 'GridAlpha', 0.25, 'GridLineStyle', '--');
    grid on;
end
hold on;
% 标注目标
targets = struct('range', {50.0, 30.0}, 'velocity', {10.0, -5.0}, ...
    'label', {'目标1 (50m, +10m/s)', '目标2 (30m, -5m/s)'});
for i = 1:length(targets)
    scatter(targets(i).velocity, targets(i).range, 120, 'w', 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 2);
    text(targets(i).velocity + 1, targets(i).range + 2, targets(i).label, ...
        'Color', 'w', 'FontSize', 10, 'FontWeight', 'bold', ...
        'BackgroundColor', 'k', 'EdgeColor', 'none', 'Margin', 2);
end
hold off;
end