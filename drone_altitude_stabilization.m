%% Drone Altitude Stabilization using PID Controller
% Plant: G(s) = 1 / (s^2 + 2s + 5)
% PID Gains: Kp = 10, Ki = 5, Kd = 2
% Author: Drone Altitude Stabilization Project

clc;
clear all;
close all;

%% ===================== SYSTEM DEFINITION =====================

% Plant transfer function: 1 / (s^2 + 2s + 5)
num_plant = [1];
den_plant = [1 2 5];
G = tf(num_plant, den_plant);

% PID Controller
Kp = 10;
Ki = 5;
Kd = 2;
C = pid(Kp, Ki, Kd);

% Closed-loop system (negative feedback)
sys_cl = feedback(C * G, 1);

fprintf('=== Drone Altitude Stabilization System ===\n');
fprintf('Plant G(s) = 1 / (s^2 + 2s + 5)\n');
fprintf('PID: Kp=%.1f  Ki=%.1f  Kd=%.1f\n\n', Kp, Ki, Kd);

%% ===================== STEP RESPONSE =====================

t = 0:0.01:10;
figure('Name', 'Drone Altitude Stabilization', 'NumberTitle', 'off', ...
       'Position', [100 100 1200 800]);

% --- Plot 1: Step Response ---
subplot(2, 2, 1);
[y, t_out] = step(sys_cl, t);
plot(t_out, y, 'b-', 'LineWidth', 2);
hold on;
yline(1, 'r--', 'Reference', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
title('Step Response (No Disturbance)');
xlabel('Time (s)');
ylabel('Altitude (m)');
grid on;
legend('PID Output y(t)', 'Reference r(t)', 'Location', 'southeast');

% Compute performance metrics
info = stepinfo(sys_cl);
fprintf('=== Step Response Performance ===\n');
fprintf('Rise Time    : %.4f s\n', info.RiseTime);
fprintf('Settling Time: %.4f s\n', info.SettlingTime);
fprintf('Overshoot    : %.2f %%\n', info.Overshoot);
fprintf('Peak Value   : %.4f\n', info.Peak);
fprintf('Steady State : %.4f\n\n', dcgain(sys_cl));

% Annotate overshoot on plot
[peak_val, peak_idx] = max(y);
plot(t_out(peak_idx), peak_val, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(t_out(peak_idx)+0.1, peak_val, sprintf('Peak: %.2f', peak_val), ...
     'FontSize', 9, 'Color', 'red');

%% ===================== DISTURBANCE RESPONSE =====================

subplot(2, 2, 2);

% Simulate with disturbance at t=5s using lsim
dt = 0.01;
t_sim = 0:dt:15;
r = ones(size(t_sim));       % reference = 1 (desired altitude)
d = zeros(size(t_sim));      % disturbance signal
d(t_sim >= 5 & t_sim < 6) = 0.5;   % wind gust from t=5 to t=6

% Disturbance enters at plant input
% Closed loop with disturbance: Y = (CG/(1+CG))*R + (G/(1+CG))*D
sys_r_to_y = feedback(C * G, 1);           % reference to output
sys_d_to_y = feedback(G, C);               % disturbance to output

y_ref  = lsim(sys_r_to_y, r, t_sim);
y_dist = lsim(sys_d_to_y, d, t_sim);
y_total = y_ref + y_dist;

plot(t_sim, y_total, 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, r, 'r--', 'LineWidth', 1.5);
plot(t_sim, d, 'g:', 'LineWidth', 1.5);
yline(1, 'r--', 'LineWidth', 1);
title('Step Response with Wind Disturbance');
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('Output y(t)', 'Reference r(t)', 'Disturbance D(t)', 'Location', 'southeast');
grid on;

%% ===================== BODE PLOT =====================

subplot(2, 2, 3);
% Open loop transfer function
L = C * G;
[mag, phase, wout] = bode(L);
mag_db = 20*log10(squeeze(mag));
phase_deg = squeeze(phase);

yyaxis left;
semilogx(wout, mag_db, 'b-', 'LineWidth', 2);
ylabel('Magnitude (dB)');
yline(0, 'k--', '0 dB', 'LineWidth', 1);

yyaxis right;
semilogx(wout, phase_deg, 'r-', 'LineWidth', 2);
ylabel('Phase (deg)');
yline(-180, 'k:', '-180 deg', 'LineWidth', 1);

title('Bode Plot (Open Loop)');
xlabel('Frequency (rad/s)');
grid on;
legend('Magnitude', 'Phase', 'Location', 'southwest');

% Gain and Phase margins
[Gm, Pm, Wgm, Wpm] = margin(L);
fprintf('=== Stability Margins ===\n');
fprintf('Gain Margin  : %.2f dB at %.2f rad/s\n', 20*log10(Gm), Wgm);
fprintf('Phase Margin : %.2f deg at %.2f rad/s\n\n', Pm, Wpm);

%% ===================== POLE-ZERO MAP =====================

subplot(2, 2, 4);
pzmap(sys_cl);
title('Pole-Zero Map (Closed Loop)');
grid on;

p = pole(sys_cl);
fprintf('=== Closed-Loop Poles ===\n');
for i = 1:length(p)
    if imag(p(i)) >= 0
        fprintf('Pole %d: %.4f + %.4fi\n', i, real(p(i)), imag(p(i)));
    end
end

%% ===================== SUMMARY =====================

fprintf('\n=== Summary ===\n');
if all(real(p) < 0)
    fprintf('System is STABLE (all poles in left half plane)\n');
else
    fprintf('System is UNSTABLE\n');
end
fprintf('Steady-state error = %.4f\n', abs(1 - dcgain(sys_cl)));

sgtitle('Drone Altitude Stabilization — PID Control', 'FontSize', 14, 'FontWeight', 'bold');

