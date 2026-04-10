function [x_pri, y_pri, qd, qdd, tvec] = trajectoryGeneration(path_x, path_y, v_max, a_max,Ts, v0)

x = path_x;
y = path_y;

% Calcolo dell'ascissa curvilinea
dist = sqrt(diff(x).^2 + diff(y).^2);
s = [0,cumsum(dist)];

% Parametri per il profilo di velocità trapezoidale
d_s = s(end) - s(1); % Distanza totale
t_acc = (v_max-v0) / a_max; % Tempo di accelerazione
t_dec = v_max / a_max;
s_acc = 0.5 * a_max * t_acc^2; % Distanza durante l'accelerazione
s_dec = 0.5 * a_max * t_dec^2; % Distanza durante la decelerazione
s_const = d_s - s_acc - s_dec; % Distanza costante
t_const = s_const / v_max; % Tempo costante
t_total =  t_acc + t_const + t_dec; % Tempo totale

% Profilo di velocità trapezoidale
% t = linspace(0, t_total, 100);
t=0:Ts:t_total;
v = zeros(size(t));

v(t <= t_acc) = v0 + a_max * t(t <= t_acc); % Accelerazione
v(t_acc < t & t <= t_acc + t_const) = v_max; % Velocità costante
v(t_acc + t_const < t & t <= t_total) = v_max - a_max * (t(t_acc + t_const < t & t <= t_total) - t_acc - t_const); % Decelerazione

% Profilo di accelerazione
a = zeros(size(t));
a(t <= t_acc) = a_max; % Accelerazione massima
a(t_acc + t_const < t & t <= t_total) = -a_max; % Decelerazione massima

% Profilo di spazio percorso
s_profile = zeros(size(t));
s_profile(t <= t_acc) = 0.5 * a_max * t(t <= t_acc).^2; % Distanza durante l'accelerazione
s_profile(t_acc < t & t <= t_acc + t_const) = s_acc + v_max * (t(t_acc < t & t <= t_acc + t_const) - t_acc); % Distanza costante
s_profile(t_acc + t_const < t & t <= t_total) = s(end) - 0.5 * a_max * (t_total - t(t_acc + t_const < t & t <= t_total)).^2; % Distanza durante la decelerazione

x_pri = interp1(s,x,s_profile,"spline");
y_pri = interp1(s,y,s_profile,"spline");
qd = v;
qdd = a;
tvec = t;
end
