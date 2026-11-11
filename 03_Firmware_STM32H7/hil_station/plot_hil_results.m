%% Skrypt post-processingu badan HIL (STM32H7 + CAN FD + Simulink)
% 1. Wyciagniecie danych telemetrycznych z obiektu wyjsciowego Simulinka
% 2. Obliczenie calkowych i ekstremalnych wskaznikow jakosci regulacji
% 3. Wygenerowanie wykresow publikacyjnych do pracy magisterskiej

close all; clc;

%% 1. Rozpakowanie macierzy danych telemetrycznych
time     = out.tout;          % Wektor czasu symulacji [s]
data     = out.sim_data;      % Macierz pomiarow (N x 6)

d        = data(:, 1);        % Dystans rzeczywisty miedzy pojazdami [m]
d_target = data(:, 2);        % Dynamiczny dystans zadany z prawa CTH (ze STM32) [m]
v_lead   = data(:, 3);        % Predkosc pojazdu poprzedzajacego (lidera) [m/s]
v_ego    = data(:, 4);        % Predkosc wlasna naszego pojazdu [m/s]
a_des    = data(:, 5);        % Przyspieszenie zadane przez regulator STM32 [m/s^2]
a_ego    = data(:, 6);        % Przyspieszenie rzeczywiste pojazdu (po inercji) [m/s^2]
zone     = data(:, 7);        % Strefa bezpieczenstwa (0 = SAFE, 1 = WARNING, 2 = CRITICAL, 3 = FAULT)

% Korekta ewentualnej probki inicjalizacyjnej UDP w chwili t = 0 s
if d_target(1) == 0
    d_target(1) = d_target(2);
end

dt  = time(2) - time(1);      % Automatyczne wyznaczenie kroku probkowania [s]
e_d = d - d_target;           % Wektor uchybu regulacji odleglosci [m]

%% 2. Obliczenie wskaznikow jakosci regulacji (Calosc vs Faza Sledzenia t >= 12 s)
% A) Dla calego eksperymentu (0..60 s - lacznie z dojazdem z 60 m)
IAE_total = sum(abs(e_d)) * dt;
ISE_total = sum(e_d .^ 2) * dt;

% B) Dla fazy wlasciwego podazania za liderem (t >= 12 s: hamowanie, postoj i start)
idx_track = (time >= 12.0);
idx_jerk_track = (time(2:end) >= 12.0);
IAE_track = sum(abs(e_d(idx_track))) * dt;
ISE_track = sum(e_d(idx_track) .^ 2) * dt;
max_err_track = max(abs(e_d(idx_track)));

% C) Wskazniki bezpieczenstwa (ISO 26262) i komfortu jazdy (ISO 15622)
d_min    = min(d);
jerk_vec = diff(a_ego) / dt;
jerk_max = max(abs(jerk_vec));
jerk_max_track = max(abs(jerk_vec(idx_jerk_track)));

fprintf("=========================================================================\n");
fprintf(" WYNIKI BADAN HIL (Scenariusz Stop & Go, Ts_STM32 = 100 ms, dt = %.2f s)\n", dt);
fprintf("=========================================================================\n");
fprintf(" [Calosc 0..60 s]      IAE = %6.2f m*s  | ISE = %7.2f m^2*s\n", IAE_total, ISE_total);
fprintf(" [Sledzenie 12..60 s]  IAE = %6.2f m*s  | ISE = %7.2f m^2*s | Max |e_d| = %.2f m\n", ...
    IAE_track, ISE_track, max_err_track);
fprintf(" [Bezpieczenstwo/ISO]  Min d = %5.2f m   | Max |Jerk| = %.2f m/s^3 | Max |Jerk| tracking = %.2f m/s^3\n", ...
    d_min, jerk_max, jerk_max_track);
fprintf("=========================================================================\n");

%% 3. Generowanie wykresow publikacyjnych (Biale tlo + zsynchronizowane osie)
fig = figure('Name', 'Wyniki HIL - Tempomat ACC', 'Color', 'w', 'Position', [100, 100, 1000, 720]);

ax1 = subplot(4, 1, 1);
plot(time, d, 'b-', time, d_target, 'r--', 'LineWidth', 1.6);
ylabel('Dystans [m]', 'FontWeight', 'bold');
title('Walidacja HIL układu ACC (STM32H7 + CAN FD) – Scenariusz Stop & Go', 'FontSize', 12);
legend('d (rzeczywisty)', 'd_{target} (CTH ze STM32)', 'Location', 'northeast');
grid on; grid minor;

ax2 = subplot(4, 1, 2);
plot(time, v_lead, 'k--', time, v_ego, 'b-', 'LineWidth', 1.6);
ylabel('Prędkość [m/s]', 'FontWeight', 'bold');
legend('v_{lead} (pojazd poprzedzający)', 'v_{ego} (pojazd własny)', 'Location', 'northeast');
grid on; grid minor;

ax3 = subplot(4, 1, 3);
plot(time, a_des, 'r--', time, a_ego, 'b-', 'LineWidth', 1.6);
xlabel('Czas [s]', 'FontWeight', 'bold');
ylabel('Przyspieszenie [m/s^2]', 'FontWeight', 'bold');
legend('a_{des} (zadane ze STM32)', 'a_{ego} (rzeczywiste po inercji)', 'Location', 'northeast');
grid on; grid minor;

ax4 = subplot(4, 1, 4);
plot(time, e_d, 'm-', 'LineWidth', 1.5);
yline(-2.0, 'r--', 'Prog WARNING (-2 m)', 'LineWidth', 1.2)
yline(0.0, 'k:')
xlabel('Czas [s]', 'FontWeight', 'bold');
ylabel('Uchyb odleglosci [m]', 'FontWeight', 'bold');
legend('Uchyb odleglosci w czasie', 'Location', 'northeast');
grid on; grid minor;

% Sprzezenie osi X wszystkich 3 wykresow (wspolne przyblizanie lupka)
linkaxes([ax1, ax2, ax3, ax4], 'x');
xlim([0, max(time)]);