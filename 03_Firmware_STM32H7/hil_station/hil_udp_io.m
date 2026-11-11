function y = hil_udp_io(u)
% HIL_UDP_IO Wymiana danych UDP z mostem Pythona (can_udp_bridge.py)
% Wejscie u (wektor 3x1 z Simulinka): [dist_m; rel_vel_mps; ego_vel_mps]
% Wyjscie y (wektor 3x1 do Simulinka): [a_des_mps2; d_target_m; zone_id]

    persistent udpObj last_y;
    
    % 1. Jednorazowa inicjalizacja gniazda UDP (port 5005) przy starcie symulacji
    if isempty(udpObj)
        udpObj = udpport("byte", "LocalHost", "127.0.0.1", "LocalPort", 5005);
        % Domyslny stan poczatkowy: a_des = 0.0, d_target = 35.0 m, zone = 0 (SAFE)
        last_y = [0.0; 0.0; 0.0];
    end
    
    % 2. Wyslanie aktualnego stanu fizyki pojazdu z Simulinka do Pythona (port 5006)
    txFloats = single([u(1), u(2), u(3)]);
    txBytes  = typecast(txFloats, 'uint8');
    write(udpObj, txBytes, "uint8", "127.0.0.1", 5006);
    
    % 3. Odbior najnowszego pakietu 12 bajtow (3 x float) ze STM32
    while udpObj.NumBytesAvailable >= 12
        rxBytes  = read(udpObj, 12, "uint8");
        rxFloats = typecast(uint8(rxBytes), 'single');
        % Rozpakowanie wszystkich 3 wartosci przyslanych przez most Pythona:
        % rxFloats(1) -> fDesiredAccelMps2 [m/s^2]
        % rxFloats(2) -> fTargetDistanceMeters [m]
        % rxFloats(3) -> eCurrentZone (0..3)
        last_y = double([rxFloats(1); rxFloats(2); rxFloats(3)]);
    end
    
    y = last_y;
end