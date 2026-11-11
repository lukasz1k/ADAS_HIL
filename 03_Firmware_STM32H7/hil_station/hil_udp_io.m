function a_des = hil_udp_io(u)
% HIL_UDP_IO Wymiana danych UDP z mostem Pythona (can_udp_bridge.py)
% Wejscie u (wektor 3x1 z Simulinka): [dist_m; rel_vel_mps; ego_vel_mps]
% Wyjscie a_des (skalar do Simulinka): zadane przyspieszenie z STM32 [m/s^2]

    persistent udpObj last_a_des;
    
    % Inicjalizacja gniazda UDP przy pierwszym kroku symulacji
    if isempty(udpObj)
        udpObj = udpport("byte", "LocalHost", "127.0.0.1", "LocalPort", 5005);
        last_a_des = 0.0;
    end
    
    % 1. Wysłanie aktualnego stanu fizyki pojazdu z Simulinka do Pythona (port 5006)
    txFloats = single([u(1), u(2), u(3)]);
    txBytes  = typecast(txFloats, 'uint8');
    write(udpObj, txBytes, "uint8", "127.0.0.1", 5006);
    
    % 2. Odbiór najnowszego rozkazu a_des z STM32 (jeśli przyszedł nowy pakiet 12B)
    while udpObj.NumBytesAvailable >= 12
        rxBytes  = read(udpObj, 12, "uint8");
        rxFloats = typecast(uint8(rxBytes), 'single');
        last_a_des = double(rxFloats(1)); % Pierwszy float to fDesiredAccelMps2
    end
    
    a_des = last_a_des;
end