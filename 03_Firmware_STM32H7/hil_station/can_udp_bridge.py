import serial
import socket
import struct
import time

# 1. Konfiguracja sprzętowa portu COM6 (CANable V2.0 PRO SLCAN)
SERIAL_PORT = "COM6"
BAUDRATE = 115200

# 2. Konfiguracja gniazd UDP (komunikacja Python <-> Simulink na tym samym PC)
UDP_IP = "127.0.0.1"
UDP_PORT_TO_SIMULINK = 5005    # Python wysyła do Simulinka: (a_des, d_target_cth, zone)
UDP_PORT_FROM_SIMULINK = 5006  # Simulink przysyła do Pythona: (d_m, dv_mps, v_ego_mps)

# 3. Formaty struktur binarnych Little-Endian (\x3c oznacza znak mniejszości)
STM32_RX_FMT = "\x3cHBBff"
STM32_TX_FMT = "\x3cHhH"
UDP_TX_FMT = "\x3cfff"
UDP_RX_FMT = "\x3cfff"

ZONE_NAMES = {
    0: "SAFE (GREEN)",
    1: "WARN (YELLOW)",
    2: "CRIT (RED)",
    3: "FAULT (3xLED)"
}


def build_slcan_fd_frame_0x200(dist_m, rel_vel_mps, ego_vel_mps):
    """Pakuje pomiary z Simulinka w 64-bajtową ramkę CAN FD z BRS (b200F...)"""
    dist_cm = max(0, min(65535, int(round(dist_m * 100.0))))
    rel_vel_cms = max(-32768, min(32767, int(round(rel_vel_mps * 100.0))))
    ego_vel_cms = max(0, min(65535, int(round(ego_vel_mps * 100.0))))

    payload_6b = struct.pack(STM32_TX_FMT, dist_cm, rel_vel_cms, ego_vel_cms)
    full_payload_64b = payload_6b + (b"\x00" * 58)

    hex_str = full_payload_64b.hex().upper()
    return f"b200F{hex_str}\r".encode("ascii")


def main():
    print(f"[BRIDGE] Otwieranie portu {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.005)
    # Wymuszenie flag DTR/RTS dla sterownika USB CDC w systemie Windows
    ser.dtr = True
    ser.rts = True
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Inicjalizacja CANable: 500 kbps (S6) / 2 Mbps BRS (Y2)
    for cmd in [b"\r\r", b"C\r", b"S6\r", b"Y2\r", b"O\r"]:
        ser.write(cmd)
        ser.flush()
        time.sleep(0.05)

    # Konfiguracja nieblokujących gniazd UDP dla Simulinka
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx.bind((UDP_IP, UDP_PORT_FROM_SIMULINK))
    sock_rx.setblocking(False)

    # Domyślny stan początkowy pojazdu (zanim Simulink zacznie nadawać)
    sim_dist_m = 60.0      # 60 metrów przed nami
    sim_rel_vel_mps = 0.0  # Prędkość względna 0 m/s
    sim_ego_vel_mps = 20.0 # Prędkość własna 20 m/s (72 km/h)
    simulink_connected = False

    last_tx_time = time.time()
    rx_buffer = ""
    raw_debug_count = 0

    print("[BRIDGE] Most CAN FD <-> UDP aktywny!")
    print("[BRIDGE] WSKAZOWKA: Jesli nic sie nie pojawia, wcisnij czarny przycisk RESET na plytce Nucleo!\n")

    try:
        while True:
            # --- KROK A: Odbiór najnowszego stanu fizyki z Simulinka po UDP ---
            try:
                while True:
                    udp_data, _ = sock_rx.recvfrom(1024)
                    if len(udp_data) == 12:
                        sim_dist_m, sim_rel_vel_mps, sim_ego_vel_mps = struct.unpack(
                            UDP_RX_FMT, udp_data
                        )
                        if not simulink_connected:
                            print("\n>>> [UDP] Polaczono z modelem fizyki w Simulinku! <<<\n")
                            simulink_connected = True
            except BlockingIOError:
                pass

            # --- KROK B: Cykliczne wysyłanie ramki 0x200 do STM32 (co 50 ms = 20 Hz) ---
            now = time.time()
            if (now - last_tx_time) >= 0.050:
                last_tx_time = now
                frame_cmd = build_slcan_fd_frame_0x200(
                    sim_dist_m, sim_rel_vel_mps, sim_ego_vel_mps
                )
                ser.write(frame_cmd)
                ser.flush()

            # --- KROK C: Odbiór ramki 0x100 ze STM32 i wysyłka po UDP do Simulinka ---
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode("ascii", errors="ignore")
                rx_buffer += chunk

                while "\r" in rx_buffer:
                    line, rx_buffer = rx_buffer.split("\r", 1)
                    line = line.strip()
                    if not line:
                        continue

                    # Diagnostyka: wypisz pierwsze 3 surowe komunikaty z portu COM6
                    if raw_debug_count < 3:
                        print(f"[RAW COM6] Odebrano: {line[:35]}...")
                        raw_debug_count += 1

                    line_up = line.upper()
                    # Obsługa zarówno małych jak i wielkich liter (b100f / b100F / d100f)
                    if line_up.startswith("B100F") or line_up.startswith("D100F"):
                        hex_payload = line_up[5:]
                        if len(hex_payload) >= 24:
                            raw_12b = bytes.fromhex(hex_payload[:24])
                            dist_cm, zone, alive, a_des, d_target = struct.unpack(
                                STM32_RX_FMT, raw_12b
                            )

                            udp_packet = struct.pack(
                                UDP_TX_FMT, a_des, d_target, float(zone)
                            )
                            sock_tx.sendto(udp_packet, (UDP_IP, UDP_PORT_TO_SIMULINK))

                            zone_str = ZONE_NAMES.get(zone, f"ZONE({zone})")
                            mode_str = "SIMULINK" if simulink_connected else "DEFAULT "
                            print(
                                f"[{mode_str}] Alive:{alive:3d} | "
                                f"d:{dist_cm/100.0:6.2f}m (Cel:{d_target:5.2f}m) | "
                                f"dv:{sim_rel_vel_mps:+6.2f}m/s | "
                                f"v:{sim_ego_vel_mps:5.2f}m/s | "
                                f"{zone_str:13s} | "
                                f"a_des:{a_des:+6.2f} m/s^2"
                            )

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[BRIDGE] Zamykanie magistrali CAN FD i gniazd UDP...")
        ser.write(b"C\r")
        ser.close()
        sock_tx.close()
        sock_rx.close()


if __name__ == "__main__":
    main()