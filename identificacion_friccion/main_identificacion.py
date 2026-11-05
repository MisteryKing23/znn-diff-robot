"""
Identificación experimental de la fricción de cada rueda — solo Raspberry + OpenCR (sin ROS).
Requiere el firmware openCR_friccion.ino cargado en el OpenCR.
"""
import argparse
import csv
import math
import socket
import sys
import time
import ajuste_friccion

# =========================== CONFIGURACIÓN ===========================
PUERTO = '/dev/ttyACM0'
MODO = 'suelo'              # 'suelo' o 'aire'
SIGNO = {'IZQ': +1, 'DER': +1}

LAPTOP_IP = None            # p.ej. '192.168.1.50' para ver la gráfica en vivo; None = desactivado
PUERTO_UDP = 5005

VELOCIDADES = [0.3, 0.5, 0.8, 1.2, 1.6, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]
REPETICIONES = 3            # veces que se recorre la lista completa
T_ASENTAR = 1.0             # s a velocidad constante antes de empezar a promediar (alineación caster)
T_MEDIR = 1.2               # s promediando en suelo
T_REPOSO = 1.0              # s en 0 entre pares de mesetas

RUEDAS_AIRE = ['IZQ', 'DER']
T_MEDIR_AIRE = 2.0

# PI de velocidad del OpenCR ajustado para evitar oscilaciones (ciclos límite)
KP = 4.0                    # u/(rad/s)
KI = 25.0                   # u/(rad/s)/s
ACEL = 4.0                  # rad/s^2 de la rampa entre mesetas

DIST_MAX = 1.5              # m: si el carrito se aleja más del punto de partida se corta
TOL_VEL = 0.08              # rad/s + 8 %: tolerancia para considerar alcanzada la meseta
LIMITE_UNIDADES = 400       # debe coincidir con LIMITE_CORRIENTE del firmware
# =====================================================================

MA = 2.69           # mA por unidad de corriente
RPM = 0.229         # rpm por unidad de velocidad
RADPS = RPM * 2 * math.pi / 60
TICKS = 4096.0
RADIO = 0.033       # m
RUEDAS = ('IZQ', 'DER')

ser = None
udp = None
datos = []
mesetas = []
ultimo_envio = -1.0
odo = {'ref': None, 'acum': {'IZQ': 0.0, 'DER': 0.0}, 'prev': None}

class LimiteDistancia(Exception):
    pass

class FirmwareIncorrecto(Exception):
    pass

def enviar(texto):
    ser.write((texto + '\n').encode())

def enviar_udp(texto):
    if udp:
        try:
            udp.sendto(texto.encode(), (LAPTOP_IP, PUERTO_UDP))
        except OSError:
            pass

def leer_muestra():
    """Devuelve la siguiente línea X del OpenCR ya convertida a sentido lógico y unidades SI."""
    lineas_t = 0
    while True:
        linea = ser.readline().decode(errors='ignore').strip()
        if not linea:
            continue
        if linea.startswith('T,'):
            lineas_t += 1
            if lineas_t > 100:
                raise FirmwareIncorrecto()
            continue
        if not linea.startswith('X,'):
            print(f"\n  [OpenCR] {linea}")
            continue
        p = linea.split(',')
        if len(p) != 15:
            continue
        try:
            v = [float(x) for x in p[1:]]
        except ValueError:
            continue
        s = SIGNO
        m = dict(t=v[0] / 1e6, modo=int(v[1]),
                 ref={'IZQ': s['IZQ'] * v[2], 'DER': s['DER'] * v[3]},
                 goal={'IZQ': s['IZQ'] * v[4] * MA, 'DER': s['DER'] * v[5] * MA},
                 cur={'IZQ': s['IZQ'] * v[6] * MA, 'DER': s['DER'] * v[7] * MA},
                 vel={'IZQ': s['IZQ'] * v[8] * RADPS, 'DER': s['DER'] * v[9] * RADPS},
                 pos={'IZQ': s['IZQ'] * int(v[10]), 'DER': s['DER'] * int(v[11])},
                 n=int(v[12]), lat_us=int(v[13]))
        actualizar_odometria(m)
        enviar_udp(f"D,{m['t']:.3f},{m['goal']['IZQ']:.1f},{m['goal']['DER']:.1f},"
                   f"{m['cur']['IZQ']:.1f},{m['cur']['DER']:.1f},"
                   f"{m['vel']['IZQ'] / RADPS * RPM:.2f},{m['vel']['DER'] / RADPS * RPM:.2f}")
        return m

def actualizar_odometria(m):
    if odo['prev'] is not None:
        for r in RUEDAS:
            d = (m['pos'][r] - odo['prev'][r] + 2048) % 4096 - 2048
            odo['acum'][r] += d / TICKS * 2 * math.pi
    odo['prev'] = dict(m['pos'])

def distancia_recorrida():
    return RADIO * (odo['acum']['IZQ'] + odo['acum']['DER']) / 2

def ciclo(ref, etiqueta=''):
    global ultimo_envio
    m = leer_muestra()
    if ultimo_envio < 0 or m['t'] - ultimo_envio >= 0.05 or m['t'] < ultimo_envio:
        enviar(f"V,{SIGNO['IZQ'] * ref.get('IZQ', 0.0):.4f},{SIGNO['DER'] * ref.get('DER', 0.0):.4f}")
        ultimo_envio = m['t']
    datos.append([round(m['t'], 4), etiqueta, m['modo'],
                  m['ref']['IZQ'], m['ref']['DER'], m['goal']['IZQ'], m['goal']['DER'],
                  m['cur']['IZQ'], m['cur']['DER'], m['vel']['IZQ'], m['vel']['DER'],
                  m['pos']['IZQ'], m['pos']['DER'], m['n'], m['lat_us']])
    if MODO == 'suelo' and abs(distancia_recorrida()) > DIST_MAX:
        raise LimiteDistancia()
    return m

def esperar(segundos):
    m = ciclo({})
    t_fin = m['t'] + segundos
    while m['t'] < t_fin:
        m = ciclo({})

def pendiente(t, y):
    n = len(t)
    if n < 3:
        return 0.0
    tm, ym = sum(t) / n, sum(y) / n
    den = sum((a - tm) ** 2 for a in t)
    return sum((a - tm) * (b - ym) for a, b in zip(t, y)) / den if den > 0 else 0.0

def meseta(ref, t_medir, id_meseta):
    ruedas = [r for r in RUEDAS if r in ref]
    etiqueta = f'm{id_meseta}'
    # 1) rampa
    m = ciclo(ref, etiqueta + '_rampa')
    t_lim = m['t'] + 2 * max(abs(v) for v in ref.values()) / max(ACEL, 0.1) + 3.0
    while any(abs(m['ref'][r] - ref[r]) > 1e-3 for r in ruedas) and m['t'] < t_lim:
        m = ciclo(ref, etiqueta + '_rampa')
    # 2) asentar
    t_fin = m['t'] + T_ASENTAR
    while m['t'] < t_fin:
        m = ciclo(ref, etiqueta + '_asentar')
    # 3) medir
    ventana = []
    t_fin = m['t'] + t_medir
    while m['t'] < t_fin:
        m = ciclo(ref, etiqueta + '_medir')
        ventana.append(m)
        txt = f"\r  meseta {id_meseta:3d} medir"
        for r in ruedas:
            txt += f" | {r} ref={ref[r]:+5.2f} vel={m['vel'][r]:+5.2f} rad/s cur={m['cur'][r]:+6.1f} mA"
        print(txt + f" | x={distancia_recorrida():+.2f} m   ", end='', flush=True)

    alcanzada = {}
    max_std_vel = 0.45 if MODO == 'suelo' else 0.18
    for r in ruedas:
        n_tot = sum(w['n'] for w in ventana) or 1
        prom = lambda k: sum(w[k][r] * w['n'] for w in ventana) / n_tot
        eta, i_med, i_goal = prom('vel'), prom('cur'), prom('goal')
        vels = [w['vel'][r] for w in ventana]
        sd = lambda xs: math.sqrt(sum((x - sum(xs) / len(xs)) ** 2 for x in xs) / max(len(xs) - 1, 1))
        eta_std_val = sd(vels)
        deriva = pendiente([w['t'] for w in ventana], vels) * t_medir
        tol = TOL_VEL + 0.08 * abs(ref[r])
        motivo = ''
        if abs(eta - ref[r]) > tol:
            motivo = 'no_alcanza_velocidad'
        elif max(abs(w['goal'][r]) for w in ventana) >= 0.95 * LIMITE_UNIDADES * MA:
            motivo = 'satura_corriente'
        elif abs(deriva) > tol * 1.5:
            motivo = 'no_estacionario'
        elif eta_std_val > max_std_vel:
            motivo = 'oscilacion_pi'
        alcanzada[r] = motivo != 'no_alcanza_velocidad'
        mesetas.append(dict(meseta=id_meseta, rueda=r, modo=MODO, ref=round(ref[r], 4),
                            eta_media=round(eta, 5), eta_std=round(eta_std_val, 5),
                            i_medida_mA=round(i_med, 3), i_goal_mA=round(i_goal, 3),
                            i_std_mA=round(sd([w['cur'][r] for w in ventana]), 3),
                            n_ciclos=sum(w['n'] for w in ventana),
                            valida=int(motivo == ''), motivo=motivo))
        print(f"\n    {r}: eta={eta:+.3f} rad/s  i={i_med:+.2f} mA (goal {i_goal:+.2f})"
              f"  -> tau={ajuste_friccion.KTAU * i_med:+.2f} mN m  {motivo or 'OK'}", end='')
    print()
    return all(alcanzada.values())

def iniciar_motores():
    if not args.sim:
        time.sleep(1.0)
    ser.reset_input_buffer()
    enviar("REBOOT")
    if not args.sim:
        time.sleep(1.0)
    listos = 0
    t_fin = time.monotonic() + 1.5
    while time.monotonic() < t_fin and listos < 2:
        s = ser.readline().decode(errors='ignore').strip()
        if s.startswith('DIAG'):
            print("  ", s)
            if 'com=OK' in s and 'modo=0,' in s and 'torque=1,' in s and 'hw_error=0,' in s:
                listos += 1
    if listos < 2:
        print("\nAlgún motor no está listo (revisa modo/torque/hw_error). Abortando.")
        enviar("STOP")
        raise SystemExit(1)
    enviar(f"G,{KP},{KI}")
    enviar(f"A,{ACEL}")
    enviar("TEL,EXT")
    t_fin = time.monotonic() + 1.0
    while time.monotonic() < t_fin:
        s = ser.readline().decode(errors='ignore').strip()
        if s.startswith('OK:G,'):
            partes = s.split(',')
            kp_real = int(partes[1]) / 100
            ki_real = int(partes[2]) / 100
            if abs(kp_real - KP) > 0.1 or abs(ki_real - KI) > 0.1:
                print(f"\n  ERROR: ganancias no coinciden. Esperado kp={KP} ki={KI}, "
                      f"recibido kp={kp_real} ki={ki_real}")
                raise SystemExit(1)
            print(f"  Ganancias verificadas: kp={kp_real} ki={ki_real}")
        elif s.startswith('DIAG,pi,'):
            print(f"  [OpenCR] {s}")
        elif s.startswith('OK:TEL'):
            break
        elif s:
            print(f"  [OpenCR] {s}")
    print(f"Motores listos. PI: kp={KP} ki={KI} u/(rad/s), rampa {ACEL} rad/s^2")
    if udp:
        print(f"Enviando datos en vivo a {LAPTOP_IP}:{PUERTO_UDP}")

def ensayo_suelo():
    k = 0
    for rep in range(1, REPETICIONES + 1):
        print(f"\n--- Suelo: repetición {rep} ---")
        for v in VELOCIDADES:
            ok = True
            for s in (+1, -1):
                k += 1
                ok &= meseta({'IZQ': s * v, 'DER': s * v}, T_MEDIR, k)
            esperar(T_REPOSO)
            # Solo corta si falla a velocidades altas (> 2.5 rad/s) por saturación real
            if not ok and v > 2.5:
                print(f"  {v} rad/s no se alcanza: se omiten velocidades mayores.")
                break

def ensayo_aire():
    k = 0
    for grupo in RUEDAS_AIRE:
        ruedas = grupo if isinstance(grupo, tuple) else (grupo,)
        for rep in range(1, REPETICIONES + 1):
            print(f"\n--- Aire: ruedas {'+'.join(ruedas)}, repetición {rep} ---")
            for v in VELOCIDADES:
                ok = True
                for s in (+1, -1):
                    k += 1
                    ok &= meseta({r: s * v for r in ruedas}, T_MEDIR_AIRE, k)
                if not ok and v > 2.5:
                    print(f"  {v} rad/s no se alcanza: se omiten velocidades mayores.")
                    break
            esperar(T_REPOSO)

def guardar():
    with open('friccion_datos.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'fase', 'modo', 'ref_izq_radps', 'ref_der_radps', 'goal_izq_mA', 'goal_der_mA',
                    'cur_izq_mA', 'cur_der_mA', 'vel_izq_radps', 'vel_der_radps',
                    'pos_izq_ticks', 'pos_der_ticks', 'n_ciclos', 'lat_max_us'])
        w.writerows(datos)
    if mesetas:
        with open('friccion_mesetas.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(mesetas[0].keys()))
            w.writeheader()
            w.writerows(mesetas)
    print(f"\nDatos en friccion_datos.csv ({len(datos)} muestras) y friccion_mesetas.csv "
          f"({len(mesetas)} renglones)")
    if datos:
        lat = max(d[-1] for d in datos)
        print(f"Ciclo de control del OpenCR: máximo {lat} us (debe ser < 2000 us)")

# ------------------------------ MAIN --------------------------------
ap = argparse.ArgumentParser(description='Identificación de fricción de las ruedas')
ap.add_argument('--sim', action='store_true', help='usa un OpenCR simulado (sin robot)')
ap.add_argument('--modo', choices=('suelo', 'aire'), default=None)
ap.add_argument('--puerto', default=None)
args = ap.parse_args()
if args.modo:
    MODO = args.modo
if args.puerto:
    PUERTO = args.puerto
if args.sim:
    from sim_opencr import OpenCRSimulado
    ser = OpenCRSimulado(signo_fisico=SIGNO, ktau=ajuste_friccion.KTAU)
    print("MODO SIMULACIÓN. Fricción verdadera:", ser.verdad)
else:
    import serial
    ser = serial.Serial(PUERTO, 3000000, timeout=0.1)
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if LAPTOP_IP else None
try:
    iniciar_motores()
    esperar(1.0)
    if MODO == 'suelo':
        print(f"\nSUELO: el carrito irá y regresará en línea recta (máx. {DIST_MAX} m). "
              f"Velocidad máx. {max(VELOCIDADES) * RADIO:.2f} m/s.")
        ensayo_suelo()
    else:
        print("\nAIRE: el robot debe estar sobre un soporte, ruedas sin tocar el piso.")
        ensayo_aire()
except FirmwareIncorrecto:
    print("\nEl OpenCR responde con telemetría 'T' (no 'X'): carga openCR_friccion.ino.")
except LimiteDistancia:
    print(f"\nEl carrito se alejó más de {DIST_MAX} m del inicio: se corta la prueba.")
except KeyboardInterrupt:
    print("\nDetenido por el usuario.")
finally:
    try:
        for _ in range(3):
            enviar("C,0,0")
            if not args.sim:
                time.sleep(0.01)
        enviar("STOP")
        enviar("TEL,STD")
        ser.close()
    except Exception:
        pass

guardar()
if mesetas:
    mesetas_leidas = ajuste_friccion.leer_mesetas('friccion_mesetas.csv')
    res = ajuste_friccion.ajustar(mesetas_leidas)
    est = ajuste_friccion.friccion_estatica('arranque_resultados.csv')
    ajuste_friccion.imprimir(res, est)
    ajuste_friccion.guardar(res, est)
    if args.sim:
        print("\nVerdad del simulador:", ser.verdad)
    ajuste_friccion.graficar(res, mesetas_leidas)
    sys.exit(0)