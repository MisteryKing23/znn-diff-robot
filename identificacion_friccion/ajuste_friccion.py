"""
Ajuste automático de los coeficientes de fricción de cada rueda a partir de
las mesetas medidas por identificacion_friccion.py.
Incluye filtrado adaptativo para modo suelo/aire y aislamiento automático de la región
lineal (|eta| >= 2.5 rad/s) si detecta arrastre de la rueda loca a bajas velocidades.
"""
import argparse
import csv
import json
import math
import os
import numpy as np

KTAU = 1.05          # N m / A
EPS = 0.1            # rad/s, ancho de tanh en ddmr.Params
ETA_MIN = 0.3        # rad/s: umbral base
ETA_LINEAL = 2.5     # rad/s: umbral de régimen lineal libre de giro de rueda loca
RUEDAS = ('DER', 'IZQ')
SUFIJO = {'DER': 'r', 'IZQ': 'l'}

def mc(X, y):
    """Mínimos cuadrados con errores estándar. Devuelve (theta, se, r2, rmse)."""
    theta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ theta
    n, p = X.shape
    gl = max(n - p, 1)
    s2 = float(res @ res) / gl
    try:
        cov = s2 * np.linalg.inv(X.T @ X)
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(p, np.nan)
    sst = float(((y - y.mean()) ** 2).sum())
    r2 = 1 - float(res @ res) / sst if sst > 0 else float('nan')
    return theta, se, r2, math.sqrt(float(res @ res) / n)

def leer_mesetas(ruta):
    with open(ruta, newline='') as f:
        filas = list(csv.DictReader(f))
    out = []
    for r in filas:
        modo = r.get('modo', 'suelo')
        ref = float(r.get('ref', r['eta_media']))
        eta = float(r['eta_media'])
        eta_std = float(r.get('eta_std', 0.0))
        # Criterio adaptativo: en suelo tolera hasta 0.45 rad/s de vibración de rodadura
        max_std = 0.45 if modo == 'suelo' else 0.18
        vel_ok = abs(eta - ref) <= (0.08 + 0.08 * abs(ref)) * 1.5
        std_ok = eta_std <= max_std
        valida_csv = r['valida'] in ('1', 'True', 'true')
        out.append(dict(rueda=r['rueda'], modo=modo, eta=eta,
                        i_mA=float(r['i_medida_mA']), i_goal_mA=float(r['i_goal_mA']),
                        valida=bool(valida_csv or (vel_ok and std_ok))))
    return out

def _ajustar_subconjunto(pts, ktau, usar):
    eta = np.array([m['eta'] for m in pts])
    i_a = np.array([(m['i_mA'] if usar == 'medida' else m['i_goal_mA']) for m in pts]) * 1e-3
    tau = ktau * i_a
    sg = np.sign(eta)
    ambos = (sg > 0).any() and (sg < 0).any()
    r = {'n': len(pts), 'eta': eta.tolist(), 'tau': tau.tolist()}

    th, se, r2, rmse = mc(np.column_stack([eta, sg]), tau)
    r.update(d=float(th[0]), c=float(th[1]), se_d=float(se[0]), se_c=float(se[1]),
             r2=r2, rmse=rmse)
    if not ambos:
        r['aviso'] = 'solo un sentido de giro: c incluye cualquier offset de corriente'

    asim = {}
    for nombre, msk in (('pos', sg > 0), ('neg', sg < 0)):
        if msk.sum() >= 2 and np.ptp(eta[msk]) > 0:
            th2, se2, r22, _ = mc(np.column_stack([eta[msk], sg[msk]]), tau[msk])
            asim[nombre] = dict(d=float(th2[0]), c=float(th2[1]), se_d=float(se2[0]),
                                se_c=float(se2[1]), r2=r22, n=int(msk.sum()))
    r['asimetrico'] = asim

    if ambos and len(pts) >= 4:
        th3, se3, _, _ = mc(np.column_stack([eta, sg, np.ones_like(eta)]), tau)
        r['con_sesgo'] = dict(d=float(th3[0]), c=float(th3[1]), b=float(th3[2]),
                              se_b=float(se3[2]))
    return r

def ajustar(mesetas, ktau=KTAU, eta_min=ETA_MIN, usar='medida'):
    """Ajusta (d, c) y verifica automáticamente si se requiere aislar la zona lineal (|eta| >= 2.5)."""
    resultado = {'ktau': ktau, 'eta_min': eta_min, 'corriente_usada': usar, 'ruedas': {}}
    for rueda in RUEDAS:
        pts_base = [m for m in mesetas if m['rueda'] == rueda and m['valida'] and abs(m['eta']) >= eta_min]
        if len(pts_base) < 3:
            print(f"  {rueda}: solo {len(pts_base)} mesetas válidas; no se ajusta.")
            continue
        r_base = _ajustar_subconjunto(pts_base, ktau, usar)

        # Verificación automática de consistencia física:
        # Si d <= 0.0002 o hay pendiente negativa en algún sentido por efecto de la rueda loca a baja vel,
        # usa automáticamente las mesetas de régimen lineal (|eta| >= 2.5 rad/s).
        pts_lin = [m for m in pts_base if abs(m['eta']) >= max(eta_min, ETA_LINEAL)]
        d_pos = r_base['asimetrico'].get('pos', {}).get('d', 1.0)
        d_neg = r_base['asimetrico'].get('neg', {}).get('d', 1.0)
        necesita_lin = (r_base['d'] <= 0.0002 or d_pos <= 0.0 or d_neg <= 0.0)

        if necesita_lin and len(pts_lin) >= 6:
            r_lin = _ajustar_subconjunto(pts_lin, ktau, usar)
            if r_lin['d'] > r_base['d'] and r_lin['c'] > 0:
                r_lin['nota_auto'] = f"Ajustado en zona lineal (|eta| >= {ETA_LINEAL} rad/s) para eliminar transitorio de baja velocidad"
                resultado['ruedas'][rueda] = r_lin
                continue

        resultado['ruedas'][rueda] = r_base
    return resultado

def friccion_estatica(ruta, ktau=KTAU):
    if not ruta or not os.path.exists(ruta):
        return {}
    acc = {}
    with open(ruta, newline='') as f:
        for r in csv.DictReader(f):
            if r.get('tipo') == 'arranque':
                acc.setdefault(r['rueda'], []).append(abs(float(r['goal_mA'])))
    return {k: dict(i_mA=float(np.mean(v)), tau_s=ktau * float(np.mean(v)) * 1e-3, n=len(v))
            for k, v in acc.items()}

def imprimir(res, estatica=None):
    print("\n================ FRICCIÓN IDENTIFICADA ================")
    print(f"KTAU = {res['ktau']} N m/A   (|eta| >= {res['eta_min']} rad/s, corriente {res['corriente_usada']})")
    for rueda, r in res['ruedas'].items():
        s = SUFIJO[rueda]
        print(f"\n  Rueda {rueda}  ({r['n']} mesetas)")
        if 'nota_auto' in r:
            print(f"    [AUTO] {r['nota_auto']}")
        print(f"    d_{s} = {r['d']:.5f} ± {r['se_d']:.5f}  N m/(rad/s)")
        print(f"    c_{s} = {r['c']:.5f} ± {r['se_c']:.5f}  N m      "
              f"(= {r['c'] / res['ktau'] * 1e3:.1f} mA)")
        print(f"    R^2 = {r['r2']:.4f}   RMSE = {r['rmse'] * 1e3:.2f} mN m")
        for nombre, a in r['asimetrico'].items():
            print(f"    sentido {nombre}: d = {a['d']:.5f}  c = {a['c']:.5f}  (n={a['n']})")
        if 'con_sesgo' in r:
            b = r['con_sesgo']
            print(f"    sesgo b = {b['b'] * 1e3:+.2f} ± {b['se_b'] * 1e3:.2f} mN m"
                  f"{'   <- significativo' if abs(b['b']) > 2 * b['se_b'] else ''}")
        if 'aviso' in r:
            print(f"    AVISO: {r['aviso']}")
        if estatica and rueda in estatica:
            e = estatica[rueda]
            print(f"    estática (prueba_arranque): {e['tau_s']:.5f} N m = {e['i_mA']:.1f} mA "
                  f"-> tau_s / c = {e['tau_s'] / r['c']:.2f}" if r['c'] > 0 else "")
    print("\nPara ddmr.Params:")
    for rueda, r in res['ruedas'].items():
        s = SUFIJO[rueda]
        print(f"    d_{s}: float = {r['d']:.5f}")
        print(f"    c_{s}: float = {r['c']:.5f}")

def guardar(res, estatica=None, eps=EPS, prefijo='friccion'):
    params = {'eps': eps}
    for rueda, r in res['ruedas'].items():
        s = SUFIJO[rueda]
        params[f'd_{s}'] = round(r['d'], 6)
        params[f'c_{s}'] = round(r['c'], 6)
    with open(f'{prefijo}_params.json', 'w') as f:
        json.dump(params, f, indent=2)
    with open(f'{prefijo}_resultados.json', 'w') as f:
        json.dump({**res, 'estatica': estatica or {}}, f, indent=2)
    print(f"\nParámetros en {prefijo}_params.json (cárgalos con run_robot.py --params {prefijo}_params.json)")
    print(f"Reporte completo en {prefijo}_resultados.json")

def graficar(res, mesetas, ruta='friccion_ajuste.png', eps=EPS):
    try:
        import matplotlib
        if not os.environ.get('DISPLAY') and os.name != 'nt':
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib no está instalado; se omite la gráfica.")
        return
    ruedas = [r for r in RUEDAS if r in res['ruedas']]
    if not ruedas:
        return
    fig, ax = plt.subplots(2, len(ruedas), figsize=(6 * len(ruedas), 7), squeeze=False,
                           gridspec_kw={'height_ratios': [3, 1]})
    ktau = res['ktau']
    for k, rueda in enumerate(ruedas):
        r = res['ruedas'][rueda]
        a, b = ax[0][k], ax[1][k]
        todas = [m for m in mesetas if m['rueda'] == rueda]
        etas_usadas = set(round(x, 5) for x in r['eta'])
        exc = [m for m in todas if round(m['eta'], 5) not in etas_usadas]
        if exc:
            a.plot([m['eta'] for m in exc], [ktau * m['i_mA'] * 1e-3 for m in exc], 'x',
                   color='gray', label='excluidas')
        eta, tau = np.array(r['eta']), np.array(r['tau'])
        a.plot(eta, tau, 'o', color='tab:blue', label='mesetas válidas')
        lim = max(abs(eta).max(), 0.5) * 1.1
        e = np.linspace(-lim, lim, 400)
        a.plot(e, r['d'] * e + r['c'] * np.tanh(e / eps), 'k-',
               label=f"d={r['d']:.4f}, c={r['c']:.4f}")
        for nombre, col in (('pos', 'tab:green'), ('neg', 'tab:red')):
            if nombre in r['asimetrico']:
                s = r['asimetrico'][nombre]
                em = e[e > 0.05] if nombre == 'pos' else e[e < -0.05]
                a.plot(em, s['d'] * em + s['c'] * np.sign(em), '--', color=col, lw=1,
                       label=f"{nombre}: d={s['d']:.4f}, c={s['c']:.4f}")
        a.axhline(0, color='k', lw=0.5)
        a.axvline(0, color='k', lw=0.5)
        a.set_title(f"Rueda {rueda}   R²={r['r2']:.4f}")
        a.set_ylabel('tau = KTAU·i  [N m]')
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
        res_ = tau - (r['d'] * eta + r['c'] * np.sign(eta))
        b.plot(eta, res_ * 1e3, 'o', color='tab:blue')
        b.axhline(0, color='k', lw=0.5)
        b.set_xlabel('eta [rad/s]')
        b.set_ylabel('residuo [mN m]')
        b.grid(alpha=0.3)
    fig.suptitle('Identificación de fricción viscosa + Coulomb (régimen permanente)')
    fig.tight_layout()
    fig.savefig(ruta, dpi=150)
    print(f"Gráfica en {ruta}")
    if os.environ.get('DISPLAY') or os.name == 'nt':
        plt.show()

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mesetas', nargs='?', default='friccion_mesetas.csv')
    ap.add_argument('--ktau', type=float, default=KTAU)
    ap.add_argument('--eta-min', type=float, default=ETA_MIN)
    ap.add_argument('--eps', type=float, default=EPS)
    ap.add_argument('--corriente', choices=('medida', 'goal'), default='medida')
    ap.add_argument('--arranque', default='arranque_resultados.csv')
    ap.add_argument('--sin-grafica', action='store_true')
    a = ap.parse_args()
    mesetas = leer_mesetas(a.mesetas)
    res = ajustar(mesetas, a.ktau, a.eta_min, a.corriente)
    est = friccion_estatica(a.arranque, a.ktau)
    imprimir(res, est)
    guardar(res, est, a.eps)
    if not a.sin_grafica:
        graficar(res, mesetas, eps=a.eps)

if __name__ == '__main__':
    main()