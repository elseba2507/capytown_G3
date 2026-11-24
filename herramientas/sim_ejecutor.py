"""Prueba offline de Core (ejecutor.py) con una planta imperfecta como el carrito medido."""
import math, sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "carrito"))
from ejecutor import Core

def corre(modo, vueltas=1, k_d=1.005, k_g=1.03, ruido_giro=0.0, ve=1.0, sesgo=0.003, semilla=1, **kw):
    rng = np.random.default_rng(semilla)
    c = Core(modo=modo, vueltas=vueltas, **kw)
    x = y = th = 0.0; s_odom = 0.0; va = wa = 0.0; dt = 1 / 30; tau = 0.27
    giro_extra = 1.0; prev = c.estado
    real_llegadas = []
    for _ in range(int(400 / dt)):
        v, w = c.update(dt)
        if c.estado == "GIRA" and prev != "GIRA":
            giro_extra = 1.0 + rng.normal(0, ruido_giro)   # derrape aleatorio de este giro
        if prev == "PARADA" and c.estado != "PARADA":
            real_llegadas.append((x, y, th))
        prev = c.estado
        va += (v - va) * dt / tau; wa += (w - wa) * dt / tau
        wr = wa * giro_extra
        x += va * k_d * math.cos(th) * dt; y += va * k_d * math.sin(th) * dt; th += wr * dt
        s_odom += abs(va) * dt
        c.imu((wa + (wr - wa) * ve) / k_g + sesgo, dt); c.odom(s_odom)
        if c.estado == "FIN":
            fin = locals().get("fin", 0) + 1
            if fin > 90: break
    return c, (x, y, th), real_llegadas

c, real, _ = corre("giro")
print(f"giro: real {math.degrees(real[2]):.2f} deg, creido {math.degrees(c.th):.2f}, t {c.t:.1f}s")
c, real, _ = corre("recta")
print(f"recta: real {real[0]*100:.1f} cm lateral {real[1]*100:.2f}, creido x {c.x*100:.1f}, t {c.t:.1f}s")
for rg, ve in ((0.0, 1.0), (0.05, 1.0), (0.05, 0.6), (0.05, 0.0)):
  for sem in (1, 2):
    c, real, rl = corre("cuadrado", vueltas=1, ruido_giro=rg, ve=ve, semilla=sem)
    errs = []
    for (k, xc, yc, thc), (xr, yr, thr) in zip(c.llegadas, rl):
        xd, yd = [(0,0),(2,0),(2,2),(0,2)][k]
        errs.append(math.hypot(xr - xd, yr - yd) * 100)
    print(f"derrape {rg} ve {ve} sem {sem}: error real en cada dock (cm) {np.round(errs,1)}  t {c.t:.0f}s estado {c.estado}")
