#!/usr/bin/env python3
"""GranPrix capytown_G3 · Control: planificador de tramo + ejecutor.

Pose creida (x, y, th):
    th  = integral del giroscopio (/imu angular_velocity.z, menos su sesgo)
    x,y = avance de /odom proyectado con th
Planificador: desde la pose creida al dock siguiente
    phi = atan2(yd - y, xd - x) - th        L = hypot(xd - x, yd - y)
Escalas (real = escala * sensor), las corrige el estimador en cada pit:
    th += ca * (wz - sesgo) * dt          ds_real = cd * ds_odom
Ejecutor (lazo abierto respecto al mundo):
    gira hasta que th avance phi          (equivale a phi / ca en el giroscopio)
    avanza hasta recorrer L - freno       (equivale a L / cd en /odom), rumbo sostenido con th

Topicos: suscribe /odom, /imu · publica /cmd_vel
Uso (desde la carpeta final/ del robot):
    python3 ejecutor.py --ros-args -p modo:=giro  -p log_csv:=ej_giro1.csv
    python3 ejecutor.py --ros-args -p modo:=recta -p log_csv:=ej_recta1.csv
    python3 ejecutor.py --ros-args -p modo:=cuadrado -p vueltas:=1 -p log_csv:=ej_cuad1.csv
"""
import csv
import math

DOCKS = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]   # D1..D4 respecto a D1 (m)


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Core:
    """Maquina de estados sin ROS: SESGO -> (GIRA -> AVANZA -> PARADA)* -> FIN."""

    def __init__(self, modo="cuadrado", vueltas=1, v=0.10, w_max=0.5, cd=1.005, ca=1.03,
                 freno=0.027, kp_giro=3.0, w_min=0.12, tol_giro=math.radians(1.0),
                 kp_rumbo=2.0, t_parada=2.0, t_sesgo=1.0):
        self.modo, self.v, self.w_max, self.cd, self.ca = modo, v, w_max, cd, ca
        self.freno, self.kp_giro, self.w_min, self.tol = freno, kp_giro, w_min, tol_giro
        self.kp_rumbo, self.t_parada, self.t_sesgo = kp_rumbo, t_parada, t_sesgo
        if modo == "giro":
            self.plan = [("GIRA", math.pi / 2)]
        elif modo == "recta":
            self.plan = [("AVANZA", 2.0)]
        else:                                   # cuadrado: dock k -> dock k+1, parando en cada uno
            self.plan = [("IR", (k + 1) % 4) for k in range(4 * vueltas)]
        self.x = self.y = self.th = 0.0
        self.estado, self.t, self.t_est = "SESGO", 0.0, 0.0
        self.sesgo_n, self.sesgo_sum, self.sesgo = 0, 0.0, 0.0
        self.s_prev = None
        self.llegadas = []                      # (dock, x, y, th) creidos al parar
        self.dock = 0

    # ---- sensores -------------------------------------------------------------
    def imu(self, wz, dt):
        if self.estado == "SESGO":
            self.sesgo_n += 1
            self.sesgo_sum += wz
        else:
            self.th += self.ca * (wz - self.sesgo) * dt

    def odom(self, s):
        """s = avance acumulado segun /odom (m, con signo: + hacia adelante)."""
        if self.s_prev is not None and self.estado != "SESGO":
            ds = self.cd * (s - self.s_prev)
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        self.s_prev = s

    # ---- control --------------------------------------------------------------
    def _siguiente(self):
        if not self.plan:
            self.estado = "FIN"
            return
        tipo, arg = self.plan.pop(0)
        if tipo == "IR":                        # planificador de tramo
            self.dock = arg
            xd, yd = DOCKS[arg]
            phi = norm(math.atan2(yd - self.y, xd - self.x) - self.th)
            L = math.hypot(xd - self.x, yd - self.y)
            self.plan[:0] = [("GIRA", phi), ("AVANZA", L), ("PARADA", arg)]
            return self._siguiente()
        self.estado, self.t_est = tipo, 0.0
        if tipo == "GIRA":
            self.th0, self.obj, self.t_tol = self.th, arg, 0.0
        elif tipo == "AVANZA":
            self.s0, self.rumbo = self.s_prev, self.th
            self.obj = max(0.0, arg - self.freno)

    def update(self, dt):
        self.t += dt
        self.t_est += dt
        v = w = 0.0
        if self.estado == "SESGO":
            if self.t_est >= self.t_sesgo and self.s_prev is not None:
                self.sesgo = self.sesgo_sum / max(1, self.sesgo_n)
                self._siguiente()
        elif self.estado == "GIRA":
            e = self.obj - (self.th - self.th0)
            w = clamp(self.kp_giro * e, -self.w_max, self.w_max)
            if abs(e) < self.tol:
                w = 0.0                             # dentro de la tolerancia: quieto
            elif abs(w) < self.w_min:
                w = math.copysign(self.w_min, e)    # minimo para vencer el roce
            self.t_tol = self.t_tol + dt if abs(e) < self.tol else 0.0
            if self.t_tol >= 0.2 or self.t_est > 15.0:
                w = 0.0
                self._siguiente()
        elif self.estado == "AVANZA":
            v = self.v
            w = clamp(self.kp_rumbo * norm(self.rumbo - self.th), -self.w_max, self.w_max)
            if self.cd * (self.s_prev - self.s0) >= self.obj or self.t_est > 60.0:
                v = w = 0.0
                self._siguiente()
        elif self.estado == "PARADA":
            if self.t_est >= self.t_parada:       # ya quieto: esta es la llegada creida
                self.llegadas.append((self.dock, self.x, self.y, self.th))
                self._siguiente()
        return v, w


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Twist

    class Ejecutor(Node):
        def __init__(self):
            super().__init__("ejecutor")
            d = Core()
            self.declare_parameters("", [
                ("modo", "cuadrado"), ("vueltas", 1), ("linear_speed", d.v), ("max_angular", d.w_max),
                ("cd", d.cd), ("ca", d.ca), ("freno", d.freno), ("t_parada", d.t_parada),
                ("log_csv", "")])
            g = lambda n: self.get_parameter(n).value
            self.c = Core(modo=g("modo"), vueltas=int(g("vueltas")), v=float(g("linear_speed")),
                          w_max=float(g("max_angular")), cd=float(g("cd")), ca=float(g("ca")),
                          freno=float(g("freno")), t_parada=float(g("t_parada")))
            self.s = 0.0
            self.xy = None
            self.t_imu = None
            self.wz = 0.0
            self.rx = {"odom": None, "imu": None}
            self.log = None
            if g("log_csv"):
                self.f = open(g("log_csv"), "w", newline="")
                self.log = csv.writer(self.f)
                self.log.writerow(["t", "estado", "x", "y", "th_deg", "s_odom", "v", "w", "wz"])
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.create_subscription(Imu, "/imu", self.on_imu, 50)
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_timer(1.0 / 30.0, self.loop)
            self.reportado = 0
            self.fin_dicho = False
            self.get_logger().info(f"ejecutor listo: modo={g('modo')} cd={self.c.cd} ca={self.c.ca} "
                                   f"freno={self.c.freno}. Quieto 1 s para medir el sesgo del giroscopio...")

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_odom(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            yaw_o = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if self.xy is not None:            # avance con signo: el vaiven al girar se cancela
                dx, dy = p.x - self.xy[0], p.y - self.xy[1]
                self.s += dx * math.cos(yaw_o) + dy * math.sin(yaw_o)
            self.xy = (p.x, p.y)
            self.c.odom(self.s)
            self.rx["odom"] = self.now()

        def on_imu(self, m):
            t = self.now()
            self.wz = m.angular_velocity.z
            if self.t_imu is not None:
                self.c.imu(self.wz, t - self.t_imu)
            self.t_imu = t
            self.rx["imu"] = t

        def loop(self):
            t = self.now()
            cmd = Twist()
            vivos = all(r is not None and t - r < 0.5 for r in self.rx.values())
            if vivos and self.c.estado != "FIN":
                v, w = self.c.update(1.0 / 30.0)
                cmd.linear.x, cmd.angular.z = float(v), float(w)
            self.pub.publish(cmd)
            if self.log:
                c = self.c
                self.log.writerow([f"{c.t:.3f}", c.estado, f"{c.x:.4f}", f"{c.y:.4f}",
                                   f"{math.degrees(c.th):.2f}", f"{self.s:.4f}",
                                   f"{cmd.linear.x:.3f}", f"{cmd.angular.z:.3f}", f"{self.wz:.4f}"])
            while self.reportado < len(self.c.llegadas):
                k, x, y, th = self.c.llegadas[self.reportado]
                self.get_logger().info(f"LLEGADA E{k + 1}: cree x={x * 100:.1f} cm  y={y * 100:.1f} cm  "
                                       f"th={math.degrees(th):.1f} deg")
                self.reportado += 1
            if self.c.estado == "FIN" and not self.fin_dicho:
                self.get_logger().info(f"FIN. giro acumulado (giroscopio) = {math.degrees(self.c.th):.1f} deg, "
                                       f"avance /odom = {self.s * 100:.1f} cm, t = {self.c.t:.1f} s")
                self.fin_dicho = True

    rclpy.init(args=args)
    node = Ejecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        if node.log:
            node.f.close()


if __name__ == "__main__":
    main()
