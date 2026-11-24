# capytown_G3 · Reto final GranPrix Delivery Semiautónomo (rama `final`, avance)

Rama de trabajo del reto final. El repositorio de entrega será `capytown_eqNN_final` (fork de `granprix-esan`); esta rama se mueve allí cuando el grupo tenga el link del repo del curso y el número de equipo.

Integrantes: César Sebastián Patazca Rios (Integración) · Roger Nahim Georgy Patiño Rodulfo (Control) · Herman Nicolás Figuera Ferrer (Percepción).
Carrito: Yahboom MicroROS-Pi5 n.º 8 (roscar8), ROS_DOMAIN_ID 8 en su WiFi propia, `odom_corrector` con b_eff 0.2132 y r 0.0311.

## Contenido

| Carpeta | Qué hay |
| --- | --- |
| `carrito/ejecutor.py` | Control v1: planificador de tramo y ejecutor con giros cerrados por el giroscopio |
| `mediciones/` | 5 rectas de 2 m y 5 giros de 90° (23–24/11): CSV del nodo del RC-1 y bags `/odom /imu /cmd_vel` |
| `herramientas/analiza_bags.py` | Compara el giro de `/odom` con el giroscopio integrado y saca avance y desvío de cada bag |
| `herramientas/sim_ejecutor.py` | Prueba offline de `ejecutor.py` con una planta imperfecta como la medida |

## Mediciones del carrito (Nahim, 23–24/11)

Hechas con `cuadrado_controller.py` del RC-1 sin modificar (recta: kp 2, ki 0, kd 4; giro: kp_yaw 5, kd_yaw 0.4; 0.10 m/s y 0.5 rad/s). Piso con papeles de 50 × 50 cm, regla de 30 cm y transportador; los avances de las rectas 3–5 se leyeron de fotos.

| Recta | Real (cm) | /odom en FIN (cm) | /odom final (cm) | real ÷ odom final | Lateral /odom (cm) |
| --- | --- | --- | --- | --- | --- |
| 1 | 205.5 | 200.7 | 203.0 | 1.012 | −1.5 |
| 2 | 205.5 | 200.1 | 203.4 | 1.010 | −1.3 |
| 3 | ≈ 204.4 | 200.8 | 202.9 | 1.007 | +1.7 |
| 4 | ≈ 203.5 | 200.6 | 202.3 | 1.006 | −0.4 |
| 5 | ≈ 201 | 200.5 | 203.5 | 0.988 | −1.0 |

| Giro | Real (°) | /odom (°) | Giroscopio integrado (°) | Tiempo (s) |
| --- | --- | --- | --- | --- |
| 1 | ≈ 90 | 92.0 | 92.4 | 4.1 |
| 2 | ≈ 100 | 91.4 | 94.9 | 4.2 |
| 3 | ≈ 93 | 92.5 | 90.3 | 8.4 |
| 4 | 90 | 92.3 | 86.1 | 4.2 |
| 5 | ≈ 98 | 90.1 | 93.7 | 8.5 |

- **Distancia:** el carrito frena ≈ 2.7 cm después de la orden de parar. Con la /odom final, el error de escala es +0.5 % ± 1.0 %.
- **Giro:** /odom marca 90–92.5° siempre y no ve el derrape de las 4 ruedas (correlación con el real −0.69, error σ 5.3°). El giroscopio lo sigue mejor (correlación 0.76, error σ 3.0°). Al girar, el centro del carrito se corre hasta 4 cm mientras /odom marca 0.2 cm.
- `/imu` no trae orientación: hay que integrar `angular_velocity.z`.

## Simulador del curso y predicción v0

Paso de calibración elegido α = 0.35 (con 0.10, 0.60 y 0.90 sale 28, 24 y 32 % fuera de tolerancia, contra 21 %).

| Escenario (reset + recalibración) | Error medio (cm) | Fuera de tolerancia | Reintentos ≈ | Tiempo por vuelta (s) |
| --- | --- | --- | --- | --- |
| Valores por defecto | 2.9 | 21 % | 3 | 119 |
| Nuestros números, giros con /odom (ruido de giro 3°, máximo del slider) | 9.9 | 79 % | 13 | 127 |
| Meta con giros por giroscopio (ruido 1.5°) | 6.2 | 63 % | 10 | 125 |

**Predicción v0:** ≈ 13 reintentos y 127 s por vuelta. **Meta v1:** ≈ 10 reintentos y 125 s.

## Control v1 (`carrito/ejecutor.py`)

- Pose creída: `th += ca·(wz − sesgo)·dt` con el giroscopio; `x, y` con el avance de `/odom` × cd proyectado con `th`.
- Planificador: `phi = atan2(yd − y, xd − x) − th`, `L = hypot(xd − x, yd − y)`.
- Giro: P sobre el ángulo del giroscopio (kp 3, mínimo 0.12 rad/s, tolerancia 1° durante 0.2 s).
- Recta: rumbo sostenido con el giroscopio (kp 2) y corte en `L − freno`.
- Valores iniciales: cd 1.005, ca 1.03, freno 0.027 m. Sesgo del giroscopio medido 1 s quieto al arrancar.

```bash
python3 ejecutor.py --ros-args -p modo:=giro -p log_csv:=ej_giro1.csv
python3 ejecutor.py --ros-args -p modo:=recta -p log_csv:=ej_recta1.csv
python3 ejecutor.py --ros-args -p modo:=cuadrado -p vueltas:=1 -p log_csv:=ej_cuad1.csv
```

Simulación offline (`python3 herramientas/sim_ejecutor.py`): error en cada dock ≤ 1.1 cm si el giroscopio ve todo el derrape; 2–19 cm si ve el 60 %; 7–48 cm si no lo ve. El reset con el tag en el pit sigue siendo necesario.

Pendiente de probar en el robot (sesión 3).
