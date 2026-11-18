# capytown_G3 — RC-1: Control PID, del simulador al robot real

Robótica (08079) · Universidad ESAN · 2026-2 · Prof. Marks Calderón Niquin ·
Sección **S-003** · Equipo **3** · Tag de entrega: `v1.0-rc1`

## Integrantes y roles

| Integrante | Rol |
| --- | --- |
| César Sebastián Patazca Rios | Integración + Planificación |
| Roger Nahim Georgy Patiño Rodulfo | Control |
| Herman Nicolas Figuera Ferrer | Percepción |

## Montaje

- **Robot:** Yahboom MicroROS-Pi5, carro n.º 8 (`roscar8`, 10.42.0.1)
- **ROS_DOMAIN_ID:** 8
- **Simulador:** `yahboomcar_sim`, mundo `arena_rc1`, ruido `noise:=low`
- **Odometría del robot:** `/odom` de `odom_corrector` (solo encoders, sin EKF), ≈ 11 Hz

## Ganancias finales

| | Recta (kp, ki, kd) | Giro (kp_yaw, ki_yaw, kd_yaw) | Velocidades |
| --- | --- | --- | --- |
| Simulador | 3.0, 0.5, 4.0 | 5.0, 0.0, 0.4 | 0.20 m/s, max_angular 2.0 |
| Robot real | 2.0, 0.0, 4.0 | 5.0, 0.0, 0.4 | 0.10 m/s, max_angular 0.5 |

Re-sintonías en el robot (una por prueba): 1) `max_angular` 1.0 → 0.5 (derrape al girar); 2) `ki` 0.5 → 0 (wind-up); 3) `kp` 3 → 2 (curva inicial más suave). Calibración de `wheel_separation` (b_eff) 0.2259 → 0.2132 m en `config/wheel_params_rc1.yaml`.

## Resultados del robot

| Criterio | Medido | Meta |
| --- | --- | --- |
| E5 giro 90° | 0°, 0°, 0° | ≤ 3° |
| E6 recta 1D | −1.5, +13.5, +5, +12, +3, +7.5 cm (kp 2) | ≤ 5 cm |
| E7 cuadrado | 3A 15 cm · 3B 75, 44, 54.5 cm (según `/odom`: 2.2, 3.2, 1.4 cm) | 3B ≤ 25 cm y < 3A |

La diferencia entre `/odom` y la cinta viene del derrape de las ruedas al girar, que los encoders no ven (ver `informe.pdf`).

## Contenido del repositorio (RC-1)

| Ruta | Qué es |
| --- | --- |
| `config/rc1_pid.yaml` | Ganancias finales del simulador y del robot |
| `config/wheel_params_rc1.yaml` | Parámetros de odometría usados en el RC-1 (b_eff 0.2132) |
| `csv/` | Un CSV por prueba (`exp*_sim`, `exp*_real`), `hoja_resultados.csv` y `tabla_sim_robot.csv` |
| `plots/` | Un PNG por prueba, generado con `09_plot_csv.py` |
| `bags/rc1_real_*` | Bags del robot (`/odom`, `/cmd_vel`, `/tf`) |
| `lab_log.txt` | Verificación del bringup y anotaciones del laboratorio |
| `informe.pdf` | Informe de una página del RC-1 |

## Cómo reproducir

Bringup en el robot (dentro del contenedor, tras `./ros2_humble.sh`):

```bash
ros2 launch capytown_esan bringup.launch.py wheel_params:=/root/yahboomcar_ws/capytown_G3/rc1/wheel_params_rc1.yaml
```

Cuadrado 3B con las ganancias finales:

```bash
python3 cuadrado_controller.py --ros-args -p experimento:=cuadrado -p kp:=2.0 -p ki:=0.0 -p kd:=4.0 -p kp_yaw:=5.0 -p ki_yaw:=0.0 -p kd_yaw:=0.4 -p linear_speed:=0.10 -p max_angular:=0.5 -p log_csv:=exp3b_real_run1.csv
```

Gráfica de un CSV:

```bash
python3 09_plot_csv.py csv/exp3b_real_run1.csv --experimento cuadrado --lado 2.0
```

---

## RC-0: Primer arranque y calibración

Robótica (08079) · Universidad ESAN · 2026-2 · Prof. Marks Calderón Niquin ·
Sección **S-003** · Equipo **3** · Tag de entrega: `v1.0-rc0`

### Integrantes y roles

| Integrante | Rol |
| --- | --- |
| César Sebastián Patazca Rios | Integración + Planificación |
| Roger Nahim Georgy Patiño Rodulfo | Control |
| Herman Nicolas Figuera Ferrer | Percepción |

El grupo tiene 3 integrantes, por eso Sebastián asume dos roles.

### Montaje

- **Robot:** Yahboom MicroROS-Pi5, carro n.º 8
- **ROS_DOMAIN_ID:** 20 (Pi5 y placa)
- **Variante de calibración:** A1 Torre (recta B4–C4 oeste → este, giro de 360° en B2 antihorario)
- **Variante de validación:** A2 Escalón (recta C4–D4 este → oeste, giro de 360° en A1 horario)

### Resultados

| Parámetro | Valor inicial | Valor final |
| --- | --- | --- |
| r (`wheel_radius`) | 0.0325 m | **0.0311 m** |
| b_eff (`wheel_separation`) | 0.130 m (= W, ancho físico) | **0.2259 m** |

b_eff se calibró primero el 27/10 (0.2389 m) con la cruz de cinta bajo las ruedas. El 30/10 se recalibró solo sobre el film, porque la cinta alteraba el giro (ver informe).

### Contenido del repositorio

| Ruta | Qué es |
| --- | --- |
| `config/wheel_params.yaml` | r y b_eff finales |
| `setup_log.txt` | Nodos, tópicos y frecuencias del arranque (Parte A) |
| `linea_recta.csv` | Corridas de la recta de 1.0 m (Parte B), distancias en metros |
| `calibration_log.csv` | Iteraciones de b_eff (Parte C), generado por `calibrate_beff`: filas 1–9 del 27/10 y el resto del 30/10. Las fechas vienen del reloj de la Raspberry, que está desfasado |
| `validacion.csv` | Validación en A2 sin recalibrar (Parte D). Los valores reales son aproximados (no se anotaron en el momento); la odometría sale de los bags |
| `bags/` | `rc0_recta`, `rc0_giro`, `rc0_val_recta`, `rc0_val_recta_2`, `rc0_val_giro` |
| `capytown_esan/plot_odom.py` | Lee un bag y genera las gráficas x-y y θ(t) (Parte E) |
| `plots/` | Gráficas generadas |
| `informe_rc0.pdf` | Informe de una página del RC-0 |

### Cómo reproducir

Bringup con los parámetros del grupo (dentro del contenedor del robot):

```bash
ros2 launch capytown_esan bringup.launch.py wheel_params:=/root/yahboomcar_ws/src/capytown_esan/config/wheel_params.yaml
```

Gráficas de un bag (desde la carpeta `plots/`):

```bash
python3 ../capytown_esan/plot_odom.py ../bags/rc0_recta
```
