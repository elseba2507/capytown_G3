# capytown_G3 — RC-0: Primer arranque y calibración

Robótica (08079) · Universidad ESAN · 2026-2 · Prof. Marks Calderón Niquin ·
Sección **S-003** · Equipo **3** · Tag de entrega: `v1.0-rc0`

## Integrantes y roles

| Integrante | Rol |
| --- | --- |
| César Sebastián Patazca Rios | Integración + Planificación |
| Roger Nahim Georgy Patiño Rodulfo | Control |
| Herman Nicolas Figuera Ferrer | Percepción |

El grupo tiene 3 integrantes, por eso Sebastián asume dos roles.

## Montaje

- **Robot:** Yahboom MicroROS-Pi5, carro n.º 8
- **ROS_DOMAIN_ID:** 20 (Pi5 y placa)
- **Variante de calibración:** A1 Torre (recta B4–C4 oeste → este, giro de 360° en B2 antihorario)
- **Variante de validación:** A2 Escalón (recta C4–D4 este → oeste, giro de 360° en A1 horario)

## Resultados

| Parámetro | Valor inicial | Valor final |
| --- | --- | --- |
| r (`wheel_radius`) | 0.0325 m | **0.0311 m** |
| b_eff (`wheel_separation`) | 0.130 m (= W, ancho físico) | **0.2259 m** |

b_eff se calibró primero el 27/10 (0.2389 m) con la cruz de cinta bajo las ruedas. El 30/10 se recalibró solo sobre el film, porque la cinta alteraba el giro (ver informe).

## Contenido del repositorio

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
| `informe.pdf` | Informe de una página |

## Cómo reproducir

Bringup con los parámetros del grupo (dentro del contenedor del robot):

```bash
ros2 launch capytown_esan bringup.launch.py wheel_params:=/root/yahboomcar_ws/src/capytown_esan/config/wheel_params.yaml
```

Gráficas de un bag (desde la carpeta `plots/`):

```bash
python3 ../capytown_esan/plot_odom.py ../bags/rc0_recta
```
