# ftg_rv — Controlador reactivo Follow the Gap (F1Tenth)

Paquete ROS 2 (Humble, `ament_python`) que implementa un controlador reactivo
**Follow the Gap** para el simulador F1Tenth, con **contador de vueltas** y
**cronómetro por vuelta**. Proyecto del primer parcial de Vehículos Autónomos
(ESPOL). Mapa asignado: **SaoPaulo** (Interlagos).

El paquete soporta **dos escenarios ejecutables**, cada uno con su propio
mapa y su propio tuning:

| Escenario | Mapa | Parámetros | Comando |
|---|---|---|---|
| **Parte 1** — pista limpia | `SaoPaulo_map` (simulador) | `config/params.yaml` | `ros2 launch ftg_rv controller.launch.py` (+ simulador) |
| **Parte 2** — obstáculos fijos | `maps/SaoPaulo_obs_map` (este paquete) | `config/params_obs.yaml` | `ros2 launch ftg_rv controller_obs.launch.py` (todo en uno) |

---

## 1. Instalación y ejecución

### 1.1 Prerrequisitos

- ROS 2 Humble y el workspace del curso (`F1Tenth-Repository`) con el
  simulador `f1tenth_gym_ros` ya compilado y funcionando.

### 1.2 Instalar este paquete

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository/src
git clone https://github.com/Raulvillaes/ftg_rv.git ftg_rv
cd ~/F1Tenth-Repository
colcon build --packages-select ftg_rv
source install/setup.bash
```

### 1.3 Configurar el mapa SaoPaulo (una sola vez, solo Parte 1)

La Parte 2 no necesita este paso: su mapa con obstáculos viaja dentro de este
paquete y se instala solo con el `colcon build` de arriba.

1. Descargar el mapa oficial (imagen **y** yaml) del repositorio
   [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks/tree/main/SaoPaulo)
   y copiarlos a `src/f1tenth_gym_ros/maps/`:

   ```bash
   cd ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps
   wget https://raw.githubusercontent.com/f1tenth/f1tenth_racetracks/main/SaoPaulo/SaoPaulo_map.png
   wget https://raw.githubusercontent.com/f1tenth/f1tenth_racetracks/main/SaoPaulo/SaoPaulo_map.yaml
   ```

2. Editar `src/f1tenth_gym_ros/config/sim.yaml` y apuntar `map_path` al
   mapa (ruta absoluta, **sin** extensión):

   ```yaml
   map_path: '/home/TU_USUARIO/F1Tenth-Repository/src/f1tenth_gym_ros/maps/SaoPaulo_map'
   ```

3. **Recompilar el simulador** para que el cambio llegue a `install/`
   (el launch lee el `sim.yaml` instalado, no el de `src/`):

   ```bash
   cd ~/F1Tenth-Repository
   colcon build --packages-select f1tenth_gym_ros
   ```

### 1.4 Ejecutar la Parte 1 (pista limpia)

Terminal 1 — simulador:

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository && source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Terminal 2 — controlador (tuning de `config/params.yaml`):

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository && source install/setup.bash
ros2 launch ftg_rv controller.launch.py
```

> **Nota (mapa en RViz):** si RViz muestra *"No map received"*, el
> `map_server` del simulador no llegó a activarse (condición de carrera de su
> launch; el simulador funciona igual). Se activa a mano con:
>
> ```bash
> ros2 lifecycle set /map_server configure
> ros2 lifecycle set /map_server activate
> ```

### 1.5 Ejecutar la Parte 2 (obstáculos fijos)

Un solo comando levanta el simulador con el mapa con obstáculos **y** el
controlador con el tuning conservador (`config/params_obs.yaml`):

```bash
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository && source install/setup.bash
ros2 launch ftg_rv controller_obs.launch.py
```

No hay que tocar `sim.yaml` para alternar entre las dos partes: el launch
`sim_obs.launch.py` sobreescribe los parámetros propios de este escenario —
`map_path` del bridge y del `map_server` hacia el mapa instalado en el share
de este paquete, y `stheta` (la orientación inicial del vehículo, girada a
-60° para no arrancar de frente al primer obstáculo) — y todo lo demás
(tópicos, número de agentes) se sigue leyendo del `sim.yaml` del simulador.

En la consola del controlador se ve la meta fijada al arrancar y, al
completar cada vuelta, un bloque con el número de vuelta, su tiempo y el
acumulado:

```
==============================================
  VUELTA 3 COMPLETADA
  Tiempo de vuelta:     61.42 s
  Tiempo acumulado:    184.90 s
==============================================
```

### 1.6 Tunear parámetros

Editar `config/params.yaml` (Parte 1) o `config/params_obs.yaml` (Parte 2),
volver a ejecutar `colcon build --packages-select ftg_rv` (solo copia el yaml
a `install/`) y relanzar el controlador. La lista completa de parámetros está
en la sección 5.

## 2. Enfoque: el algoritmo Follow the Gap

Follow the Gap es un algoritmo **reactivo**: no usa mapa ni planificación
global; decide cada comando de dirección y velocidad mirando únicamente el
scan actual del LiDAR. La idea central es *apuntar siempre hacia el hueco
libre más grande*, esquivando implícitamente los obstáculos. Por eso el mismo
controlador sirve para las dos partes sin cambios de lógica: los obstáculos
fijos aparecen en `/scan` igual que cualquier pared, y solo cambia el tuning.

Sobre cada mensaje de `/scan` (a ~250 Hz) se ejecuta este pipeline:

1. **Recorte del FOV** (`_compute_fov_indices`): el LiDAR cubre ~270°, pero
   solo interesan los rayos delanteros (±90° por defecto). Lo que queda
   detrás de los hombros del auto no ayuda a decidir hacia dónde avanzar y
   puede producir "gaps falsos" hacia atrás.

2. **Preprocesado** (`preprocess_lidar`):
   - Lecturas inválidas (`NaN`) se tratan como obstáculo (0 m): lo
     conservador ante un dato corrupto es asumir que allí hay algo.
   - Lecturas infinitas o enormes se **saturan** a `max_range` (10 m): sin
     esto, una lectura de 30 m dominaría siempre la elección del objetivo.
   - **Media móvil** de `smoothing_window` rayos: elimina ruido puntual del
     sensor sin deformar la geometría de la pista.

3. **Burbuja de seguridad** (`apply_safety_bubble`): se busca el punto más
   cercano del scan y se anulan (distancia = 0) todos los rayos dentro de un
   radio físico `bubble_radius` a su alrededor. El radio en metros se
   convierte a número de rayos con `arctan(radio / distancia)`: el mismo
   obstáculo ocupa más rayos cuanto más cerca está. Esta burbuja es lo que
   obliga al auto a *rodear* el obstáculo más próximo en vez de rozarlo.

4. **Gap máximo** (`find_max_gap`): se umbraliza el scan (`gap_threshold`,
   rayos con más de 2 m son "libres") y se busca la **racha contigua más
   larga** de rayos libres, de forma vectorizada con NumPy (sin bucles
   Python, importante a 250 Hz).

5. **Mejor punto** (`find_best_point`): dentro del gap se mezcla el punto
   **más lejano** (rápido: apura las curvas) con el **centro del gap**
   (seguro: se aleja de las paredes) según `best_point_bias`
   (0.0 = solo el más lejano, 1.0 = solo el centro). Es la perilla principal
   de tuning del comportamiento en curva. Detalle importante: en rectas
   largas muchos rayos saturan a `max_range` y empatan como "más lejano";
   entre los empatados se elige el más cercano al centro del gap, porque
   `argmax` a secas devolvería el borde del empate y el auto zigzaguearía
   (frenándose) en plena recta.

6. **Actuación**: el índice del mejor punto se convierte a ángulo real del
   rayo (`angle_min + índice · angle_increment`), se satura al límite físico
   del servo (±0.42 rad) y se publica en `/drive`. La **velocidad es el
   mínimo de dos criterios independientes** (`compute_speed`):
   - *Por ángulo de volante* (escalonada): `max_speed` casi recto,
     `mid_speed` en curvas suaves y `corner_speed` en curvas cerradas.
   - *Por espacio libre al frente* (lineal): `corner_speed` con
     `brake_distance` o menos de pista libre, `max_speed` a partir de
     `full_speed_distance`. Sin este criterio el auto aceleraría en el
     vértice de la curva (donde el volante apunta momentáneamente recto a
     la salida) y no frenaría al final de las rectas; con él, frena
     *entrando* a la curva y acelera *saliendo*, como se espera.

Caso degenerado: si ningún rayo supera el umbral (no hay gap), el auto apunta
al rayo más largo disponible a velocidad de curva, en vez de seguir a ciegas
la última orden.

## 3. Contador de vueltas y cronómetro

Ambos viven en `odom_callback` (suscripción a `/ego_racecar/odom`):

- **Meta**: la posición del *primer* mensaje de odometría (donde aparece el
  auto). Una vuelta se cuenta cuando el auto vuelve a entrar al círculo de
  radio `finish_line_tolerance` alrededor de la meta.
- **Anti doble conteo** (dos protecciones independientes):
  - *Histéresis*: tras contar una vuelta hay que **salir** del círculo
    (alejarse a más de 2× la tolerancia) antes de poder contar la siguiente.
  - *Tiempo mínimo de vuelta* (`min_lap_time`): cruces separados por menos
    de 10 s se ignoran.
- **Cronómetro**: usa `self.get_clock().now()` (reloj de ROS). Al completar
  cada vuelta se imprime en consola el bloque mostrado en la sección 1.5.

## 4. Estructura del código

```
ftg_rv/
├── ftg_rv/
│   └── reactive_node.py            # Todo el controlador (un solo nodo)
├── launch/
│   ├── controller.launch.py        # Parte 1: solo el nodo, con params.yaml
│   ├── sim_obs.launch.py           # Simulador con el mapa con obstáculos
│   └── controller_obs.launch.py    # Parte 2: sim_obs + nodo con params_obs.yaml
├── config/
│   ├── params.yaml                 # Tuning Parte 1 (pista limpia)
│   └── params_obs.yaml             # Tuning Parte 2 (obstáculos, conservador)
├── maps/
│   ├── SaoPaulo_obs_map.png        # Mapa SaoPaulo con los 5 obstáculos fijos
│   └── SaoPaulo_obs_map.yaml       # Metadatos del mapa (mismos que el original)
├── package.xml / setup.py / setup.cfg
└── README.md
```

Funciones principales de `reactive_node.py`:

| Función | Qué hace |
|---|---|
| `lidar_callback` | Orquesta el pipeline Follow the Gap y publica `/drive` |
| `_compute_fov_indices` | Calcula (una vez) los índices del recorte de FOV |
| `preprocess_lidar` | Limpieza de NaN/inf, saturación y suavizado |
| `apply_safety_bubble` | Anula los rayos alrededor del punto más cercano |
| `find_max_gap` | Racha contigua más larga de rayos libres |
| `find_best_point` | Mezcla lejano/centro del gap (`best_point_bias`) |
| `compute_speed` | Mínimo entre velocidad por ángulo y por espacio libre |
| `odom_callback` | Vueltas (histéresis + tiempo mínimo) y cronómetro |
| `publish_drive` | Publica el `AckermannDriveStamped` con el clamp del servo |

### El mapa con obstáculos (Parte 2)

Los 5 obstáculos fijos se dibujaron directamente sobre el PNG del mapa con
GIMP (píxeles negros = ocupado para el simulador y el `map_server`), sobre
una copia renombrada a `SaoPaulo_obs_map.png` que vive en este paquete con su
propio yaml de metadatos. La imagen conserva las dimensiones del original
(2000×2000 px), por lo que la resolución y el origen del yaml no cambian. Así
el mapa original del simulador queda intacto y ambos escenarios conviven.

Para lanzarlo sin modificar el simulador, `sim_obs.launch.py` replica los
nodos de `gym_bridge_launch.py` pero pasa dos fuentes de parámetros al
bridge: `[sim.yaml, {'map_path': <mapa de este paquete>}]`. En `launch_ros`
la última fuente gana, así que solo se sobreescribe el mapa y el resto de la
configuración sigue viniendo del `sim.yaml` del simulador.

## 5. Parámetros

Cada escenario carga su propio yaml (sección 1.6). Valores actuales:

| Parámetro | Parte 1 | Parte 2 | Significado |
|---|---|---|---|
| `max_speed` | 12.0 m/s | 10.0 m/s | Velocidad en recta |
| `mid_speed` | 7.0 m/s | 6.5 m/s | Velocidad en curva suave |
| `corner_speed` | 4.0 m/s | 4.0 m/s | Velocidad en curva cerrada |
| `steering_threshold_low/high` | 10° / 20° | 10° / 17.5° | Umbrales de los 3 niveles de velocidad |
| `brake_distance` | 2.5 m | 5.0 m | Espacio libre frontal al que se va a `corner_speed` |
| `full_speed_distance` | 8.0 m | 8.0 m | Espacio libre frontal que permite `max_speed` |
| `fov_angle` | 90° | 90° | Semiancho del FOV útil |
| `smoothing_window` | 5 | 5 | Ventana de la media móvil |
| `max_range` | 10.0 m | 10.0 m | Saturación del LiDAR |
| `gap_threshold` | 2.0 m | 2.0 m | Distancia mínima de un rayo "libre" |
| `bubble_radius` | 0.4 m | 0.4 m | Radio de la burbuja de seguridad |
| `best_point_bias` | 0.4 | 0.3 | 0 = punto más lejano, 1 = centro del gap |
| `finish_line_tolerance` | 4.0 m | 4.0 m | Radio de detección de la meta |
| `min_lap_time` | 10.0 s | 10.0 s | Tiempo mínimo entre cruces de meta |

Lógica del tuning de cada parte:

- **Parte 1** (pista limpia): ajustado iterativamente en SaoPaulo. La
  configuración inicial conservadora (5.0/3.0/1.5 m/s, sin freno por espacio
  libre) daba vueltas de ~67 s; la final logra **~38.7 s por vuelta** de
  forma estable y sin colisiones.
- **Parte 2** (obstáculos): más conservador donde importa para sobrevivir al
  slalom: menor tope en recta (10 m/s), frenado anticipado (`brake_distance`
  5.0 m: los obstáculos aparecen en el scan con menos preaviso que una curva)
  y `best_point_bias` 0.3 para apuntar algo más lejos dentro del gap y
  suavizar la trayectoria entre paredes alternadas.

---

**Autor:** Raulvillaes — Vehículos Autónomos, ESPOL.
