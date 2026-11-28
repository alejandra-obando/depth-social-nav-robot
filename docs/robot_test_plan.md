# Plan de pruebas en el robot real

Cómo comprobar, paso a paso, que el control convencional funciona en el robot: primero cada pieza sola (cada
cámara por separado), luego juntándolas, hasta la prueba completa del examen. **No se pasa al paso siguiente
hasta que el anterior cumple su criterio.** Al final hay una tabla para anotar los resultados.

Qué hace cada pieza: [`conventional_control.md`](conventional_control.md). Instalación y comandos de referencia:
[`../software/README.md`](../software/README.md).

## Antes de empezar

**Seguridad (igual que en el segundo parcial):** no hay parada física. En toda prueba con el robot en el suelo,
una persona junto al interruptor de la batería, espacio despejado y velocidades bajas. Parada por software desde
cualquier shell de la Pi:

```bash
ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: true}"    # parar
ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: false}"   # reanudar
```

**Material:** cinta métrica, cinta de enmascarar para marcar el suelo, los dos tags impresos (tag36h11, ID 0 y
ID 1, cuadrado negro de 16 cm con margen blanco), una caja de cartón, una pila de 10 cm (o algo así de bajo), el
bafle conectado a la Pi.

**Dos terminales en el PC:**
- **PC:** en la carpeta del repo (`~/depth-social-nav-robot`).
- **Pi:** `ssh wallens@<ip de la Pi>` y luego, en cada shell nueva, `source ~/wallens/scripts/ros-env.sh`.

En la Pi, para no repetir la ruta del archivo de parámetros:

```bash
P=~/ros_ws/install/bringup/share/bringup/config/social_nav.yaml
```

Los parámetros se cambian en el PC (`software/src/bringup/config/social_nav.yaml`) y se llevan a la Pi con
`software/scripts/deploy.sh`. Así lo que se mide queda guardado en el repo.

---

## Paso 0 · En el PC: el código está bien

```bash
cd software/src && ../../.venv/bin/python -m pytest -q && cd ../..
.venv/bin/python -m simulation.run_conventional --scenario full --seed 0
```

**Pasa si:** todos los tests pasan y la corrida termina con `phase1_completed` y `phase2_completed` en `true`.

## Paso 1 · Instalar en la Pi

Solo la primera vez (detalle en `software/README.md`, "On the robot"):

```bash
# PC
WALLENS_IP=<ip> software/scripts/deploy.sh --no-tests
# Pi
sudo ~/social_nav/scripts/install-pi.sh deps
~/social_nav/scripts/install-pi.sh
```

**Pasa si:** `ros2 pkg executables bringup` lista `social_nav_node`, `speaker_node`, `sensor_check` y `make_voice`;
`ls -l /dev/cam_rear` existe con la C270 conectada; `ls ~/.config/social_nav/voice/` tiene 8 archivos `.wav`.

## Paso 2 · La base del segundo parcial

### 2a. Servicio y tópicos

```bash
systemctl is-active wallens                   # active
ros2 topic hz /scan                           # >= 5 Hz
ros2 topic hz /odometry/filtered              # >= 20 Hz
```

### 2b. Huella sin pinza en el freno

Copiar el `footprint` de `software/config/footprint_no_gripper.yaml` a
`ros/wallens_safety/config/wallens_safety.yaml` **y** a `ros/wallens_nav/config/nav2.yaml` del repo wallens
(su test compara los dos), desplegar con su `scripts/desplegar.sh` y reiniciar `wallens`.

**Pasa si:** con una caja a 0,5 m al frente, esta orden **no** llega a tocarla (frena ~15 cm antes):

```bash
ros2 topic pub -r 10 --times 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

### 2c. Odometría (Fase 0 del plan del parcial)

Marcar el punto de partida en el suelo. Avanzar 2 m (0,10 m/s durante 20 s) y medir con la cinta; luego un
giro de 180° (0,5 rad/s durante ~6,3 s). Repetir 3 veces.

```bash
ros2 topic echo --once /odometry/filtered | grep -A3 "position:"      # antes y después
ros2 topic pub -r 10 --times 200 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}}"
ros2 topic pub -r 10 --times 63 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

**Pasa si:** en las 3 corridas, la diferencia entre la cinta y la odometría es ≤ 5 cm y ≤ 3°. Si el giro patina,
bajar la velocidad de giro (`omega_max`) y la aceleración del driver.

## Paso 3 · LiDAR visto por nuestro código

```bash
ros2 run bringup sensor_check --ros-args -p check:=lidar --params-file $P -r __node:=social_nav_node
```

1. Caja a 0,5 m **al frente** → la línea `front:` dice el punto más cercano ~0,5 m a **~0°**.
2. Caja a la **izquierda** → ángulo **positivo** (~+90°); a la derecha, negativo.
3. Robot a ~1 m de una **esquina** de la sala, mirándola → `corners:` muestra una esquina a ~1,4 m con paredes
   a ~90°.

**Pasa si:** las tres coinciden. Si el ángulo sale al revés o girado, revisar `lidar.yaw` / `lidar.reversed`
en el YAML (deben coincidir con el TF `base_link -> base_laser` del segundo parcial).

## Paso 4 · Medir y anotar la posición de las cámaras

Con la cinta, desde el **centro del eje de las ruedas** (x hacia adelante, y a la izquierda) y la **altura desde
el suelo**; la inclinación con un transportador o el nivel del celular. Anotar en `social_nav.yaml`:
`front_camera`, `depth_camera` (D415, la misma montura) y `rear_camera` (C270, `yaw: 3.14159` porque mira hacia
atrás; `pitch` positivo = mira hacia arriba). Medir también `start_to_corner`: distancia de la posición de
salida del robot a la esquina de partida. Desplegar (`software/scripts/deploy.sh`).

## Paso 5 · D415 color sola (tag de la espalda, ID 0)

```bash
# Pi, terminal 1: solo los drivers de las cámaras, sin controlador
ros2 launch bringup social_nav.launch.py controller:=false
# Pi, terminal 2
ros2 topic hz /d415/color/image_raw                             # ~15 Hz
ros2 run bringup sensor_check --ros-args -p check:=front --params-file $P -r __node:=social_nav_node
```

Debe decir `calibrated` al arrancar: la D415 trae su calibración de fábrica. Una persona con el tag de la
espalda **a ~0,30 m del suelo** se pone delante, de espaldas al robot, y se mide con la cinta desde el **borde
delantero** del robot:

| Distancia real al borde | 0,7 m | 1,0 m | 1,5 m | 2,0 m |
|---|---|---|---|---|
| `from the front edge` | | | | |

Luego, a 1 m: un paso a la izquierda → `bearing` positivo; a la derecha → negativo. Probar también caminando
despacio y girando un poco (±30°) el tag.

**Pasa si:** error ≤ 5 cm hasta 1,5 m y ≤ 10 cm a 2 m; el signo del ángulo es el correcto; el tag se sigue
viendo caminando. Si la distancia tiene un error fijo, corregir `front_camera.x`; si la altura (`height`) sale
muy distinta de 0,30 m, revisar `z` y `pitch`.

## Paso 6 · C270 sola (tag del pecho, ID 1)

```bash
# con el launch del paso 5 todavía corriendo
ros2 topic hz /c270/image_raw                                    # ~15-30 Hz
ros2 run bringup sensor_check --ros-args -p check:=rear --params-file $P -r __node:=social_nav_node
```

La persona se pone **detrás** del robot, de frente a él, con el tag del pecho a **~1,10 m del suelo**:

| Distancia real al borde trasero | 0,7 m | 1,0 m | 1,2 m | 1,5 m |
|---|---|---|---|---|
| `from the rear edge` | | | | |

La C270 no trae calibración: el nodo usa `rear_camera.fovy_deg` (34,7°, el valor nominal). Si las distancias
tienen todas el mismo error en porcentaje, ajustar ese valor: si el programa mide `d_prog` cuando la cinta dice
`d_real`, el nuevo valor es `2·atan(tan(fovy/2) · d_prog / d_real)`. Ejemplo: mide 1,10 m cuando son 1,00 m con
34,7° → `2·atan(tan(17,35°) · 1,10) ≈ 37,9°`.

**Pasa si:** error ≤ 5 cm entre 0,7 y 1,5 m, y signos de `bearing` correctos (para la cámara trasera, la
izquierda **del robot**).

## Paso 7 · D415 profundidad: obstáculos bajos

```bash
ros2 run bringup sensor_check --ros-args -p check:=depth --params-file $P -r __node:=social_nav_node
```

1. Sin nada delante → `no low obstacle points` (el suelo no debe aparecer como obstáculo).
2. Pila de 10 cm a 0,6 m al frente → puntos a ~0,6 m y ~0°.
3. La misma pila en el paso 3 (`check:=lidar`) → el LiDAR **no** la ve (está a 16 cm). Por eso hace falta esta cámara.

**Pasa si:** las tres se cumplen. Si el suelo aparece como obstáculo, revisar `depth_camera.z` y `pitch`.

## Paso 8 · Las dos cámaras a la vez: CPU y temperatura

Con el launch del paso 5 y un `sensor_check` de cada cámara corriendo:

```bash
top -b -n 1 | head -15
cat /sys/class/thermal/thermal_zone0/temp            # milésimas de °C
```

**Pasa si:** las imágenes siguen a su frecuencia (`ros2 topic hz`), la CPU total < ~80 % y la temperatura < 75 °C.
Si no, bajar `camera_rate` o apagar la profundidad (`use_depth: false`).

## Paso 9 · El bafle solo

```bash
aplay -L                                              # nombre del bafle -> speaker_node.device en el YAML
aplay ~/.config/social_nav/voice/corner.wav
# o con el nodo:
ros2 run bringup speaker_node --ros-args --params-file $P
ros2 topic pub --once /social_nav/alert std_msgs/msg/String "{data: start}"     # y lost, found, corner, guide, wait, done, obstacle
```

**Pasa si:** se oyen las 8 frases, y se entienden con los motores girando (robot levantado, paso 10) a 2 m. Si
la voz de espeak-ng no se entiende, grabar las frases propias con el mismo nombre de archivo (`arecord -f S16_LE
-r 22050 -c 1 lost.wav`).

## Paso 10 · Todo junto, en seco: las ruedas no se mueven

```bash
# Pi, terminal 1: cámaras + controlador + bafle; la velocidad va a cmd_vel_dry, que nadie escucha
ros2 launch bringup social_nav.launch.py dry_run:=true
# Pi, terminal 2 y 3
ros2 topic echo /social_nav/state
ros2 topic echo /cmd_vel_dry
```

Con el robot quieto y una persona con los tags:

| Qué hace la persona | Qué debe pasar |
|---|---|
| Nada visible al principio | Estado `FOLLOW`, `v = 0`: espera al tag. |
| Se pone de espaldas a 1,5 m | Dice "Sígueme"; `linear.x > 0`. |
| A 0,9 m del centro del robot (~0,75 m del borde) | `linear.x` ≈ 0. |
| Se corre a la izquierda | `angular.z > 0`. |
| Se tapa el tag 1 s | Pasa a `LOST`, `v = 0`, dice "No te veo, espérame"; al destaparlo, "Te encontré" y `FOLLOW`. |
| Mantiene tapado el tag 10 s | Gira hacia donde estaba (aquí no gira, solo se ve `angular.z`) y a los 5 s barre. |

**Pasa si:** se cumple toda la tabla. La fase 2 no se puede probar en seco: la odometría no cambia si el robot
no se mueve.

## Paso 11 · Seguir en un pasillo (fase 1 real)

Robot en el suelo, persona junto al interruptor. Desde el PC:

```bash
software/scripts/start.sh            # arranca y guarda la pose de salida
ssh wallens@<ip> journalctl -u social-nav -f
software/scripts/stop.sh             # para
```

1. La persona camina recto por un pasillo de 5 m a paso lento, se detiene, sigue.
2. Gira suave a un lado y al otro.
3. Se tapa el tag con la mano 2 s mientras camina.
4. Otra persona cruza entre el robot y ella.

**Pasa si:** el robot la sigue a ~0,9 m sin tocarla, frena cuando no ve el tag y la reencuentra sola, la
distancia nunca pasa de 1,5 m desde el borde y ninguna pérdida dura más de 5 s. Esta es la condición de salida
de la Fase 1 del plan del parcial.

## Paso 12 · Esquina lejana y cambio de fase

En un rincón con dos paredes: la persona camina hacia la esquina y se detiene cerca de ella.

**Pasa si:** con el robot a menos de 1,8 m de la esquina, dice "Fase uno completa. Ahora te guío de regreso",
espera 3 s, gira hacia la salida, y **no avanza** hasta ver el tag del pecho con la C270; entonces dice
"Sígueme". Que no confunda con una esquina dos cajas puestas en ángulo en medio del camino.

## Paso 13 · Guiar de regreso (fase 2)

Desde el paso 12, la persona camina detrás del robot hasta la salida.

1. Una vez se queda quieta 3 s y se aleja a más de 1,2 m → el robot espera ("Te espero") y sigue cuando ella se
   acerca ("Te encontré").
2. El robot llega a la zona de salida → "Llegamos. Fin del recorrido".

**Pasa si:** nunca la deja a más de 1,5 m y termina en la zona de la esquina de salida (anotar la distancia a la
marca de salida). Si termina lejos, medir mejor `start_to_corner` (paso 4).

## Paso 14 · Obstáculos

Repetir los pasos 11 a 13 con: dos pilas de 10 cm en el camino, un maletín, y otra persona que cruza.

**Pasa si:** sin choques; la persona guía puede rodear los obstáculos y el robot los rodea también.

## Paso 15 · Ensayo completo cronometrado

En 7×7 m o lo más parecido (pasillo largo + rincón con dos paredes), con obstáculos y terceros caminando,
**como en el examen**: un solo arranque con `start.sh`, sin tocar el PC después.

**Pasa si:** ida y vuelta completas en menos de 20 min, sin choques, sin pérdidas de más de 5 s, distancia
≤ 1,5 m, señal de fin de fase audible. Repetir al menos 3 veces seguidas.

---

## Registro de resultados

| Paso | Fecha | ¿Pasa? | Valores medidos / qué se cambió |
|---|---|---|---|
| 0 · Código en el PC | | | |
| 1 · Instalación | | | |
| 2 · Base (freno, odometría) | | | |
| 3 · LiDAR | | | |
| 4 · Posición de las cámaras | | | |
| 5 · D415 color | | | |
| 6 · C270 | | | |
| 7 · Profundidad | | | |
| 8 · CPU y temperatura | | | |
| 9 · Bafle | | | |
| 10 · En seco | | | |
| 11 · Seguir | | | |
| 12 · Esquina y cambio de fase | | | |
| 13 · Guiar de regreso | | | |
| 14 · Obstáculos | | | |
| 15 · Ensayo completo | | | |

## Si algo falla

| Síntoma | Dónde mirar |
|---|---|
| `sensor_check` se queda en "waiting for data" | `ros2 topic list`: el nombre del tópico de la cámara no coincide con `topics.*` del YAML. |
| El tag no se detecta | Tag arrugado o con reflejos; margen blanco cortado; más de ~2,5 m; mucha inclinación. |
| La distancia del tag tiene un error fijo | Posición de la cámara (`x`) mal medida. |
| La distancia del tag tiene un error en porcentaje | Calibración: `fovy_deg` (C270) o `tag_size` (¿el cuadrado negro mide 16 cm?). |
| El robot no se mueve aunque el estado es `FOLLOW` | `/parada` activa; el freno ve algo (`/freno_obstaculo`); batería baja (`/battery_state`). |
| No se oye nada | `speaker_node.device` vacío o mal (`aplay -L`); `journalctl -u social-nav` dice "aplay not found". |
| El robot gira al revés | Signo del LiDAR (paso 3) o `yaw` de la cámara (paso 4). |
