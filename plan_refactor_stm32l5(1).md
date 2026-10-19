# Plan de refactor: control FOC en STM32L5

Cambios recomendados, ordenados de mayor a menor importancia. Cada punto indica el problema, la idea general de la solución y los archivos afectados.

**Prioridades**

- **P1: Bugs y seguridad.** Hacen que el sistema falle, se cuelgue o dañe hardware. Se arreglan antes de mover nada.
- **P2: Robustez.** Comportamiento correcto solo "de casualidad" o difícil de depurar.
- **P3: Arquitectura.** Separación de módulos y dependencias.
- **P4: Limpieza.** Legibilidad y mantenimiento.

> **Versión 2.** Incorpora la revisión de `general.c`, `general.h` y `user_constants.h`. Los puntos 1, 2, 13, 14, 15 y 21 son nuevos, y el 9, el 17 y el 20 se ampliaron. Cambió la numeración respecto de la versión anterior.

---

## Resumen

| # | Cambio | Prioridad | Archivo(s) |
|---|--------|-----------|------------|
| 1 | Estado seguro del PWM (parar, IDLE, falla, error de init) | P1 | `general.c`, `corriente.c` |
| 2 | Entrada a CONTROL: referencia y estados inicializados | P1 | `general.c`, `posicion.c`, `corriente.c`, `interpolador.c` |
| 3 | Saturar tensión y duty en el lazo de corriente | P1 | `corriente.c` |
| 4 | Rehacer la protección de sobrecorriente | P1 | `corriente.c`, `general.c`, `current_sense.c` |
| 5 | Arreglar bucle infinito en `medir_Ld` | P1 | `corriente.c` |
| 6 | Arreglar `medir_Lq` (excitación, destino, cola) | P1 | `corriente.c` |
| 7 | Reescribir la salida UART/DMA (`console_print`) | P1 | `corriente.c`, `general.c` |
| 8 | Estimador de velocidad único y filtrado | P1 | `corriente.c`, `posicion.c` |
| 9 | Rehacer el interpolador (NaN, race, saltos, fin, firma) | P1 | `interpolador.c`, `general.c` |
| 10 | Overflow de TIM5 y uso de `double` | P1 | `corriente.c` |
| 11 | Corregir `current_sense.c` | P2 | `current_sense.c` |
| 12 | Verificar el polo del IIR de corriente | P2 | `current_sense.c` |
| 13 | Encoder: cero persistente, resolución y ángulo eléctrico | P2 | `general.c`, `posicion.c`, `user_constants.h` |
| 14 | `init_sistema` y `alinear_rotor` más robustos | P2 | `general.c` |
| 15 | Verificar parámetros del motor y tipos de constantes | P2 | `user_constants.h` |
| 16 | Eliminar dependencias circulares | P3 | todos |
| 17 | Separar capas (algoritmos / drivers / control / app) | P3 | todos |
| 18 | Módulo `encoder` con API única | P3 | `posicion.c`, `mt6835.c` |
| 19 | `motor_params` como única fuente de verdad | P3 | `corriente.c`, `posicion.c` |
| 20 | Partir `user_constants.h` y `general.*` | P3 | `lib/General` |
| 21 | Máquina de estados dueña de las transiciones | P3 | `general.c`, `app_freertos.c` |
| 22 | Orquestador con flujo de datos explícito | P3 | `corriente.c`, `posicion.c` |
| 23 | Convenciones de nombres y unidades SI | P3 | todos |
| 24 | Tests en PC de la matemática pura | P3 | `lib/tests` |
| 25 | Limpieza menor | P4 | varios |

---

# P1: Bugs y seguridad

## 1. Estado seguro del PWM (parar, IDLE, falla, error de init)

**Problemas.**

- `parar_lazos()` solo deshabilita las interrupciones de TIM3 y TIM6. Los CCR quedan con el último duty calculado, así que en IDLE el motor sigue alimentado con el último vector de tensión. `calibrar_adcs()` y `medir_Ld()` asumen justamente que en IDLE no circula corriente.
- Hay tres implementaciones distintas del "apagado":
  - `parada_emergencia()` para el PWM pero deja habilitadas las interrupciones de los lazos.
  - `handle_sobrecorriente()` para el PWM y deshabilita las interrupciones, pero no avisa a nadie.
  - `lazo_corriente()` repite la lógica inline y además manda el error a la cola.
- En `init_sistema()`, si falla un `init_*` se manda el error a la cola y se retorna, pero el PWM ya iniciado sigue con su CCR.
- No está definido qué significa "seguro". TIM3 usa solo los canales 1 a 3 (sin complementarios), así que lo que hace `HAL_TIM_PWM_Stop` en el puente depende del driver: freno con low-sides encendidos o alta impedancia.
- Con tu fórmula `duty = 0.5 + v/VCC`, tensión cero es 50 %, no 0 %.

**Cómo.**

1. Crear un único `pwm_disable()` y su par `pwm_enable()`. Definir el estado seguro según el driver. Si tiene pin de enable, usarlo para dejar el puente en alta impedancia.
2. Toda salida de CONTROL (parar, parada de emergencia, falla, error de init) pasa por la misma secuencia: PWM seguro, deshabilitar interrupciones de TIM3/TIM6 y limpiar sus flags pendientes.
3. Cuando se para el lazo pero no se corta el PWM, escribir el punto neutro (`0.5 * ARR` en los tres canales) en lugar de dejar el último duty.
4. En `init_sistema()`, cualquier error llama a esa misma rutina antes de retornar.

```c
void pwm_safe_state(void) {
    // 1) puente en estado seguro (enable del driver o duty neutro)
    // 2) deshabilitar ITs de TIM3/TIM6
    // 3) limpiar flags pendientes para que no entre un lazo espurio
}
```

## 2. Entrada a CONTROL: referencia y estados inicializados

**Problemas.**

- `iniciar_lazos()` habilita las interrupciones sin inicializar nada. El interpolador arranca con `consigna_posicion = 0`, o con la última consigna del movimiento anterior. Si el rotor está en otra posición (lo moviste a mano en IDLE, o quedó tras un movimiento), al pasar a CONTROL la referencia salta a ese valor. El PID pide torque máximo y el movimiento es brusco.
- `integral`, `prev_error`, `prev_position` (en `posicion.c`) y `prev_tita_m` (en `corriente.c`) son `static` locales y no se pueden resetear. Arrastran windup de la sesión anterior y la primera muestra de `wm` da un pico.

**Cómo.**

1. Mover el estado de cada etapa a un struct con `init()` y `reset()` (`pid_t`, `current_ctrl_t`, `traj_t`, `speed_est_t`).
2. Definir una acción de entrada a CONTROL, que ejecute la máquina de estados (punto 21) con las interrupciones aún deshabilitadas:
   1. Leer la posición actual `x`.
   2. `traj_hold(x)`: la referencia queda en `x` con velocidad cero.
   3. `pid_reset(x)`, `speed_est_reset(x)`, `current_ctrl_reset()`, de modo que el primer `wm` sea cero.
   4. Limpiar flags y recién ahí habilitar PWM y las interrupciones.
3. Como las interrupciones están deshabilitadas mientras se resetea, no hace falta sincronización extra.

## 3. Saturar tensión y duty en el lazo de corriente

**Problema.** En `lazo_corriente` el duty se calcula como `0.5 + v/VCC` y se castea directo a `uint32_t`. Si `v` es lo bastante negativo, el duty es negativo y el cast es comportamiento indefinido. Si supera 1.0, el CCR queda mayor que el ARR. Además no se limita el vector `vd`/`vq`.

**Cómo.**

1. Agregar `clampf()` en un módulo de utilidades.
2. Limitar el vector de tensión antes de la transformada inversa. El radio máximo es `VCC/2` con PWM sinusoidal simple (tu fórmula actual) o `VCC/√3` si agregás inyección de modo común (min-max o tercer armónico). Priorizar `vd` y recortar `vq` con el margen restante.
3. Saturar cada duty a `[0, 1]` antes de convertirlo a CCR, idealmente en `pwm_set_duties()` para que ningún llamador pueda saltearse la protección.

```c
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void voltage_limit(float *vd, float *vq, float vmax) {
    float vd_l = clampf(*vd, -vmax, vmax);
    float vq_max = sqrtf(fmaxf(vmax * vmax - vd_l * vd_l, 0.0f));
    *vd = vd_l;
    *vq = clampf(*vq, -vq_max, vq_max);
}
```

Como el lazo de corriente no tiene integrador, no hace falta anti-windup ahí. Sí conviene si más adelante agregás uno.

## 4. Rehacer la protección de sobrecorriente

**Problemas.**

- Se evalúa sobre la corriente filtrada por el IIR, así que el disparo se retrasa.
- Es una comparación por software dentro del lazo. TIM3 no tiene entrada de break configurada, así que hoy es tu única protección, salvo que el driver tenga una falla por hardware.
- La lógica de disparo está duplicada: `lazo_corriente` la repite inline y en `general.c` existe `handle_sobrecorriente()` que hace casi lo mismo pero no avisa a la cola de errores.

**Cómo.**

1. Crear el módulo `protection` con `protection_check(const abc_t *i_raw)` que devuelve un código de falla.
2. Alimentarlo con la corriente sin filtrar, o usar un watchdog analógico del ADC (AWD) que dispare una interrupción con umbral de hardware.
3. Si el driver o el hardware lo permiten, usar la entrada BKIN de TIM3 como corte por hardware de segundo nivel.
4. Unificar la acción de disparo en `fault_trip(code)`: llama a `pwm_safe_state()` (punto 1), latchea la falla y avisa a la máquina de estados. No debe tocar la consola ni el LED directamente.
5. Borrar `handle_sobrecorriente()` y el bloque inline. El lazo solo hace `if (protection_check(&i_abc)) return;`.

## 5. Arreglar el bucle infinito en `medir_Ld`

**Problema.** `while (corrientes_fase.a > id_final);` compara contra una copia local que se leyó una sola vez, así que nunca cambia. O el bucle no termina, o sale inmediatamente si la corriente inicial fue negativa.

**Cómo.**

1. Releer `get_corrientes_fase()` en cada iteración.
2. Agregar un timeout, porque si la corriente nunca cae el sistema se cuelga.
3. Manejar el signo: comparar `fabsf(i)` contra `0.368f * fabsf(i0)`.
4. Validar el resultado (`i0` mínima, `t` dentro de un rango razonable) antes de aceptar `Ld`.
5. Usar un temporizador libre (o el DWT cycle counter) leído directo, en lugar de contar overflows.
6. Mover `medir_Ld` a un módulo `commissioning`, fuera de `corriente.c`. Ver punto 17.

```c
uint32_t t0 = timer_now();
while (fabsf(get_ia()) > i_target) {
    if (timer_elapsed(t0) > TIMEOUT_TICKS) { return ERR_TIMEOUT; }
}
uint32_t ticks = timer_elapsed(t0);
```

## 6. Arreglar `medir_Lq`

**Problemas.**

1. **Excitación equivocada.** Poner B y C en alto y A en bajo da `i_a = -I`, `i_b = i_c = I/2`. Eso sigue siendo el eje d, con polaridad opuesta. Para excitar q necesitás `i_a = 0` e `i_b = -i_c`.
2. **Destino equivocado.** El resultado se guarda en `motor.Ld` en lugar de `motor.Lq`, y no se actualiza `controlador.Pq`.
3. **Cola mal usada.** `xQueueReceive(cola_consola, NULL, ...)` con una cola de ítems no nulos es inválido y puede dar assert o hard fault. Hay que pasar un buffer.
4. Tiene el mismo bucle infinito que `medir_Ld`.

**Cómo.**

1. Con el rotor bloqueado y alineado (d sobre fase A), aplicar tensión entre B y C dejando A en el promedio de ambos duties, para que `i_a ≈ 0`.
2. Medir la caída de la corriente en fase B (o `i_b - i_c`) con la misma rutina de medición de τ que en `medir_Ld`. Conviene una única función `measure_tau()` parametrizada por qué corriente observar.
3. Guardar en `Lq` y recalcular `Pq` desde `motor_params_set_Lq()` (punto 19).
4. Pasar un buffer real a `xQueueReceive`, o mejor un helper `console_wait_enter()`.

## 7. Reescribir la salida por UART/DMA

**Problemas.**

- En `medir_Lq`, `cadena[32]` es una variable local, pero `HAL_UART_Transmit_DMA` retorna antes de terminar. Cuando el DMA lee el buffer, el stack ya no es válido.
- En `leer_posicion()` ya resolviste eso con `static char cadena[16]`, pero dos llamadas seguidas pisan el buffer mientras el DMA anterior sigue leyéndolo.
- Varios `HAL_UART_Transmit_DMA` seguidos pueden devolver `HAL_BUSY` y perder mensajes silenciosamente. Pasa en cada comando de `general.c` que imprime un mensaje.
- Los largos (8, 19, 26, 31, 41…) están hardcodeados. Hoy coinciden con los textos (los revisé), pero cualquier cambio de texto los desincroniza.

**Cómo.**

1. Crear `console_print(const char *fmt, ...)` con un ring buffer propio.
2. Una tarea (o el callback `HAL_UART_TxCpltCallback` con un semáforo) vacía el ring y lanza el siguiente DMA solo cuando el anterior terminó.
3. Calcular el largo con `strlen()`/`snprintf()`, nunca a mano.
4. Agregar `console_wait_enter()` para las pausas "ENTER para continuar".
5. Imprimir valores con `float` (por ejemplo µH), no con enteros escalados.

## 8. Estimador de velocidad único y filtrado

**Problemas.**

- Con 0.18° por cuenta (`ENCODER_PPR = 2000`), la velocidad por diferencia tiene una resolución de `0.18°/Ts` por cuenta. Con Ts = 50 µs son unos 3600°/s (~10 rev/s) por cuenta. Ese `wm` ruidoso entra directo al BEMF y al desacople del lazo de corriente.
- Se calcula dos veces, en `corriente.c` (rad/s) y en `posicion.c` (°/s), con unidades y períodos distintos.
- El término D del PID usa el `wm` ruidoso de `posicion.c`.

**Cómo.**

1. Crear un módulo `speed_estimator` que reciba el ángulo y entregue `theta_est` y `w_est`.
2. Opción simple: diferencia + filtro pasa-bajos de primer orden. Opción mejor: PLL o observador de tipo 2, que da un ángulo suave y velocidad sin retardo grande.
3. Que ambos lazos consuman la misma salida, en rad/s.
4. Si el lazo de corriente es mucho más rápido que el de posición, actualizar el estimador en el lazo rápido o pasar el último valor del lento.
5. Mejorar la resolución del encoder (punto 13) ayuda a todo esto.

```c
// PLL sobre ángulo mecánico (wrap a [-pi, pi])
float err = wrap_pi(theta_meas - est->theta);
est->w     += est->ki * err * Ts;
est->theta += (est->w + est->kp * err) * Ts;
```

## 9. Rehacer el interpolador

**Problemas.**

1. **NaN por signo.** `a_max` se multiplica por ±1 y queda modificado para el movimiento siguiente. Si el previo fue negativo y el nuevo es triangular, `sqrtf(d / a_max)` recibe un valor negativo y da NaN. Lo mismo pasa con `v_max`.
2. **Fin de trayectoria.** Cuando `t > t_total` se devuelve la última consigna calculada, no `posicion_final`.
3. **Race condition.** `consigna_nueva()` se llama desde una tarea (vía `mover()` en `general.c`) y `interpolar()` corre en una ISR. Es una estructura de varios campos, así que la ISR puede ver un estado a medio actualizar.
4. **Salto de velocidad.** Un comando nuevo a mitad de movimiento arranca desde la posición medida con velocidad cero, no desde la referencia actual.
5. **Firma inconsistente.** `mover()` llama `consigna_nueva(get_posicion(), angulo)` con 2 argumentos, y el `interpolador.c` que me pasaste define `consigna_nueva(const float posicion_final)` con 1. Uno de los dos archivos está desactualizado. Confirmá cuál es el vigente.

**Cómo.**

1. Separar los límites (`v_max`, `a_max`, `dt`, constantes) del estado del perfil (por movimiento).
2. Trabajar con magnitudes positivas y una variable `dir = ±1`. La posición es `x0 + dir * s(t)`, donde `s(t)` sale del perfil triangular o trapezoidal.
3. Al final del perfil devolver exactamente `posicion_final`.
4. Planificar en una función pura que llena un struct local y publicarlo con un mailbox de dos slots (la tarea escribe en el slot inactivo y luego cambia el índice activo; la ISR solo lee el índice). Así no hay estructuras a medio copiar.
5. Al replanificar, tomar como posición inicial la última referencia generada (`x_ref`) y, si querés suavidad total, también su velocidad. Solo usar la posición medida cuando el sistema viene de IDLE (ver punto 2).
6. La posición actual entra por parámetro, no por `get_posicion()`: `traj_plan(x_actual, x_final)`.

```c
typedef struct { float x0, x1, dir, t_acc, t_const, t_total, v_pk, a; } traj_t;
void  traj_plan(traj_t *out, float x0, float x1, float vmax, float amax);
float traj_eval(const traj_t *p, float t);   // pura, testeable en PC
```

## 10. Overflow de TIM5 y uso de `double`

**Problemas.**

- `tim5_of * 110000000` se calcula en `uint32_t`, y desborda cuando `tim5_of` supera ~39.
- `motor.Ld * 1073741824.0` usa `double` y el Cortex-M33 no tiene FPU de doble precisión, así que se hace por software. Tampoco es necesario para imprimir.

**Cómo.**

1. Acumular el tiempo en `uint64_t`, o pasar a segundos con `float` en cada término.
2. Si TIM5 es de 32 bits, medir con el contador directo y contar overflows solo si hace falta.
3. Imprimir con `float` en µH: `console_print("Ld = %.1f uH\r\n", Ld * 1e6f);`. Vos ya usás `%f` en `leer_posicion()`, así que el soporte de `printf` con float está habilitado.

---

# P2: Robustez

## 11. Corregir `current_sense.c`

**Problemas.**

- `raw_adcs` lo escribe el DMA y debería ser `volatile`.
- `HAL_ADC_ConvCpltCallback` no verifica `hadc->Instance`. Además, redefinir un callback débil de HAL dentro de una librería es frágil. Conviene que el callback viva en un solo lugar y llame a `current_sense_on_conv_complete()`.
- `ADC_0A - (sum_adc1 >> 6)` mezcla `uint32_t` con `int` y puede dar resultados inesperados.
- `adc + offset - ADC_0A` es equivalente a `adc - promedio_en_reposo`, así que basta guardar el promedio.
- `get_corrientes_fase` puede copiar `a` y `b` de muestras distintas si la interrupción lo interrumpe, y `c` queda inconsistente.
- `set_adc_offsets` usa `HAL_Delay` y muestrea con delays, lo cual está bien en arranque. Pero se llama también desde `calibrar_adcs()` en IDLE, donde hoy el PWM puede seguir con el último duty (punto 1). Con corriente circulando, el offset queda mal medido.

**Cómo.**

1. Declarar `static volatile uint32_t raw_adcs`.
2. Guardar `offset` como el promedio medido (`int32_t`/`float`) y calcular `(raw - offset) * scale`.
3. En `get_corrientes_fase`, copiar solo `a` y `b` con una sección crítica corta (o `__disable_irq` breve, o un doble buffer) y calcular `c = -(a + b)` sobre la copia.
4. Guardar también la corriente sin filtrar para la protección (punto 4).
5. Antes de calibrar, forzar el estado neutro del PWM.

## 12. Verificar el polo del IIR de corriente

**Problema.** El filtro IIR (`IIR_ALPHA`) agrega fase dentro del lazo cerrado. Si su polo no está bastante por encima de `POLO_CORRIENTE`, degrada el margen de fase y la respuesta que diseñaste con `Kp = α·L`.

**Cómo.**

1. Calcular la frecuencia de corte equivalente: `fc ≈ α / (2π·Ts)` para α chico.
2. Elegir `fc` al menos 5 a 10 veces mayor que el ancho de banda del lazo (`POLO_CORRIENTE / 2π`, unos 800 Hz con 5000 rad/s).
3. Si no se puede, incluir el retardo del filtro en el diseño (o filtrar solo la señal que usan otras funciones y dejar la del lazo sin filtrar).

## 13. Encoder: cero persistente, resolución y ángulo eléctrico

**Problemas.**

- **El cero mecánico cambia en cada arranque.** `alinear_rotor()` corre siempre y define el cero como "donde quedó el rotor con el eje d alineado con la fase A". Con `PP = 7`, ese cero cambia entre arranques en múltiplos de 360°/7 ≈ 51.4°.
- **La alineación es de lazo abierto.** Se aplica 3 % de duty en la fase A y se espera 100 ms fijos, sin verificar que el rotor se asentó. Con fricción o carga puede quedar mal alineado, y el FOC funciona entonces con el ángulo eléctrico desfasado sin avisar.
- **Resolución baja.** El MT6835 es un encoder absoluto (21 bits por SPI), pero se usa como incremental de 2000 cuentas/rev (0.18°). Un pulso perdido corrompe el ángulo eléctrico hasta el próximo reinicio.
- **Definición de `ENCODER_PPR`.** El comentario de `user_constants.h` pide que sea par porque el código lo divide por 2 al configurar el MT6835. Eso indica que el timer cuenta x2 (y en ese caso el encoder tiene 1000 PPR programados).
- `tita_e = tita_m * PP` en grados y sin wrap. Al crecer, el `float` pierde resolución.

**Cómo.**

1. Calibrar **una sola vez** el offset eléctrico (alineación d con fase A) y guardarlo en flash. Al arrancar, leer el ángulo absoluto por SPI para inicializar el contador, sin mover el rotor.
2. Separar el cero mecánico (lo define el usuario y se persiste) del cero eléctrico (propiedad del montaje motor-encoder). Renombrar `ENCODER_CNT_ZERO` según cuál sea.
3. Mejorar la resolución. Revisá en el datasheet la resolución ABZ máxima configurable del MT6835 y, si en `tim.c` el timer está en modo TI1 (x2), pasar a TI12 (x4) duplica las cuentas sin tocar el encoder. Otra opción es leer el ángulo absoluto por SPI con DMA en el lazo lento y usar ABZ solo para interpolar entre lecturas.
4. Definir constantes que no dependan de un comentario:

```c
#define ENCODER_ABZ_PPR      1000                  // lo que se programa en el MT6835
#define ENCODER_COUNT_MULT   2                     // 2 = TI1, 4 = TI12
#define ENCODER_CPR          (ENCODER_ABZ_PPR * ENCODER_COUNT_MULT)
_Static_assert(ENCODER_CPR % PP == 0, "CPR debe ser multiplo de PP");
```

5. Calcular el ángulo eléctrico desde las cuentas: `cnt_e = ((int32_t)(cnt - ZERO_ELEC) * PP) % ENCODER_CPR`, y convertirlo a radianes (o a índice de tabla/CORDIC) solo al final.
6. Evaluar usar el CORDIC del STM32L5 para `sin`/`cos` en las transformadas.

## 14. `init_sistema` y `alinear_rotor` más robustos

**Problemas.**

- `init_sistema()` repite cinco veces el mismo bloque de 4 líneas (llamar, comparar, encolar error, retornar).
- En `alinear_rotor()`, `encoder_t encoder_conf` es local: si `init_encoder` guarda ahí datos que se usan después, se pierden al retornar.
- `0.03*__HAL_TIM_GET_AUTORELOAD(...)` usa un literal `double` (sin `f`), lo que fuerza aritmética de doble precisión por software. Pasa igual en `corriente.c`.
- `init_adcs()` valida con `raw_adcs == 0` en un único instante, que es una comprobación débil. Es mejor verificar que los promedios de reposo estén cerca de mitad de escala (`ADC_0A ± tolerancia`), lo que detecta un sensor desconectado o un ADC trabado.
- Las funciones `init_*` y `alinear_rotor` están declaradas públicas en `general.h`, pero solo las usa `init_sistema()`.

**Cómo.**

1. Reducir el flujo a una tabla de pasos o a una secuencia con `goto fail`, con un único punto de salida que llama a `fault_trip(codigo)` (punto 4).
2. Dejar la configuración del encoder dentro del módulo `encoder` (punto 18).
3. Reescribir la alineación como una rutina de commissioning con rampa de duty y espera hasta que el ángulo se estabilice (variación menor a un umbral durante N ms). Además debería ejecutarse solo cuando hace falta (punto 13).
4. Usar sufijo `f` en todos los literales de coma flotante.
5. Marcar como `static` las funciones que solo se usan dentro del archivo.

## 15. Verificar parámetros del motor y tipos de constantes

**Problemas.**

- Los comentarios de `user_constants.h` no coinciden con los valores: `LQ 0.0006` dice "0.3 mH" y `LD 0.0005` dice "0.2 mH", pero los valores son 0.6 mH y 0.5 mH. Hay que decidir cuál es el correcto.
- `LAMBDA 0.0001f` da `Kt = 1.5·7·0.0001 ≈ 1.05 mN·m/A`. Confirmá que ese sea el valor real, porque el BEMF y la conversión torque a corriente dependen directo de él.
- Tipos mezclados: `R_FASE`, `LQ` y `LD` son `double` (sin `f`) mientras que `LAMBDA` es `float`. Las constantes de compilación no cuestan nada, pero se cuelan en cuentas en tiempo de ejecución.
- Los nombres `PP`, `LD`, `LQ` son muy cortos y pueden colisionar con macros de HAL/CMSIS.

**Cómo.**

1. Verificar los valores contra la hoja de datos o contra las mediciones de `medir_Ld`/`medir_Lq` (una vez corregidos), y corregir los comentarios.
2. Agregar `f` a todos los literales y prefijar con `MOTOR_` (`MOTOR_PP`, `MOTOR_RS`, `MOTOR_LD`, `MOTOR_LQ`, `MOTOR_LAMBDA`).
3. Dejarlos como valores por defecto de `motor_params` (punto 19).

---

# P3: Arquitectura

## 16. Eliminar dependencias circulares

**Problemas.**

- `posicion.c` → `interpolador` (`interpolar()`) y `interpolador.c` → `posicion` (`get_posicion()`).
- `corriente.c` ↔ `posicion.c` (`get_posicion`, `get_consigna_torque`).
- `general.c` incluye casi todo (`adc`, `usart`, `tim`, `spi`, `mt6835`, `posicion`, `current_sense`, `interpolador`), y `corriente.c` depende de globales de `general` (`estado`, colas).

**Cómo.**

1. Regla: las dependencias van en un solo sentido (app → control → drivers → HAL) y los módulos de algoritmos no dependen de nadie.
2. El interpolador recibe la posición por parámetro (`traj_plan(x_actual, x_final)`) y no incluye `posicion.h`.
3. Los lazos no se llaman entre sí: el orquestador (punto 22) pasa los datos de uno al otro.
4. Los lazos no conocen la máquina de estados ni las colas: reciben una señal simple (`enabled`) o son habilitados y deshabilitados desde afuera.

## 17. Separar capas

**Problemas.**

- `corriente.c` mezcla matemática FOC, escritura de CCR de TIM3, protección (cola RTOS + LED), consola UART y medición de parámetros.
- `posicion.c` mezcla el PID con lectura de TIM2.
- `general.*` mezcla cuatro responsabilidades distintas (ver mapeo abajo).

**Cómo.** Dividir por responsabilidad:

```
lib/
  algo/                (C puro, sin HAL, testeable en PC)
    clarke_park/  pid/  trajectory/  utils/ (clampf, wrap_angle, filtros)
  drivers/
    pwm/  current_sense/  encoder/  (mt6835 detrás de encoder)
  control/
    current_loop/  position_loop/  motor_params/  protection/
  app/
    state_machine/  console/  commissioning/  system_init/
  config/
    hw_config.h  motor_config.h  control_config.h
test/
```

Dónde va cada cosa de `general.*`:

| Hoy en `general.*` | Destino |
|---|---|
| `init_pwm`, `init_adcs`, `init_timer_*`, `init_sistema` | `app/system_init` (con `init_*` como `static`) |
| `alinear_rotor` | `app/commissioning` |
| `estados_e` y la lógica de transiciones | `app/state_machine` |
| `iniciar_lazos`, `parar_lazos`, `parada_emergencia`, `mover`, `leer_posicion`, `calibrar_adcs`, `estado_sistema` | `app/console` (solo parsean y mandan eventos, ver punto 21) |
| `error_codes_e`, `handle_sobrecorriente` | `control/protection` (`fault_trip`) |
| `medir_Ld`, `medir_Lq` (hoy en `corriente.c`) | `app/commissioning` |

Migración sugerida, sin romper todo de una vez:

1. Mover primero lo puro (`clark_park`, `utils`, `trajectory`). No tiene dependencias, es fácil.
2. Extraer `pwm` (con `pwm_set_duties` y `pwm_safe_state`).
3. Extraer `protection` y `commissioning`.
4. Dejar `corriente.c` y `posicion.c` como `current_loop`/`position_loop`, ya limpios.

Con esto el lazo queda así:

```c
current_sense_get(&i_abc);
if (protection_check(&i_abc)) return;
current_ctrl_step(&ctrl, &i_abc, theta_e, w_est, torque_ref, &duty);   // puro
pwm_set_duties(&duty);                                                  // clamp adentro
```

## 18. Módulo `encoder` con API única

**Problema.** `get_posicion()` vive en `posicion.c`, lee TIM2 directo, y la configuración del MT6835 vive en `general.c` (`alinear_rotor`), sin relación clara entre ambos.

**Cómo.**

1. Crear `encoder.h` con `encoder_init()`, `encoder_get_counts()`, `encoder_get_angle_mech()`, `encoder_get_angle_elec()`.
2. Implementar el backend seleccionable (TIM2 ABZ o MT6835 por SPI) detrás de esa API. La configuración del MT6835 y su `encoder_t` quedan dentro del módulo.
3. Que el resto del código no incluya `tim.h` ni `mt6835.h`.
4. Poner los ceros (mecánico y eléctrico) como parte de este módulo y persistirlos (punto 13).

## 19. `motor_params` como única fuente de verdad

**Problemas.**

- `Kt` está calculado dos veces: `inv_Kt` en `corriente.c` y `KT` en `posicion.c`.
- Hay macros en `user_constants.h`, un `motor_specs_t` mutable y `Pq`/`Pd` derivados que se recalculan solo a veces (por ejemplo, `medir_Lq` no actualiza `Pq`).

**Cómo.**

1. Un módulo `motor_params` con un struct (`Rs`, `Ld`, `Lq`, `lambda`, `PP`) y funciones `set_*`.
2. Cada `set_*` recalcula lo derivado (`Kt`, `inv_Kt`, `Pd`, `Pq`) en un solo lugar.
3. Los valores de `user_constants.h` quedan solo como valores por defecto.
4. Los lazos leen del módulo, nunca calculan `Kt` por su cuenta.

## 20. Partir `user_constants.h` y `general.*`

**Hallazgos.**

- `user_constants.h` no concentra todas las constantes. Solo tiene el encoder, el polo de corriente, las ganancias PID y los parámetros del motor. Otras constantes que usa el código (`VCC`, `DT`, `DTi`, `I_LIM`, `I_MAX`, `V_MAX`, `A_MAX`, `ADC_0A`, `ADC_3A`, `IIR_ALPHA`, `ENCODER_CNT_ZERO`, `DEG2RAD`) están en otros headers que todavía no vi. El problema es entonces la dispersión, no solo la mezcla.
- `general.h` no es el hub de globales. Solo declara los enums y las funciones. Pero `general.c` y `corriente.c` usan `estado`, `cola_errores`, `cola_consola`, `enviar_cola_estados`, `uart_receive_dma`, `mensaje_cli_t` y `tim5_of`, que se declaran en otro lado (probablemente `main.h` o `app_freertos.h`). Ahí está el hub real.

**Cómo.**

1. Dividir en `hw_config.h` (pines, timers, ADC, encoder, VCC, escala de corriente), `motor_config.h` (R, L, λ, PP) y `control_config.h` (polos, ganancias, límites, períodos, IIR).
2. Sacar las colas, el estado y los códigos de error a módulos con dueño claro (`state_machine`, `console`, `protection`), con funciones de acceso en lugar de `extern`.
3. Eliminar `general` cuando quede vacío (ver el mapeo del punto 17).

## 21. Máquina de estados dueña de las transiciones

**Problemas.**

- Los comandos de consola actúan directo sobre periféricos desde la tarea de consola: `iniciar_lazos()` habilita interrupciones de timers, `parada_emergencia()` para el PWM, `calibrar_adcs()` toca los ADC.
- Cada comando consulta `estado` (una global sin protección) antes de actuar. Entre el chequeo y la acción puede llegar una falla o un segundo comando, y como `estado` solo cambia cuando la cola de estados se procesa, dos comandos seguidos ven el mismo estado.
- El código de "comando no permitido en este estado" se repite en cada función con su `switch`/`if` y su mensaje.
- No está claro qué diferencia a `PARADA` de `FALLA`.

**Cómo.**

1. Los comandos de consola solo parsean y publican eventos en una cola: `EV_START`, `EV_STOP`, `EV_ESTOP`, `EV_MOVE(x)`, `EV_CALIBRATE_ADC`, `EV_FAULT(code)`.
2. Una única tarea `fsm` consume los eventos, es la única que cambia `estado` y ejecuta las acciones de entrada y salida de cada estado (puntos 1 y 2).
3. Una tabla estado × evento define qué se permite, y el rechazo ("Reiniciar sistema", "Iniciar lazos de control"…) se genera en un solo lugar.
4. Definir la diferencia entre `PARADA` (detención pedida por el usuario, recuperable) y `FALLA` (latcheada, requiere reinicio o confirmación).
5. Exponer el estado con `fsm_state()` en lugar de `extern`.

| Estado | START | STOP | ESTOP | MOVE | CAL_ADC | FAULT |
|---|---|---|---|---|---|---|
| IDLE | → CONTROL | ya detenido | → PARADA | rechazar | ejecutar | → FALLA |
| CONTROL | ya iniciado | → IDLE | → PARADA | ejecutar | rechazar | → FALLA |
| PARADA / FALLA | rechazar | rechazar | ignorar | rechazar | rechazar | ignorar |

## 22. Orquestador con flujo de datos explícito

**Problema.** Hoy `lazo_corriente` "tira" de `get_posicion()` y `get_consigna_torque()`, y `lazo_posicion` de `interpolar()`. El orden y la temporización son implícitos y difíciles de razonar. Además, no sé todavía en qué orden se ejecutan el callback del ADC (que actualiza la corriente filtrada) y la interrupción de TIM3 (que corre el lazo de corriente).

**Cómo.**

1. Un módulo `control_task` (o el handler de la ISR) que llame a cada etapa en orden y pase las entradas y salidas explícitamente.
2. Cada etapa es una función `step(&estado, &entradas, &salidas)`.
3. Documentar en un solo lugar la frecuencia y la prioridad de cada lazo (lazo de corriente en la ISR de TIM3, posición en TIM6).
4. Si los lazos corren a distinta frecuencia, el orquestador maneja el submuestreo y el dato compartido.

## 23. Convenciones de nombres y unidades SI

**Problemas.**

- Mezcla de idiomas: `corriente`/`posicion`/`estado` frente a `current_sense`/`clark_park`/`init_*`.
- Carpetas con mayúsculas inconsistentes (`Control`, `General`, `Current_sense`, `Interpolador` frente a `clark_park`).
- Nombres confusos: `DT` frente a `DTi`.
- Unidades mezcladas: grados y °/s en `posicion.c`, rad/s en `corriente.c`.

**Cómo.**

1. Elegir un idioma para el código (yo iría con inglés para identificadores y español en comentarios si querés).
2. Todo en minúsculas para carpetas y archivos.
3. Usar SI internamente (rad, rad/s, N·m, A, V, s) y convertir solo en la interfaz de usuario y la consola.
4. Sufijos en constantes que lo necesiten: `TS_CURRENT_S`, `TS_POSITION_S`, en lugar de `DT`/`DTi`.
5. Poner la unidad en el nombre cuando no sea SI (`angle_deg`, `speed_rpm`).

**Ojo con las ganancias del PID.** Hoy `PID_P`, `PID_I` y `PID_D` están expresadas para posición en grados. Al pasar a radianes hay que multiplicarlas por 180/π ≈ 57.3 (las tres, porque el error, su integral y la velocidad escalan igual). Si no, el lazo queda con una ganancia 57 veces menor. Conviene hacer el cambio con un test que verifique la equivalencia.

## 24. Tests en PC de la matemática pura

**Cómo.**

1. Una vez separados los módulos de `algo/`, compilarlos en PC sin HAL.
2. Con PlatformIO: entorno `native` y carpeta `test/` con Unity.
3. Casos útiles: Clarke/Park e inversas (ida y vuelta), perfil trapezoidal y triangular (posición final exacta, velocidad y aceleración acotadas, movimientos en ambos sentidos), PID (saturación y anti-windup), estimador de velocidad (rampa y escalón), y la máquina de estados (tabla del punto 21).
4. Sumar un test de "motor simulado" (modelo dq de primer orden) para validar el lazo de corriente antes de probar en hardware.

---

# P4: Limpieza

## 25. Limpieza menor

- Usar `clampf()` para simplificar el anti-windup y las saturaciones en `posicion.c`.
- Borrar el comentario obsoleto sobre el `0.18f` hardcodeado en `get_posicion()` y el "Pd = 5000 * Ld" en `medir_Ld`.
- Renombrar `distancia_acc` en `interpolador.c`. Es `v²/a`, es decir, la distancia de aceleración más la de frenado.
- Quitar los `volatile` sobre todo el struct del controlador de `posicion.c`. No sincronizan nada, y solo deberían ir en lo que realmente comparten una ISR y una tarea.
- Reemplazar números mágicos (`0.03f`, `0.368f`, `110000000`, `100` de timeout de cola) por constantes con nombre (`COMMISSIONING_DUTY`, `TAU_FRACTION`, `TIM5_CLK_HZ`, `INIT_QUEUE_TIMEOUT_MS`).
- Reemplazar `while (cond);` sin timeout por funciones con salida por tiempo.
- `estado_sistema()` hoy solo imprime "Estado". Que imprima el estado real, los errores latcheados y algunas variables útiles (posición, corrientes).
- `leer_posicion()` imprime el número sin terminador de línea. Si es intencional para un protocolo, documentarlo; si no, agregar `\r\n`.

---

# Orden de trabajo sugerido

| Fase | Contenido | Puntos |
|------|-----------|--------|
| 1 | Bugs de seguridad y de funcionamiento, sin mover archivos | 1, 2, 3, 4, 5, 6, 7, 10 |
| 2 | Interpolador puro + estimador de velocidad | 9, 8 |
| 3 | Robustez del sensado, encoder e inicialización | 11, 12, 13, 14, 15 |
| 4 | Reorganización de carpetas, dependencias y máquina de estados | 16, 17, 18, 19, 20, 21, 22 |
| 5 | Convenciones y tests | 23, 24, 25 |

Hacer la fase 1 antes de reorganizar reduce el riesgo: los bugs se ven mejor en el código que ya conocés, y después se mueven ya corregidos. Los puntos 1 y 2 se pueden implementar dentro de `general.c` con un `pwm_safe_state()` y una función `control_enter()`, y más adelante se mudan a la máquina de estados.

## Información pendiente

- `corriente.h`, `posicion.h`, `interpolador.h`, `current_sense.h`: para ubicar las constantes dispersas (`VCC`, `DT`, `DTi`, `I_LIM`, `I_MAX`, `V_MAX`, `A_MAX`, `ADC_0A`, `IIR_ALPHA`, `ENCODER_CNT_ZERO`).
- `main.h` y `app_freertos.c/.h`: para ver dónde viven `estado`, las colas, `enviar_cola_estados`, `mensaje_cli_t` y `tim5_of`, y qué hacen las tareas de estados, fallas y consola.
- `main.c` y `stm32l5xx_it.c`: quién llama a cada lazo, con qué prioridades y a qué frecuencia.
- `tim.c` y `adc.c`: configuración de TIM3 (frecuencia, si hay break), modo del encoder en TIM2 (TI1 o TI12) y trigger del ADC.
- `mt6835.c/.h`: `init_encoder`, resolución ABZ configurable y si el cero se escribe en EEPROM o en registro.
- `clark_park.c`: unidades y costo de `sinf`/`cosf`.
- `tests.c`, y si usás PlatformIO o CubeIDE.
- Hardware: modelo del driver de puente (si tiene pin de enable, entrada de falla y qué hace con las entradas PWM en bajo), para definir el estado seguro del punto 1.
