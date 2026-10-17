// La tabla "Todos los parámetros" del capítulo 9, como código.
//
// Cada campo lleva al lado de dónde sale. Si un valor no tiene procedencia, no va acá:
// va en supuestos.toml marcado como tal, que es donde se ve la deuda de fidelidad.
//
// Se carga desde parametros.toml. Nada de esto se recompila para afinar.
#pragma once

#include <string>

namespace estab {

// Las tres formas de tratar el cuadro que no llena el presupuesto. Se eligen por
// parametro para que las cuatro corran contra el mismo escenario y la misma semilla.
enum class Variante {
  hoy,        // como estaba: el lazo no se entera y se autoengana
  remiendo,   // detectar que no se usa el presupuesto y dejar de tantear (BBR, WebRTC)
  relleno,    // completar con paquetes dummy para que el lazo vea un flujo saturado (Vidaptive)
  tren,       // medir la dispersion del tren de paquetes de cada cuadro (SQP)
};

const char* nombre(Variante v);
Variante variante_por_nombre(const std::string& s);

struct Parametros {
  Variante variante = Variante::hoy;

  // ─── el cuadro que no llena el presupuesto ───────────────────────────────
  double app_umbral = 0.70;      // por debajo de esta fraccion del presupuesto, el
                                 // codificador no esta usando lo que tiene: la muestra
                                 // no es evidencia de nada (WebRTC usa 0,65)
  double tren_pacing = 2.0;      // kappa: cada cuadro se manda a 2x la estimacion (SQP)
  double tren_destino = 0.90;    // eta: se usa el 90 % de lo medido. El 10 % que sobra
                                 // drena las colas solo, sin sondeo y sin transitorio
  // Los cuadros sobre los que se ajusta el ancho por dispersión. El ancho sale de la
  // pendiente del lapso contra el tamaño, así que la ventana tiene que juntar cuadros de
  // tamaños distintos: dos segundos cubren un cuadro clave y unos cien P.
  double tren_ventana = 2.0;     // segundos
  // Cuántas veces se vuelve a ajustar la recta con los trenes que quedaron por debajo: la demora
  // que agrega el camino sólo alarga el lapso, y el caño es el borde de abajo. 0, la recta común.
  int    tren_envolvente = 2;
  // Cuánto del presupuesto calculado se usa de verdad. Por debajo de 1 queda holgura
  // permanente, y con holgura las colas se drenan solas — que es la alternativa de SQP
  // al sondeo periódico del piso. El canje entre los dos se mide en su propio ensayo.
  double destino = 1.0;
  // Por debajo de esto un cuadro no tiene dispersión. Los de un menú, de uno o dos paquetes, no la
  // tienen ni completados con paridad: el encabezado admite a lo sumo tres de paridad para un
  // cuadro de un paquete, y un tren de cuatro entra entero en la ráfaga que un router deja pasar
  // ocioso (15 KB en el del banco): llega junto y no muestra el caño.
  double tren_min_paquetes = 6;
                                 // medible: se lee el ruido del reloj, no el cano
  // Cuánto puede subir el ancho medido por segundo, como fracción de sí mismo. Baja sin
  // límite. 0 no limita.
  double tren_subida_max = 0.10;
  // Cuánto dura el ancho medido sin muestras nuevas antes de dejar de ser techo, en segundos.
  double tren_vigencia = 3.0;
  // Con la fila vacía, el ancla de la curva de subida es al menos el destino del tren, hasta
  // este múltiplo del presupuesto. 0 no lo usa.
  double tren_ancla = 1.5;
  // Y sólo tras estos segundos sin fila.
  double tren_ancla_espera = 6.0;

  // ─── el codificador que entrega menos de lo que se le pide ─────────────
  // Cuánto más que la imagen se le puede llegar a pedir, y en cuánto tiempo se aprende lo que
  // le falta. 1 apaga la compensación.
  // 1,6: con el techo del tren el presupuesto ya no se pasa del caño, y libx264 entrega alrededor
  // de un 70 % de lo que se le pide; con 1,3 la compensación queda clavada en su tope.
  double codificador_compensacion_max = 1.6;
  double codificador_constante = 2.0;   // segundos
  // Cuántos pasos seguidos con la fila sobre el objetivo hacen falta para soltarla (por encima del
  // doble se suelta en el acto). 1 la suelta con el primero.
  int    codificador_suelta_ciclos = 3;
  // ─── el ritmo del lazo ───────────────────────────────────────────────────
  double Tc = 0.100;                  // cap. 9: 100 ms en red doméstica. A 300 ms la
                                      // retención sola aporta 150 ms de tiempo perdido
  double fps = 60.0;

  // ─── el filtro del crecimiento ───────────────────────────────────────────
  // Cap. 9: NO es fijo. El mejor valor se corre a la izquierda cuando sube el viaje,
  // y un valor fijo es malo en los dos extremos.
  double filtro_lento = 0.250;        // con viaje corto
  double filtro_rapido = 0.150;       // con viaje largo
  double filtro_rtt_corte = 0.100;    // dónde se cruza
  int    muestras_para_recta = 3;     // cap. 6: el ajuste gana recién con ~3

  // ─── el objetivo de fila ─────────────────────────────────────────────────
  // Cap. 3: con 10 ms fijos y 250 ms de viaje, la fila se vacía el 54 % del tiempo y
  // el lazo se queda ciego. Por eso el objetivo se ata al camino, no al lazo.
  double dref_piso = 0.010;
  double dref_fraccion = 0.25;        // sobre (viaje + cuadro)
  // Y nunca bajo el ruido del camino (Medicion::ruido) por este factor: en un Wi-Fi o por internet
  // la fila sube y vuelve sola decenas de milisegundos, y un objetivo de 10 ms la leía como cola.
  double dref_ruido = 1.5;
  double ruido_ventana = 0.35;      // s: la envolvente de abajo de la fila, con ruido
  double filtro_ruido = 0.75;       // s: la ventana del ajuste de la fila en un camino con ruido
  // Una cola que se vació en menos de esto, con un caño que entrega lo que se mandaba antes de
  // ella, fue un corte del aire y no un caño más angosto: el presupuesto vuelve a lo de antes.
  // 0 la desactiva, y está apagada: el banco no la distinguió del ruido entre corridas. Con 1,5 s,
  // por SUNSHINE_ESTAB_PARAMS, para probarla en un camino real.
  double vaciado_episodio = 0;      // s
  // Cuánto tiene que subir la fila por segundo para que la cola sea la de un corte del aire y no la
  // del propio lazo
  double vaciado_rampa = 0.5;       // s de fila por s
  // En cuánto tiene que poder vaciarse la cola de un corte como el último: fija qué parte de ese
  // caño se usa. Los cortes de un Wi-Fi lejano vienen cada un par de segundos.
  double vaciado_drenaje = 0.5;     // s
  int    percentil_viaje_dref = 5;
  int    tau_del_piso = 0;            // si τ̂ sale del piso del viaje en vez del percentil_viaje    // el percentil del ida y vuelta con que se mide el viaje

  // ─── las ventanas de estimación ──────────────────────────────────────────
  double ventana_piso = 10.0;         // cap. 6: mismo valor que BBR, mismas razones
  double ventana_capacidad = 1.5;
  int    percentil_capacidad = 50;    // la mediana: la entrega de los mismos cuadros ya no está sesgada
  int    percentil_viaje = 80;        // cap. 5 y 3: las ganancias se mueven mucho más
                                      // despacio que el lazo, o la resintonía desestabiliza
  double constante_capacidad = 0.30;  // suavizado de Ĉ

  // Cuánto puede quedarse en el camino antes de que se considere que NO está saliendo todo
  // lo que se pone. Es la puerta de la única señal que no depende del piso del retardo, así
  // que no puede ser ni tan chica que la abra el ruido de medición ni tan grande que la
  // tape una pérdida real.
  double margen_egreso = 0.05;
  // La cola ajena: un episodio de fila en el que, durante al menos ajena_s, la fila se quedó alta
  // y quieta mientras llegaba lo mismo que se mandaba —ni se llenaba por lo nuestro ni se vaciaba
  // al bajar—. Si termina solo antes de ajena_max_s, el caño no cambió: se vuelve al presupuesto
  // y a la capacidad de antes, y la calma se explora sin esperar reexplorar_s. 0 lo apaga.
  double ajena_s = 0.5;
  double ajena_max_s = 4.0;
  double ajena_tolerancia = 0.15;     // cuánto pueden diferir lo que llega y lo que se mandó
  // Con la entrega de unos mismos cuadros (Medicion::entrega), el caño está lleno si llegan más
  // lento que esto respecto de cómo salieron. 0 no la usa y compara las tasas.
  double entrega_llena = 0.93;
  // Si el techo demostrado desenrolla la curva de subida al recortar.
  int    techo_no_enrolla = 1;
  // Si la fila también sale de la edad del último cuadro bueno (el medidor de GameStream).
  int    fila_edad = 1;
  // Cuánto tiene que quedar el mínimo de la ventana del piso por encima del de toda la sesión para
  // tomarlo como la cola de otro flujo, que no se vacía (0 lo apaga), y qué percentil de la ventana
  // pasa a ser el piso en ese caso.
  double competidor_umbral = 0.05;
  int    competidor_percentil = 25;
  double competidor_sostenido = 8.0;
  double competidor_dref = 0.0;
  double competidor_perdida_s = 10.0;
  int    capacidad_sostenida = 0;
  double codificador_suelta_perdida = 0.0;
  double edad_cadencia = 2.5;
  // De cuánto tiempo atrás es el mínimo que cuenta como la edad de ahora, en segundos: el resumen
  // sale cada 50 ms y el último cuadro bueno puede llevar hasta un cuadro esperando, así que el
  // menor de varios es el que menos suma de eso.
  double edad_reciente = 0.15;
  // Segundos con los que se suavizan lo emitido y lo recibido antes de compararlos. Un
  // codificador real entrega cuadros de tamaño muy desparejo, y en una ventana de un cuarto de
  // segundo lo que llega queda un 5 % por debajo de lo que salió uno de cada catorce pasos sin
  // que haya cola ni pérdida. Un caño que se angosta sostiene la diferencia; el ruido, no.
  double egreso_constante = 0.5;

  // Cuánto se puede autorizar por encima de la capacidad que se pudo DEMOSTRAR. No es un
  // objetivo de operación —en régimen R y Ĉ quedan pegados— sino el margen que le queda al
  // lazo para tantear caño nuevo. Si fuera 1,0 exacto, R y Ĉ se realimentarían y el lazo no
  // podría descubrir que el caño creció.
  double techo_sobre_C = 1.1;

  // ─── el chequeo de coherencia (cap. 5) ───────────────────────────────────
  double coh_bajada_minima = 0.005;   // por ciclo. Por debajo el modelo no afirma nada
  double coh_clavada = 0.002;         // ±2 ms por ciclo: ni baja ni sigue creciendo
  double coh_margen = 0.015;          // +15 ms sobre el objetivo. Sin esto dispara
                                      // en el punto de equilibrio normal
  int    coh_ciclos = 3;              // se actúa primero y se deshace después

  // ─── la mezcla y los márgenes (cap. 4) ───────────────────────────────────
  double rho = 5.0;                   // 5 a 1. Pasando de ahí casi no se gana adelanto
  double margen_fase = 0.7853981634;  // 45°, en radianes

  // ─── el salto por pérdida (cap. 5) ───────────────────────────────────────
  double salto_factor = 0.85;         // sobre la tasa recibida medida. Tomado de GCC
  double salto_perdida = 0.05;        // pérdida creíble para dispararlo
  double salto_margen_fila = 0.020;   // y la fila bien por encima del objetivo.
  // El salto rápido: si llega esta fracción menos de lo que se emitió un viaje antes, salta
  // a la tasa recibida sin esperar a la fila. 0 lo apaga.
  double salto_rapido = 0.3;
  // También salta si los cuadros de un informe —al menos tantos como esto— cruzaron un caño
  // esa fracción más angosto que el presupuesto. 0 no mira los cuadros.
  int    salto_trenes = 2;
  // La ley no baja el presupuesto por debajo de esta fracción de lo que el caño entrega. 0 no
  // tiene piso.
  double piso_recibida = 0.7;
  // Y sólo mientras la fila esté por debajo de este múltiplo del objetivo.
  double salto_rapido_fila = 2;
  // Y sólo si lo que llega quedó corto en tantos ciclos seguidos.
  int    salto_rapido_ciclos = 2;
  // La pérdida por encima de esto, sostenida tantos segundos sin que se vea fila, es la cola de
  // un camino que el retardo no ve; y por tantos segundos después la subida no acelera. 0 apaga.
  double perdida_congestion = 0.06;
  double perdida_congestion_s = 0.5;
  double ciego_s = 30.0;
  // Si la muestra cruda de la fila sube más que esto en un paso, y ya pasa el doble del
  // objetivo, se salta a esta fracción de lo que llega. 0 apaga.
  double salto_escalon = 0.02;
  double salto_escalon_factor = 0.6;
  // La alarma entre pasos (Control::alarma): la fila tiene que crecer más que esto en 50 ms y pasar
  // de esto (o de 1,3 veces el objetivo); el destino es alarma_factor del caño menos lo que vacía
  // la cola en alarma_vaciado segundos. alarma_escalon 0 la apaga.
  // 15 ms y no 20: una caída de 12 a 4 Mbit/s con Sunshine de verdad hace crecer la fila unos 20 ms
  // por intervalo, justo en el borde; si no dispara, la cola llega a 220 ms y la invalidación pide
  // un cuadro de veinte atrás, fuera de las dieciséis referencias: un cuadro clave.
  double alarma_escalon = 0.015;
  double alarma_fila = 0.025;
  // La fila de antes del escalón no puede pasar de esto: una descarga arma una cola de cientos de
  // milisegundos que crece y baja a su ritmo, y eso no es un caño que se angosta. Y la propia, con
  // el caño lleno, anda entre 30 y 50 ms: con 60, "vacía hace poco" pide bajar de 30 y la alarma
  // queda muda justo en la caída desde un caño lleno, que es la de siempre.
  double alarma_fila_previa = 0.080;
  // En cuántos intervalos seguidos entre muestras tiene que crecer la fila: con dos se saca el ruido
  // de acuses que saltan, con uno se gana una muestra —un ida y vuelta más la cola—, y en una caída
  // de 12 a 4 Mbit/s la cola del router desborda a los cien milisegundos. Con 0, el crecimiento es
  // el de una ventana fija de 50 ms, que con pings seguidos junta varias muestras.
  int    alarma_intervalos = 2;
  // De cuánto tiempo atrás sale la fila de antes del escalón: su mínimo. 0 es la de dos muestras atrás.
  double alarma_previa_s = 0.25;
  // Cuánto después de la alarma una fila de vuelta en su piso la desmiente (0 nunca). Un acuse
  // demorado sube la muestra de ENet de un golpe y la siguiente vuelve al piso; la cola de un caño
  // que se angostó tarda cientos de milisegundos en vaciarse.
  double alarma_desmentir_s = 0.25;
  double alarma_factor = 0.7;
  double alarma_vaciado = 0.2;
  // Si la alarma pide un cuadro clave en el acto: lo que salió en los últimos cien milisegundos se
  // está perdiendo, y el primer cuadro entero que llegue ya es el clave. Apagado, porque en una caída
  // que la cola absorbe no se pierde nada y el cuadro clave sobra.
  int    cuadro_clave_en_caida = 0;
  // Cuántos cuadros puede haber en vuelo —el ida y vuelta por los cuadros por segundo— antes de que
  // el host codifique menos. El cliente se entera de lo perdido un ida y vuelta más tarde, y para
  // repararlo invalidando referencias el último cuadro que tiene debe seguir entre las dieciséis del
  // codificador. 0 lo apaga. Con doce, detrás de la cola de una descarga —doscientos milisegundos,
  // quince cuadros a 60 por segundo— el host codifica la mitad y la reparación llega a tiempo; con
  // el camino libre no actúa.
  int    cuadros_en_vuelo = 12;
  // Y al disparar la alarma, uno de cada cuántos cuadros por seis décimas de segundo: lo que desbordó
  // la cola se nota recién cuando llega el primer cuadro bueno de detrás, una cola entera después.
  int    cuadros_alarma = 4;

  // ─── la curva de subida (cap. 4) ─────────────────────────────────────────
  double curva_duracion = 2.0;        // la S tímida: lenta, acelera, frena al llegar
  double curva_exploracion = 0.02;    // cuánto sigue subiendo pasado el ancla
  double curva_convexa = 0.002;       // y cuánto acelera esa subida mientras no aparece fila (1/s³)
  double arranque_rapido = 1.0;       // hasta la primera fila, la subida por segundo sobre lo que se manda
  // La exploración en crucero (sección 7a del control): cuánto multiplica por paso, qué fila cruda
  // cuenta como pegada al piso y cuántos segundos de crucero hacen falta. 0 la apaga.
  double crucero_ganancia = 0.0;
  double crucero_fila = 0.004;
  double crucero_espera = 2.0;
  // El arranque rápido otra vez después de tantos segundos sin fila: el caño pudo crecer. Con
  // Sunshine de verdad, de 4 a 12 Mbit/s, llegar baja de catorce segundos a cinco. 0 no.
  double reexplorar_s = 4.0;
  int    fila_requiere_lleno = 0;     // una fila de menos del doble del objetivo sólo cuenta con el caño lleno
  double ancla_derrumbe = 0.8;        // si la fila reaparece bien por debajo del ancla,
  double ancla_mezcla = 0.5;          // el ancla baja a la mitad del camino

  // ─── la reparación (cap. 7) ──────────────────────────────────────────────
  double fec_min = 0.05;              // especificaciones previas del proyecto
  double fec_max = 0.50;
  double fec_subida = 0.60;           // subir rápido
  double fec_bajada_tau = 5.0;        // bajar lento: ~5 s
  double fec_z = 2.0;                 // f ≈ p + z·√(p/D)
  double bloques_esperados = 30.0;    // D típico; lo pisa la planta con el real

  // ─── el sondeo del piso (cap. 5) ─────────────────────────────────────────
  double sondeo_cada = 8.0;
  double sondeo_dura = 0.35;
  double sondeo_recorte = 0.15;       // cuesta 0,66 % del caudal

  // ─── los topes ───────────────────────────────────────────────────────────
  double bitrate_min = 1e6;
  double bitrate_max = 30e6;
  // Cap. 5: arrancar abajo y subir no es simétrico. Con la ley sola, arrancar abajo dejaba
  // al lazo plantado por debajo del caño —sube tanteando y se detiene en cuanto la fila toca
  // el objetivo—, y 0,4 costaba diez puntos de uso permanentes. Con el tren como ancla de la
  // curva de subida (`tren_ancla`) la subida mide el caño en vez de tantearlo, y arrancar
  // abajo ya no cuesta uso. Arrancar arriba sí cuesta: con una descarga ya en el caño, el
  // primer segundo a 0,7 de lo negociado rompía cinco cuadros clave seguidos.
  double arranque_fraccion = 0.3;

  // ─── lo que el presupuesto paga antes de llegar a la imagen ──────────────
  // Procedencia de cada uno en supuestos.toml. Los que están marcados "inventado" ahí
  // son deuda, no decisiones.
  int    mtu = 1024;                  // carga útil por paquete
  int    cabeceras = 56;              // IP + UDP + cabecera propia
  double audio_bps = 294400.0;        // sobre el cable, con sus cabeceras
  int    gf_shards = 255;             // GF(256): un bloque no pasa de 255 fragmentos
  int    bloques_por_cuadro = 4;
  // Moonlight le pide al host un mínimo de DOS paquetes de paridad por bloque, y lo
  // dice con todas las letras en su código: "si un cuadro tiene menos fragmentos de
  // datos de los que generarían 2 de paridad, se sube el porcentaje para ese cuadro
  // por encima del valor pedido (llegando hasta 200 %)".
  // Con cuadros chicos ese mínimo, y no el porcentaje, es el que manda.
  int    fec_minimo_paquetes = 2;
  // El mínimo mientras no hubo pérdida en los últimos fec_limpio_s segundos (−1: siempre el de
  // arriba). El cliente arma cada bloque con la paridad que dice el porcentaje de sus paquetes, sea
  // cual sea; a 4 Mbit/s un cuadro tiene cinco paquetes, y dos de paridad son el 40 % del cable.
  // Sin pérdida no se manda paridad: si aparece, el cuadro perdido se repara invalidando
  // referencias, y la primera pérdida o la alarma devuelven el mínimo del cliente.
  int    fec_minimo_limpio = 0;
  double fec_limpio_s = 10.0;

  // Cómo se pasa de la decisión de reparación a paquetes de paridad.
  //   0  porcentaje: techo(k·f) con el mínimo de arriba, que es lo que hace Sunshine. Con
  //      cuadros chicos la perilla no responde —el mínimo tapa al porcentaje— y la imagen
  //      se calcula con la paridad pedida aunque salga el doble.
  //   1  paquetes: por bloque, la menor paridad m (con el mismo mínimo) que deja la
  //      probabilidad de perder el bloque por debajo de `paridad_falla_bloque` con la
  //      pérdida estimada, y la imagen se calcula con la paridad que de verdad salió.
  int    paridad_en_paquetes = 1;
  // Uno en diez mil. En un Wi-Fi con pérdida en ráfagas, uno en mil deja unas cuatro imágenes por
  // minuto reparadas (un tirón corto cada una); uno en diez mil las baja a la mitad por unos tres
  // puntos de uso. Más estricto no ayuda: lo que queda son imágenes que pierden la mitad de sus
  // paquetes de un saque, y eso no lo cubre ninguna paridad razonable.
  double paridad_falla_bloque = 0.0001;
  // El cuadro que repara una invalidación es el único que no se puede perder: si se pierde, se
  // invalida otra vez desde él, y con dos o tres seguidas la última referencia sana queda fuera de
  // las dieciséis del codificador y sale un cuadro clave. Detrás de la cola de una descarga pasa.
  // Su bloque se protege con esta meta, y su paridad puede llegar a este múltiplo de los datos.
  double paridad_falla_reparacion = 1e-5;
  double fec_max_reparacion = 1.0;
  // Segundos en que la paridad olvida un pico de pérdida (se decide con el mayor entre la
  // pérdida media y el pico, que decae con esta constante). 0 la decide con la media.
  double paridad_memoria = 10.0;
  // Pérdida media por encima de la cual se cuenta una tanda, para saber si se repite.
  double paridad_tanda = 0.01;
  // El ida y vuelta de ENet, del lado del host: cada cuántos milisegundos el host manda su
  // ping, y si se lee la muestra de cada acuse (1) o el valor que ENet suaviza con 1/8 (0).
  double enet_ping_ms = 25;
  int    enet_rtt_crudo = 1;
  // Los primeros segundos, la paridad se decide con al menos esta pérdida.
  double paridad_arranque = 0.10;
  double paridad_arranque_s = 2.0;
  // Carga desde TOML plano. Devuelve vacío si salió bien, o el motivo del error.
  std::string cargar(const std::string& archivo);
  // Uno solo, por nombre. Devuelve false si el nombre no existe.
  bool fijar(const std::string& clave, const std::string& valor);
  std::string volcar() const;
};

// Las dos ganancias, derivadas del margen de fase y de la mezcla ρ.
// Cap. 4: no son constantes de sintonía, son consecuencia. Se recalculan cada ciclo.
struct Sintonia {
  double kg_n = 0;            // k_g = kg_n · Ĉ / τ̂
  double kp_n = 0;            // k_p = kp_n · Ĉ / τ̂²
  double wc_tau = 0;          // ω_c·τ
  double w180_tau = 0;        // ω_180·τ
  double margen_ganancia = 0; // PREDICCIÓN: el lazo oscila acá
  double periodo_tau = 0;     // y con este período, en unidades de τ
};

Sintonia derivar(const Parametros& p);

}  // namespace estab
