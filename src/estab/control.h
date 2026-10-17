// LA TEORÍA.
//
// Entra una medición, sale una decisión. No hay red, ni hilos, ni entrada/salida, ni
// dependencias fuera de la biblioteca estándar. Eso no es purismo: es lo que permite que
// este archivo se copie a Sunshine/src/ sin editarlo, se compile a WebAssembly para el
// simulador de la presentación, y se enlace en el emisor UDP del banco — las tres cosas
// corriendo exactamente el mismo código, que es la única forma de que no se desincronicen.
//
// Todo lo numérico vive en Parametros. Acá no hay ninguna constante suelta.
#pragma once

#include <deque>
#include <vector>

#include "parametros.h"

namespace estab {

// Lo único que el controlador sabe del mundo. Todo esto llega TARDE: el reporte del
// cliente también viaja. Esa tardanza es τ, y es la que fija la velocidad del lazo.
struct Medicion {
  double t = 0;              // hora del host, segundos
  double owd_min = 0;        // menor retardo en un solo sentido del período informado
  double owd_medio = 0;      // su promedio
  double perdida = 0;        // fracción de paquetes que el cliente no vio
  double rtt = 0;            // ida y vuelta, medido con eco
  double tasa_recibida = 0;  // bits/s que el cliente contó. La que permite bajar a un
                             // valor medido en vez de a una fracción inventada
  double entrega = 0;        // de unos mismos cuadros, lo que tardaron en salir sobre lo que
                             // tardaron en llegar: menos de 1 es un caño que los frena. 0: no se sabe
  int    muestras = 0;       // cuántos paquetes respaldan lo de arriba

  // Lo que el host sabe SIN ayuda del cliente: cuántos bits puso de verdad en el cable.
  // Si eso está muy por debajo del presupuesto, el codificador no tuvo qué codificar y
  // la medición de recién no dice nada sobre la red.
  double tasa_emitida = 0;

  // La dispersión del tren de paquetes del último cuadro, para la variante SQP:
  // cuántos bytes tenía y cuánto tardó en atravesar, de su primer paquete al último.
  double tren_bytes = 0;
  double tren_lapso = 0;     // T_fin − T_ini, medido en el cliente
  double tren_lleno = 0;     // el tamaño que ese cuadro habría tenido sin undershoot
  // Todos los trenes del período, si quien arma la Medicion los tiene; si no, vale el de
  // arriba. Con uno solo por período se pierde justo el que importa: el cuadro de sondeo
  // casi nunca es el último.
  struct Tren { double bytes = 0, lapso = 0; };
  std::vector<Tren> trenes;
  // La paridad que de verdad salió, sobre los datos, en el último ciclo. El host la conoce
  // sin ayuda del cliente. Con cuadros chicos no es la pedida: el mínimo por bloque manda.
  double paridad_emitida = 0;
  // Paquetes perdidos por cuadro que perdió alguno, en el período: el largo de una ráfaga.
  // Cero si no hubo pérdida o no se sabe.
  double rafaga_perdida = 0;
  // Cuántos reportes llegaron después de que su cuadro se dio por perdido, desde el arranque.
  // Sólo para el registro: si crece, el plazo del apagón es corto para este camino.
  int reportes_tardios = 0;
  // Lo que entregó el codificador, en bits/s de datos de imagen —sin paridad ni cabeceras de
  // red—. Cero si no se sabe.
  double tasa_datos = 0;
  // Lo que la fila sube y vuelve sola en un camino real (Wi-Fi, internet), en segundos: no es cola
  // propia. Cero si no se sabe.
  double ruido = 0;
  // El caño que vació la última cola, en bits/s: lo que el host mandó entre dos ida y vuelta que
  // la encontraron llena, sobre lo que tardaron en volver. Mientras hay cola el cuello de botella
  // no para, así que eso es lo que entrega. Cero si desde la medición anterior no se vació ninguna.
  double vaciado = 0;
  double vaciado_desde = 0;  // cuándo empezó a llenarse esa cola, hora del host en segundos
  double vaciado_subida = 0; // cuánto tardó en llegar a su pico, en segundos
  double vaciado_altura = 0; // y cuánto subió hasta ahí, en segundos de fila
};

// Por qué se movió. Sin esto, depurar el lazo es adivinar (cap. 9).
enum class Motivo {
  arranque,
  quieto,           // en el objetivo, sin novedad
  sin_evidencia,    // el codificador no llenó el presupuesto: la muestra no dice nada
  fila_crece,       // el término de crecimiento mandó
  fila_alta,        // el corrector de nivel
  salto_perdida,    // desborde: pérdida alta con la fila llena
  tanteando,        // subiendo por la curva en S
  reanclado,        // bajar no mejoró: no era una fila
  sondeando,        // recorte deliberado para releer el piso
  topeado,
};

const char* nombre(Motivo m);

// La decisión, más todo lo que el capítulo 9 manda registrar. Los campos de auditoría no
// son opcionales: la resintonía es lo primero que se rompe, y sin verla no se nota.
struct Decision {
  double presupuesto = 0;    // bits/s al cable, entero
  double imagen = 0;         // de eso, lo que queda para la imagen
  // Lo que hay que pedirle al codificador para que entregue `imagen`. Es la imagen dividida
  // por lo que el codificador viene entregando de lo que se le pide (ver
  // `codificador_compensacion_max`); sin compensación, igual a `imagen`.
  double pedido_codificador = 0;
  double reparacion = 0;     // fracción de paridad
  Motivo motivo = Motivo::arranque;
  bool   app_limited = false;   // el codificador no esta usando el presupuesto
  double ancho_tren = 0;        // la estimacion por dispersion, si la variante la usa

  double fila = 0;           // lo que el controlador cree que hay de cola
  double piso = 0;           // la referencia con la que lo calculó
  double dref = 0;           // y el objetivo vigente
  double crecimiento = 0;    // la derivada por ciclo, ya filtrada
  double zona_muerta = 0;    // la vara con la que se mide el nivel
  double C_est = 0;          // capacidad estimada
  double tau_est = 0;        // tiempo perdido estimado
  double kg = 0, kp = 0;     // las ganancias que salieron de las dos de arriba
  double perdida_est = 0;
  double rafaga_perdida = 1;   // paquetes que se lleva cada tropiezo, para la paridad
  int    paridad_minima = 2;   // el mínimo de paridad por bloque (fec_minimo_limpio)
  int    muestras = 0;       // con cuánta evidencia se decidió
  bool   sondeando = false;
  bool   reanclo = false;
  bool   topeo = false;
};

// Cuántos paquetes de paridad lleva un bloque de k datos cuando la reparación se decide en
// paquetes (`paridad_en_paquetes`). Es la menor m —con el mínimo que pide el cliente— que deja
// la probabilidad de perder el bloque por debajo de `paridad_falla_bloque`, si la pérdida es
// `perdida` y llega en tropiezos de `rafaga` paquetes seguidos: los tropiezos caen con
// probabilidad perdida/rafaga por paquete, y el bloque aguanta los que entran en su paridad.
//
// Con el techo de `fec_max`. Sin él, la pérdida de una cola que desborda —de congestión— se
// contesta con más paridad, y eso es sumarle carga al caño que se cae. Distinguir el
// desborde por la fila no alcanza: con una descarga en el caño el piso del retardo se
// envenena y la fila parece baja.
//
// La llama el host por cada bloque, con `perdida_est`, `rafaga_perdida` y `paridad_minima` de la
// última decisión; `minimo` negativo es `fec_minimo_paquetes`.
int paridad_de_bloque(int k, double perdida, double rafaga, const Parametros& p, int minimo = -1);

class Control {
 public:
  explicit Control(const Parametros& p);

  // Deja el lazo como recién arrancado. `negociado` es el techo que pidió el cliente.
  void reiniciar(double negociado);

  // Se llama cada Tc con lo último que llegó del cliente.
  Decision paso(const Medicion& m);

  // Entre dos pasos, con cada muestra del ida y vuelta: la fila de esa muestra sobre su piso y
  // cuánto creció en los últimos 50 ms, en segundos, y lo último que se midió que llega. Si la
  // fila pegó un escalón que sólo un caño angosto explica, recorta el presupuesto ahí mismo —sin
  // esperar al paso— y devuelve true; el presupuesto nuevo es el de presupuesto().
  bool alarma(double t, double fila, double crecimiento, double recibida);
  // Con la muestra siguiente a una alarma: si la fila ya volvió a su piso, el escalón fue un acuse
  // demorado y no un caño que se angosta —la cola de un caño angosto no se vacía en una muestra—, y
  // el lazo vuelve a como estaba antes de la alarma. Devuelve true si la deshizo; el presupuesto
  // vuelve a ser el de presupuesto().
  bool desmentir_alarma(double t, double fila);
  double presupuesto() const { return R_ * p_.destino; }
  // Si la alarma pide además un cuadro clave en el acto (cuadro_clave_en_caida). Con el caño
  // angosto, lo que salió en los últimos cien milisegundos se está perdiendo, y el cliente lo nota
  // recién cuando le llega el primer cuadro entero, un tercio de segundo después; desde ahí
  // descarta todo hasta el cuadro clave que pida. Mandarlo ya hace que ese primer cuadro sea él.
  bool alarma_pide_cuadro_clave() const { return p_.cuadro_clave_en_caida != 0; }
  // El mínimo de paridad por bloque vigente; la alarma lo vuelve a subir en el acto.
  int paridad_minima() const { return paridad_minima_; }
  // Los cuadros por segundo que el host codifica de verdad (cuadros_en_vuelo): la paridad mínima es
  // un costo por cuadro, y contarla a la tasa nominal dejaba la imagen en el piso.
  void cuadros_por_segundo(double fps) { fps_ = fps > 0 ? fps : p_.fps; }

  // Para el experimento de la ganancia última: multiplica k_g y k_p por el mismo factor.
  // El modelo predice que el lazo empieza a oscilar de forma sostenida en un valor
  // concreto — es la predicción que puede matarlo.
  void escalar_ganancias(double factor) { escala_ = factor; }

  // Para el experimento del error de estimación: fuerza Ĉ en vez de deducirla.
  void forzar_capacidad(double bps) { C_forzada_ = bps; }

  // Modo ingenuo: sin término de crecimiento, con ganancias fijas. Es el contrafáctico
  // del capítulo 4 — dos integraciones sin derivada.
  void modo_ingenuo(bool si) { ingenuo_ = si; }

  const Parametros& parametros() const { return p_; }
  const Sintonia&   sintonia() const { return s_; }

 private:
  struct Muestra { double t = 0; double v = 0; };

  static void   podar(std::deque<Muestra>& v, double t, double ventana);
  static double percentil(const std::deque<Muestra>& v, int q);
  // El valor de la ventana más cercano a un instante pasado. Devuelve 0 si la ventana está
  // vacía; si el instante pedido cae antes de lo que se guarda, devuelve el más viejo.
  static double valor_en(const std::deque<Muestra>& v, double t);

  // Cap. 6: promedio mientras haya pocas muestras, ajuste de recta a partir de ~3.
  // La mediana no sirve para pendientes. Devuelve la pendiente y deja el residuo,
  // que es de donde sale el ancho de la zona muerta.
  double pendiente(double& residuo_rms) const;
  double constante_filtro(double rtt) const;

  void   estimar(const Medicion& m, double fila, double dref);
  bool   chequeo_coherencia(double fila, double dref, double delta, double dR);
  void   sondeo_del_piso(double t);
  void   subir_tanteando(double dref, double fila);
  void   no_enrollar(double techo);

  Parametros p_;
  Sintonia   s_;

  // el lazo
  double R_ = 0;             // el presupuesto, lo único que decide
  double f_ = 0;             // la reparación
  double R_sano_ = 0;        // último presupuesto con la fila realmente vacía
  double C_sano_ = 0;        // y la capacidad estimada en ese momento
  // Un episodio de fila: lo que había antes y cuánto de él la fila fue de otro (ajena_s)
  struct Episodio {
    bool   activo = false;
    double t0 = 0, R = 0, C = 0;
    double ajeno = 0;        // segundos con la fila alta, quieta y sin que la explique lo nuestro
  } episodio_;
  bool   tras_ajena_ = false;  // la última fila fue de otro y terminó: la calma se explora ya
  double fila_prev_ = 0;
  double fila_cruda_prev_ = 0;   // la muestra cruda de la fila del paso anterior
  double fila_cruda_prev2_ = 0;  // y la de dos pasos atrás
  bool   habia_fila_ = false;
  bool   primera_ = true;
  double t_ = 0;

  // la señal
  std::deque<Muestra> piso_;     // ventana del piso del retardo en un sentido
  std::deque<Muestra> viajes_;   // ventana del ida y vuelta
  std::deque<Muestra> tasas_;    // ventana de la tasa recibida
  // Lo que el host puso en el cable, con su hora. Hace falta guardado y no sólo el último:
  // el informe del cliente llega un viaje tarde, así que la tasa que informa corresponde a
  // lo que se emitió un viaje antes. Compararla contra lo que se está emitiendo AHORA hace
  // que toda subida de tasa parezca pérdida.
  std::deque<Muestra> emitidas_;
  std::deque<Muestra> filas_;    // ventana corta, para el ajuste de recta
  std::deque<Muestra> minimos_;  // la fila cruda de los últimos ruido_ventana s: su envolvente de abajo
  std::deque<Muestra> presupuestos_;  // R en los últimos segundos, para volver a lo de antes de una cola
  double piso_vigente_ = 0;
  double piso_fresco_ = 0;
  bool   competidor_ = false;  // el retardo de la ventana tiene el vaivén de otro flujo (competidor_umbral)
  double t_sin_vaiven_ = 0;    // la última vez que la ventana no tenía ese vaivén
  double perdida_suave_ = 0;
  double perdida_pico_ = 0;    // la de la paridad: el último pico, que se olvida despacio
  double t_perdida_ = -1e9;    // la última vez que hubo pérdida, o una alarma
  int    paridad_minima_ = 2;
  double fps_ = 60;              // los cuadros por segundo codificados
  double t_tanda_ = -1e9;      // cuándo empezó la última tanda de pérdida
  bool   en_tanda_ = false, recurrente_ = false;
  double t_arranque_ = -1;     // la hora del primer paso
  double rafaga_ = 0;          // largo de las ráfagas del canal, sin las del desborde
  double C_est_ = 0, tau_est_ = 0;
  double C_forzada_ = 0;
  double escala_ = 1.0;
  bool   ingenuo_ = false;

  // la curva de subida
  double ancla_ = 0, curva_t_ = 0, curva_R0_ = 0;
  double t_ultima_fila_ = 0;     // la última vez que hubo fila
  bool   en_curva_ = false;

  // el sondeo del piso
  bool   sondeando_ = false;
  double sondeo_hasta_ = 0;
  int    quieto_ = 0;

  // el cuadro que no llena el presupuesto
  bool   app_limited_ = false;
  // Llega al cable más de lo que sale del otro lado: hay cola o hay descarte, y en los dos
  // casos el caño está lleno. Es lo único que sigue siendo cierto cuando el piso del
  // retardo se envenenó.
  bool   caño_lleno_ = false;
  double ancho_tren_ = 0;
  double t_ancho_ = 0;            // cuándo se actualizó el ancho por última vez
  double emitida_entonces_ = 0;   // lo emitido un viaje antes del informe vigente
  double emitida_suave_ = 0, recibida_suave_ = 0;   // las dos, suavizadas para compararlas
  double t_salto_rapido_ = -1e9;  // cuándo disparó por última vez el salto rápido
  int llega_menos_seguidos_ = 0;  // ciclos seguidos en que llegó bastante menos de lo emitido
  double t_perdida_sin_fila_ = -1;  // desde cuándo se pierde de más sin que se vea fila
  double ciego_hasta_ = -1e9;       // hasta cuándo el retardo no es de fiar como señal de cola
  bool arranque_hecho_ = false;     // ya apareció la primera fila o pérdida de la sesión
  double entrega_suave_ = 0;        // la entrega de los mismos cuadros, suavizada a medio segundo
  double dref_ultimo_ = 0;          // el objetivo de fila del último paso, para la alarma
  double t_alarma_ = -1e9;
  std::deque<Muestra> entregado_;  // lo que llegó en el último segundo, para salir de la exploración
  double t_crucero_ = 0;           // desde cuándo el camino está en crucero
  bool   explorando_ = false;      // subiendo multiplicativo en crucero          // cuándo disparó por última vez la alarma entre pasos
  double eficiencia_cod_ = 1;     // lo que el codificador entrega sobre lo que se le pide
  double pedido_prev_ = 0;        // lo que se le pidió en el paso anterior
  int    fila_sobre_objetivo_ = 0;  // pasos seguidos con la fila por encima del objetivo
  // Lo que la última alarma cambió, para deshacerlo si la muestra siguiente la desmiente
  struct AntesDeAlarma {
    double R = 0, ancla = 0, t_alarma = 0, t_perdida = 0, t_salto_rapido = 0;
    bool en_curva = false, arranque_hecho = false;
    int paridad_minima = 0;
  };
  AntesDeAlarma antes_alarma_;
  bool alarma_abierta_ = false;
  // Los trenes de la ventana: cuántos bits tenía cada cuadro y cuánto tardó en cruzar. El
  // ancho sale de la pendiente entre los dos, no de cada cociente por separado.
  struct Tren { double t = 0; double bits = 0; double lapso = 0; };
  std::deque<Tren> trenes_;
  // El ancho por dispersión de la ventana; false si ningún cuadro alcanzó a separarse.
  bool ancho_por_regresion(double& ancho) const;

  // el chequeo de coherencia
  struct Prueba { bool activa = false; double t = 0; double R = 0; double fila = 0; };
  Prueba prueba_;
  int    sospecha_ = 0;
};

}  // namespace estab
