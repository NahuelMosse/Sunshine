// De las señales que GameStream ya trae a la Medicion del controlador.
//
// GameStream no dice cuándo llegó cada paquete ni cada cuadro. Lo que sí trae, sin agregar
// nada al protocolo (specs/propuesta-solucion/10-senales-gamestream.md):
//
//   · el ida y vuelta que ENet mide en el canal de control, que cruza el mismo cuello de
//     botella que el video —siempre que el host lo haga medir: con el ping del cliente cada
//     100 ms el host nunca está ocioso y nunca manda el suyo—;
//   · el estado de FEC de cada cuadro que necesitó reparación o se perdió: cuántos paquetes
//     de datos y de paridad tenía y cuántos llegaron;
//   · el resumen de pérdida, cada 50 ms, con el último cuadro que llegó entero y cuántos se
//     perdieron desde el anterior. Moonlight lo manda a Sunshine desde que se implementó lo
//     que faltaba; un cliente de serie no lo manda, y el medidor funciona sin él.
//
// Con eso se arma todo: la fila sale del ida y vuelta, la pérdida y su ráfaga del estado de
// FEC, y lo que llega del avance del último cuadro bueno, con los tamaños que el host conoce.
//
// Igual que control.h: sin red, sin hilos, sin dependencias. Se prueba en el banco y se copia
// a Sunshine sin editar.
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "../control.h"

namespace estab {
namespace gamestream {

// Lo que el host puso en el cable para un cuadro. Horas del reloj del host, en µs.
struct CuadroEnviado {
  uint32_t indice = 0;
  int64_t  t_primero_us = 0;   // cuándo salió el primer paquete
  int64_t  t_ultimo_us = 0;    // y el último
  int      paquetes = 0;       // datos y paridad
  int      paquetes_datos = 0; // de esos, los de datos: el resto es la paridad que salió
  double   bytes_rtp = 0;      // lo que el cliente va a contar de esos paquetes
  double   bytes_cable = 0;    // eso más IP, UDP y cifrado: lo que cobra el cuello de botella
  double   bytes_lleno = 0;    // en bytes de cable, el tamaño que habría tenido usando todo el
                               // presupuesto (lo usa sólo la medición por dispersión)
};

// El estado de FEC de un cuadro, tal como lo manda moonlight-common-c (SS_FRAME_FEC_STATUS).
// Sólo llega para cuadros que necesitaron paridad o no se pudieron rearmar. Si se rearmó, el
// cliente lo manda apenas junta los paquetes de datos que hacen falta: lo recibido es justo eso y
// lo que no llegó todavía no se perdió, así que lo perdido son los huecos detrás del último
// recibido (missingPacketsBeforeHighestReceived; −1 si no se conoce).
struct EstadoFec {
  uint32_t indice = 0;
  int datos_total = 0, paridad_total = 0;
  int datos_recibidos = 0, paridad_recibida = 0;
  int faltantes = -1;
};

// La llegada de un cuadro, como la informa el mensaje Frame Stats de GameStream (reservado en
// todas las generaciones del protocolo y nunca implementado; su contenido es éste). Horas del
// reloj del CLIENTE, truncadas a 32 bits: nunca se comparan con las del host salvo por diferencia.
struct TrenRecibido {
  uint32_t indice = 0;
  uint32_t t_primero_us = 0;
  uint32_t t_ultimo_us = 0;
  uint32_t bytes = 0;
  uint16_t paquetes = 0;
};

// El resumen de pérdida periódico (IDX_LOSS_STATS): cuadros perdidos desde el anterior y el
// último que llegó entero.
struct ResumenPerdida {
  uint32_t perdidos = 0;
  uint32_t ultimo_bueno = 0;
};

class MedidorSenales {
 public:
  MedidorSenales() = default;

  void enviado(const CuadroEnviado& c);
  void estado_fec(const EstadoFec& e);
  void resumen(int64_t llega_us, const ResumenPerdida& r);
  // La llegada de un cuadro (Frame Stats). Con ella el control mide la dispersión: cuánto se
  // abrieron los paquetes de un cuadro al cruzar el cuello de botella, que dice el ancho del caño
  // aunque el cuadro sea chico. Un cliente que no la manda deja la medición como sin ella.
  void tren(const TrenRecibido& r) { trenes_rx_.push_back(r); }
  // El ida y vuelta de ENet del peer del canal de control, en segundos, tal como el host lo lee.
  // Uno de menos de la mitad del piso no es un camino más corto sino una lectura rota —en Windows
  // aparece de vez en cuando un ida y vuelta de 1 ms—, y como piso envenena diez segundos de fila:
  // se descarta, salvo que se repita, que es lo que haría un camino que de verdad se acortó.
  void rtt(double segundos);
  void audio(double bps) { audio_bps_ = bps; }
  // El cliente pidió un cuadro clave, y el próximo cuadro que va a salir es `proximo`. Hasta que
  // llegue, el cliente descarta los cuadros enteros que recibe —no tiene de qué decodificarlos—
  // y el resumen los cuenta perdidos: esos no son pérdida de la red, y no entran en la cuenta.
  void pedido_idr(uint32_t proximo) { if (!idr_pendiente_) { idr_pendiente_ = true; idr_desde_ = proximo; } }
  // El ping periódico del cliente, con su reloj monotónico en milisegundos. Lo que su llegada
  // se despega de ese reloj es la cola del sentido de subida, por la que vuelven los resúmenes.
  void ping_cliente(int64_t llega_us, uint32_t ms_cliente);
  // Una muestra cruda del ida y vuelta de ENet, en ms, apenas llega. Devuelve su fila sobre el
  // mínimo de diez segundos y cuánto creció desde la de hace 50 ms, en segundos; false si la
  // muestra es la misma de antes o todavía no hay con qué comparar.
  bool muestra_enet(int64_t t_us, double ms, double& fila, double& crecimiento);
  // Lo que la fila sube y vuelve sola, en segundos (Medicion::ruido)
  double ruido() const { return ruido_; }
  // Qué partes opcionales usa: la fila por la edad del último cuadro bueno (fila_edad) y lo
  // entregado sobre los mismos cuadros (entrega_llena > 0).
  void configurar(const Parametros& p) {
    usar_edad_ = p.fila_edad != 0;
    edad_reciente_us_ = static_cast<int64_t>(p.edad_reciente * 1e6);
    cadencia_max_us_ = p.edad_cadencia > 0 ? static_cast<int64_t>(p.edad_cadencia * 1e6 / p.fps) : 0;
    usar_misma_ = p.entrega_llena > 0;
    fila_previa_max_ = p.alarma_fila_previa;
    intervalos_alarma_ = p.alarma_intervalos;
    previa_us_ = static_cast<int64_t>(p.alarma_previa_s * 1e6);
  }

  // Arma la Medicion con lo que llegó desde la llamada anterior. Devuelve false si todavía no
  // hay con qué: sin ida y vuelta medido ni cuadros enviados.
  bool medicion(int64_t ahora_us, Medicion& m);

  // Si llegó al menos un resumen de pérdida: con un cliente de serie, lo que llega se estima
  // en vez de medirse.
  bool hay_resumen() const { return hubo_resumen_; }

 private:
  std::map<uint32_t, CuadroEnviado> enviados_;
  struct Tramo { int64_t t_us; double lapso_s; double bits; };
  std::deque<Tramo> ventana_rx_, ventana_tx_, ventana_datos_;
  std::deque<Tramo> ventana_paquetes_tx_, ventana_paquetes_perdidos_;
  double bits_emitidos_ = 0, bits_datos_ = 0;       // desde la llamada anterior
  double paquetes_tx_ = 0, datos_tx_ = 0;
  double paquetes_perdidos_ = 0;                     // del estado de FEC y los cuadros perdidos
  double cuadros_con_perdida_ = 0, perdidos_en_cuadros_ = 0;
  int      fallidos_con_estado_ = 0;                 // cuadros con estado que no se rearmaron
  double   bits_llegados_ = 0;                       // por el avance del último cuadro bueno
  int64_t  resumen_llega_us_ = 0;
  uint32_t ultimo_bueno_ = 0;
  bool     hubo_resumen_ = false;
  bool     idr_pendiente_ = false;                   // esperando el cuadro clave pedido
  uint32_t idr_desde_ = 0;                           // el primero que pudo ser ese cuadro
  int64_t  ultima_llamada_us_ = 0;
  double   rtt_ = 0;
  double   audio_bps_ = 0;
  double   paquetes_por_cuadro_ = 0;                 // media, para los cuadros que se perdieron enteros

  // La entrega sobre los mismos cuadros: los resúmenes recientes, y lo último que dieron
  struct Resumen { int64_t llega_us; uint32_t bueno; double perdidos; };
  double   perdidos_acum_ = 0;
  std::deque<Resumen> resumenes_;
  double   tasa_misma_ = 0;                          // bits/s entregados, draft-cheng
  double   entrega_ = 0;                             // lapso de salida sobre lapso de llegada
  int64_t  t_misma_us_ = 0;

  // La edad del último cuadro bueno y la deriva del ping, con sus mínimos de diez segundos
  struct Muestra { int64_t t_us; double v; };
  std::deque<Muestra> edades_, vueltas_, rtts_, crudas_;
  // El ruido del camino: cuánto sobresale cada muestra de ENet sobre las de alrededor (las de antes
  // y las de después), en los últimos quince segundos. Ver sumar_ruido.
  std::deque<Muestra> excursiones_;
  int64_t  centro_ruido_us_ = 0;                     // la última muestra ya juzgada
  double   ruido_ = 0;                               // segundos, el percentil 90
  void     sumar_ruido(int64_t t_us);
  // El vaciado de cada cola: las muestras de ENet desde su pico hasta que vuelve al piso. Ver
  // seguir_vaciado.
  struct Bajada { int64_t t_us; int64_t sale_us; double ms; };
  std::vector<Bajada> bajada_;
  bool     en_cola_ = false;
  int64_t  cola_desde_us_ = 0;
  double   vaciado_ = 0;                             // bits/s, el último sin informar
  int64_t  vaciado_desde_us_ = 0;
  int64_t  vaciado_pico_us_ = 0;                     // cuándo salió el ping del pico
  double   cola_desde_ms_ = 0, vaciado_altura_ = 0;   // el ida y vuelta al empezar la cola, y cuánto subió
  void     seguir_vaciado(int64_t t_us, double ms, double piso);
  double   bits_entre(int64_t desde_us, int64_t hasta_us) const;
  bool     usar_edad_ = false;
  int64_t  edad_reciente_us_ = 150000;
  int64_t  cadencia_max_us_ = 41667;                 // el hueco entre cuadros que todavía es cadencia
  int      intervalos_alarma_ = 2;
  int64_t  previa_us_ = 0;
  bool     usar_misma_ = false;
  double   fila_previa_max_ = 0.025;                 // la fila de la que puede arrancar un escalón
  int64_t  t_piso_enet_us_ = 0;                      // la última muestra de ENet que tocó el piso
  bool     verosimil(double ms, std::deque<Muestra>& historia, int& seguidas);
  std::vector<TrenRecibido> trenes_rx_;              // las llegadas desde la medición anterior
  int      rtt_bajos_ = 0, enet_bajas_ = 0;          // lecturas seguidas bajo la mitad del piso
};

}  // namespace gamestream
}  // namespace estab
