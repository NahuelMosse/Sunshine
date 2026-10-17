#include "senales.h"

#include <algorithm>

namespace estab {
namespace gamestream {
namespace {

// Cuánto se guarda lo enviado: tiene que cubrir holgado el ida y vuelta más la cadencia del
// resumen de pérdida, que es cuando se cruza con lo que llegó.
constexpr int64_t kRetencionUs = 3000000;

// La ventana de las tasas y de la pérdida. El resumen llega cada 50 ms y el estado de FEC
// sólo con pérdida: un cuarto de segundo junta varios de cada uno.
constexpr int64_t kVentanaUs = 250000;

// Sobre cuánto se mide la entrega: con la cuantización de un cuadro en cada punta, 300 ms dejan
// un error de alrededor de un 2 %.
constexpr int64_t kLapsoEntregaUs = 300000;

// Los mínimos de la edad, de la deriva del ping y del ida y vuelta, como el piso del control.
constexpr int64_t kPisoUs = 10000000;

// Cuánto más tiene que tardar un cuadro en llegar que en salir para que su dispersión sea la del
// cuello de botella y no la del emisor.
constexpr double kDispersionMinima = 1.1;

}  // namespace

void MedidorSenales::enviado(const CuadroEnviado& c) {
  enviados_[c.indice] = c;
  bits_emitidos_ += c.bytes_cable * 8.0;
  if (c.paquetes > 0) bits_datos_ += c.bytes_cable * 8.0 * c.paquetes_datos / c.paquetes;
  paquetes_tx_ += c.paquetes;
  datos_tx_ += c.paquetes_datos;
  paquetes_por_cuadro_ = paquetes_por_cuadro_ > 0
                             ? paquetes_por_cuadro_ + 0.05 * (c.paquetes - paquetes_por_cuadro_)
                             : c.paquetes;
  for (auto it = enviados_.begin(); it != enviados_.end();) {
    if (c.t_primero_us - it->second.t_primero_us > kRetencionUs) it = enviados_.erase(it);
    else break;
  }
}

void MedidorSenales::estado_fec(const EstadoFec& e) {
  const int total = e.datos_total + e.paridad_total;
  const int recibidos = e.datos_recibidos + e.paridad_recibida;
  // Rearmado, lo que falta hasta el total es la paridad que no llegó a hacer falta: lo perdido son
  // los huecos. Contar hasta el total da siempre la paridad del bloque: la ráfaga parece de dos a
  // cuatro paquetes cuando la del canal es de uno y algo, y la paridad se va a su techo.
  const int faltan = recibidos >= e.datos_total && e.faltantes >= 0
                         ? std::max(e.faltantes, e.datos_total - e.datos_recibidos)
                         : std::max(0, total - recibidos);
  if (faltan > 0) {
    paquetes_perdidos_ += faltan;
    perdidos_en_cuadros_ += faltan;
    cuadros_con_perdida_ += 1;
  }
  // Reed-Solomon rearma con cualquier k de los n: si llegaron menos que los datos, el cuadro
  // se perdió aunque haya llegado parte
  if (recibidos < e.datos_total) ++fallidos_con_estado_;
}

void MedidorSenales::resumen(int64_t llega_us, const ResumenPerdida& r) {
  if (!hubo_resumen_) {
    hubo_resumen_ = true;
    ultimo_bueno_ = r.ultimo_bueno;
    resumen_llega_us_ = llega_us;
    fallidos_con_estado_ = 0;
    return;
  }
  resumen_llega_us_ = llega_us;
  if (r.ultimo_bueno <= ultimo_bueno_) return;   // sin avance, o un resumen viejo

  // Lo que llegó desde el resumen anterior son los cuadros hasta el último bueno, menos los
  // que se perdieron en el camino. Los tamaños los conoce el host.
  double bytes = 0;
  int conocidos = 0;
  for (auto it = enviados_.upper_bound(ultimo_bueno_);
       it != enviados_.end() && it->first <= r.ultimo_bueno; ++it) {
    bytes += it->second.bytes_cable;
    ++conocidos;
  }
  // Si en el tramo llegó el cuadro clave pedido, los perdidos que cuenta el resumen son los que
  // el cliente descartó esperándolo, que llegaron enteros. La pérdida que llevó al pedido ya la
  // dijo el estado de FEC del cuadro que no se pudo rearmar.
  const bool descartes = idr_pendiente_ && r.ultimo_bueno >= idr_desde_;
  if (descartes) idr_pendiente_ = false;
  // Y el cliente cuenta perdido todo índice que no le llegó, también los que el host nunca mandó
  // —los que Sunshine tira cuando se le llena la cola del codificador—. Esos no son pérdida de la
  // red: son los del tramo que el host no tiene anotados.
  const double no_salieron = std::max(0.0, static_cast<double>(r.ultimo_bueno - ultimo_bueno_) - conocidos);
  const uint32_t perdidos = descartes ? 0
      : static_cast<uint32_t>(std::max(0.0, static_cast<double>(r.perdidos) - no_salieron));

  const double por_cuadro = conocidos > 0 ? bytes / conocidos : 0.0;
  bits_llegados_ += std::max(0.0, bytes - perdidos * por_cuadro) * 8.0;

  // Los cuadros perdidos enteros no mandan estado de FEC: no llegó ningún paquete del que el
  // cliente pudiera hablar. Son los perdidos que el resumen cuenta y el estado no explica.
  const int enteros = std::max(0, static_cast<int>(perdidos) - fallidos_con_estado_);
  paquetes_perdidos_ += enteros * paquetes_por_cuadro_;
  fallidos_con_estado_ = 0;
  ultimo_bueno_ = r.ultimo_bueno;
  perdidos_acum_ += perdidos;

  // ── lo entregado, sobre los mismos cuadros ────────────────────────────────
  // Comparar lo que llega ahora con lo que salió un viaje antes compara cuadros distintos, y el
  // codificador varía de un cuadro a otro bastante más que el caño: con Sunshine de verdad, la
  // relación entre las dos tasas variaba entre un 9 y un 17 % en régimen, y eso disparaba saltos
  // con la cola vacía. Acá se toman los cuadros que avanzó el último bueno en al menos 300 ms, y
  // se divide lo que pesaban por el mayor entre lo que tardaron en salir y lo que tardaron en
  // llegar: la estimación de la tasa de entrega de BBR (draft-cheng-iccrg-delivery-rate-estimation).
  // Cuánto más lento llegaron que lo que salieron dice si el caño los está frenando.
  resumenes_.push_back({llega_us, r.ultimo_bueno, perdidos_acum_});
  while (resumenes_.size() > 2 && llega_us - resumenes_[1].llega_us >= kLapsoEntregaUs) resumenes_.pop_front();
  const Resumen& desde = resumenes_.front();
  if (llega_us - desde.llega_us >= kLapsoEntregaUs) {
    const auto a = enviados_.find(desde.bueno);
    const auto b = enviados_.find(r.ultimo_bueno);
    if (a != enviados_.end() && b != enviados_.end()) {
      double bytes = 0;
      int n = 0;
      for (auto it = enviados_.upper_bound(desde.bueno); it != enviados_.end() && it->first <= r.ultimo_bueno; ++it) {
        bytes += it->second.bytes_cable;
        ++n;
      }
      if (n > 0) bytes -= std::min(static_cast<double>(n), perdidos_acum_ - desde.perdidos) * bytes / n;
      const double salida = (b->second.t_ultimo_us - a->second.t_ultimo_us) / 1e6;
      const double llegada = (llega_us - desde.llega_us) / 1e6;
      if (salida > 0 && llegada > 0 && bytes > 0) {
        tasa_misma_ = bytes * 8.0 / std::max(salida, llegada);
        entrega_ = salida / llegada;
        t_misma_us_ = llega_us;
      }
    }
  }

  // ── la edad del último cuadro bueno ───────────────────────────────────────
  // Desde que salió hasta que llegó el resumen que lo nombra: el viaje del video más el de
  // vuelta. Es la única medida de retardo que cruza la cola del VIDEO: el ida y vuelta de ENet
  // viaja por el canal de control, y detrás de una cola que reparte por flujo no la ve.
  // Pero sólo con los cuadros saliendo a su cadencia: mientras el cliente descarta esperando un
  // cuadro clave o una invalidación, o el host saltea cuadros (el divisor), el último bueno se
  // queda quieto y su edad suma la espera hasta el próximo, que no es cola.
  const auto c = enviados_.find(r.ultimo_bueno);
  if (c != enviados_.end() && (cadencia_max_us_ <= 0 || (!descartes && !idr_pendiente_))) {
    const auto sig = std::next(c);
    const int64_t proximo_us = sig != enviados_.end() ? sig->second.t_primero_us : llega_us;
    if (cadencia_max_us_ <= 0 || proximo_us - c->second.t_primero_us <= cadencia_max_us_) {
      edades_.push_back({llega_us, (llega_us - c->second.t_ultimo_us) / 1000.0});
    }
  }
}

bool MedidorSenales::verosimil(double ms, std::deque<Muestra>& historia, int& seguidas) {
  if (historia.empty()) return true;
  double piso = historia.front().v;
  for (const Muestra& x : historia) piso = std::min(piso, x.v);
  if (ms >= 0.5 * piso) { seguidas = 0; return true; }
  if (++seguidas < 3) return false;
  // Tres seguidas: el camino se acortó de verdad, y el piso viejo ya no sirve
  seguidas = 0;
  historia.clear();
  return true;
}

void MedidorSenales::rtt(double segundos) {
  if (segundos <= 0 || !verosimil(segundos * 1000.0, rtts_, rtt_bajos_)) return;
  rtt_ = segundos;
}

// El ruido de un camino real —Wi-Fi, internet— son muestras que suben y vuelven solas: el aire se
// corta un rato, un acuse se demora. La cola propia no hace eso: un caño que se angosta la sube como
// una rampa que no vuelve, y un camino que se alarga la sube de un escalón que se queda. Lo que
// separa las tres cosas es mirar a los dos lados: una muestra cuenta como ruido por lo que sobresale
// sobre el menor de antes y el menor de después, en kRuidoLadoUs a cada lado. Una rampa no sobresale
// del menor de después (sigue subiendo) y un escalón tampoco (se queda). Se juzga cada muestra con
// ese retraso. El ruido es la altura de los saltos: el percentil 90 de las muestras que sobresalen
// más de kSaltoMin en los últimos kRuidoVentanaUs. Los cortes de un Wi-Fi ocupan pocas muestras —un
// par cada pocos segundos—, y el percentil 90 de todas caía entre ellos: el ruido salía del vaivén
// de fondo y los saltos seguían leyéndose como cola. Sin saltos, es el de todas.
void MedidorSenales::sumar_ruido(int64_t t_us) {
  constexpr int64_t kRuidoLadoUs = 250000, kRuidoVentanaUs = 30000000;
  constexpr double kSaltoMin = 0.005;
  constexpr size_t kSaltosMin = 5;
  for (size_t c = 0; c < crudas_.size(); ++c) {
    const Muestra& x = crudas_[c];
    if (x.t_us <= centro_ruido_us_) continue;
    if (t_us - x.t_us < kRuidoLadoUs) break;           // todavía no se ve el después
    double antes = x.v, despues = x.v;
    bool hay_antes = false, hay_despues = false;
    for (size_t j = 0; j < crudas_.size(); ++j) {
      const Muestra& y = crudas_[j];
      if (y.t_us < x.t_us && x.t_us - y.t_us <= kRuidoLadoUs) { antes = hay_antes ? std::min(antes, y.v) : y.v; hay_antes = true; }
      if (y.t_us > x.t_us && y.t_us - x.t_us <= kRuidoLadoUs) { despues = hay_despues ? std::min(despues, y.v) : y.v; hay_despues = true; }
    }
    centro_ruido_us_ = x.t_us;
    if (!hay_antes || !hay_despues) continue;
    excursiones_.push_back({x.t_us, std::max(0.0, x.v - std::max(antes, despues)) / 1000.0});
  }
  while (!excursiones_.empty() && t_us - excursiones_.front().t_us > kRuidoVentanaUs) excursiones_.pop_front();
  if (excursiones_.size() < 20) { ruido_ = 0; return; }
  std::vector<double> v, saltos;
  v.reserve(excursiones_.size());
  for (const Muestra& e : excursiones_) { v.push_back(e.v); if (e.v > kSaltoMin) saltos.push_back(e.v); }
  if (saltos.size() >= kSaltosMin) v.swap(saltos);
  const size_t k = static_cast<size_t>(0.9 * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
  ruido_ = v[k];
}

// Lo que el host puso en el cable entre dos horas suyas: los cuadros, repartidos en lo que tardaron
// en salir, y el audio, que sale parejo.
double MedidorSenales::bits_entre(int64_t desde_us, int64_t hasta_us) const {
  if (hasta_us <= desde_us) return 0;
  double bits = audio_bps_ * (hasta_us - desde_us) / 1e6;
  for (const auto& [indice, c] : enviados_) {
    (void)indice;
    if (c.t_ultimo_us < desde_us) continue;
    if (c.t_primero_us > hasta_us) break;
    const int64_t dura = c.t_ultimo_us - c.t_primero_us;
    const double parte = dura <= 0 ? 1.0
        : static_cast<double>(std::min(hasta_us, c.t_ultimo_us) - std::max(desde_us, c.t_primero_us)) / dura;
    bits += c.bytes_cable * 8.0 * std::max(0.0, parte);
  }
  return bits;
}

// El caño que vacía una cola, medido con el ping de ENet, que hace la misma cola que el video. Una
// cola FIFO no deja de servir mientras tiene algo: entre dos pings que la encontraron llena salió
// todo lo que el host puso en el cable entre que salió uno y el otro, ni un byte más ni uno menos.
// Lo que tardaron en volver es lo que tardó eso en salir. Es la estimación de la entrega de BBR
// (draft-cheng-iccrg-delivery-rate-estimation): con cola, lo entregado por unidad de tiempo es el
// cuello de botella. No depende del tamaño de los cuadros, así que sirve igual con cuadros de un
// paquete, que no tienen dispersión que medir.
//
// Se mide desde el pico hasta que la cola vuelve al piso, porque es lo que dice cómo quedó el caño:
// cuando se corta el aire, mientras la cola crece el caño entrega casi nada, y lo que la vacía es el
// caño de siempre; cuando el caño se angosta, lo que la vacía es el caño nuevo. Si en el medio la cola
// se vació un instante, el lapso suma tiempo sin entregar y la medida sale baja: equivocarse es
// quedarse corto. Y con otro flujo en la cola sale la parte del video, también corta.
void MedidorSenales::seguir_vaciado(int64_t t_us, double ms, double piso) {
  constexpr double kLlenaMs = 20, kVaciaMs = 5;       // cuándo hay cola y cuándo ya no
  constexpr int64_t kEpisodioMaxUs = 3000000;
  const double cola = ms - piso;
  const int64_t sale_us = t_us - static_cast<int64_t>(ms * 1000.0);
  if (!en_cola_) {
    if (cola < kLlenaMs) return;
    en_cola_ = true;
    cola_desde_us_ = sale_us;
    cola_desde_ms_ = ms;
    bajada_.assign(1, {t_us, sale_us, ms});
    return;
  }
  if (t_us - cola_desde_us_ > kEpisodioMaxUs) { en_cola_ = false; bajada_.clear(); return; }
  if (cola >= kVaciaMs) {
    // Desde el último pico: lo que sube todavía es la cola llenándose. Un par de milisegundos de más
    // son el vaivén del acuse, no la cola.
    if (ms >= bajada_.front().ms || ms > bajada_.back().ms + 2) bajada_.clear();
    bajada_.push_back({t_us, sale_us, ms});
    return;
  }
  en_cola_ = false;
  if (bajada_.size() >= 3 && bajada_.back().t_us - bajada_.front().t_us >= 30000) {
    // La recta de lo enviado contra la hora de vuelta: su pendiente es lo que el caño entregó
    const Bajada& a = bajada_.front();
    double st = 0, sb = 0;
    std::vector<double> b(bajada_.size());
    for (size_t i = 0; i < bajada_.size(); ++i) {
      b[i] = bits_entre(a.sale_us, bajada_[i].sale_us);
      st += (bajada_[i].t_us - a.t_us) / 1e6;
      sb += b[i];
    }
    const double n = static_cast<double>(bajada_.size()), mt = st / n, mb = sb / n;
    double num = 0, den = 0;
    for (size_t i = 0; i < bajada_.size(); ++i) {
      const double x = (bajada_[i].t_us - a.t_us) / 1e6 - mt;
      num += x * (b[i] - mb);
      den += x * x;
    }
    if (den > 0 && num > 0) {
      vaciado_ = num / den;
      vaciado_desde_us_ = cola_desde_us_;
      vaciado_pico_us_ = a.sale_us;
      vaciado_altura_ = (a.ms - cola_desde_ms_) / 1000.0;
    }
  }
  bajada_.clear();
}

bool MedidorSenales::muestra_enet(int64_t t_us, double ms, double& fila, double& crecimiento) {
  if (ms <= 0 || !verosimil(ms, crudas_, enet_bajas_)) return false;
  // El host lee la misma muestra varias veces entre dos acuses; una igual a la anterior no dice
  // nada que la alarma necesite —sólo importa cuando la fila crece— y contarla dos veces metía
  // intervalos de crecimiento cero justo en medio de la rampa.
  if (!crudas_.empty() && crudas_.back().v == ms) return false;
  crudas_.push_back({t_us, ms});
  while (t_us - crudas_.front().t_us > kPisoUs) crudas_.pop_front();
  sumar_ruido(t_us);
  // El crecimiento es el de los dos últimos intervalos entre muestras, el menor (o el del último,
  // con alarma_intervalos 1), llevado a 50 ms
  // con el tiempo real de cada uno: con la cola llenándose los acuses se espacian —de 25 ms a más
  // de 100— y una ventana de tiempo fija quedaba vacía justo en la rampa. Pedirlo en los dos saca
  // el ruido —muestras de acuses distintos que saltan diez milisegundos arriba y abajo— y separa
  // un caño que se angosta, que hace crecer la fila como una rampa a (R−C)/R, de un camino que se
  // alarga, que la sube de un escalón y la deja plana.
  double piso = ms;
  for (const Muestra& x : crudas_) piso = std::min(piso, x.v);
  seguir_vaciado(t_us, ms, piso);
  const size_t n = crudas_.size();
  if (n < 3) return false;
  const Muestra& a = crudas_[n - 3];
  const Muestra& b = crudas_[n - 2];
  if (t_us <= b.t_us || b.t_us <= a.t_us) return false;
  double v2 = a.v;
  fila = (ms - piso) / 1000.0;
  const double ultimo = (ms - b.v) * 50000.0 / (t_us - b.t_us);
  if (intervalos_alarma_ <= 0) {
    // Sobre una ventana fija de 50 ms, desde la última muestra de antes: con pings seguidos la
    // ventana junta varias, y el salto de un acuse suelto queda repartido en vez de multiplicado.
    size_t j = n - 1;
    while (j > 0 && t_us - crudas_[j].t_us < 50000) --j;
    if (t_us - crudas_[j].t_us < 50000) return false;
    v2 = crudas_[j].v;
    crecimiento = ultimo > 0 ? (ms - v2) * 50000.0 / (t_us - crudas_[j].t_us) / 1000.0 : 0.0;
  } else {
    crecimiento = (intervalos_alarma_ == 1 ? ultimo : std::min(ultimo, (b.v - a.v) * 50000.0 / (b.t_us - a.t_us))) / 1000.0;
  }
  // Y sólo desde una cola casi vacía que además estuvo vacía hace poco: un caño que se angosta en
  // régimen arranca sin fila, mientras que con una descarga en el caño la cola ya estaba —la arma
  // ella— y el mínimo de diez segundos queda envenenado a su nivel, así que la fila de antes
  // parecía baja sin serlo. Lo que no hace la descarga es dejar que el ida y vuelta toque el piso.
  // Vacía hace poco quiere decir bajo la mitad de lo que se admite antes del escalón: el lazo trabaja
  // con diez o veinte milisegundos de cola propia, y pedir que tocara el piso dejaba la alarma muda
  // justo cuando más falta hacía, con el caño pegado.
  // La fila de antes es la de antes de la rampa: el mínimo de los últimos alarma_previa_s. La de dos
  // muestras atrás ya es parte de la rampa cuando los acuses se espacian, y pasaba del límite a la
  // tercera muestra de una caída, antes de que el crecimiento medido llegara al umbral.
  if (previa_us_ > 0) {
    v2 = ms;
    for (size_t i = n - 1; i-- > 0 && t_us - crudas_[i].t_us <= previa_us_;) v2 = std::min(v2, crudas_[i].v);
  }
  if (ms - piso < 0.5 * fila_previa_max_ * 1000.0) t_piso_enet_us_ = t_us;
  if ((v2 - piso) / 1000.0 > fila_previa_max_ || t_us - t_piso_enet_us_ > 5000000) crecimiento = 0;
  // Ni en los primeros segundos, cuando el piso todavía no se conoce: si otro flujo ya llenaba el
  // caño al arrancar, la primera cola que se ve parece un escalón.
  if (t_us - crudas_.front().t_us < 3000000) crecimiento = 0;
  return true;
}

void MedidorSenales::ping_cliente(int64_t llega_us, uint32_t ms_cliente) {
  if (ms_cliente == 0) return;   // un cliente de serie no pone la marca
  vueltas_.push_back({llega_us, llega_us / 1000.0 - ms_cliente});
}

bool MedidorSenales::medicion(int64_t ahora_us, Medicion& m) {
  if (rtt_ <= 0 || enviados_.empty()) return false;
  const double lapso = ultima_llamada_us_ > 0 ? (ahora_us - ultima_llamada_us_) / 1e6 : 0.1;
  ultima_llamada_us_ = ahora_us;

  auto empujar = [&](std::deque<Tramo>& v, double bits) {
    if (lapso > 1e-3) v.push_back({ahora_us, lapso, bits});
    while (!v.empty() && ahora_us - v.front().t_us > kVentanaUs) v.pop_front();
  };
  auto tasa = [](const std::deque<Tramo>& v) {
    double bits = 0, dur = 0;
    for (const Tramo& x : v) { bits += x.bits; dur += x.lapso_s; }
    return dur > 1e-3 ? bits / dur : 0.0;
  };
  auto suma = [](const std::deque<Tramo>& v) {
    double s = 0;
    for (const Tramo& x : v) s += x.bits;
    return s;
  };

  empujar(ventana_tx_, bits_emitidos_);
  empujar(ventana_datos_, bits_datos_);
  empujar(ventana_rx_, bits_llegados_);
  empujar(ventana_paquetes_tx_, paquetes_tx_);
  empujar(ventana_paquetes_perdidos_, paquetes_perdidos_);

  const double enviados = suma(ventana_paquetes_tx_);
  const double perdidos = suma(ventana_paquetes_perdidos_);

  m = Medicion{};
  m.t = ahora_us / 1e6;
  m.rtt = rtt_;
  // Sin la hora de llegada de nada, el retardo que hay es el ida y vuelta: la fila es cuánto se
  // despegó de su mínimo, la misma cuenta que hace el control con el retardo en un sentido.
  m.owd_min = rtt_;
  m.owd_medio = rtt_;
  m.ruido = ruido_;
  if (vaciado_ > 0) {
    m.vaciado = vaciado_;
    m.vaciado_desde = vaciado_desde_us_ / 1e6;
    m.vaciado_subida = std::max<int64_t>(0, vaciado_pico_us_ - vaciado_desde_us_) / 1e6;
    m.vaciado_altura = std::max(0.0, vaciado_altura_);
    vaciado_ = 0;
  }
  m.perdida = enviados > 0 ? std::min(1.0, perdidos / enviados) : 0.0;
  m.tasa_emitida = tasa(ventana_tx_) + audio_bps_;
  m.tasa_datos = tasa(ventana_datos_);
  // Con el resumen de pérdida, lo que llega se mide por el avance del último cuadro bueno. Sin
  // él —un cliente de serie— sólo se puede estimar como lo que salió menos lo que se perdió. Y
  // también se estima si hace una ventana entera que no llega ninguno: el resumen viaja sin
  // garantía de entrega, y leer su ausencia como "no llegó nada" hacía saltar al lazo por una
  // caída que no existía. Lo mismo mientras el cliente espera un cuadro clave: el último cuadro
  // bueno no avanza aunque los cuadros lleguen.
  const bool resumen_fresco = hubo_resumen_ && !idr_pendiente_ &&
                              ahora_us - resumen_llega_us_ <= kVentanaUs;
  m.tasa_recibida = resumen_fresco ? tasa(ventana_rx_) + audio_bps_
                                   : m.tasa_emitida * (1.0 - m.perdida);
  if (usar_misma_ && resumen_fresco && t_misma_us_ > 0 && ahora_us - t_misma_us_ <= kVentanaUs) {
    m.tasa_recibida = tasa_misma_ + audio_bps_;
    m.entrega = entrega_;
  }

  // ── la fila: la del ida y vuelta de ENet o la de la edad del cuadro, la mayor ──
  // La edad cuenta la cola del video y también la del sentido de subida, por la que vuelve el
  // resumen; esa segunda se descuenta con la deriva del ping, que sólo viaja de subida.
  auto podar_piso = [&](std::deque<Muestra>& v) {
    while (!v.empty() && ahora_us - v.front().t_us > kPisoUs) v.pop_front();
  };
  rtts_.push_back({ahora_us, rtt_ * 1000.0});
  podar_piso(rtts_);
  podar_piso(edades_);
  podar_piso(vueltas_);
  if (usar_edad_ && !edades_.empty()) {
    double edad_min = edades_.front().v, reciente = 1e18, rtt_min = rtts_.front().v;
    for (const Muestra& x : edades_) {
      edad_min = std::min(edad_min, x.v);
      if (ahora_us - x.t_us <= edad_reciente_us_) reciente = std::min(reciente, x.v);
    }
    for (const Muestra& x : rtts_) rtt_min = std::min(rtt_min, x.v);
    double subida = 0;
    if (!vueltas_.empty()) {
      double v_min = vueltas_.front().v;
      for (const Muestra& x : vueltas_) v_min = std::min(v_min, x.v);
      subida = vueltas_.back().v - v_min;
    }
    if (reciente < 1e17) {
      const double fila_edad = std::max(0.0, reciente - edad_min - subida) / 1000.0;
      const double fila_rtt = std::max(0.0, rtt_ - rtt_min / 1000.0);
      if (fila_edad > fila_rtt) m.owd_min = m.owd_medio = rtt_ + (fila_edad - fila_rtt);
    }
  }
  // ── la dispersión de cada cuadro (Frame Stats) ──────────────────────────────
  // Sólo los cuadros que el cuello de botella abrió: uno que llega tan espaciado como salió no
  // cruzó nada más angosto que el propio emisor, y su cociente mide al emisor. El cliente cuenta
  // bytes de RTP; el cuello de botella cobra además las cabeceras, en la proporción del cuadro.
  for (const TrenRecibido& r : trenes_rx_) {
    const auto it = enviados_.find(r.indice);
    if (it == enviados_.end() || r.bytes == 0) continue;
    const CuadroEnviado& e = it->second;
    const double a_cable = e.bytes_rtp > 0 ? e.bytes_cable / e.bytes_rtp : 1.0;
    const double lapso = static_cast<uint32_t>(r.t_ultimo_us - r.t_primero_us) / 1e6;
    const double emision = (e.t_ultimo_us - e.t_primero_us) / 1e6;
    if (lapso > kDispersionMinima * emision) m.trenes.push_back({r.bytes * a_cable, lapso});
  }
  trenes_rx_.clear();

  m.paridad_emitida = datos_tx_ > 0 ? (paquetes_tx_ - datos_tx_) / datos_tx_ : 0.0;
  m.rafaga_perdida = cuadros_con_perdida_ > 0 ? perdidos_en_cuadros_ / cuadros_con_perdida_ : 0.0;
  m.muestras = static_cast<int>(paquetes_tx_);

  bits_emitidos_ = bits_datos_ = bits_llegados_ = 0;
  paquetes_tx_ = datos_tx_ = paquetes_perdidos_ = 0;
  cuadros_con_perdida_ = perdidos_en_cuadros_ = 0;
  return true;
}

}  // namespace gamestream
}  // namespace estab
