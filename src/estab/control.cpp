#include "control.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

namespace estab {

const char* nombre(Variante v) {
  switch (v) {
    case Variante::hoy:      return "hoy";
    case Variante::remiendo: return "remiendo";
    case Variante::relleno:  return "relleno";
    case Variante::tren:     return "tren";
  }
  return "?";
}

Variante variante_por_nombre(const std::string& s) {
  if (s == "remiendo") return Variante::remiendo;
  if (s == "relleno")  return Variante::relleno;
  if (s == "tren")     return Variante::tren;
  return Variante::hoy;
}

int paridad_de_bloque(int k, double perdida, double rafaga, const Parametros& p, int minimo) {
  if (minimo < 0) minimo = p.fec_minimo_paquetes;
  const int techo = std::min({p.gf_shards - k, (k * 255 + 99) / 100,
                              std::max(minimo, static_cast<int>(std::ceil(k * p.fec_max)))});
  rafaga = std::max(1.0, rafaga);
  // Los tropiezos empiezan en cada paquete con probabilidad q, y cada uno se lleva un largo
  // geométrico de media `rafaga`: la permanencia en el estado malo de un canal de Gilbert-Elliott.
  // El bloque se pierde si lo perdido suma más que m. Un largo fijo ignora la cola de las ráfagas
  // largas: con la ráfaga medida en uno y algo, deja bloques sin cubrir el tropiezo de tres.
  const double q = std::min(0.5, std::max(1e-6, perdida) / rafaga);
  const double r = 1.0 - 1.0 / rafaga;                   // seguir perdiendo dentro de un tropiezo
  for (int m = minimo; m < techo; ++m) {
    const int n = k + m;
    // P(perdidos ≤ m) = Σ_j P(j tropiezos) · P(sus largos suman ≤ m), con j binomial(n, q) y la
    // suma de j geométricos una binomial negativa: P(suma = t) = C(t−1, j−1)·(1−r)^j·r^(t−j)
    double lj = n * std::log1p(-q);                      // log P(ningún tropiezo)
    double acum = std::exp(lj);
    for (int j = 1; j <= std::min(n, m); ++j) {
      lj += std::log(double(n - j + 1) / j) + std::log(q) - std::log1p(-q);
      double suma = 1.0;                                  // con largo 1, j tropiezos son j perdidos
      if (r > 0) {
        suma = 0;
        for (int t = j; t <= m; ++t)
          suma += std::exp(std::lgamma(t) - std::lgamma(j) - std::lgamma(t - j + 1) +
                           j * std::log1p(-r) + (t - j) * std::log(r));
      }
      acum += std::exp(lj) * suma;
    }
    if (1.0 - acum < p.paridad_falla_bloque) return m;
  }
  return std::max(0, techo);
}

const char* nombre(Motivo m) {
  switch (m) {
    case Motivo::arranque:      return "arranque";
    case Motivo::quieto:        return "quieto";
    case Motivo::sin_evidencia: return "sin_evidencia";
    case Motivo::fila_crece:    return "fila_crece";
    case Motivo::fila_alta:     return "fila_alta";
    case Motivo::salto_perdida: return "salto_perdida";
    case Motivo::tanteando:     return "tanteando";
    case Motivo::reanclado:     return "reanclado";
    case Motivo::sondeando:     return "sondeando";
    case Motivo::topeado:       return "topeado";
  }
  return "?";
}

// ── la sintonía, derivada ────────────────────────────────────────────────────
// Cap. 4: k_g y k_p no son constantes elegidas, son consecuencia de pedirle al lazo un
// margen de fase y una mezcla ρ. De las mismas dos cantidades salen las predicciones que
// el capítulo 10 usa para tratar de matar el modelo.
//
// El lazo abierto tiene DOS integraciones, y es la clave de todo el capítulo 4:
//
//   la fila integra el exceso de bitrate ........ 1/(C·s)
//   la ley integra su propia corrección ......... (k_g·s + k_p)/s
//   y el reporte vuelve tarde ................... e^{−sτ}
//
//   L(s) = (k_g·s + k_p)·e^{−sτ} / (C·s²)
//
// Dos integraciones son −180° de entrada: sin el término de crecimiento no queda nada de
// margen, y por eso la derivada no es una mejora opcional sino la condición de que el
// lazo exista. Con u = ω·τ y z = k_p/k_g normalizado:
//
//   |L(u)| = kg_n·√(u² + z²) / u²        fase(u) = −π + atan(u/z) − u
constexpr double kPi = 3.14159265358979323846;

Sintonia derivar(const Parametros& p) {
  Sintonia s;
  const double phi = std::atan(p.rho);
  s.wc_tau = phi - p.margen_fase;                      // ω_c·τ
  s.kg_n = s.wc_tau / std::sqrt(1.0 + 1.0 / (p.rho * p.rho));
  s.kp_n = s.kg_n * s.wc_tau / p.rho;

  const double z = s.kp_n / s.kg_n;                    // el codo del adelanto

  // ω_180: donde la corrección vuelve media vuelta tarde. Pedir fase = −π deja
  // atan(u/z) = u, que no tiene forma cerrada — bisección.
  auto exceso = [&](double u) { return std::atan(u / z) - u; };
  double lo = s.wc_tau, hi = kPi / 2 + 1.0;            // exceso(hi) < 0 siempre
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (exceso(mid) > 0) lo = mid; else hi = mid;
  }
  s.w180_tau = 0.5 * (lo + hi);

  // A ω_180 la corrección vuelve más chica, y ese factor es cuánto se puede subir la
  // ganancia antes de que el lazo se sostenga oscilando. Es un número verificable.
  auto magnitud = [&](double u) { return s.kg_n * std::sqrt(u * u + z * z) / (u * u); };
  s.margen_ganancia = 1.0 / magnitud(s.w180_tau);
  s.periodo_tau = 2.0 * kPi / s.w180_tau;
  return s;
}

// ── utilidades de ventana ────────────────────────────────────────────────────
void Control::podar(std::deque<Muestra>& v, double t, double ventana) {
  while (!v.empty() && v.front().t < t - ventana) v.pop_front();
}

double Control::percentil(const std::deque<Muestra>& v, int q) {
  if (v.empty()) return 0;
  std::vector<double> a;
  a.reserve(v.size());
  for (const auto& m : v) a.push_back(m.v);
  const size_t k = std::min(a.size() - 1,
                            static_cast<size_t>(q / 100.0 * a.size()));
  std::nth_element(a.begin(), a.begin() + k, a.end());
  return a[k];
}

bool Control::ancho_por_regresion(double& ancho) const {
  if (trenes_.size() < 4) return false;
  // La recta del lapso contra el tamaño, sobre los trenes que queden. Devuelve false si los
  // cuadros no tuvieron tamaños distintos: con todos iguales, la pendiente no está determinada.
  auto recta = [](const std::vector<const Tren*>& v, double& pendiente, double& ordenada) {
    if (v.size() < 4) return false;
    double sx = 0, sy = 0;
    for (const Tren* x : v) { sx += x->bits; sy += x->lapso; }
    const double n = static_cast<double>(v.size());
    const double mx = sx / n, my = sy / n;
    double sxx = 0, sxy = 0;
    for (const Tren* x : v) {
      sxx += (x->bits - mx) * (x->bits - mx);
      sxy += (x->bits - mx) * (x->lapso - my);
    }
    if (sxx <= 1e-9 * mx * mx * n) return false;
    pendiente = sxy / sxx;
    ordenada = my - pendiente * mx;
    return true;
  };
  std::vector<const Tren*> v;
  v.reserve(trenes_.size());
  for (const Tren& x : trenes_) v.push_back(&x);
  double pendiente = 0, ordenada = 0;               // segundos por bit, y la ráfaga
  if (!recta(v, pendiente, ordenada)) return false;
  // Lo que demora a un cuadro en el camino —las tandas de aire de un Wi-Fi, un hilo que se
  // despertó tarde— sólo le suma lapso, nunca le resta: el caño es el borde de abajo de la nube,
  // no su medio. Se ajusta otra vez con los trenes que quedaron por debajo de la recta.
  for (int i = 0; i < p_.tren_envolvente; ++i) {
    std::vector<const Tren*> abajo;
    for (const Tren* x : v) if (x->lapso <= ordenada + pendiente * x->bits) abajo.push_back(x);
    double p2 = 0, o2 = 0;
    if (!recta(abajo, p2, o2)) break;
    v.swap(abajo);
    pendiente = p2;
    ordenada = o2;
  }
  if (pendiente <= 0) return false;                 // el lapso no crece con el tamaño
  const double c = 1.0 / pendiente;
  // Por encima del techo declarado es ruido de medición, no un descubrimiento.
  if (c > p_.bitrate_max * 4) return false;
  ancho = std::min(c, p_.bitrate_max);
  return true;
}

double Control::valor_en(const std::deque<Muestra>& v, double t) {
  if (v.empty()) return 0;
  const Muestra* mejor = &v.front();
  double dist = std::fabs(v.front().t - t);
  for (const auto& m : v) {
    const double d = std::fabs(m.t - t);
    if (d < dist) { dist = d; mejor = &m; }
  }
  return mejor->v;
}

// ── el filtro del crecimiento ────────────────────────────────────────────────
// Cap. 9: NO es fijo. El mejor valor se corre a la izquierda cuando sube el viaje, y un
// valor fijo es malo en los dos extremos.
double Control::constante_filtro(double rtt) const {
  return rtt > p_.filtro_rtt_corte ? p_.filtro_rapido : p_.filtro_lento;
}

// Cap. 6: promedio mientras haya pocas muestras, ajuste de recta a partir de ~3.
// El residuo del ajuste es literalmente el ancho de la nube del medidor, y de ahí sale
// la zona muerta — que así se angosta sola cuando hay más evidencia.
double Control::pendiente(double& residuo_rms) const {
  residuo_rms = 0;
  const size_t n = filas_.size();
  if (n < 2) return 0;

  if (n < static_cast<size_t>(p_.muestras_para_recta)) {
    // sin evidencia para una recta: diferencia simple sobre el lapso
    const double dt = filas_.back().t - filas_.front().t;
    return dt > 1e-9 ? (filas_.back().v - filas_.front().v) / dt : 0.0;
  }

  double st = 0, sv = 0;
  for (const auto& m : filas_) { st += m.t; sv += m.v; }
  const double mt = st / n, mv = sv / n;
  double num = 0, den = 0;
  for (const auto& m : filas_) {
    num += (m.t - mt) * (m.v - mv);
    den += (m.t - mt) * (m.t - mt);
  }
  const double b = den > 1e-12 ? num / den : 0.0;
  const double a = mv - b * mt;

  double ss = 0;
  for (const auto& m : filas_) {
    const double e = m.v - (a + b * m.t);
    ss += e * e;
  }
  // n-2 grados de libertad: se estimaron dos parámetros
  residuo_rms = n > 2 ? std::sqrt(ss / (n - 2)) : 0.0;
  return b;
}

// ── las estimaciones que resintonizan el lazo ────────────────────────────────
// Cap. 5 y cap. 3: se mueven mucho más despacio que el lazo, o la resintonía misma lo
// desestabiliza. Por eso percentiles sobre ventanas largas y no promedios rápidos.
void Control::estimar(const Medicion& m, double fila, double dref) {
  // Con fila hay información: si no hay cola, lo que sale es lo que pusimos y no dice nada
  // sobre la capacidad.
  //
  // Pero hay una segunda puerta, y es la que salva al lazo cuando la primera miente. Si de
  // lo que se pone en el cable llega bastante menos, algo se quedó en el camino: o hay cola
  // o hay descarte, y en los dos casos el caño está lleno. Entonces la tasa recibida ES una
  // medición de capacidad — y esta conclusión NO PASA POR EL PISO DEL RETARDO.
  //
  // Es exactamente el caso que rompía al lazo. Contra un flujo que nunca deja drenar la
  // cola, el piso se envenena, la fila medida baja de `dref`, y con la puerta vieja sola no
  // se juntaba ni una muestra de capacidad. Sin muestras, la rama de abajo empujaba Ĉ hacia
  // R: el techo se iba subiendo solo justo cuando hacía falta que no se moviera.
  // Dos cuidados, y los dos costaron una corrida cada uno:
  //
  //   · Hay que estar mandando de verdad. Si el codificador entregó una fracción del
  //     presupuesto, lo que llega mide al codificador y no al caño — y Ĉ aprendería de ahí
  //     un caño que no es. No se usa el indicador de la variante porque éste es un fondo
  //     para todas, incluida la que no lo tiene.
  //   · Hay que comparar contra lo que se emitió UN VIAJE ANTES. El informe llega tarde,
  //     así que la tasa que trae corresponde al pasado. Contra la tasa de ahora, toda
  //     subida parece pérdida — y en escena quieta, que sube y baja todo el tiempo, eso
  //     derrumbaba el presupuesto a un cuarto del caño.
  emitidas_.push_back({m.t, m.tasa_emitida});
  podar(emitidas_, m.t, 2.0);
  const double emitida_de_entonces = valor_en(emitidas_, m.t - m.rtt);
  emitida_entonces_ = emitida_de_entonces;

  const double a = p_.egreso_constante > 0 ? std::min(1.0, p_.Tc / p_.egreso_constante) : 1.0;
  if (m.tasa_recibida > 0) {
    recibida_suave_ += a * (m.tasa_recibida - recibida_suave_);
    emitida_suave_ += a * (emitida_de_entonces - emitida_suave_);
  }
  const bool mandamos_de_verdad = emitida_de_entonces > p_.app_umbral * R_;
  bool se_queda_en_el_camino =
      mandamos_de_verdad &&
      recibida_suave_ < emitida_suave_ * (1.0 - p_.margen_egreso);
  // Si la medición trae la entrega de unos mismos cuadros, es esa la que dice si el caño frena:
  // no mezcla la variación del codificador con la del caño.
  if (p_.entrega_llena > 0 && m.entrega > 0) {
    entrega_suave_ = entrega_suave_ > 0 ? entrega_suave_ + (p_.Tc / 0.5) * (m.entrega - entrega_suave_)
                                        : m.entrega;
    se_queda_en_el_camino = mandamos_de_verdad && entrega_suave_ < p_.entrega_llena;
  }
  caño_lleno_ = se_queda_en_el_camino;

  // Lo que llega sólo dice cuánto da el caño si se mandó de verdad: con la imagen quieta, lo que
  // llega es el audio, y aprendido como caño llevaba la capacidad al piso.
  if (m.tasa_recibida > 0 && ((fila > dref && mandamos_de_verdad) || se_queda_en_el_camino)) {
    tasas_.push_back({m.t, m.tasa_recibida});
  }
  podar(tasas_, m.t, p_.ventana_capacidad);

  const double medida = percentil(tasas_, p_.percentil_capacidad);  // hacia el lado seguro
  if (C_forzada_ > 0) {
    C_est_ = C_forzada_;
  } else if (medida > 0) {
    C_est_ += (p_.Tc / p_.constante_capacidad) * (medida - C_est_);
  } else if (R_ > C_est_ && !app_limited_ && !se_queda_en_el_camino) {
    // Sin cola sabemos que la capacidad es al menos lo que estamos mandando. Con dos
    // guardas, porque las dos veces que esta rama se equivocó fue creyendo que pasó por el
    // caño algo que no pasó:
    //   · si el codificador entregó una fracción del presupuesto, no salió nada y no hay
    //     nada que concluir;
    //   · si salió pero no llegó, tampoco: eso no es caño libre, es caño lleno.
    C_est_ += (p_.Tc / 1.5) * (R_ - C_est_);
  }
  C_est_ = std::max(p_.bitrate_min, C_est_);

  viajes_.push_back({m.t, m.rtt});
  podar(viajes_, m.t, p_.ventana_capacidad);
  const double viaje = percentil(viajes_, p_.tau_del_piso ? p_.percentil_viaje_dref : p_.percentil_viaje);
  const double cuadro = 1.0 / p_.fps;
  tau_est_ = viaje + cuadro + p_.Tc / 2.0 + constante_filtro(viaje);
}

// ── el chequeo de coherencia (cap. 5) ────────────────────────────────────────
// Si bajar no mejora, no era una fila: cambió el camino, y el piso guardado está podrido.
// La prueba se hace de verdad — se anota dónde estaba el presupuesto cuando empezó el
// descenso y se compara unos τ después.
bool Control::chequeo_coherencia(double fila, double dref, double delta, double dR) {
  if (ingenuo_) return false;

  // cuatro condiciones, y cada una descarta un caso distinto
  const bool bien_por_encima = fila > dref + p_.coh_margen;
  const bool clavada = std::fabs(delta) < p_.coh_clavada;
  const bool bajamos = dR < 0;

  if (!prueba_.activa && bien_por_encima && bajamos) {
    prueba_ = {true, t_, R_ - dR, fila};
    return false;
  }
  if (prueba_.activa && fila <= dref) { prueba_.activa = false; sospecha_ = 0; return false; }
  if (!prueba_.activa) return false;

  if (t_ - prueba_.t < std::max(1.2, 3.0 * tau_est_)) return false;

  const double bajo   = (prueba_.R - R_) / std::max(1.0, prueba_.R);
  const double mejoro = (prueba_.fila - fila) / std::max(1e-9, prueba_.fila);
  prueba_.activa = false;

  // bajamos de verdad, el retardo no aflojó, no hay pérdida que lo explique, y está
  // clavado en vez de seguir creciendo (que sería una caída de capacidad real)
  const bool lo_es = bajo > 0.10 && mejoro < 0.15 && perdida_suave_ < 0.02 &&
                     clavada && bien_por_encima;
  if (!lo_es) { sospecha_ = 0; return false; }
  if (++sospecha_ < p_.coh_ciclos) return false;

  sospecha_ = 0;
  piso_.clear();
  viajes_.clear();
  piso_fresco_ = t_;
  R_ = R_sano_;
  filas_.clear();
  return true;
}

// ── el sondeo del piso (cap. 5) ──────────────────────────────────────────────
// Si hace rato que ninguna medición toca el fondo, la fila nunca se vació y el mínimo de
// la ventana se la tragó. Se recorta un poco, se deja drenar, y se relee.
void Control::sondeo_del_piso(double t) {
  if (ingenuo_) return;
  // La variante por dispersión no lo necesita: el multiplicador de destino deja slack
  // permanente y las colas se drenan solas. SQP señala justamente que frenar el envío
  // cada diez segundos —el ProbeRTT de BBR, de donde salió este mecanismo— es
  // especialmente problemático para streaming interactivo.
  if (p_.variante == Variante::tren) return;
  if (sondeando_) {
    if (t > sondeo_hasta_) sondeando_ = false;
    return;
  }
  const int ciclos_quieto = static_cast<int>(2.5 / p_.Tc);
  if (t - piso_fresco_ > p_.sondeo_cada && quieto_ >= ciclos_quieto) {
    sondeando_ = true;
    sondeo_hasta_ = t + p_.sondeo_dura;
    quieto_ = 0;
  }
}

// ── subir es tantear (cap. 4) ────────────────────────────────────────────────
// La ley no puede subir sola: sin fila no hay señal. La S tímida es la que menos pico de
// latencia deja en todo el rango de recuperación, y en un juego eso es lo que se siente.
// Cuando un techo recorta el presupuesto, la curva de subida no puede seguir avanzando por
// debajo como si nada: seguía hacia su ancla —el valor con el que se arrancó, por ejemplo— y el
// día que el techo desaparecía el presupuesto saltaba de una al valor de la curva. Con 4 Mbit/s
// de caño, de 3,3 a 9,3 en un ciclo. El ancla baja al techo y la curva vuelve a empezar desde ahí.
void Control::no_enrollar(double techo) {
  if (std::getenv("ESTAB_DEBUG_ALARMA")) std::fprintf(stderr, "no_enrollar t=%.2f R=%.0f techo=%.0f C=%.0f\n", t_, R_, techo, C_est_);
  ancla_ = std::min(ancla_, techo);
  en_curva_ = false;
}

void Control::subir_tanteando(double dref, double fila) {
  if (!en_curva_) {
    en_curva_ = true;
    curva_t_ = 0;
    curva_R0_ = R_;
    if (ancla_ < R_) ancla_ = R_;
  }
  curva_t_ += p_.Tc;
  const double T = p_.curva_duracion, a = 9.2 / T;
  const double sg = 1.0 / (1.0 + std::exp(-a * (curva_t_ - T / 2)));
  const double s0 = 1.0 / (1.0 + std::exp(a * T / 2));
  double objetivo = curva_R0_ + (ancla_ - curva_R0_) * (sg - s0) / (1.0 - 2.0 * s0);
  // Pasada la curva sigue explorando: el ancla es una memoria, no una pared. Primero despacio
  // y después cada vez más rápido, como la parte convexa de CUBIC: cuanto más tiempo pasa sin
  // que aparezca fila, menos vale el ancla como estimación del caño. Sin el tren no hay otra
  // forma de saber que el caño creció, y a paso constante pasar de 4 a 12 Mbit/s llevaba minutos.
  if (curva_t_ > T) {
    const double x = curva_t_ - T;
    // La aceleración cuenta desde que el retardo vuelve a valer: el tiempo a ciegas no se acumula,
    // y al vencer no salta de golpe a lo que habría alcanzado
    const double xc = std::min(x, std::max(0.0, t_ - ciego_hasta_));
    objetivo += ancla_ * (p_.curva_exploracion * x + p_.curva_convexa * xc * xc * xc);
  }
  // Hasta la primera fila de la sesión nada dice dónde está el caño, y el ancla es sólo el valor
  // con que se arrancó, que se eligió bajo a propósito. Mientras tanto se sube en proporción a lo
  // que ya se manda, como el arranque de BBR: con Sunshine de verdad y un caño de 12 Mbit/s,
  // subiendo desde el ancla la acción tardaba treinta y cinco segundos en llegar al caño.
  //
  // Y lo mismo cuando hace rato que no hay fila: el caño pudo haber crecido, y el ancla es la
  // memoria de uno más angosto. Es el sondeo acelerado en la recuperación que pedía el panel del
  // 29/8. Con la curva sola, de un caño de 4 a uno de 12 Mbit/s, llegar tardaba quince segundos.
  // Después de una cola ajena que terminó sola, la calma que sigue es corta —la película vuelve en
  // segundos— y no se puede esperar a que pase el plazo: se explora desde ya.
  const bool sin_fila = (p_.reexplorar_s > 0 && t_ - t_ultima_fila_ > p_.reexplorar_s) || tras_ajena_;
  if (p_.arranque_rapido > 0 && (!arranque_hecho_ || sin_fila)) {
    objetivo = std::max(objetivo, R_ * (1.0 + p_.arranque_rapido * p_.Tc));
  }
  R_ = std::max(R_, std::min(p_.bitrate_max, objetivo));
  (void)dref; (void)fila;
}

// ── la alarma: la caída de caño, sin esperar al paso ─────────────────────────
// Sin pérdida hace falta R ≤ C_nuevo·(1 + B/T_d), con B el buffer del router y T_d lo que tarda el
// lazo en ver y actuar. Con el paso de 100 ms y el ida y vuelta suavizado, T_d andaba en 0,25 a
// 0,3 s, y bajar de 12 a 4 Mbit/s desbordaba un buffer de 280 ms antes de que el lazo se enterara.
// Con cada muestra del ida y vuelta —el host hace ping cada 25 ms— T_d baja a un ida y vuelta más
// una muestra.
//
// El escalón es la fila creciendo más de lo que ninguna otra cosa hace crecer: 15 ms en 50 ms es
// mandar más de 1,4 veces el caño. La cola que arma una descarga crece de a un paquete por viaje.
// Y el destino no es una fracción del caño sino lo que además vacía la cola en un tiempo dado,
// como el drenaje de Pudica (NSDI 2024): 0,85 del caño menos la cola sobre el tiempo de vaciado.
bool Control::alarma(double t, double fila, double crecimiento, double recibida) {
  if (p_.alarma_escalon <= 0 || recibida <= 0 || ingenuo_) return false;
  if (crecimiento < p_.alarma_escalon) return false;
  if (fila < std::max(p_.alarma_fila, 1.3 * dref_ultimo_)) return false;
  // El caño nuevo: lo que llega todavía es el viejo —mide cuadros que salieron antes—, pero la
  // pendiente de la fila ya es el nuevo, porque crece a (R−C)/R. De 12 a 4 Mbit/s, lo que llegaba
  // decía 9 y la pendiente decía 5,5.
  const double pendiente = std::min(0.9, std::max(0.0, crecimiento / 0.05));
  // Con la imagen quieta lo que llega es el audio: no dice nada del caño
  const double caño = std::min({C_est_, app_limited_ ? C_est_ : recibida, R_ * (1.0 - pendiente)});
  const double destino = std::max({p_.bitrate_min, 0.25 * caño,
                                   caño * (p_.alarma_factor - fila / p_.alarma_vaciado)});
  // Una alarma cada dos segundos, como los saltos; pero la puede disparar aunque el salto del paso
  // haya disparado recién, si va más abajo: ese mira lo que llega, que en ese momento todavía es el
  // caño viejo, y con la guarda común la alarma quedaba bloqueada justo cuando hacía falta.
  if (t - t_alarma_ < 2.0) return false;
  if (R_ <= destino / 0.9) return false;
  antes_alarma_ = {R_, ancla_, t_alarma_, t_perdida_, t_salto_rapido_, en_curva_, arranque_hecho_, paridad_minima_};
  alarma_abierta_ = true;
  t_alarma_ = t;
  t_perdida_ = t;
  paridad_minima_ = p_.fec_minimo_paquetes;
  if (std::getenv("ESTAB_DEBUG_ALARMA")) std::fprintf(stderr, "alarma t=%.2f fila=%.3f crec=%.3f R=%.0f destino=%.0f\n", t, fila, crecimiento, R_, destino);
  R_ = destino;
  no_enrollar(caño);
  t_salto_rapido_ = t;
  arranque_hecho_ = true;
  return true;
}

bool Control::desmentir_alarma(double t, double fila) {
  if (!alarma_abierta_) return false;
  // Sólo la muestra de enseguida puede desmentirla: pasado ese lapso, lo que pase ya es otra cosa
  if (t - t_alarma_ > p_.alarma_desmentir_s) { alarma_abierta_ = false; return false; }
  if (fila >= 0.5 * dref_ultimo_) return false;
  alarma_abierta_ = false;
  const AntesDeAlarma& a = antes_alarma_;
  R_ = a.R; ancla_ = a.ancla; t_alarma_ = a.t_alarma; t_perdida_ = a.t_perdida;
  t_salto_rapido_ = a.t_salto_rapido; en_curva_ = a.en_curva; arranque_hecho_ = a.arranque_hecho;
  paridad_minima_ = a.paridad_minima;
  return true;
}

// ── el lazo ──────────────────────────────────────────────────────────────────
Control::Control(const Parametros& p) : p_(p), s_(derivar(p)) {
  reiniciar(p.bitrate_max);
}

void Control::reiniciar(double negociado) {
  // Cap. 5: arrancar abajo y subir cuesta unos segundos de imagen más pobre; arrancar
  // arriba y bajar cuesta imagen ROTA más el riesgo de disparar el salto de emergencia.
  // No es simétrico.
  R_ = std::max(p_.bitrate_min, std::min(p_.bitrate_max, negociado * p_.arranque_fraccion));
  R_sano_ = R_;
  ancla_ = R_;
  f_ = p_.fec_min;
  C_est_ = negociado;
  tau_est_ = 0.3;
  piso_.clear(); viajes_.clear(); tasas_.clear(); filas_.clear(); minimos_.clear(); emitidas_.clear();
  presupuestos_.clear();
  piso_vigente_ = 0; piso_fresco_ = 0; competidor_ = false; t_sin_vaiven_ = -1e9; perdida_suave_ = 0; perdida_pico_ = 0; rafaga_ = 0;
  t_tanda_ = -1e9; en_tanda_ = false; recurrente_ = false; t_arranque_ = -1;
  t_perdida_ = -1e9; paridad_minima_ = p_.fec_minimo_paquetes; fps_ = p_.fps;
  fila_prev_ = 0; fila_cruda_prev_ = 0; fila_cruda_prev2_ = 0; habia_fila_ = false; primera_ = true;
  en_curva_ = false; sondeando_ = false; quieto_ = 0; sospecha_ = 0;
  prueba_ = {};
  C_sano_ = negociado; episodio_ = {}; tras_ajena_ = false;
  app_limited_ = false;
  caño_lleno_ = false;
  ancho_tren_ = 0;
  trenes_.clear();
  t_ancho_ = 0;
  t_salto_rapido_ = -1e9;
  llega_menos_seguidos_ = 0;
  entrega_suave_ = 0;
  entregado_.clear(); t_crucero_ = 0; explorando_ = false;
  t_alarma_ = -1e9;
  arranque_hecho_ = false;
  t_perdida_sin_fila_ = -1;
  ciego_hasta_ = -1e9;
  eficiencia_cod_ = 1;
  fila_sobre_objetivo_ = 0;
  pedido_prev_ = 0;
  emitida_suave_ = recibida_suave_ = 0;
  t_ultima_fila_ = 0;
  t_ = 0;
}

Decision Control::paso(const Medicion& m) {
  t_ = m.t;
  Decision d;
  presupuestos_.push_back({m.t, R_});
  podar(presupuestos_, m.t, 5.0);
  // Del caño que vació la cola, lo que se puede usar: lo que deja vaciar en vaciado_drenaje una cola
  // como la de recién. Un corte de h segundos mandando R deja R·h en la cola, que se vacía a C − R:
  // R ≤ C·T/(h + T). Usar el caño entero deja las colas de un corte encimándose con las del siguiente.
  const double vaciado_util = m.vaciado > 0
      ? std::min(p_.tren_destino, p_.vaciado_drenaje / (m.vaciado_subida + p_.vaciado_drenaje)) * m.vaciado
      : 0.0;
  // Un corte del aire sube la fila casi tanto como pasa el tiempo —el caño no entrega nada—; la cola
  // que arma el propio lazo, al ritmo de lo que se manda de más, una fracción de eso. Sólo una cola
  // que subió como un corte, en un camino con ruido, dice algo que la ley no sabe.
  const bool corte = p_.vaciado_episodio > 0 && m.vaciado > 0 && m.ruido > 0.002 &&
                     m.vaciado_altura >= p_.vaciado_rampa * std::max(1e-3, m.vaciado_subida);

  // ── 0. ¿el codificador usó el presupuesto? ─────────────────────────────────
  // Es lo primero que hay que saber, porque decide si lo que sigue es evidencia o ruido.
  // El host lo sabe sin ayuda del cliente: son los bits que él mismo puso en el cable.
  if (p_.variante != Variante::hoy && p_.variante != Variante::relleno) {
    app_limited_ = m.tasa_emitida > 0 && m.tasa_emitida < p_.app_umbral * R_;
  } else {
    app_limited_ = false;   // en `relleno` nunca lo está: el pacer completa con dummies
  }

  // La muestra por dispersión del tren de paquetes (SQP). No depende de la tasa promedio
  // sino de cómo se abren los paquetes de UN cuadro al cruzar el cuello de botella, así
  // que sigue sirviendo aunque el cuadro sea chico.
  // Lo mismo, sólo con los cuadros de este informe: el caño que cruzaron los últimos cuadros.
  // Es ruidoso, pero es lo primero que muestra que el caño se angostó —antes que cualquier
  // promedio de tasas—, y con los cuadros espaciados a κ·R mide el caño nuevo apenas cae por
  // debajo de lo que se manda.
  double bits_recientes = 0, lapso_reciente = 0;
  int trenes_recientes = 0;
  if (p_.variante == Variante::tren &&
      (!m.trenes.empty() || (m.tren_lapso > 1e-6 && m.tren_bytes > 0))) {
    // Un cuadro de dos o tres paquetes no tiene dispersión que medir: lo que se lee es
    // el ruido del reloj, no el cuello de botella. SQP lo reconoce y es su límite
    // conocido. Con menos de este piso de paquetes, la muestra se descarta.
    auto sumar = [&](double bytes, double lapso) {
      const double paquetes = bytes / static_cast<double>(p_.mtu + p_.cabeceras);
      if (lapso > 1e-6 && paquetes >= p_.tren_min_paquetes) {
        trenes_.push_back({m.t, bytes * 8.0, lapso});
        // n paquetes abren n − 1 huecos
        bits_recientes += bytes * 8.0 * (paquetes - 1.0) / paquetes;
        lapso_reciente += lapso;
        ++trenes_recientes;
      }
    };
    if (!m.trenes.empty()) {
      for (const auto& tr : m.trenes) sumar(tr.bytes, tr.lapso);
    } else {
      sumar(m.tren_bytes, m.tren_lapso);
    }
    // Con un cuadro solo, "bits sobre lapso" sería el ancho. Pero si el cuello de botella
    // deja pasar B bytes de golpe cuando está ocioso —el token bucket de un router, la
    // agregación de Wi-Fi—, un tren de S bytes no tarda S/C sino (S − B)/C, y un cuadro
    // chico que entra entero en la ráfaga no tarda nada: el cociente da un caño infinito.
    // Pasa justo en la escena quieta, donde los cuadros son chicos y el enlace está ocioso.
    //
    // Ajustando el lapso contra el tamaño sobre varios cuadros, la pendiente es 1/C y la
    // ráfaga queda en la ordenada, así que no contamina la estimación. Si todos los cuadros
    // cupieron en la ráfaga no hay pendiente que ajustar, y el ancho no se mueve: no hay
    // evidencia, y sin evidencia no se sube.
    while (!trenes_.empty() && m.t - trenes_.front().t > p_.tren_ventana) trenes_.pop_front();
    // Baja de golpe y sube despacio. Bajar es lo que protege; los saltos para arriba son casi
    // siempre del cuello de botella: si deja pasar ráfagas, cada vez que la cola se vacía
    // recupera el crédito, los cuadros pasan en parte de golpe y el ancho sale inflado un par
    // de segundos, justo cuando la ley ve la fila vacía y sube.
    double ancho = 0;
    if (ancho_por_regresion(ancho)) {
      // El caño entrega por lo menos lo que acaba de llegar. Un cuadro que el cliente procesó
      // tarde —sin CPU, los paquetes se leen de a tandas— tiene un lapso enorme, y la regresión
      // con pocas muestras daba un ancho casi nulo: bajando de golpe, una sola muestra así tiraba
      // el presupuesto al mínimo con el caño entregando cuatro veces más.
      if (m.tasa_recibida > 0) ancho = std::max(ancho, m.tasa_recibida);
      const double dt = std::max(0.0, m.t - t_ancho_);
      if (p_.tren_subida_max > 0 && ancho_tren_ > 0 && ancho > ancho_tren_) {
        ancho = std::min(ancho, ancho_tren_ * (1.0 + p_.tren_subida_max * dt));
      }
      ancho_tren_ = ancho;
      t_ancho_ = m.t;
    }
  }
  // Una medida vieja no es techo. Con el presupuesto bajo y muchos cuadros por segundo, los
  // cuadros quedan por debajo del mínimo de paquetes que tiene dispersión medible y no entran
  // muestras nuevas; sin vencimiento, el techo de la última medida —la de la caída, por
  // ejemplo— frenaba al presupuesto por debajo de un caño que ya había vuelto, y el presupuesto
  // bajo mantenía los cuadros chicos. Vencida, manda la ley con su techo demostrado.
  if (p_.variante == Variante::tren && ancho_tren_ > 0 && m.t - t_ancho_ > p_.tren_vigencia) {
    ancho_tren_ = 0;
  }

  // ── 1. el piso del retardo: la referencia de toda la medición ──────────────
  // Cap. 6: mínimo sobre 10 s, mismo valor que BBR y mismas razones. El reloj del
  // cliente tiene un desfase desconocido, así que el nivel absoluto no significa nada;
  // lo único utilizable es cuánto se despegó del mínimo reciente.
  piso_.push_back({m.t, m.owd_min});
  podar(piso_, m.t, p_.ventana_piso);
  piso_vigente_ = piso_.front().v;
  for (const auto& x : piso_) piso_vigente_ = std::min(piso_vigente_, x.v);
  if (m.owd_min < piso_vigente_ + 0.003) piso_fresco_ = m.t;
  const double piso_minimo = piso_vigente_;
  // Con otro flujo llenando el caño la cola no se vacía nunca, y el mínimo de diez segundos queda
  // donde lo deja el valle más hondo del otro: un TCP que retrocede mucho baja el piso, el lazo cree
  // suya la cola del otro y le cede el caño; uno que retrocede poco lo deja alto y el lazo compite.
  // Lo que delata al otro no es el nivel sino el vaivén: su diente de sierra mueve el retardo de la
  // ventana bastante más que la fila propia, que el lazo mantiene en decenas de milisegundos. Con
  // ese vaivén el piso pasa a ser un percentil bajo de la ventana, que no depende de cuán hondos son
  // los valles. Un vaivén propio —una caída, un salto de ruta— dura lo que tarda el lazo en vaciar
  // su fila; el del otro, lo que dura el otro: sólo cuenta el que se sostiene competidor_sostenido.
  // La fila propia se mueve con su objetivo, que crece con el viaje: en un camino largo el umbral es
  // competidor_dref veces ese objetivo, y el vaivén es propio salvo que algo diga lo contrario. Lo
  // dice la pérdida —un flujo que se guía por ella llena el buffer hasta desbordarlo— en los
  // últimos competidor_perdida_s; o, si el lazo ya quedó tan abajo que ni pierde, el percentil bajo
  // despegado del mínimo más que el objetivo: la fila propia no ocupa tres cuartos de la ventana.
  competidor_ = false;
  if (p_.competidor_umbral > 0 && piso_.size() >= 10) {
    std::vector<double> v;
    v.reserve(piso_.size());
    for (const auto& x : piso_) v.push_back(x.v);
    const size_t j = v.size() * 9 / 10;
    std::nth_element(v.begin(), v.begin() + j, v.end());
    const double umbral = std::max(p_.competidor_umbral, p_.competidor_dref * dref_ultimo_);
    if (v[j] - piso_vigente_ <= umbral) {
      t_sin_vaiven_ = m.t;
    } else if (m.t - t_sin_vaiven_ >= p_.competidor_sostenido) {
      const size_t k = std::min(v.size() - 1, v.size() * static_cast<size_t>(p_.competidor_percentil) / 100);
      std::nth_element(v.begin(), v.begin() + k, v.end());
      const bool perdio = p_.competidor_perdida_s <= 0 || m.t - t_perdida_ < p_.competidor_perdida_s;
      if (perdio || v[k] - piso_vigente_ > dref_ultimo_) {
        competidor_ = true;
        piso_vigente_ = v[k];
      }
    }
  } else {
    t_sin_vaiven_ = m.t;
  }

  const double fila_cruda = std::max(0.0, m.owd_medio - piso_vigente_);
  // En un camino con ruido, la ley mira la envolvente de abajo: el mínimo de los últimos
  // ruido_ventana. Un corte del aire arma cola de verdad —el caño cae a casi nada por un par de
  // décimas— pero se va solo, y contado entero bajaba el presupuesto de golpe cada dos segundos sin
  // que la subida, que va por la curva tímida, lo recuperara antes del corte siguiente. Una cola
  // que se sostiene aparece con ese retraso; la alarma y el salto, que miran la muestra cruda, no
  // cambian.
  double fila_ley = fila_cruda;
  minimos_.push_back({m.t, fila_cruda});
  podar(minimos_, m.t, p_.ruido_ventana);
  if (m.ruido > 0.002) {
    double minimo = fila_cruda;
    for (const auto& x : minimos_) minimo = std::min(minimo, x.v);
    fila_ley = minimo;
  }
  filas_.push_back({m.t, fila_ley});
  // Y con ruido, la recta se ajusta sobre más tiempo: un salto de cien milisegundos dominaba una
  // ventana de doscientos cincuenta y se leía como la fila creciendo.
  podar(filas_, m.t, m.ruido > 0.002 ? std::max(constante_filtro(m.rtt), p_.filtro_ruido) : constante_filtro(m.rtt));

  double residuo = 0;
  const double pend = pendiente(residuo);          // ajuste de recta, cap. 6
  const double delta = pend * p_.Tc;               // crecimiento por ciclo
  // la fila que el lazo usa es la del ajuste, no la muestra cruda
  double fila = 0;
  { double st = 0; for (const auto& x : filas_) st += x.v; fila = st / filas_.size(); }

  perdida_suave_ += 0.30 * (std::max(0.0, m.perdida) - perdida_suave_);
  // La pérdida con la que se decide la paridad recuerda el último pico. La que sigue a una
  // descarga por TCP llega en tandas —cada diez segundos, unos cientos de milisegundos— y la
  // media vuelve a cero entre una y otra: la paridad estaba en el mínimo justo cuando llegaba
  // la tanda siguiente.
  //
  // Sólo si la pérdida se repite: una tanda que llega antes de que pase el doble de la memoria
  // desde la anterior. La pérdida de una caída de capacidad es una sola, y recordarla dejaba la
  // paridad alta —y la imagen baja— mucho después de que el lazo se hubiera acomodado.
  const bool en_tanda = perdida_suave_ > p_.paridad_tanda;
  if (en_tanda && !en_tanda_) {
    recurrente_ = m.t - t_tanda_ < 2.0 * p_.paridad_memoria;
    t_tanda_ = m.t;
  }
  en_tanda_ = en_tanda;
  if (p_.paridad_memoria > 0 && recurrente_ && m.t - t_tanda_ < 2.0 * p_.paridad_memoria) {
    perdida_pico_ = std::max(perdida_suave_, perdida_pico_ * std::exp(-p_.Tc / p_.paridad_memoria));
  } else {
    perdida_pico_ = perdida_suave_;
  }
  // El mínimo de paridad baja sólo lejos de toda pérdida y del arranque: con una descarga en el
  // caño la pérdida vuelve cada pocos segundos, y un bloque con un solo paquete de paridad no
  // aguanta la tanda de la cola que desborda.
  if (m.perdida > 0) t_perdida_ = m.t;
  const bool limpio = p_.fec_minimo_limpio >= 0 && t_arranque_ >= 0 &&
                      m.t - std::max(t_perdida_, t_arranque_) > p_.fec_limpio_s;
  paridad_minima_ = limpio ? std::min(p_.fec_minimo_limpio, p_.fec_minimo_paquetes) : p_.fec_minimo_paquetes;
  // Y al arrancar, antes de haber medido nada, se protege como si hubiera pérdida. El primer
  // segundo sale a un presupuesto que nadie verificó, y si otro flujo ya llena el caño se pierde
  // una cuarta parte de lo que se manda justo cuando la paridad está en el mínimo, porque todavía
  // no midió pérdida: cada cuadro roto ahí es un cuadro clave más.
  if (t_arranque_ < 0) t_arranque_ = m.t;
  if (m.t - t_arranque_ < p_.paridad_arranque_s) {
    perdida_pico_ = std::max(perdida_pico_, p_.paridad_arranque);
  }

  // ── 2. el objetivo de fila, atado al CAMINO y no al lazo ───────────────────
  // Cap. 3: con 10 ms fijos y 250 ms de viaje la fila se vacía el 54 % del tiempo y el
  // lazo se queda ciego. El objetivo no es una constante.
  //
  // Con el piso del ida y vuelta, no con su percentil alto: cuando el ida y vuelta es la única
  // medida de retardo —el de ENet, por GameStream— incluye la cola, y un objetivo atado al
  // percentil alto crecía con la propia fila, que crecía con el objetivo.
  const double viaje_base = viajes_.empty() ? m.rtt : percentil(viajes_, p_.percentil_viaje_dref);
  //
  // Y nunca bajo el ruido del camino: lo que la fila sube y vuelve sola no es cola propia. Atado al
  // ruido y no a la fila, el objetivo no crece con la cola que arma el propio lazo.
  const double dref = ingenuo_
      ? p_.dref_piso
      : std::max({p_.dref_piso, p_.dref_fraccion * (viaje_base + 1.0 / p_.fps), p_.dref_ruido * m.ruido});

  dref_ultimo_ = dref;
  // La capacidad aprende de la fila que se sostiene en las últimas capacidad_sostenida muestras, no
  // de un pico: una sola muestra de jitter sobre el objetivo la llevaba de un golpe a lo que el lazo
  // mandaba, y de ahí no salía, porque mide con lo que el lazo manda. La cola de verdad no se vacía
  // entre dos muestras; el jitter sí.
  double fila_sostenida = fila;
  {
    int n = 0;
    for (auto it = filas_.rbegin(); it != filas_.rend() && n < p_.capacidad_sostenida; ++it, ++n) {
      fila_sostenida = std::min(fila_sostenida, it->v);
    }
  }
  estimar(m, fila_sostenida, dref);
  sondeo_del_piso(m.t);

  // ── 3. las ganancias, recalculadas ─────────────────────────────────────────
  const double C = ingenuo_ ? p_.bitrate_max : C_est_;
  const double tau = ingenuo_ ? 0.2 : std::max(0.05, tau_est_);
  const double kg = escala_ * s_.kg_n * C / tau;
  const double kp = escala_ * s_.kp_n * C / (tau * tau);

  // ── 4. la zona muerta: el ancho de la nube del medidor (cap. 6) ────────────
  // No es un valor a elegir. Es la dispersión del propio ajuste, dividida por la raíz de
  // la evidencia: se angosta sola cuando hay más muestras.
  const double n = static_cast<double>(std::max<size_t>(1, filas_.size()));
  const double zona = ingenuo_ ? 0.0 : std::min(0.020, 2.0 * residuo / std::sqrt(n));

  // ── 5. LA LEY (cap. 9) ─────────────────────────────────────────────────────
  //   ΔR = −k_g·(d_k − d_{k−1}) − k_p·T_c·(d_k − d_ref)
  double dR = 0;
  double caño_reciente = 0;      // el caño de los últimos cuadros, si se pudo medir
  Motivo motivo = Motivo::quieto;
  if (primera_) {
    primera_ = false;
    motivo = Motivo::arranque;
  } else if (ingenuo_) {
    // el contrafáctico del capítulo 4: mira el VALOR, no si crece. Dos integraciones
    // sin derivada no se estabilizan con ninguna ganancia.
    dR = -kg * 0.8 * (fila - dref);
    if (perdida_suave_ > 0.02) dR -= R_ * 0.22;
    motivo = Motivo::fila_alta;
  } else {
    const double nivel = std::fabs(fila - dref) > zona ? (fila - dref) : 0.0;
    dR = -kg * delta - kp * p_.Tc * nivel;
    motivo = std::fabs(delta) > 0.001 ? Motivo::fila_crece
           : (nivel != 0.0 ? Motivo::fila_alta : Motivo::quieto);

    // ── el salto por pérdida (cap. 5) ────────────────────────────────────────
    // Pérdida creíble CON la fila llena es desborde: el cuello de botella ya está
    // tirando. Con la fila vacía, la misma pérdida no mueve un bit — eso es trabajo del
    // lazo de reparación.
    //
    // "Llena", no "creciendo": la fila deja de crecer al llenarse el buffer, y para
    // cuando la pérdida es creíble el crecimiento ya desapareció. Pedir las dos cosas
    // juntas hace que el salto no dispare nunca.
    //
    // Y un salto borra lo que la curva de subida recordaba. El ancla es el caudal al que apareció
    // la última fila; después de un desborde ese caudal está demostrado que no entra, y si la
    // curva seguía viva, en cuanto la fila bajaba volvía de un paso al ancla vieja. Con una
    // descarga en el caño y Sunshine de verdad, de 1,9 a 10,7 Mbit/s en un ciclo, justo antes de
    // pasar del menú a la acción.
    if (perdida_suave_ > p_.salto_perdida && fila > dref + p_.salto_margen_fila) {
      dR = std::min(dR, -(R_ - p_.salto_factor * C_est_));
      motivo = Motivo::salto_perdida;
    }
    // El mismo salto, sin esperar a la fila. Cuando el caño se angosta de golpe, la fila que
    // ve el lazo llega un cuarto de segundo tarde —el informe, el viaje, el filtro—, y para
    // entonces el buffer ya desbordó. Pero el caño angosto se ve antes por otro lado: llega
    // bastante menos de lo que se emitió un viaje antes, aunque todavía no se pierda nada. Y
    // lo que llega en ese momento es exactamente el caño nuevo, mientras que Ĉ todavía
    // recuerda el viejo.
    //
    // Lo que tiene que ser de verdad es la emisión, no su relación con R: al salir de un menú
    // R es el valor sin verificar con el que se arrancó, y lo emitido puede quedar justo por
    // debajo de cualquier fracción de él. Alcanza con que haya sido bastante más que el piso.
    //
    // Sólo con la fila todavía por debajo del doble del objetivo, y no dos veces seguidas. Con
    // otro flujo en el caño lo que llega queda por debajo de lo que sale todo el tiempo —el otro
    // se lleva su parte— y el salto disparaba en cada ciclo, cada vez sobre lo que había quedado:
    // un trinquete que le regalaba el caño entero a una descarga. Anticiparse a la cola sólo tiene
    // sentido mientras la cola no se armó; armada, manda el salto de arriba. Mirar si lo que llega
    // se derrumbó no sirve para distinguirlos: el diente de sierra de TCP también lo derrumba.
    //
    // El doble y no el objetivo mismo porque el informe llega tarde: cuando el caño se angosta, la
    // fila que se ve ya pasó el objetivo para cuando lo que llega muestra la caída, y con la guarda
    // en el objetivo el salto no disparaba justo en la caída —con Sunshine de verdad, de 12 a
    // 4 Mbit/s, la cola llegaba a 264 ms antes de que el salto por pérdida la cortara—. Más arriba
    // del doble vuelve el trinquete contra la descarga.
    //
    // Y el caño que cruzaron los últimos cuadros avisa todavía antes que lo que llega, que es un
    // promedio de un cuarto de segundo. Lo mismo vale para el destino: justo después de la caída
    // lo que llega todavía arrastra el caño viejo, y saltar ahí dejaba la cola llenándose.
    // Con un piso en la mitad de lo que llegó, por la misma razón que el ancho del tren: un
    // cuadro procesado tarde no es un caño angosto. La mitad y no todo porque justo después de
    // una caída lo que llega todavía arrastra el caño viejo.
    double ancho_reciente =
        p_.salto_trenes > 0 && trenes_recientes >= p_.salto_trenes && lapso_reciente > 0
            ? bits_recientes / lapso_reciente : 0.0;
    if (ancho_reciente > 0) ancho_reciente = std::max(ancho_reciente, 0.5 * m.tasa_recibida);
    caño_reciente = ancho_reciente;
    // Un ciclo solo puede ser el informe que se demoró, no el caño: se pide que se repita.
    const bool corto = p_.entrega_llena > 0 && m.entrega > 0
                           ? m.entrega < 1.0 - p_.salto_rapido
                           : m.tasa_recibida > 0 && m.tasa_recibida < (1.0 - p_.salto_rapido) * emitida_entonces_;
    llega_menos_seguidos_ = corto ? llega_menos_seguidos_ + 1 : 0;
    const bool llega_menos = llega_menos_seguidos_ >= std::max(1, p_.salto_rapido_ciclos);
    // Y sólo sin cola todavía. Con otro flujo en la cola los paquetes de cada cuadro salen
    // intercalados con los suyos, y la dispersión mide la parte del caño que le toca al video,
    // no el caño: "cruzaron un caño angosto" pasaba a ser cierto a cada rato, y cada salto le
    // cedía más caño a una descarga. Una caída desde el régimen empieza con la fila vacía.
    const bool cruza_menos = ancho_reciente > 0 && fila < dref &&
                             ancho_reciente < (1.0 - p_.salto_rapido) * R_;
    if (p_.salto_rapido > 0 && m.tasa_recibida > 0 &&
        emitida_entonces_ > 2.0 * p_.bitrate_min && fila < p_.salto_rapido_fila * dref &&
        m.t - t_salto_rapido_ > 2.0 && (llega_menos || cruza_menos)) {
      double destino = p_.salto_factor * std::min(C_est_, m.tasa_recibida);
      if (ancho_reciente > 0) destino = std::min(destino, p_.salto_factor * ancho_reciente);
      if (R_ > destino) {
        dR = std::min(dR, -(R_ - destino));
        motivo = Motivo::salto_perdida;
        t_salto_rapido_ = m.t;
      }
    }

    // ── la pérdida sostenida sin fila ────────────────────────────────────────
    // El único retardo que da GameStream es el ida y vuelta de ENet, que viaja por el canal de
    // control y no por el del video. Detrás de una cola que reparte por flujo —fq_codel, CAKE: los
    // routers con gestión de cola— el control tiene su propia cola y no ve la del video, que se
    // llena igual: la fila creída queda en cero con doscientos milisegundos de fila de verdad, y
    // el salto por pérdida, que pide ver la fila, no dispara nunca. La pérdida aleatoria de un
    // enlace no dura así; si se sostiene por encima de lo que la paridad cubre, es la cola.
    //
    // Y a partir de ahí el retardo no es evidencia de que haya lugar: la subida no acelera.
    if (p_.perdida_congestion > 0 && perdida_suave_ > p_.perdida_congestion && fila <= dref) {
      if (t_perdida_sin_fila_ < 0) t_perdida_sin_fila_ = m.t;
      if (m.t - t_perdida_sin_fila_ >= p_.perdida_congestion_s && m.tasa_recibida > 0) {
        const double destino = p_.salto_factor * std::min(C_est_, m.tasa_recibida);
        if (R_ > destino) {
          dR = std::min(dR, -(R_ - destino));
          motivo = Motivo::salto_perdida;
        }
        t_perdida_sin_fila_ = -1;
        ciego_hasta_ = m.t + p_.ciego_s;
      }
    } else {
      t_perdida_sin_fila_ = -1;
    }

    // ── el salto por el escalón de la fila ───────────────────────────────────
    // Un caño que se angosta a un tercio desborda el buffer de un router en un quinto de segundo:
    // antes de que llegue la primera pérdida, y antes de que la fila filtrada pase el objetivo por
    // mucho. Lo que sí se ve en ese mismo ciclo es la muestra cruda de la fila pegando un escalón,
    // y lo que llega, que ya es el caño nuevo. Se salta ahí, y más abajo que en los otros saltos:
    // la cola que se armó tiene que vaciarse antes de que salga el cuadro clave que el cliente
    // va a pedir, o se pierde también y lo vuelve a pedir.
    //
    // Una descarga por TCP arma la cola de a un paquete por ida y vuelta, sin escalones; y el
    // salto no se repite antes de dos segundos.
    if (p_.salto_escalon > 0 && fila_cruda - fila_cruda_prev_ > p_.salto_escalon &&
        fila_cruda > 2.0 * dref && m.tasa_recibida > 0 && m.t - t_salto_rapido_ > 2.0) {
      // El caño nuevo: lo que llega todavía es el viejo, pero la fila ya crece a (R−C)/R. Con el
      // menor de los dos últimos crecimientos: un camino que se alarga sube la fila de un escalón
      // y la deja plana, y leído como pendiente recortaba el presupuesto a casi nada.
      const double pendiente = std::min(0.9, std::max(0.0,
          std::min(fila_cruda - fila_cruda_prev_, fila_cruda_prev_ - fila_cruda_prev2_) / p_.Tc));
      const double caño_nuevo = std::min({C_est_, app_limited_ ? C_est_ : m.tasa_recibida, R_ * (1.0 - pendiente)});
      const double destino = p_.salto_escalon_factor * caño_nuevo;
      if (R_ > destino) {
        dR = std::min(dR, -(R_ - destino));
        motivo = Motivo::salto_perdida;
        t_salto_rapido_ = m.t;
        // Éste no pide cuadro clave: no distingue una descarga que arma su cola de un caño que
        // se angosta, y un cuadro clave dentro de la cola de una descarga se pierde, y el cliente
        // pide uno por cuadro. La alarma sí los distingue.
      }
    }
  }

  // ── 5b. sin evidencia no se sube ───────────────────────────────────────────
  // Con el codificador entregando una fracción del presupuesto, la fila está vacía porque
  // NO MANDAMOS, no porque haya lugar. El término proporcional no puede distinguirlo: ve la
  // fila muy por debajo del objetivo y ordena subir, y con las ganancias del capítulo 4 eso
  // son unos 73 kbit/s por ciclo — once megabits en quince segundos de menú, contra un caño
  // que no cambió.
  //
  // Bloquear el tanteo no alcanzaba: el tanteo es el mecanismo explícito de subir, pero la
  // ley misma sube igual, y es la ley la que hace casi todo el daño.
  //
  // Se corta en una sola dirección a propósito. Subir sin evidencia es inventar caño;
  // bajar sin evidencia es prudente, y además es lo que hay que hacer si mientras el
  // codificador calla la red empeoró.
  if (app_limited_ && dR > 0) {
    dR = 0;
    if (motivo != Motivo::salto_perdida) motivo = Motivo::sin_evidencia;
  }

  quieto_ = std::fabs(delta) < 0.002 ? quieto_ + 1 : 0;

  const double antes = R_;
  R_ = R_ + dR;
  // Bajar vacía la cola; bajar por debajo de lo que el caño entrega sólo la vacía más rápido, y
  // pasado un punto lo único que agrega es una imagen peor. Cuando el caño se angosta, los
  // cuadros que ya estaban codificados para el viejo siguen saliendo un par de ciclos, la fila
  // sigue creciendo, y el término de crecimiento seguía cortando: con Sunshine de verdad, de 12
  // a 4 Mbit/s, el presupuesto terminaba en el mínimo con el caño entregando 4,25. Y con el
  // presupuesto en el mínimo, esos mismos cuadros tardaban todavía más en salir.
  //
  // El piso es una fracción de lo que llega —del caño de los últimos cuadros, si es menor, porque
  // justo después de una caída lo que llega todavía arrastra el caño viejo—, y no rige en el paso
  // de un salto, que ya eligió su destino con los mismos números.
  if (p_.piso_recibida > 0 && dR < 0 && motivo != Motivo::salto_perdida && !app_limited_ &&
      m.tasa_recibida > 0) {
    const double entrega = caño_reciente > 0 ? std::min(m.tasa_recibida, caño_reciente) : m.tasa_recibida;
    R_ = std::max(R_, std::min(antes, p_.piso_recibida * entrega));
  }
  if (R_ < p_.bitrate_min) { R_ = p_.bitrate_min; d.topeo = true; }
  if (R_ > p_.bitrate_max) { R_ = p_.bitrate_max; d.topeo = true; }
  if (fila <= dref) R_sano_ = R_;
  if (motivo == Motivo::salto_perdida || perdida_suave_ > p_.paridad_tanda) arranque_hecho_ = true;

  // ── 6. las defensas ────────────────────────────────────────────────────────
  if (chequeo_coherencia(fila, dref, delta, R_ - antes)) {
    d.reanclo = true;
    motivo = Motivo::reanclado;
  }

  // ── 7. bajar es corregir, subir es tantear ─────────────────────────────────
  // Histéresis: para declarar la fila vacía hay que bajar bastante más de lo que hizo
  // falta para declararla llena.
  //
  // Y una fila chica con el caño sin llenar no es el caño: son las ráfagas de cada cuadro. Si lo
  // que sale sigue llegando al mismo ritmo, esa fila no marca dónde está el caño y la subida sigue.
  // Contada como fila, cada vez que cruzaba el objetivo la curva volvía a empezar desde el ancla
  // vieja, y de un caño de 4 a uno de 12 Mbit/s se tardaba quince segundos en llegar.
  const bool fila_de_rafagas = p_.fila_requiere_lleno && !caño_lleno_ && fila < 2.0 * dref &&
                               perdida_suave_ < p_.paridad_tanda;
  const bool hay_fila = !fila_de_rafagas && (habia_fila_ ? (fila > dref * 0.3) : (fila > dref));
  if (hay_fila) {
    if (!habia_fila_) {
      // el ancla es el caudal del FLANCO en que apareció la fila. Tocarla en cada ciclo
      // la derrumba; no dejarla bajar lleva el pico de fila a 62 ms.
      if (R_ > ancla_) ancla_ = R_;
      else if (R_ < ancla_ * p_.ancla_derrumbe) ancla_ = p_.ancla_mezcla * (ancla_ + R_);
    }
    en_curva_ = false;
    t_ultima_fila_ = m.t;
    arranque_hecho_ = true;
  } else if (!ingenuo_ && !app_limited_ && motivo != Motivo::salto_perdida) {
    // Nunca en el mismo paso de un salto: la curva devolvía el presupuesto a donde estaba antes
    // de saltar, y el salto no bajaba nada.
    //
    // Y si el tren mide más caño que el ancla, el ancla es el tren. Sin esto, pasada la curva la
    // subida es la exploración —un porcentaje fijo del ancla vieja por segundo—, y con Sunshine de
    // verdad, de un caño de 4 a uno de 12 Mbit/s, pasaban veinticinco segundos en 7. El tren
    // sabe cuánto caño hay sin haberlo llenado; con la fila vacía, que es la única forma de
    // llegar acá, no hay otro flujo que lo desmienta.
    //
    // Pero sólo si la última fila quedó varios segundos atrás. Si la fila va y viene, el ancla
    // que dejó su flanco es la que vale: detrás de un router que deja pasar ráfagas el tren sale
    // inflado cada vez que la cola se vacía, y apuntar la curva ahí la hacía chocar una y otra vez.
    if (p_.tren_ancla > 0 && p_.variante == Variante::tren && ancho_tren_ > 0 &&
        m.t - t_ultima_fila_ > p_.tren_ancla_espera) {
      ancla_ = std::max(ancla_, std::min(p_.tren_destino * ancho_tren_, p_.tren_ancla * R_));
    }
    // Con la fila vacía se tantea hacia arriba. Pero si la fila está vacía porque NO
    // MANDAMOS NADA, subir es inventar: no hay ninguna evidencia de que haya lugar.
    subir_tanteando(dref, fila);
    if (motivo == Motivo::quieto) motivo = Motivo::tanteando;
  } else if (app_limited_) {
    en_curva_ = false;          // al volver la escena, la curva arranca de cero
    if (motivo == Motivo::quieto) motivo = Motivo::sin_evidencia;
  }

  // ── la cola ajena ──────────────────────────────────────────────────────────
  // Una película en régimen baja un pedazo cada pocos segundos a toda la velocidad del caño y
  // llena la cola del router mientras dura. Para el lazo es igual a una caída: corta, y lo que
  // llega mientras tanto —lo poco que manda— entra a Ĉ como si fuera el caño. Cuando la película
  // termina el pedazo, la cola desaparece de golpe, pero Ĉ y el ancla quedaron en el mínimo, y en
  // los dos segundos de calma la subida no llega a nada: con Sunshine de verdad, el video se
  // quedaba en 1 Mbit/s con 5 libres.
  //
  // Lo que distingue a la cola de otro es lo que hace después del corte. La propia, la de una
  // caída, se vacía: llega más de lo que se manda. La de otro se queda alta y quieta mientras
  // llega lo mismo que se manda —el video pasa entero, detrás de la cola ajena—. Si eso dura y el
  // episodio termina solo y corto, el caño no cambió y se vuelve a lo de antes. Una descarga sin
  // pausa no termina, y ahí sigue mandando la ley.
  //
  // Terminar es vaciarse de verdad, contra el mínimo del retardo y no contra el piso del capítulo 5:
  // con una descarga continua ese piso es un percentil de la ventana, y la fila medida contra él se
  // vacía a ratos con doscientos milisegundos de cola. Leído como el fin de un pedazo de película,
  // el presupuesto exploraba hasta el máximo adentro de la cola de la descarga de un juego.
  const double fila_verdadera = std::max(0.0, m.owd_medio - piso_minimo);
  if (fila <= dref) C_sano_ = C_est_;
  if (p_.ajena_s > 0 && !ingenuo_) {
    if (hay_fila && !habia_fila_) episodio_ = {true, m.t, R_sano_, C_sano_, 0.0};
    if (hay_fila) tras_ajena_ = false;
    if (hay_fila && episodio_.activo && !app_limited_ && m.tasa_recibida > 0 && emitida_entonces_ > 0) {
      const double r = m.tasa_recibida / emitida_entonces_;
      const bool igual = std::fabs(r - 1.0) < p_.ajena_tolerancia;
      const bool quieta = fila_verdadera > 2.0 * dref && fila_cruda >= fila_cruda_prev_ - 0.010;
      // La fila propia en equilibrio también se ve quieta y con todo llegando; la diferencia es que
      // la propia se vacía si se baja, y ésta ya sobrevivió a bajar a la mitad, o al piso
      const bool bajamos = R_ <= std::max(0.5 * episodio_.R, 1.5 * p_.bitrate_min);
      if (igual && quieta && bajamos) episodio_.ajeno += p_.Tc;
    }
    if (!hay_fila && habia_fila_ && episodio_.activo) {
      const bool ajeno = episodio_.ajeno >= p_.ajena_s && m.t - episodio_.t0 <= p_.ajena_max_s &&
                         fila_verdadera < dref;
      if (ajeno && episodio_.R > R_) {
        R_ = std::min(p_.bitrate_max, episodio_.R);
        C_est_ = std::max(C_est_, episodio_.C);
        tasas_.clear();
        // Y el tren empieza de nuevo: los cuadros que cruzaron detrás de la cola de otro salieron
        // intercalados con sus paquetes, y su dispersión mide la parte que le tocó al video, no el
        // caño. Tampoco sirve la medida de antes: en una calma de un par de segundos no entran
        // muestras nuevas, y con Sunshine de verdad quedaba de techo para siempre la de la primera.
        ancho_tren_ = 0;
        trenes_.clear();
        ancla_ = std::max(ancla_, R_);
        en_curva_ = false;
        d.reanclo = true;
      }
      tras_ajena_ = ajeno;
      episodio_.activo = false;
    }
  }
  habia_fila_ = hay_fila;

  // ── 7a. la exploración en crucero (HyStart++, RFC 9406) ────────────────────
  // Pasada la primera fila, la curva sube como una cúbica desde el ancla, y de un caño de 4 a uno de
  // 12 Mbit/s eso son diez segundos. Si el camino está en crucero —la fila cruda pegada al piso, lo
  // que sale llega entero y al mismo ritmo, sin pérdida— durante unos segundos, no hay evidencia de
  // estar cerca del caño, y se sube multiplicando, como el arranque. Se sale en cuanto la fila
  // cruda despega, igual que HyStart++ sale cuando el mínimo de la ronda sube, y no se vuelve al
  // presupuesto al que se llegó —ya se pasó— sino al máximo que de verdad se entregó, como BBR.
  if (p_.crucero_ganancia > 0 && !ingenuo_) {
    entregado_.push_back({m.t, m.tasa_recibida});
    podar(entregado_, m.t, 1.0);
    const bool limpio = fila_cruda < p_.crucero_fila && !app_limited_ && perdida_suave_ < p_.paridad_tanda &&
                        (m.entrega <= 0 || entrega_suave_ >= 0.97) && motivo != Motivo::salto_perdida;
    if (!limpio) t_crucero_ = m.t;
    if (explorando_) {
      if (fila_cruda >= p_.crucero_fila || motivo == Motivo::salto_perdida || perdida_suave_ > p_.paridad_tanda) {
        double maximo = 0;
        for (const auto& x : entregado_) maximo = std::max(maximo, x.v);
        if (maximo > 0 && motivo != Motivo::salto_perdida) {
          R_ = std::max(p_.bitrate_min, std::min(R_, 0.95 * maximo));
          ancla_ = std::max(ancla_, maximo);
        }
        explorando_ = false;
        en_curva_ = false;
        t_crucero_ = m.t;
      } else {
        R_ = std::min(p_.bitrate_max, R_ * (1.0 + p_.crucero_ganancia));
      }
    } else if (arranque_hecho_ && m.t - t_crucero_ >= p_.crucero_espera) {
      explorando_ = true;
    }
  }
  fila_prev_ = fila;
  fila_cruda_prev2_ = fila_cruda_prev_;
  fila_cruda_prev_ = fila_cruda;

  // ── 7b. la variante por dispersión (SQP) ───────────────────────────────────
  // Acá el presupuesto no sale de integrar correcciones sobre el retardo: sale de lo que
  // se midió que el caño entrega, recortado por el multiplicador de destino. Ese recorte
  // reemplaza al sondeo del piso — el 10 % que sobra drena las colas solo, sin frenar el
  // envío cada ocho segundos.
  if (p_.variante == Variante::tren && ancho_tren_ > 0) {
    // La dispersión mide la capacidad del ENLACE, no la parte que queda libre: con otro
    // flujo en la cola, los paquetes de un cuadro igual salen separados a la velocidad del
    // caño. Si reemplazara a la ley, el lazo ignoraría a cualquier competidor —contra una
    // descarga se quedaba con 16 de 20 Mbit/s y rompía el 6 % de los cuadros—. Como techo
    // sobre la ley aporta lo que la ley no tiene —saber cuánto caño hay sin haber mandado
    // nada, que es el caso del menú— y deja a la ley lo que sí sabe: si hay cola.
    //
    // Frena siempre la subida, pero baja el presupuesto sólo con evidencia de que el caño no
    // alcanza —fila, pérdida, lo emitido que no llega— o con el presupuesto sin verificar, que es
    // el del codificador callado. Un techo por debajo de lo que se está mandando sin cola ni
    // pérdida es una medida que la propia red desmiente: con la CPU escasa el tren medía al
    // emisor y no al caño, cada muestra bajaba el presupuesto a lo que ya se mandaba, y el
    // presupuesto se quedaba en la mitad del caño subiendo y bajando.
    const double techo = p_.tren_destino * ancho_tren_;
    if (R_ > techo) {
      const bool no_alcanza = caño_lleno_ || fila > dref || perdida_suave_ > p_.salto_perdida * 0.5;
      R_ = (no_alcanza || app_limited_) ? std::max(p_.bitrate_min, techo)
                                        : std::min(R_, std::max(techo, antes));
      if (motivo == Motivo::quieto || motivo == Motivo::tanteando) motivo = Motivo::topeado;
      no_enrollar(R_);
    }
  }

  // ── 7b'. la cola que se vació sola ─────────────────────────────────────────
  // Un corte del aire —un Wi-Fi lejano, el punto de acceso atendiendo a otro— arma en un cuarto de
  // segundo la misma rampa de fila que un caño que se angosta, y la alarma, el escalón y la ley la
  // cortan igual: cada corte bajaba el presupuesto a 1 Mbit/s y la curva tímida tardaba en
  // devolverlo. Mientras pasa no hay forma de distinguirlos; cuando la cola se vacía, sí: lo que la
  // vació es el caño que quedó. Si entrega lo que se mandaba antes de la cola, se vuelve ahí, hasta la
  // parte de ese caño que deja vaciar a tiempo la cola del corte siguiente, y nunca más abajo. En el
  // banco, con 8 Mbit/s y un corte cada dos segundos, no se separó del ruido entre corridas (uso del
  // 55-59 % con y sin esto): queda apagado hasta probarlo en un camino real.
  if (corte && !ingenuo_ && m.t - m.vaciado_desde <= p_.vaciado_episodio) {
    double antes_cola = 0;
    for (const auto& x : presupuestos_) {
      if (x.t > m.vaciado_desde) break;
      if (x.t >= m.vaciado_desde - 0.3) antes_cola = std::max(antes_cola, x.v);
    }
    const double destino = std::min({antes_cola, p_.bitrate_max, vaciado_util});
    if (std::getenv("ESTAB_DEBUG_ALARMA")) std::fprintf(stderr, "vaciado t=%.2f desde=%.2f caño=%.0f antes=%.0f R=%.0f\n", m.t, m.vaciado_desde, m.vaciado, antes_cola, R_);
    // Sólo si el caño entrega por lo menos lo que se mandaba: si entrega menos, se angostó, y la cola
    // que dejó la caída todavía se está yendo detrás del recorte. Volver ahí la llenaba de nuevo: con
    // Sunshine de verdad, de 12 a 4 Mbit/s, los descartes del router pasaban de 18 a 73.
    if (m.vaciado >= antes_cola && destino > R_) {
      if (alarma_abierta_ || t_alarma_ >= m.vaciado_desde) {
        paridad_minima_ = antes_alarma_.paridad_minima;
        t_perdida_ = antes_alarma_.t_perdida;
        alarma_abierta_ = false;
      }
      R_ = destino;
      ancla_ = std::max(ancla_, destino);
      C_est_ = std::max(C_est_, destino);
      en_curva_ = false;
      motivo = Motivo::reanclado;
      d.reanclo = true;
    }
  }

  // ── 7c. el techo que no pasa por el piso del retardo ───────────────────────
  // Toda la ley de los capítulos 3 y 4 se apoya en una sola medición: cuánto se despegó el
  // retardo de su piso. Cuando ese piso se envenena —y contra un flujo que nunca deja
  // drenar la cola se envenena siempre, a los diez segundos exactos— la ley no se degrada:
  // se queda sin entrada, y un integrador sin entrada se va a donde lo empujen.
  //
  // Ĉ es lo que de verdad llegó al cliente, medido en el percentil bajo. No pasa por el
  // piso ni por el retardo. Saturar R contra eso no reemplaza a la ley: la deja operar
  // normalmente mientras haya señal, y le pone un fondo cuando no la hay.
  //
  // Va DESPUÉS de todo lo que toca R, tanteo y dispersión incluidos: un techo que alguna
  // rama pueda saltear no es un techo.
  //
  // Y rige sólo cuando hay EVIDENCIA de caño lleno. El mismo argumento, usado al revés: si
  // todo lo que se pone en el cable llega y no se pierde nada, el caño no está lleno —y eso
  // tampoco pasa por el piso—, así que saturar ahí no protege de nada y frena al lazo justo
  // cuando puede crecer. Sin esta guarda, recuperar de 8 a 20 Mbit/s tardaba 16,5 s en vez
  // de 4,3, y el mecanismo de holgura permanente se quedaba en el 55 % del caño.
  const double techo_demostrado = p_.techo_sobre_C * C_est_;
  const bool hay_evidencia = caño_lleno_ || perdida_suave_ > p_.salto_perdida * 0.5;
  if (hay_evidencia && R_ > techo_demostrado) {
    R_ = std::max(p_.bitrate_min, techo_demostrado);
    // Sin esto la curva de subida seguía hacia su ancla por debajo del techo, y el presupuesto
    // saltaba cada ciclo entre el techo y la curva: con Sunshine de verdad, de 12,9 a 8,2 Mbit/s
    // y vuelta, diez veces por segundo. El mismo anti-enrollado que el techo del tren.
    if (p_.techo_no_enrolla) no_enrollar(R_);
    d.topeo = true;
    if (motivo == Motivo::tanteando || motivo == Motivo::quieto) motivo = Motivo::topeado;
  }

  // ── 8. la reparación (cap. 7) ──────────────────────────────────────────────
  // f ≈ p + z·√(p/D): la paridad que hace falta para que un bloque de D fragmentos
  // aguante la pérdida medida. Subir rápido, bajar lento.
  const double D = std::max(8.0, p_.bloques_esperados);
  const double obj = std::min(p_.fec_max,
      std::max(p_.fec_min, perdida_suave_ + p_.fec_z * std::sqrt(std::max(0.0, perdida_suave_) / D)));
  const double alfa = obj > f_ ? p_.fec_subida : (p_.Tc / p_.fec_bajada_tau);
  f_ += alfa * (obj - f_);

  // ── 9. el presupuesto único (cap. 7) ───────────────────────────────────────
  // Lo que sale al cable es R, entero. La imagen es lo que queda después de pagar el
  // audio, las cabeceras y la paridad. Por eso mover f no agrega un solo bit al caño.
  double egreso = R_ * p_.destino;
  if (sondeando_) egreso *= (1.0 - p_.sondeo_recorte);
  if (sondeando_) motivo = Motivo::sondeando;
  const double ovh = static_cast<double>(p_.mtu) / (p_.mtu + p_.cabeceras);
  double imagen = std::max(0.3e6, (egreso - p_.audio_bps) * ovh / (1.0 + f_));
  if (p_.paridad_en_paquetes) {
    // Con la paridad decidida en paquetes se paga la que el host va a poner en un cuadro del
    // tamaño presupuestado: la misma cuenta por bloque, sobre k paquetes. El mínimo de dos por
    // bloque es un costo fijo por cuadro, no una fracción.
    //
    // No la que salió: cuando el codificador entrega cuadros más chicos que lo pedido, la
    // proporción de paridad que sale sube —dos de paridad sobre tres de datos es 67 %—, y
    // pagarla achicaba la imagen, que achicaba los cuadros, que subía la proporción. Con
    // libx264 sobre un caño de 8 Mbit/s la imagen se iba a la mitad del presupuesto.
    //
    // La mayor imagen cuya paridad, calculada para su propio tamaño, entra en el presupuesto. La
    // paridad sube a saltos con k, así que iterar no converge: la última pasada pagaba la de un
    // cuadro más grande, y subir el presupuesto un 4 % llegaba a bajar la imagen un 12 %.
    const double util = (egreso - p_.audio_bps) * ovh;
    const double por_paquete = p_.mtu * 8.0 * fps_;
    imagen = 0.3e6;
    for (int k = std::max(1, static_cast<int>(std::ceil(util / por_paquete))); k >= 1; --k) {
      const int m_par = paridad_de_bloque(k, perdida_pico_, std::max(1.0, rafaga_), p_, paridad_minima_);
      const double cabe = std::min(k * por_paquete, util - m_par * por_paquete);
      if (cabe > (k - 1) * por_paquete) {
        imagen = std::max(0.3e6, cabe);
        break;
      }
    }
  }

  d.presupuesto = egreso;
  d.imagen = imagen;

  // ── 10. el codificador, compensado ─────────────────────────────────────────
  // Un codificador de tasa fija con el buffer de un cuadro —libx264 como lo configura Sunshine—
  // entrega un cuarto menos de lo que se le pide aunque la escena lo pida todo, y el caño queda
  // a medio usar sin que el lazo pueda hacer nada: el presupuesto le sobra. Se le pide de más en
  // la proporción que viene faltando, para que lo que entregue sea la imagen presupuestada.
  //
  // Sólo se aprende con el codificador usando el presupuesto —en un menú entrega poco porque no
  // tiene qué, y compensar eso sería pedirle de más justo antes del golpe—, sin cola y sin
  // pérdida. Y con techo: si en un cambio de escena el codificador sí llega a lo pedido, lo que
  // sale pasa el presupuesto a lo sumo por ese techo, y la ley lo corrige como a cualquier exceso.
  //
  // Y se suelta de golpe ante la primera señal de congestión: fila por encima del objetivo, el
  // caño lleno o un salto. Medido con Sunshine y Moonlight de verdad, pedir de más cuando el caño
  // se angosta multiplicaba los cuadros rotos de una caída por diez: el exceso del codificador cae
  // justo en el peor momento. Suelta, vuelve a aprender despacio. La pérdida sola no es señal: en
  // un Wi-Fi la hay siempre, y soltar con ella deja al codificador entregando un sexto menos. La
  // fila apenas por encima del objetivo tampoco, si es un paso suelto: en un Wi-Fi las tandas de aire
  // la pasean alrededor del objetivo todo el tiempo y la compensación no llega a aprenderse. Por
  // encima del doble, o sostenida, sí; una caída pasa del doble en el primer informe.
  fila_sobre_objetivo_ = fila > dref ? fila_sobre_objetivo_ + 1 : 0;
  const bool fila_congestion = p_.codificador_suelta_ciclos <= 1
                                   ? fila > dref
                                   : fila > 2.0 * dref || fila_sobre_objetivo_ >= p_.codificador_suelta_ciclos;
  const bool congestion = fila_congestion || caño_lleno_ || motivo == Motivo::salto_perdida ||
                          (p_.codificador_suelta_perdida > 0 && perdida_suave_ > p_.codificador_suelta_perdida);
  if (congestion) {
    eficiencia_cod_ = 1.0;
  } else if (p_.codificador_compensacion_max > 1.0 && m.tasa_datos > 0 && pedido_prev_ > 0 &&
             m.tasa_emitida >= p_.app_umbral * R_) {
    const double muestra = std::min(1.0, m.tasa_datos / pedido_prev_);
    eficiencia_cod_ += p_.Tc / p_.codificador_constante * (muestra - eficiencia_cod_);
  }
  const double eficiencia = std::max(1.0 / std::max(1.0, p_.codificador_compensacion_max),
                                     std::min(1.0, eficiencia_cod_));
  d.pedido_codificador = imagen / eficiencia;
  pedido_prev_ = d.pedido_codificador;
  d.reparacion = f_;
  d.motivo = motivo;
  d.fila = fila;
  d.piso = piso_vigente_;
  d.dref = dref;
  d.crecimiento = delta;
  d.zona_muerta = zona;
  d.C_est = C_est_;
  d.tau_est = tau;
  d.kg = kg;
  d.kp = kp;
  d.perdida_est = perdida_pico_;
  d.paridad_minima = paridad_minima_;
  // El largo de las ráfagas del canal. Con la fila llena la pérdida es de la propia cola,
  // que tira todo lo que llega mientras dura el desborde: eso no dice nada del canal, y si
  // entrara al promedio la paridad se quedaría protegiendo contra ráfagas que no existen.
  const bool desborde = fila > dref + p_.salto_margen_fila;
  if (m.rafaga_perdida > 0 && !desborde) {
    rafaga_ = rafaga_ > 0 ? rafaga_ + 0.2 * (m.rafaga_perdida - rafaga_) : m.rafaga_perdida;
  }
  d.rafaga_perdida = std::max(1.0, rafaga_);
  d.muestras = m.muestras;
  d.sondeando = sondeando_;
  d.app_limited = app_limited_;
  d.ancho_tren = ancho_tren_;
  return d;
}

}  // namespace estab
