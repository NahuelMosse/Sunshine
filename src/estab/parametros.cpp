#include "parametros.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace estab {
namespace {

// Un TOML plano alcanza: clave = valor, una por línea, # para comentar. Prefiero treinta
// líneas propias a una dependencia que después hay que arrastrar hasta Sunshine.
struct Campo {
  const char* nombre;
  enum { D, I } tipo;
  size_t desplazamiento;
};

#define CAMPO_D(n) { #n, Campo::D, offsetof(Parametros, n) }
#define CAMPO_I(n) { #n, Campo::I, offsetof(Parametros, n) }

const std::vector<Campo>& tabla() {
  static const std::vector<Campo> t = {
      CAMPO_D(Tc), CAMPO_D(fps),
      CAMPO_D(filtro_lento), CAMPO_D(filtro_rapido), CAMPO_D(filtro_rtt_corte),
      CAMPO_I(muestras_para_recta),
      CAMPO_D(dref_piso), CAMPO_D(dref_fraccion), CAMPO_D(dref_ruido), CAMPO_D(ruido_ventana), CAMPO_D(filtro_ruido), CAMPO_D(vaciado_episodio), CAMPO_D(vaciado_rampa), CAMPO_D(vaciado_drenaje),CAMPO_I(percentil_viaje_dref), CAMPO_I(tau_del_piso),
      CAMPO_D(ventana_piso), CAMPO_D(ventana_capacidad),
      CAMPO_I(percentil_capacidad), CAMPO_I(percentil_viaje),
      CAMPO_D(constante_capacidad), CAMPO_D(margen_egreso), CAMPO_D(ajena_s), CAMPO_D(ajena_max_s), CAMPO_D(ajena_tolerancia), CAMPO_D(entrega_llena), CAMPO_I(techo_no_enrolla), CAMPO_I(fila_edad), CAMPO_D(competidor_umbral), CAMPO_I(competidor_percentil), CAMPO_D(competidor_sostenido), CAMPO_D(competidor_dref), CAMPO_D(competidor_perdida_s), CAMPO_I(capacidad_sostenida), CAMPO_D(codificador_suelta_perdida), CAMPO_D(edad_cadencia),CAMPO_D(edad_reciente),CAMPO_D(egreso_constante),CAMPO_D(techo_sobre_C),      CAMPO_D(coh_bajada_minima), CAMPO_D(coh_clavada), CAMPO_D(coh_margen),
      CAMPO_I(coh_ciclos),
      CAMPO_D(rho), CAMPO_D(margen_fase),
      CAMPO_D(salto_factor), CAMPO_D(salto_perdida), CAMPO_D(salto_margen_fila), CAMPO_D(salto_rapido), CAMPO_D(salto_rapido_fila), CAMPO_I(salto_rapido_ciclos), CAMPO_D(perdida_congestion), CAMPO_D(perdida_congestion_s), CAMPO_D(ciego_s), CAMPO_D(salto_escalon), CAMPO_D(salto_escalon_factor), CAMPO_D(alarma_escalon), CAMPO_D(alarma_fila), CAMPO_D(alarma_fila_previa), CAMPO_I(alarma_intervalos), CAMPO_D(alarma_previa_s),CAMPO_D(alarma_desmentir_s), CAMPO_D(alarma_factor), CAMPO_D(alarma_vaciado), CAMPO_I(cuadro_clave_en_caida), CAMPO_I(cuadros_en_vuelo), CAMPO_I(cuadros_alarma),CAMPO_D(piso_recibida), CAMPO_I(salto_trenes),
      CAMPO_D(curva_duracion), CAMPO_D(curva_exploracion), CAMPO_D(curva_convexa), CAMPO_D(arranque_rapido), CAMPO_D(crucero_ganancia), CAMPO_D(crucero_fila), CAMPO_D(crucero_espera), CAMPO_D(reexplorar_s), CAMPO_I(fila_requiere_lleno),
      CAMPO_D(ancla_derrumbe), CAMPO_D(ancla_mezcla),
      CAMPO_D(fec_min), CAMPO_D(fec_max), CAMPO_D(fec_subida),
      CAMPO_D(fec_bajada_tau), CAMPO_D(fec_z), CAMPO_D(bloques_esperados),
      CAMPO_D(sondeo_cada), CAMPO_D(sondeo_dura), CAMPO_D(sondeo_recorte),
      CAMPO_D(destino), CAMPO_D(app_umbral), CAMPO_D(tren_pacing),
      CAMPO_D(tren_destino), CAMPO_D(tren_ventana), CAMPO_I(tren_envolvente), CAMPO_D(tren_min_paquetes),      CAMPO_D(tren_subida_max), CAMPO_D(tren_ancla), CAMPO_D(tren_ancla_espera), CAMPO_D(tren_vigencia),
      CAMPO_D(codificador_compensacion_max), CAMPO_D(codificador_constante), CAMPO_I(codificador_suelta_ciclos),      CAMPO_D(bitrate_min), CAMPO_D(bitrate_max), CAMPO_D(arranque_fraccion),
      CAMPO_I(mtu), CAMPO_I(cabeceras), CAMPO_D(audio_bps),
      CAMPO_I(gf_shards), CAMPO_I(bloques_por_cuadro), CAMPO_I(fec_minimo_paquetes), CAMPO_I(fec_minimo_limpio), CAMPO_D(fec_limpio_s),
      CAMPO_I(paridad_en_paquetes), CAMPO_D(paridad_falla_bloque), CAMPO_D(paridad_falla_reparacion), CAMPO_D(fec_max_reparacion), CAMPO_D(paridad_memoria), CAMPO_D(paridad_tanda), CAMPO_D(paridad_arranque), CAMPO_D(paridad_arranque_s), CAMPO_D(enet_ping_ms), CAMPO_I(enet_rtt_crudo),  };
  return t;
}

std::string limpiar(std::string s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

}  // namespace

std::string Parametros::cargar(const std::string& archivo) {
  std::ifstream f(archivo);
  if (!f) return "no se pudo abrir " + archivo;

  std::string linea;
  int n = 0;
  while (std::getline(f, linea)) {
    ++n;
    const auto h = linea.find('#');
    if (h != std::string::npos) linea = linea.substr(0, h);
    linea = limpiar(linea);
    if (linea.empty() || linea[0] == '[') continue;

    const auto eq = linea.find('=');
    if (eq == std::string::npos) {
      return "linea " + std::to_string(n) + ": falta el =";
    }
    const std::string clave = limpiar(linea.substr(0, eq));
    const std::string valor = limpiar(linea.substr(eq + 1));
    if (!fijar(clave, valor)) {
      return "linea " + std::to_string(n) + ": parametro desconocido '" + clave + "'";
    }
  }
  return "";
}

// Un parámetro desconocido es un error, no algo para ignorar: casi siempre es un nombre
// mal escrito, y silenciarlo hace que la corrida use un valor que nadie puso.
bool Parametros::fijar(const std::string& clave, const std::string& valor) {
  if (clave == "variante") { variante = variante_por_nombre(valor); return true; }
  for (const auto& c : tabla()) {
    if (clave != c.nombre) continue;
    auto* base = reinterpret_cast<char*>(this);
    if (c.tipo == Campo::D) {
      *reinterpret_cast<double*>(base + c.desplazamiento) = std::strtod(valor.c_str(), nullptr);
    } else {
      *reinterpret_cast<int*>(base + c.desplazamiento) =
          static_cast<int>(std::strtol(valor.c_str(), nullptr, 10));
    }
    return true;
  }
  return false;
}

std::string Parametros::volcar() const {
  std::ostringstream o;
  o.precision(10);
  for (const auto& c : tabla()) {
    const auto* base = reinterpret_cast<const char*>(this);
    o << c.nombre << " = ";
    if (c.tipo == Campo::D) o << *reinterpret_cast<const double*>(base + c.desplazamiento);
    else                    o << *reinterpret_cast<const int*>(base + c.desplazamiento);
    o << "\n";
  }
  return o.str();
}

}  // namespace estab
