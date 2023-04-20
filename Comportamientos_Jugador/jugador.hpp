#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

// Estado del nivel 0
struct stateN0
{
  ubicacion jugador;
  ubicacion sonambulo;

  bool operator==(const stateN0 &x) const
  {
    return (jugador == x.jugador && sonambulo.f == x.sonambulo.f && sonambulo.c == x.sonambulo.c);
  }
};

class ComportamientoJugador : public Comportamiento
{
public:
  ComportamientoJugador(unsigned int size) : Comportamiento(size)
  {
    // Inicializar Variables de Estado
  }
  ComportamientoJugador(std::vector<std::vector<unsigned char>> mapaR) : Comportamiento(mapaR)
  {
    // Inicializar Variables de Estado
    hayPlan = false;
  }
  ComportamientoJugador(const ComportamientoJugador &comport) : Comportamiento(comport) {}
  ~ComportamientoJugador() {}

  Action think(Sensores sensores);
  int interact(Action accion, int valor);

  // Funcion para la busqueda en anchura del nivel 0
  bool AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

private:
  // Declarar Variables de Estado
  list<Action> plan; // Almacena el plan
  bool hayPlan;      // Indica si existe plan o no

  // Nos dice si una casilla es transitable (si no es muro o precipicio)
  bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa);

  // Nos indica la siguiente casilla en caso de avanzar en función de la orientación del agente
  ubicacion NextCasilla(const ubicacion &pos);

  //Devuelve el estado de aplicar a un estado una determinada accion
  stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa);

  //Encuentra si el elemento item está en la lista
  bool Find(const stateN0 &item, const list<stateN0> &lista);
};

#endif
