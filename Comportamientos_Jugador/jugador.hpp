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

// Definicion del tipo nodo para el nivel0
struct nodeN0
{
  stateN0 st;
  list<Action> secuencia;

  bool operator==(const nodeN0 &n) const
  {
    return (st == n.st);
  }

  bool operator<(const nodeN0 &n) const
  {
    if (st.jugador.f < n.st.jugador.f)
    {
      return true;
    }
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
    {
      return true;
    }
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula < n.st.jugador.brujula)
    {
      return true;
    }
    else
    {
      return false;
    }
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
  list<Action> AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

private:
  // Declarar Variables de Estado
  list<Action> plan; // Almacena el plan
  bool hayPlan;      // Indica si existe plan o no
  stateN0 c_state;   // Estado actual del agente
  ubicacion goal;    // Lugar a donde queremos llegar

  // Nos dice si una casilla es transitable (si no es muro o precipicio)
  bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa);

  // Nos indica la siguiente casilla en caso de avanzar en función de la orientación del agente
  ubicacion NextCasilla(const ubicacion &pos);

  // Devuelve el estado de aplicar a un estado una determinada accion
  stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa);

  // Encuentra si el elemento item está en la lista
  bool Find(const stateN0 &item, const list<stateN0> &lista);
  bool Find(const stateN0 &item, const list<nodeN0> &lista);

  // Función que pone a 0 todos los elementos de una matriz
  void AnulaMatriz(vector<vector<unsigned char>> &matriz);

  // Permite pintar sobre el mapa del simulador el plan partiendo desde el estado st
  void VisualizarPlan(const stateN0 &st, const list<Action> &plan);
};

#endif
