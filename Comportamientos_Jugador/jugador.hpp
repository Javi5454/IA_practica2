#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "../include/comportamientos/comportamiento.hpp"

#include <list>

// Estado del nivel
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

// Definicion del tipo nodo para el nivel1
struct nodeN1
{
  stateN0 st;
  list<Action> secuencia;

  bool operator==(const nodeN1 &n) const
  {
    return (st == n.st);
  }

  bool operator<(const nodeN1 &n) const
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
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f < n.st.sonambulo.f)
    {
      return true;
    }
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c < n.st.sonambulo.c)
    {
      return true;
    }
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula == n.st.jugador.brujula && st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c == n.st.sonambulo.c && st.sonambulo.brujula < n.st.sonambulo.brujula)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

// Definicion de estado para el nivel 2
struct stateN2
{
  ubicacion jugador;
  ubicacion sonambulo;
  bool bikini;
  bool zapatillas;

  bool operator==(const stateN2 &x) const
  {
    return (jugador == x.jugador && sonambulo == x.sonambulo && bikini == x.bikini && zapatillas == x.zapatillas);
  }

  bool operator<(const stateN2 &st) const
  {
    if (jugador.f < st.jugador.f)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c < st.jugador.c)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula < st.jugador.brujula)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini < st.bikini)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini == st.bikini && zapatillas < st.zapatillas)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

// Definicion de nodo para el nivel 2
struct nodeN2
{
  stateN2 st;
  list<Action> secuencia;
  int coste_acumulado = 0;

  bool operator==(const nodeN2 &n) const
  {
    return (st == n.st);
  }

  bool operator<(const nodeN2 &n) const
  {
    return (coste_acumulado > n.coste_acumulado); // Lo ponemos mayor para que lo use bien la priority_queue
  }

  bool operator>(const nodeN2 &n) const
  {
    return (coste_acumulado > n.coste_acumulado);
  }
};

// Definicion de estado para el nivel 3
struct stateN3
{
  ubicacion jugador;
  ubicacion sonambulo;
  bool bikini_j;
  bool zapas_j;
  bool bikini_s;
  bool zapas_s;

  bool operator==(const stateN3 &st) const
  {
    return (jugador == st.jugador && sonambulo == st.sonambulo && bikini_j == st.bikini_j && zapas_j == st.zapas_j && bikini_s == st.bikini_s && zapas_s == st.zapas_s);
  }

  bool operator<(const stateN3 &st) const
  {
    if (jugador.f < st.jugador.f)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c < st.jugador.c)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula < st.jugador.brujula)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j < st.bikini_j)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j == st.bikini_j && zapas_j < st.zapas_j)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j == st.bikini_j && zapas_j == st.zapas_j && bikini_s < st.bikini_s)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j == st.bikini_j && zapas_j == st.zapas_j && bikini_s == st.bikini_s && zapas_s < st.zapas_s)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j == st.bikini_j && zapas_j == st.zapas_j && bikini_s == st.bikini_s && zapas_s == st.zapas_s && sonambulo.f < st.sonambulo.f)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j == st.bikini_j && zapas_j == st.zapas_j && bikini_s == st.bikini_s && zapas_s == st.zapas_s && sonambulo.f == st.sonambulo.f && sonambulo.c < st.sonambulo.c)
    {
      return true;
    }
    else if (jugador.f == st.jugador.f && jugador.c == st.jugador.c && jugador.brujula == st.jugador.brujula && bikini_j == st.bikini_j && zapas_j == st.zapas_j && bikini_s == st.bikini_s && zapas_s == st.zapas_s && sonambulo.f == st.sonambulo.f && sonambulo.c == st.sonambulo.c && sonambulo.brujula < st.sonambulo.brujula)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

// Nodo nivel 3
struct nodeN3
{
  stateN3 st;
  list<Action> secuencia;
  int coste = 0;
  int heuristica = 0;
  int suma = 0;

  bool operator==(const nodeN3 &n) const
  {
    return (st == n.st);
  }

  bool operator<(const nodeN3 &n) const
  {
    return (suma > n.suma);
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

  // Funcion para la busqueda en anchura del nivel 1
  list<Action> AnchuraSonambulo(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

  // Funcion para la busqueda en de Dijkstra del nivel 2
  list<Action> DijkstraSoloJugador(const stateN2 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

  // Funcion para la búsqueda A* del nivel 3
  list<Action> AEstrellaSonambulo(const stateN3 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

private:
  // Declarar Variables de Estado
  list<Action> plan;            // Almacena el plan
  bool hayPlan;                 // Indica si existe plan o no
  ubicacion jugador, sonambulo; // Para sustituir a el c_state
  ubicacion goal;               // Lugar a donde queremos llegar

  // Nos dice si una casilla es transitable (si no es muro o precipicio)
  bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa);

  // Nos indica la siguiente casilla en caso de avanzar en función de la orientación del agente
  ubicacion NextCasilla(const ubicacion &pos);

  // Devuelve el estado de aplicar a un estado una determinada accion en el nivel 0 y 1
  stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa);

  // Devuelve el estado de aplicar a un estado una determinada accion en el nivel 2
  stateN2 apply(const Action &a, const stateN2 &st, const vector<vector<unsigned char>> &mapa);

  // Devuelve el estado de aplicar a un estado una determinada accion en el nivel 3
  stateN3 apply(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa);

  // Encuentra si el elemento item está en la lista
  bool Find(const stateN0 &item, const list<stateN0> &lista);
  bool Find(const stateN0 &item, const list<nodeN0> &lista);

  // Función que pone a 0 todos los elementos de una matriz
  void AnulaMatriz(vector<vector<unsigned char>> &matriz);

  // Permite pintar sobre el mapa del simulador el plan partiendo desde el estado st
  void VisualizarPlan(const ubicacion &jugador, const ubicacion &sonambulo, const list<Action> &plan);

  // Indica si el sonámbulo está en nuestro rango de visión
  bool VeoSonambulo(const stateN0 &st);
  bool VeoSonambulo(const stateN3 &st);

  // Nos permite calcular el coste de una accion en el nivel 2 y 3
  int CalcularCoste(const Action &a, const stateN2 &st, const vector<vector<unsigned char>> &mapa);
  int CalcularCoste(const Action &a, const stateN3 &st, const vector<vector<unsigned char>> &mapa, const unsigned char tipo);

  // Heurística para nuestro algoritmo de A* para el agente sonámbulo
  int distanciaChebyshev(const ubicacion &inicio, const ubicacion &final);
};

#endif
