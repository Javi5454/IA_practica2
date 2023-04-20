#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;

	if (!hayPlan)
	{
		// Invocar metodo de búsqueda
		hayPlan = true;
	}

	if (hayPlan && plan.size() > 0)
	{
		accion = plan.front();
		plan.pop_front();
	}

	if (plan.size() == 0)
	{
		hayPlan = false;
	}

	// Incluir aquí el comportamiento del agente jugador

	return accion;
}

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

bool ComportamientoJugador::AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	stateN0 current_state = inicio;
	list<stateN0> frontier;
	list<stateN0> explored;
	frontier.push_back(current_state);

	bool solutionFound = (current_state.jugador.f == final.f && current_state.jugador.c == final.c);

	while(!frontier.empty() && !solutionFound){
		frontier.pop_front();
		explored.push_back(current_state);

		//Generar hijo actFORWARD
		stateN0 child_forward = apply(actFORWARD, current_state, mapa);

		if(child_forward.jugador.f == final.f && child_forward.jugador.c == final.c){
			current_state = child_forward;
			solutionFound = true;
		}
	}
}

bool ComportamientoJugador::CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa){
	return (mapa[x.f][x.c] != 'P' && mapa[x.f][x.c] != 'M');
}

ubicacion ComportamientoJugador::NextCasilla(const ubicacion &pos){
	ubicacion next = pos;

	switch (pos.brujula)
	{
	case norte:
		next.f--;
		break;
	
	case noreste:
		next.f--;
		next.c++;
		break;;

	case este:
		next.c++;
		break;

	case sureste:
		next.f++;
		next.c++;
		break;

	case sur:
		next.f++;
		break;

	case suroeste:
		next.f++;
		next.c--;
		break;

	case oeste:
		next.c--;
		break;

	case noroeste:
		next.f--;
		next.c--;
		break;
	
	default:
		break;
	}

	return next;
}

stateN0 ComportamientoJugador::apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa){
	stateN0 st_result = st;

	ubicacion siguiente_ubicacion;

	switch (a)
	{
	case actFORWARD:
		siguiente_ubicacion = NextCasilla(st.jugador);
		if(CasillaTransitable(siguiente_ubicacion, mapa) && !(siguiente_ubicacion == st.sonambulo)){
			st_result.jugador = siguiente_ubicacion;
		}
		break;

	case actTURN_L:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);

	case actTURN_R:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
	
	default:
		break;
	}

	return st_result;
}

bool ComportamientoJugador::Find(const stateN0 &item, const list<stateN0> &lista){
	auto it = lista.begin();

	while(it != lista.end() && !((*it) == item)){
		it++;
	}

	return (!(it == lista.end()));
}