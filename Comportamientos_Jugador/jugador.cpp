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
		cout << "Calculando un nuevo plan" << endl;
		c_state.jugador.f = sensores.posF;
		c_state.jugador.c = sensores.posC;
		c_state.jugador.brujula = sensores.sentido;
		c_state.sonambulo.f = sensores.SONposF;
		c_state.sonambulo.c = sensores.SONposC;
		c_state.sonambulo.brujula = sensores.SONsentido;
		goal.f = sensores.destinoF;
		goal.c = sensores.destinoC;

		plan = AnchuraSoloJugador(c_state, goal, mapaResultado);
		VisualizarPlan(c_state, plan);
		hayPlan = true;

		if (hayPlan)
		{
			cout << "Se encontró un plan" << endl;
		}
	}

	if (hayPlan && plan.size() > 0)
	{
		cout << "Ejecutando la siguiente acción del plan" << endl;
		accion = plan.front();
		plan.pop_front();
	}

	if (plan.size() == 0)
	{
		cout << "Se completó el plan" << endl;
		hayPlan = false;
	}

	// Incluir aquí el comportamiento del agente jugador

	return accion;
}

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

list<Action> ComportamientoJugador::AnchuraSoloJugador(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN0 current_node;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;

	bool solutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);

	frontier.push_back(current_node);

	while (!frontier.empty() && !solutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWARD
		nodeN0 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);

		if (child_forward.st.jugador.f == final.f && child_forward.st.jugador.c == final.c)
		{
			child_forward.secuencia.push_back(actFORWARD);
			current_node = child_forward;
			solutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actFORWARD);
			frontier.push_back(child_forward);
		}

		if (!solutionFound)
		{
			// Generar hijo actTURN L
			nodeN0 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN R
			nodeN0 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!solutionFound && !frontier.empty())
		{
			current_node = frontier.front();
			while(!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop_front();
				current_node = frontier.front();
			}
		}
	}

	if(solutionFound){
		plan = current_node.secuencia;
	}

	return plan;
}

bool ComportamientoJugador::CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa)
{
	return (mapa[x.f][x.c] != 'P' && mapa[x.f][x.c] != 'M');
}

ubicacion ComportamientoJugador::NextCasilla(const ubicacion &pos)
{
	ubicacion next = pos;

	switch (pos.brujula)
	{
	case norte:
		next.f--;
		break;

	case noreste:
		next.f--;
		next.c++;
		break;
		;

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

stateN0 ComportamientoJugador::apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa)
{
	stateN0 st_result = st;

	ubicacion siguiente_ubicacion;

	switch (a)
	{
	case actFORWARD:
		siguiente_ubicacion = NextCasilla(st.jugador);
		if (CasillaTransitable(siguiente_ubicacion, mapa) && !(siguiente_ubicacion == st.sonambulo))
		{
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

bool ComportamientoJugador::Find(const stateN0 &item, const list<stateN0> &lista)
{
	auto it = lista.begin();

	while (it != lista.end() && !((*it) == item))
	{
		it++;
	}

	return (!(it == lista.end()));
}

bool ComportamientoJugador::Find(const stateN0 &item, const list<nodeN0> &lista)
{
	auto it = lista.begin();

	while (it != lista.end() && !(it->st == item))
	{
		it++;
	}

	return (!(it == lista.end()));
}

void ComportamientoJugador::AnulaMatriz(vector<vector<unsigned char>> &matriz)
{
	for (int i = 0; i < matriz.size(); i++)
	{
		for (int j = 0; j < matriz[i].size(); j++)
		{
			matriz[i][j] = 0;
		}
	}
}

void ComportamientoJugador::VisualizarPlan(const stateN0 &st, const list<Action> &plan)
{
	AnulaMatriz(mapaConPlan);
	stateN0 cst = st;

	auto it = plan.begin();
	while (it != plan.end())
	{
		switch (*it)
		{
		case actFORWARD:
			cst.jugador = NextCasilla(cst.jugador);
			mapaConPlan[cst.jugador.f][cst.jugador.c] = 1;
			break;

		case actTURN_R:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 2) % 8);
			break;

		case actTURN_L:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 6) % 8);
			break;

		case actSON_FORWARD:
			cst.sonambulo = NextCasilla(cst.sonambulo);
			mapaConPlan[cst.sonambulo.f][cst.sonambulo.c] = 2;
			break;

		case actSON_TURN_SR:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 1) % 8);
			break;

		case actSON_TURN_SL:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 7) % 8);
			break;
		}

		it++;
	}
}