#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a realizar.
// Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;

	if (sensores.nivel != 4)
	{
		if (!hayPlan)
		{
			// Invocar metodo de búsqueda
			cout << "Calculando un nuevo plan" << endl;
			jugador.f = sensores.posF;
			jugador.c = sensores.posC;
			jugador.brujula = sensores.sentido;
			sonambulo.f = sensores.SONposF;
			sonambulo.c = sensores.SONposC;
			sonambulo.brujula = sensores.SONsentido;
			goal.f = sensores.destinoF;
			goal.c = sensores.destinoC;

			switch (sensores.nivel)
			{
			case 0:
				stateN0 state0;
				state0.jugador = jugador;
				state0.sonambulo = sonambulo;
				plan = AnchuraSoloJugador(state0, goal, mapaResultado);
				break;

			case 1:
				stateN0 state1;
				state1.jugador = jugador;
				state1.sonambulo = sonambulo;
				plan = AnchuraSonambulo(state1, goal, mapaResultado);
				break;

			case 2:
				stateN2 state2;
				state2.jugador = jugador;
				state2.sonambulo = sonambulo;

				if (mapaResultado[jugador.f][jugador.c] == 'K')
				{
					state2.bikini = true;
					state2.zapatillas = false;
				}
				else if (mapaResultado[jugador.f][jugador.c] == 'D')
				{
					state2.bikini = false;
					state2.zapatillas = true;
				}
				else
				{
					state2.bikini = false;
					state2.zapatillas = false;
				}

				plan = ProfundidadSoloJugador(state2, goal, mapaResultado);
				break;

			default:
				break;
			}

			if (plan.size() > 0)
			{
				VisualizarPlan(jugador, sonambulo, plan);
				hayPlan = true;
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
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
				{
					current_node = frontier.front();
				}
			}
		}
	}

	if (solutionFound)
	{
		plan = current_node.secuencia;
	}

	return plan;
}

list<Action> ComportamientoJugador::AnchuraSonambulo(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN1 current_node;
	list<nodeN1> frontier;
	set<nodeN1> explored;
	list<Action> plan;
	current_node.st = inicio;

	bool solutionFound = (current_node.st.sonambulo.f == final.f && current_node.st.sonambulo.c == final.c);

	frontier.push_back(current_node);

	while (!frontier.empty() && !solutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);

		if (VeoSonambulo(current_node.st)) // Si vemos al sonámbulo
		{
			// Generamos el hijo actSON_FORWARD
			nodeN1 child_son_forward = current_node;
			child_son_forward.st = apply(actSON_FORWARD, current_node.st, mapa);

			if (child_son_forward.st.sonambulo.f == final.f && child_son_forward.st.sonambulo.c == final.c)
			{
				child_son_forward.secuencia.push_back(actSON_FORWARD);
				current_node = child_son_forward;
				solutionFound = true;
			}
			else if (explored.find(child_son_forward) == explored.end())
			{
				child_son_forward.secuencia.push_back(actSON_FORWARD);
				frontier.push_back(child_son_forward);
			}

			if (!solutionFound)
			{
				// Generar hijo actSON_TURNsL
				nodeN1 child_son_turnsl = current_node;
				child_son_turnsl.st = apply(actSON_TURN_SL, current_node.st, mapa);

				if (explored.find(child_son_turnsl) == explored.end())
				{
					child_son_turnsl.secuencia.push_back(actSON_TURN_SL);
					frontier.push_back(child_son_turnsl);
				}

				// Generar hijo actSON_TURNSR
				nodeN1 child_son_turnsr = current_node;
				child_son_turnsr.st = apply(actSON_TURN_SR, current_node.st, mapa);

				if (explored.find(child_son_turnsr) == explored.end())
				{
					child_son_turnsr.secuencia.push_back(actSON_TURN_SR);
					frontier.push_back(child_son_turnsr);
				}
			}
		}

		if (!solutionFound)
		{
			// Generamos hijo actFORWARD
			nodeN1 child_forward = current_node;
			child_forward.st = apply(actFORWARD, current_node.st, mapa);

			if (explored.find(child_forward) == explored.end())
			{
				child_forward.secuencia.push_back(actFORWARD);
				frontier.push_back(child_forward);
			}

			// Generar hijo actTURNL
			nodeN1 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURNL
			nodeN1 child_turnr = current_node;
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
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
				{
					current_node = frontier.front();
				}
			}
		}
	}

	if (solutionFound)
	{
		plan = current_node.secuencia;
	}

	return plan;
}

list<Action> ComportamientoJugador::ProfundidadSoloJugador(const stateN2 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN2 current_node;			 // Nodo actual
	priority_queue<nodeN2> frontier; // Ahora la lista de abiertos es una cola de prioridad
	set<stateN2> explored;			 // Ahora el set de cerrados es de estados para usar el operador correcto
	list<Action> plan;
	current_node.st = inicio;
	int coste;

	bool solutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);
	frontier.push(current_node);

	while (!frontier.empty() && !solutionFound)
	{ // Realizamos la busqueda en profundidad
		frontier.pop();
		explored.insert(current_node.st);

		// Generamos el hijo actFORWARD
		nodeN2 child_forward = current_node;

		coste = CalcularCoste(actFORWARD, current_node.st, mapa); // Recordemos que el coste es en funcion de la casilla donde se parte

		child_forward.st = apply(actFORWARD, current_node.st, mapa);

		if (explored.find(child_forward.st) == explored.end())
		{ // Si hemos generado un nodo nuevo, lo metemos en abiertos
			child_forward.secuencia.push_back(actFORWARD);
			child_forward.coste_acumulado += coste;
			frontier.push(child_forward);
		}

		// Generamos el hijo actTURN_L
		nodeN2 child_turnl = current_node;

		coste = CalcularCoste(actTURN_L, current_node.st, mapa);
		child_turnl.st = apply(actTURN_L, current_node.st, mapa);

		if (explored.find(child_turnl.st) == explored.end())
		{
			child_turnl.secuencia.push_back(actTURN_L);
			child_turnl.coste_acumulado += coste;
			frontier.push(child_turnl);
		}

		// Generamos el hijo actTURN_R
		nodeN2 child_turnr = current_node;

		coste = CalcularCoste(actTURN_R, current_node.st, mapa);
		child_turnr.st = apply(actTURN_R, current_node.st, mapa);

		if (explored.find(child_turnr.st) == explored.end())
		{
			child_turnr.secuencia.push_back(actTURN_R);
			child_turnr.coste_acumulado += coste;
			frontier.push(child_turnr);
		}
		if (!frontier.empty())
		{
			current_node = frontier.top();
			while (!frontier.empty() && explored.find(current_node.st) != explored.end())
			{
				frontier.pop();

				if (!frontier.empty())
				{
					current_node = frontier.top();
				}
			}

			if (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c)
			{
				explored.insert(current_node.st);
				solutionFound = true;
			}
		}
	}

	if (solutionFound)
	{
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
		if (CasillaTransitable(siguiente_ubicacion, mapa) && !(siguiente_ubicacion.f == st.sonambulo.f && siguiente_ubicacion.c == st.sonambulo.c))
		{
			st_result.jugador = siguiente_ubicacion;
		}
		break;

	case actTURN_L:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
		break;

	case actTURN_R:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
		break;

	case actSON_FORWARD:
		siguiente_ubicacion = NextCasilla(st.sonambulo);
		if (CasillaTransitable(siguiente_ubicacion, mapa) && !(siguiente_ubicacion.f == st.jugador.f && siguiente_ubicacion.c == st.jugador.c))
		{
			st_result.sonambulo = siguiente_ubicacion;
		}
		break;

	case actSON_TURN_SL:
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 7) % 8);
		break;

	case actSON_TURN_SR:
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 1) % 8);
		break;
	}

	return st_result;
}

stateN2 ComportamientoJugador::apply(const Action &a, const stateN2 &st, const vector<vector<unsigned char>> &mapa)
{
	stateN2 st_result = st;
	ubicacion siguiente_ubicacion;

	switch (a)
	{
	case actFORWARD:
		siguiente_ubicacion = NextCasilla(st.jugador);
		if (CasillaTransitable(siguiente_ubicacion, mapa) && !(siguiente_ubicacion.f == st.sonambulo.f && siguiente_ubicacion.c == st.sonambulo.c))
		{
			st_result.jugador = siguiente_ubicacion;

			if (mapa[siguiente_ubicacion.f][siguiente_ubicacion.c] == 'K')
			{ // Conseguimos un bikini
				st_result.bikini = true;
				st_result.zapatillas = false; // No puedo tener dos objetos a la vez
			}
			else if (mapa[siguiente_ubicacion.f][siguiente_ubicacion.c] == 'D')
			{ // Consguimos unas zapatillas to guapas
				st_result.bikini = false;
				st_result.zapatillas = true;
			}
		}
		break;

	case actTURN_L:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
		break;

	case actTURN_R:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
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

void ComportamientoJugador::VisualizarPlan(const ubicacion &jugador, const ubicacion &sonambulo, const list<Action> &plan)
{
	AnulaMatriz(mapaConPlan);
	ubicacion j = jugador;
	ubicacion s = sonambulo;

	auto it = plan.begin();
	while (it != plan.end())
	{
		switch (*it)
		{
		case actFORWARD:
			j = NextCasilla(j);
			mapaConPlan[j.f][j.c] = 1;
			break;

		case actTURN_R:
			j.brujula = (Orientacion)((j.brujula + 2) % 8);
			break;

		case actTURN_L:
			j.brujula = (Orientacion)((j.brujula + 6) % 8);
			break;

		case actSON_FORWARD:
			s = NextCasilla(s);
			mapaConPlan[s.f][s.c] = 2;
			break;

		case actSON_TURN_SR:
			s.brujula = (Orientacion)((s.brujula + 1) % 8);
			break;

		case actSON_TURN_SL:
			s.brujula = (Orientacion)((s.brujula + 7) % 8);
			break;
		}

		it++;
	}
}

bool ComportamientoJugador::VeoSonambulo(const stateN0 &st)
{
	switch (st.jugador.brujula)
	{
	case norte:
		for (int i = 1; i < 4; i++)
		{
			if (st.jugador.f - i == st.sonambulo.f)
			{
				switch (i)
				{
				case 1:
					for (int j = -1; j < 2; j++)
					{
						if (st.jugador.c + j == st.sonambulo.c)
						{
							return true;
						}
					}
					break;

				case 2:
					for (int j = -2; j < 3; j++)
					{
						if (st.jugador.c + j == st.sonambulo.c)
						{
							return true;
						}
					}
					break;

				case 3:
					for (int j = -3; j < 4; j++)
					{
						if (st.jugador.c + j == st.sonambulo.c)
						{
							return true;
						}
					}
					break;
				}
			}
		}
		break;

	case este:
		for (int i = 1; i < 4; i++)
		{
			if (st.jugador.c + i == st.sonambulo.c)
			{
				switch (i)
				{
				case 1:
					for (int j = -1; j < 2; j++)
					{
						if (st.jugador.f + j == st.sonambulo.f)
						{
							return true;
						}
					}
					break;

				case 2:
					for (int j = -2; j < 3; j++)
					{
						if (st.jugador.f + j == st.sonambulo.f)
						{
							return true;
						}
					}
					break;

				case 3:
					for (int j = -3; j < 4; j++)
					{
						if (st.jugador.f + j == st.sonambulo.f)
						{
							return true;
						}
					}
					break;
				}
			}
		}
		break;

	case sur:
		for (int i = 1; i < 4; i++)
		{
			if (st.jugador.f + i == st.sonambulo.f)
			{
				switch (i)
				{
				case 1:
					for (int j = -1; j < 2; j++)
					{
						if (st.jugador.c + j == st.sonambulo.c)
						{
							return true;
						}
					}
					break;

				case 2:
					for (int j = -2; j < 3; j++)
					{
						if (st.jugador.c + j == st.sonambulo.c)
						{
							return true;
						}
					}
					break;

				case 3:
					for (int j = -3; j < 4; j++)
					{
						if (st.jugador.c + j == st.sonambulo.c)
						{
							return true;
						}
					}
					break;
				}
			}
		}
		break;

	case oeste:
		for (int i = 1; i < 4; i++)
		{
			if (st.jugador.c - i == st.jugador.c)
			{
				switch (i)
				{
				case 1:
					for (int j = -1; j < 2; j++)
					{
						if (st.jugador.f + j == st.sonambulo.f)
						{
							return true;
						}
					}
					break;

				case 2:
					for (int j = -2; j < 3; j++)
					{
						if (st.jugador.f + j == st.sonambulo.f)
						{
							return true;
						}
					}
					break;

				case 3:
					for (int j = -3; j < 4; j++)
					{
						if (st.jugador.f + j == st.sonambulo.f)
						{
							return true;
						}
					}
					break;
				}
			}
		}
		break;
	}

	return false;
}

int ComportamientoJugador::CalcularCoste(const Action &a, const stateN2 &st, const vector<vector<unsigned char>> &mapa)
{
	char casilla = mapa[st.jugador.f][st.jugador.c];

	switch (a)
	{
	case actFORWARD:
		if (casilla == 'A')
		{ // Estamos en agua
			if (st.bikini)
			{
				return 10;
			}
			else
			{
				return 100;
			}
		}
		else if (casilla == 'B')
		{ // Estamos en bosque
			if (st.zapatillas)
			{
				return 15;
			}
			else
			{
				return 50;
			}
		}
		else if (casilla == 'T')
		{ // Terreno
			return 2;
		}
		else
		{
			return 1;
		}
		break;

	case actTURN_L:
	case actTURN_R:
		if (casilla == 'A')
		{ // Estamos en agua
			if (st.bikini)
			{
				return 5;
			}
			else
			{
				return 25;
			}
		}
		else if (casilla == 'B')
		{ // Estamos en bosque
			if (st.zapatillas)
			{
				return 1;
			}
			else
			{
				return 5;
			}
		}
		else if (casilla == 'T')
		{
			return 2;
		}
		else
		{
			return 1;
		}
		break;
	}
}