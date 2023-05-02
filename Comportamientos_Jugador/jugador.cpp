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

	if (sensores.nivel != 4)
	{
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

			switch (sensores.nivel)
			{
			case 0:
				plan = AnchuraSoloJugador(c_state, goal, mapaResultado);
				break;

			case 1:
				// plan = AnchuraSonambulo(c_state, goal, mapaResultado);
				plan = AnchuraSonambulo(c_state, goal, mapaResultado);
				break;

			default:
				break;
			}

			if (plan.size() > 0)
			{
				VisualizarPlan(c_state, plan);
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

list<Action> ComportamientoJugador::AnchuraSoloJugador(const stateN &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
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

list<Action> ComportamientoJugador::AnchuraSonambulo(const stateN &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
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

		if(!solutionFound && !frontier.empty()){
			current_node = frontier.front();
			while(!frontier.empty() && explored.find(current_node) != explored.end()){
				frontier.pop_front();
				if(!frontier.empty()){
					current_node = frontier.front();
				}
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

stateN ComportamientoJugador::apply(const Action &a, const stateN &st, const vector<vector<unsigned char>> &mapa)
{
	stateN st_result = st;

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

bool ComportamientoJugador::Find(const stateN &item, const list<stateN> &lista)
{
	auto it = lista.begin();

	while (it != lista.end() && !((*it) == item))
	{
		it++;
	}

	return (!(it == lista.end()));
}

bool ComportamientoJugador::Find(const stateN &item, const list<nodeN0> &lista)
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

void ComportamientoJugador::VisualizarPlan(const stateN &st, const list<Action> &plan)
{
	AnulaMatriz(mapaConPlan);
	stateN cst = st;

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

bool ComportamientoJugador::VeoSonambulo(const stateN &st)
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