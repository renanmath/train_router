import json
from copy import deepcopy
from train_route.solver.solver import ModelSolver


class Simulator:
    """
    Simulator class. It simulates a train route operation
    """

    def __init__(self, initial_params: dict, demands: list, time_horizon: float,
                 correction_limit: float = 2.0, print_results: bool = False):
        """
        Constructor method
        :param initial_params:
        :param time_horizon:
        """

        self.__initial_params: dict = initial_params
        self.__time_horizon: float = time_horizon
        self.__print_results = print_results
        self.__iterations: int = 0
        self.__simulation_time: float = 0
        self.__correction_limit = correction_limit
        self.__solver: ModelSolver = ModelSolver(debug=True)
        self.__simulation_history: list = []
        self.__total_demands: list = demands
        self.__remaining_demands = deepcopy(demands)
        self.__remaining_demands_ratio = None
        self.__expected_remaining_demands = None
        self.__initial_demand_ratio: list = [[round(x / self.__time_horizon) for x in dm] for dm in
                                             self.__total_demands]
        self.__big_m = sum(sum(dms) for dms in self.__total_demands)

        self.__print_initial_condition()

    def __print_initial_condition(self):
        print("Iniciando otimização")
        print("Condições iniciais\n")

        num_terminals = len(self.__initial_params['terminals']['demands'])

        for ter_idx in range(num_terminals):
            print(f"Terminal {ter_idx}:")
            print(f'\tDemanda: '
                  f'{self.__initial_params["terminals"]["demands"][ter_idx]}')
            print(f'\tEstoque: '
                  f'{self.__initial_params["terminals"]["stocks"][ter_idx]}\n')

    def __solve_iteration(self, params):

        simulation_solution = self.__solver.solve(params)
        status = simulation_solution['status']
        if status != "ok":
            print("ERRO! Algo de muito errado aconteceu.")
            return self.__simulation_history

        termination_condition = simulation_solution['termination_condition']
        if termination_condition == "infeasible":
            if self.__print_results:
                print("Solução infactível")
                print("Tentando relaxar as restrições de demanda")
            go_no_restriction = True
            for f in [0.9, 0.75, 0.5, 0.25, 0.1, 0.01]:
                params_ = deepcopy(params)
                for idx, dms in enumerate(params_['terminals']['demands']):
                    params_['terminals']['demands'][idx] = [f * x for x in dms]

                simulation_solution = self.__solver.solve(params_)
                termination_condition = simulation_solution['termination_condition']
                if termination_condition == "optimal":
                    go_no_restriction = False
                    if self.__print_results:
                        print(f"Solução factível a {round(f * 100)}% das demandas")
                    break

            if go_no_restriction:
                if self.__print_results:
                    print("Tentando solução sem restrições de demanda")
                simulation_solution = self.__solver.solve(params, restrict_demand=False)

        return simulation_solution

    def simulate(self):
        """
        Simulate a train route schedule, based on demand
        :return:
        """
        previous_params = self.__initial_params
        params = self.__initial_params
        while self.__simulation_time < self.__time_horizon:
            self.__iterations += 1
            if self.__print_results:
                print(f"\n####### Iteration {self.__iterations} #######")

            partial_demands = self.__compute_partial_demand_cut(params)
            params['terminals']['demands'] = partial_demands

            simulation_solution = self.__solve_iteration(params)
            termination_condition = simulation_solution['termination_condition']

            if termination_condition == "optimal":
                self.__simulation_history.append(simulation_solution)
                if self.__print_results:
                    self.__solver.print_results()
                    print("Demanda para esta iteração:")
                    print(partial_demands)
                params = self.__compute_next_parameters(previous_params, simulation_solution)
                previous_params = params
                _ = self.__compute_queue_time(params)
                self.__simulation_time += simulation_solution['simulation_time']
                if self.__print_results:
                    print("Tempo de simulação = ", self.__simulation_time)
            else:
                if self.__print_results:
                    print("Solution: ", termination_condition)
                return self.__simulation_history, self.__remaining_demands_ratio

        print("\nDemanda inicial:")
        print(self.__total_demands)
        self.__actualize_remaining_demands(params)
        print("Percentual da demanda restante:")
        print(self.__remaining_demands_ratio)
        print("Total de vagões despachados: ",
              int(sum([sim_sol['total_wagons'] for sim_sol in self.__simulation_history])))
        return self.__simulation_history, self.__remaining_demands_ratio

    def __compute_next_parameters(self, previous_params: dict, simulation_results: dict):
        """

        :param previous_state:
        :param simulation_results:
        :return:
        """

        # find next state
        new_params = {}

        # simulation time
        tau = simulation_results["simulation_time"]

        # terminals

        load_times = previous_params['terminals']['load_times']
        unload_times = previous_params['terminals']['unload_times']
        max_num_trains = previous_params['terminals']['max_num_trains']
        demands = previous_params['terminals']['demands']
        stocks = previous_params['terminals']['stocks']

        m = len(unload_times)

        new_params['products'] = deepcopy(previous_params['products'])
        new_params['trains'] = deepcopy(previous_params['trains'])
        new_params['terminals'] = {"trains": [[] for _ in range(m)],
                                   "load_times": load_times,
                                   "unload_times": unload_times,
                                   "demands": demands,
                                   "stocks": stocks,
                                   "max_num_trains": max_num_trains}
        new_params['routes'] = [[{"max_trains": None,
                                  "transit_time": None,
                                  "eta": []} for _ in range(m)] for _ in range(m)]
        routes_info = previous_params['routes']

        trains_at_terminals = previous_params['terminals']['trains']
        prev_queue_time = {i: {"first": True, "value": None} for i in range(m)}

        all_train_locations = {}
        # recalculate the arrivals per terminal
        #############################################################################
        for k in range(m):
            prev_queue_time_k = max([x['operation_time'] for x in trains_at_terminals[k]] + [0])
            trains_arriving_at_terminal_k = []
            origins = {}
            for j in range(m):
                trains_arriving_at_terminal_k += previous_params['routes'][j][k]['eta']
                for x in previous_params['routes'][j][k]['eta']:
                    origins[x['train_id']] = j

            trains_arriving_at_terminal_k.sort(key=lambda x: x['eta'])

            for train_info in trains_arriving_at_terminal_k:
                t = train_info['train_id']
                prev_eta = train_info['eta']

                if prev_eta <= tau:
                    # train arrived at terminal k
                    all_train_locations[t] = [k]
                    queue_time_k = max(prev_queue_time_k - prev_eta, 0)
                    new_opt = queue_time_k + prev_eta + unload_times[k]
                    prev_queue_time_k = new_opt
                    new_params['terminals']['trains'][k].append({"train_id": t, "operation_time": new_opt - tau})
                else:
                    # train didn't arrive yet at terminal k
                    index_j = origins[t]
                    all_train_locations[t] = [index_j, k]
                    new_params['routes'][index_j][k]['eta'].append({"train_id": t, "eta": prev_eta - tau})

        ###################################################################################
        for j in range(m):
            control_list = []
            trains_at_terminal_j = trains_at_terminals[j]

            for k in range(m):
                if prev_queue_time[k]['value'] is None:
                    prev_queue_time[k]['value'] = max([x['operation_time'] for x in trains_at_terminals[k]] + [0])
                dispatched_trains_terminal_jk = {x['train_id']: x['to']
                                                 for x in simulation_results['trains'] if
                                                 x['from'] == j and x['to'] == k}
                routes_jk = routes_info[j][k]
                tt = routes_jk['transit_time']
                new_params['routes'][j][k]['transit_time'] = tt

                for train_info in trains_at_terminal_j:
                    t = train_info['train_id']
                    prev_opt = train_info['operation_time']

                    if t in dispatched_trains_terminal_jk:
                        # train was dispatched from terminal j to terminal k
                        assert prev_opt <= tau
                        all_train_locations[t] = [j, k]
                        time_in_travel = tau - prev_opt - load_times[j]
                        eta = tt - time_in_travel
                        new_params['routes'][j][k]['eta'].append({"train_id": t, "eta": eta})

                    elif tau < prev_opt and t not in all_train_locations:
                        # train didn't finish the operation at terminal j
                        all_train_locations[t] = [j]
                        control_list.append(t)
                        new_opt = prev_opt - tau
                        new_params['terminals']['trains'][j].append({"train_id": t, "operation_time": new_opt})
                    else:
                        pass

        new_params['trains']['locations'] = all_train_locations

        return new_params

    def __compute_partial_demand_cut(self, params: dict):
        """

        :param params:
        :return:
        """
        remaining_time_horizon = self.__time_horizon - self.__simulation_time
        expected_simulation_time = self.__compute_expected_simulation_time(params)

        self.__actualize_remaining_demands(params)
        self.__actualize_expected_remaining_demands()
        partial_demands = []

        for p, dms in enumerate(self.__remaining_demands):
            pd0 = [round(expected_simulation_time * (x / remaining_time_horizon)) for x in dms]
            fs = [
                self.__expected_remaining_demands[p][j] / self.__remaining_demands[p][j] if self.__remaining_demands[p][
                                                                                                j] > 0 else 0
                for j in range(len(pd0))]

            multiplicative_factor = [max(0, min(1, self.__correction_limit - f)) for f in fs]
            new_partial_demand = [round(multiplicative_factor[i] * pd0[i]) for i in range(len(pd0))]
            partial_demands.append(new_partial_demand)

        return partial_demands

    def __actualize_remaining_demands(self, params):
        if len(self.__simulation_history) > 0:
            simulation_solution = self.__simulation_history[-1]
            dispatched_trains = simulation_solution['trains']
            mean_weight = params['products']['mean_weight']
            total_dispacheted = [[0 for _ in range(len(mean_weight))] for _ in range(len(self.__remaining_demands))]
            for train in dispatched_trains:
                to = train['to']
                num_wagons = train['wagons']
                for i in range(len(mean_weight)):
                    weight = mean_weight[i] * num_wagons[i]
                    total_dispacheted[to][i] += weight

            if self.__print_results:
                print("Demanda realizada na iteração anterior:")
                print(total_dispacheted)
            self.__remaining_demands_ratio = []
            for i, dispatched in enumerate(total_dispacheted):
                self.__remaining_demands_ratio.append([])
                for j, value in enumerate(dispatched):
                    self.__remaining_demands[i][j] = max(self.__remaining_demands[i][j] - value, 0)
                    # self.__remaining_demands[i][j] = self.__remaining_demands[i][j] - value
                    self.__remaining_demands_ratio[i].append(
                        self.__remaining_demands[i][j] / self.__total_demands[i][j] if self.__total_demands[i][
                                                                                           j] > 0 else 0)

            if self.__print_results:
                print("Demanda total restante:")
                print(self.__remaining_demands)

    def __actualize_expected_remaining_demands(self):
        new_expected_remaining_demands = []
        for l, initial_demand in enumerate(self.__total_demands):
            new_expected_remaining_demands.append([])
            for k, dm in enumerate(initial_demand):
                initial_demand_ratio = self.__initial_demand_ratio[l][k]
                new_expected_remaining_demands[l].append(dm - self.__simulation_time * initial_demand_ratio)

        self.__expected_remaining_demands = new_expected_remaining_demands

    @staticmethod
    def __compute_expected_simulation_time(params: dict):
        """

        :param params:
        :return:
        """
        all_trains_en_route = []
        for route_i in params['routes']:
            for route_ij in route_i:
                all_trains_en_route += route_ij['eta']

        min_est = min(
            [100] + [x['eta'] + params['terminals']['unload_times'][params['trains']['locations'][x['train_id']][1]]
                     for x in all_trains_en_route])
        return min_est

    @staticmethod
    def __compute_queue_time(params: dict):
        queue_times = []
        msg = "| "
        for i, trains in enumerate(params['terminals']['trains']):
            max_op = max([0] + [train['operation_time'] for train in trains])
            min_op = min([0] + [train['operation_time'] for train in trains])
            un_time = params['terminals']['unload_times'][i]
            expected_op = min_op + un_time * max(0, len(trains) - 1)
            queue_time = max(0, max_op - expected_op)
            queue_times.append(queue_time)
            msg += f"Terminal {i}: {queue_time} horas | "

        print("Tempos de fila:")
        print(msg)
        return queue_times


if __name__ == "__main__":
    with open(
            "/home/renan/Documentos/Estudos/Pesquisa operacional e programação "
            "inteira/Projetos/train_route/train_route/input_example.json") as file:
        initial_params = json.load(file)

    total_demands = [
        [0, 12000, 0, 0],
        [0, 0, 32000, 2300],
        [0, 25000, 0, 5400],
        [0, 0, 0, 0]
    ]

    days = 7
    # best params: time horizon = 168 h and multiplicative factor = 1.897
    simulator = Simulator(initial_params, total_demands, 168, 1.897, True)
    simulator.simulate()
