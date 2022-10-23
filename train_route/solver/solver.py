from pyomo.environ import *


class ModelSolver:
    """
    Class that implement the local train route model
    """

    def __init__(self, debug: bool = False):
        """
        Constructor model
        :param debug:
        """
        self.__debug = debug
        self.__model = None

        self.__trains = None
        self.__terminals = None
        self.__products = None

        self.__num_wagons = None
        self.__mean_weight = None
        self.__load_times = None
        self.__unload_times = None
        self.__max_num_trains = None

        self.__tt_list = None
        self.__demands = None
        self.__stocks = None

        self.__remaining_op = None
        self.__train_locations = None

        self.__max_trains_per_route = None
        self.__trains_en_route = None
        self.__eta = None

        self.__results = None

    def solve(self, params_dict: dict, restrict_demand: bool = True, restrict_stock: bool = True):
        """
        Solve the MILP problem
        Receives a dictionary as parameter, with the following structure:
        {"terminals":Dict{"load_times":List[float],
                    "unload_times": List[float],
                    "max_num_trains": List[int],
                    "demands": List[List[float]],
                    "stocks": List[List[float]],
                    "trains": List[List[Dict{"train_id": int, "operation_time": float}]]},
        "products": Dict{"mean_weight": List[float]}
        "trains": Dict{"num_wagons":List[int]}
        "routes":List[List[Dict{
                                "max_trains": int,
                                "transit_time":float,
                                "eta": List[Dict{"train_id":int, "eta":float}]}]]
        }
        :param restrict_demand:
        :param params_dict:
        :return:
        """

        if self.__model is None:
            self.__model = ConcreteModel()
        else:
            self.__clear_model_parameters()

        self.__create_model_parameters(params_dict)

        self.__set_objective_function()

        self.__set_all_restrictions(restrict_demand, restrict_stock)

        optimizer = SolverFactory("glpk")
        self.__results = optimizer.solve(self.__model, tee=False)

        sim_solution = self.__create_solution_from_solver_results()
        return sim_solution

    def __create_solution_from_solver_results(self):
        solution_dict = {"status": self.__results.solver.status.value,
                         "termination_condition": self.__results.solver.termination_condition.value}
        if self.__results.solver.status.value != "ok":
            print("Deu algo muito errado")
        if self.__results.solver.termination_condition.value == 'optimal':
            solution_dict["simulation_time"] = self.__model.tau.value
            solution_dict["total_wagons"] = self.__model.obj.expr() - self.__model.tau.value
            solution_dict["trains"] = []

            for t in self.__trains:
                for i in self.__terminals:
                    for j in self.__terminals:
                        if self.__model.x[t, i, j].value > 0:
                            info = {"train_id": str(t), "from": i, "to": j, "wagons": []}
                            solution_dict['trains'].append(info)
                            for ll in self.__products:
                                info['wagons'].append(int(self.__model.nu[t, i, j, ll].value))

        return solution_dict

    def print_results(self):
        print("Solution: ", self.__results.solver.termination_condition)
        print('________________________________')
        if self.__results.solver.termination_condition != "optimal":
            return 0
        time_ = self.__model.tau.value
        total_wagons = self.__model.obj.expr() - time_

        print(f'Total de vagões despachados: {int(total_wagons)}')
        print(f'Horizonte de tempo simulado: {time_} horas')
        print("------")
        for t in self.__trains:
            for i in self.__terminals:
                for j in self.__terminals:
                    if self.__model.x[t, i, j].value is not None and self.__model.x[t, i, j].value > 0:
                        print(f'Trem {t} despachado do Terminal {i} para Terminal {j}')
                        msg = "| "
                        for k in self.__products:
                            msg += f'{int(self.__model.nu[t, i, j, k].value)} vagões do produto {k} | '
                        print(msg)
                        print("------")

    def __set_all_restrictions(self, restrict_demand, restrict_stock):

        self.__set_basic_restrictions()
        self.__set_traffic_restrictions(restrict_demand)
        self.__set_demand_and_stock_restrictions(restrict_demand, restrict_stock)
        self.__set_simulation_time_restrictions()

    def __set_simulation_time_restrictions(self):
        for t in range(len(self.__trains)):
            for i in range(len(self.__terminals)):
                for j in range(len(self.__terminals)):
                    try:
                        self.__model.constrs.add(
                            expr=self.__model.x[t, i, j] * self.__remaining_op[t][i] <= self.__model.tau)
                    except ValueError:
                        pass
                    try:
                        self.__model.constrs.add(
                            expr=self.__trains_en_route[t][i][j] * self.__model.tau <= self.__trains_en_route[t][i][
                                j] * (
                                         self.__eta[t][i][j] + self.__unload_times[j]))
                    except ValueError:
                        pass

                    for k in range(len(self.__terminals)):
                        self.__model.constrs.add(
                            expr=self.__model.tau <= self.__remaining_op[t][j] + self.__load_times[j] +
                                 self.__tt_list[j][k] + 100 * (
                                         1 - self.__model.x[t, j, k]))

    def __set_demand_and_stock_restrictions(self, restrict_demand, restrict_stock):

        # Restrictions on stock per product
        if restrict_stock:
            for ll in range(len(self.__products)):
                for j in range(len(self.__terminals)):
                    self.__model.constrs.add(
                        expr=sum(self.__model.nu[t, j, k, ll] * self.__mean_weight[ll] for t in self.__trains for k in
                                 self.__terminals) <=
                             self.__stocks[j][
                                 ll])

        # Restrictions on demands per product
        if restrict_demand:
            for ll in range(len(self.__products)):
                for k in range(len(self.__terminals)):
                    self.__model.constrs.add(
                        expr=sum(self.__model.nu[t, j, k, ll] * self.__mean_weight[ll] for t in self.__trains for j in
                                 self.__terminals) >=
                             self.__demands[k][
                                 ll])

    def __set_traffic_restrictions(self, restrict_demand: bool = False):

        # Restrictions on arrivals at a node
        for k in range(len(self.__terminals)):
            self.__model.constrs.add(
                expr=sum(self.__model.x[t, j, k] for t in self.__trains for j in self.__terminals) <=
                     self.__max_num_trains[k])

        # Restrictions on traffic on a route
        for j in range(len(self.__terminals)):
            for k in range(len(self.__terminals)):
                self.__model.constrs.add(
                    expr=sum(self.__model.x[t, j, k] for t in self.__trains) <= self.__max_trains_per_route[j][k])

        # Restrictions on number of wagons per train
        for t in range(len(self.__trains)):
            for j in range(len(self.__terminals)):
                for k in range(len(self.__terminals)):
                    self.__model.constrs.add(
                        expr=sum(self.__model.nu[t, j, k, la] for la in self.__products) <= self.__model.x[t, j, k] *
                             self.__num_wagons[t])

                    for ll in self.__products:
                        self.__model.constrs.add(
                            expr=self.__model.nu[t, j, k, ll] * self.__mean_weight[ll] >= 0.25 * self.__demands[k][
                                ll] * self.__model.x[t, j, k])

    def __set_basic_restrictions(self):

        self.__model.constrs = ConstraintList()
        # restrições de destino único
        for t in range(len(self.__trains)):
            self.__model.constrs.add(
                expr=sum(self.__model.x[t, i, j] for i in self.__terminals for j in self.__terminals) <= 1)
        # Restrições de factibilidade
        for t in range(len(self.__trains)):
            for j in range(len(self.__terminals)):
                for k in range(len(self.__terminals)):
                    self.__model.constrs.add(expr=self.__model.x[t, j, k] <= self.__train_locations[t][j])
                    if j == k:
                        self.__model.constrs.add(expr=self.__model.x[t, j, k] == 0)

    def __set_objective_function(self):
        # definição das variáveis
        self.__model.tau = Var(domain=NonNegativeReals)  # tempo da simulação
        self.__model.x = Var(self.__trains, self.__terminals, self.__terminals, within=Binary)
        self.__model.nu = Var(self.__trains, self.__terminals, self.__terminals, self.__products,
                              within=NonNegativeIntegers)
        # função objetivo (maximizar tempo de simulação e número de vagões despachados)
        self.__model.obj = Objective(expr=self.__model.tau + sum(self.__model.nu[t, j, k, ll] for t in self.__trains
                                                                 for j in self.__terminals
                                                                 for k in self.__terminals
                                                                 for ll in self.__products),
                                     sense=maximize)

    def __clear_model_parameters(self):
        self.__model = ConcreteModel()

        self.__trains = None
        self.__terminals = None
        self.__products = None

        self.__num_wagons = None
        self.__mean_weight = None
        self.__load_times = None
        self.__unload_times = None
        self.__max_num_trains = None

        self.__tt_list = None
        self.__demands = None
        self.__stocks = None

        self.__remaining_op = None
        self.__train_locations = None

        self.__max_trains_per_route = None
        self.__trains_en_route = None
        self.__eta = None

        self.__results = None

    def __create_model_parameters(self, params_dict):

        # one index parameters
        self.__trains = list(range(len(params_dict['trains']['num_wagons'])))
        self.__terminals = list(range(len(params_dict['terminals']['load_times'])))
        self.__products = list(range(len(params_dict['products']['mean_weight'])))
        self.__num_wagons = params_dict['trains']['num_wagons']
        self.__mean_weight = params_dict['products']['mean_weight']
        self.__load_times = params_dict['terminals']['load_times']
        self.__unload_times = params_dict['terminals']['unload_times']
        self.__max_num_trains = params_dict['terminals']['max_num_trains']
        # two indexes parameters
        self.__tt_list = [[x[i]['transit_time'] for i in range(len(x))]
                          for x in params_dict['routes']]
        self.__demands = params_dict['terminals']['demands']
        self.__stocks = params_dict['terminals']['stocks']
        self.__remaining_op = [[0] * len(self.__terminals) for _ in range(len(self.__trains))]
        self.__train_locations = [[0] * len(self.__terminals) for _ in range(len(self.__trains))]
        for j in self.__terminals:
            train_list = params_dict['terminals']['trains'][j]
            if len(train_list) == 0:
                continue
            for train in train_list:
                t = int(train['train_id'])
                op = train['operation_time']
                self.__remaining_op[t][j] = op
                self.__train_locations[t][j] = 1
        self.__max_trains_per_route = [[x[i]['max_trains'] for i in range(len(x))]
                                       for x in params_dict['routes']]
        # three indexes parameters

        self.__trains_en_route = [[[0] * len(self.__terminals) for _ in range(len(self.__terminals))]
                                  for _ in range(len(self.__trains))]
        self.__eta = [[[0] * len(self.__terminals) for _ in range(len(self.__terminals))]
                      for _ in range(len(self.__trains))]
        for i in self.__terminals:
            for j in self.__terminals:
                eta_info = params_dict['routes'][i][j]['eta']
                if len(eta_info) == 0:
                    continue

                for train in eta_info:
                    t = int(train['train_id'])
                    train_eta = train['eta']
                    self.__eta[t][i][j] = train_eta
                    self.__trains_en_route[t][i][j] = 1


if __name__ == "__main__":
    load_times = [8] * 4
    unload_times = [7] * 4
    max_num_trains = [4, 3, 4, 4]
    demands = [[1200, 0, 0],
               [0, 500, 800],
               [1500, 0, 600],
               [0, 0, 0]]

    stocks = [[0, 5000, 3600],
              [8000, 0, 0],
              [0, 4500, 0],
              [6000, 0, 1200]]

    num_wagons = [57] * 8 + [43] * 5

    trains_in_terminals = [[{"train_id": 0, "operation_time": 2.0}],
                           [{"train_id": 2, "operation_time": 10.0},
                            {"train_id": 12, "operation_time": 3.0}],
                           [{"train_id": 1, "operation_time": 3.0}],
                           [{"train_id": 11, "operation_time": 1.5}]]

    terminals_dict = {
        "load_times": load_times,
        "unload_times": unload_times,
        "max_num_trains": max_num_trains,
        "demands": demands,
        "stocks": stocks,
        "trains": trains_in_terminals
    }

    r00 = {"max_trains": 5, "transit_time": 0,
           "eta": []}
    r01 = {"max_trains": 5, "transit_time": 23,
           "eta": [{"train_id": 3, "eta": 9.0}, {"train_id": 4, "eta": 21.0}]}
    r02 = {"max_trains": 5, "transit_time": 27,
           "eta": []}
    r03 = {"max_trains": 5, "transit_time": 38,
           "eta": []}
    r10 = {"max_trains": 5, "transit_time": 23,
           "eta": [{"train_id": 5, "eta": 3.0}]}
    r11 = {"max_trains": 5, "transit_time": 0,
           "eta": []}
    r12 = {"max_trains": 5, "transit_time": 37,
           "eta": [{"train_id": 6, "eta": 18.0}, {"train_id": 7, "eta": 3.0}]}
    r13 = {"max_trains": 5, "transit_time": 22,
           "eta": [{"train_id": 10, "eta": 15.0}]}
    r20 = {"max_trains": 5, "transit_time": 26,
           "eta": [{"train_id": 9, "eta": 3.0}]}
    r21 = {"max_trains": 5, "transit_time": 35,
           "eta": []}
    r22 = {"max_trains": 5, "transit_time": 0,
           "eta": []}
    r23 = {"max_trains": 5, "transit_time": 25,
           "eta": [{"train_id": 8, "eta": 2.0}]}
    r30 = {"max_trains": 5, "transit_time": 41,
           "eta": []}
    r31 = {"max_trains": 5, "transit_time": 21,
           "eta": []}
    r32 = {"max_trains": 5, "transit_time": 24,
           "eta": []}
    r33 = {"max_trains": 5, "transit_time": 0,
           "eta": []}

    routes0 = [r00, r01, r02, r03]
    routes1 = [r10, r11, r12, r13]
    routes2 = [r20, r21, r22, r23]
    routes3 = [r30, r31, r32, r33]

    routes_list = [routes0, routes1, routes2, routes3]

    initial_params = {
        "terminals": terminals_dict,
        "products": {"mean_weight": [60, 52, 35]},
        "trains": {"num_wagons": num_wagons},
        "routes": routes_list
    }

    solver = ModelSolver(debug=True)
    solution = solver.solve(initial_params)
    solver.print_results()
    print(solution)
