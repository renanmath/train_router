from train_route.simulator.simulator import Simulator


class OptimizerModel:
    """
    Class responsible to perform an optmization for the train route problem
    """

    def __init__(self, initial_params: dict, total_demands: list):
        self.__initial_params = initial_params
        self.__total_demands = total_demands
        self.__best_solution = None
        self.__last_best_ratio = 1

    def run_optimization(self, time_horizon: float):
        best_params = {}
        for diff in [0, 12, 24, 36, 48]:
            for b in [2, 1.95, 1.9, 1.85, 1.8, 1.75, 1.7, 1.65, 1.6, 1.55, 1.5]:
                print(f"Simulando para {time_horizon - diff}, {b}")
                simulation_solver = Simulator(initial_params=self.__initial_params,
                                              demands=self.__total_demands,
                                              time_horizon=time_horizon - diff,
                                              correction_limit=b,
                                              )

                sim_results, ratios = simulation_solver.simulate()

                if self.__update_mean_ratio(ratios):
                    if self.__last_best_ratio < 0.1:
                        print("Solução promissora encontrada")
                        best_params["simulation_time"] = time_horizon - diff
                        best_params["corretion"] = b
                        for tt in [-0.003, -0.002, -0.001, 0.001, 0.002, 0.003]:
                            simulation_solver = Simulator(initial_params=self.__initial_params,
                                                          demands=self.__total_demands,
                                                          time_horizon=time_horizon - diff,
                                                          correction_limit=b + tt,
                                                          )

                            sim_results, ratios = simulation_solver.simulate()
                            if self.__update_mean_ratio(ratios) and self.__last_best_ratio < 0.07:
                                self.__best_solution = sim_results
                                best_params["corretion"] = b + tt
                                break

                    else:
                        print("Não deu ainda")

        if self.__best_solution is None:
            print("Só teve solução merda")

        return self.__best_solution, best_params

    def __update_mean_ratio(self, demands_ration):
        mr = 0
        length = 0
        for dms in demands_ration:
            temp_dms = [x for x in dms if x > 0]
            mr += sum(temp_dms)
            length += len(temp_dms)

        mr = mr / max(length, 1)
        print(f"mr = {mr}")

        if self.__last_best_ratio > mr:
            self.__last_best_ratio = mr
            return True
        else:
            return False


if __name__ == "__main__":
    import json

    with open(
            "/home/renan/Documentos/Estudos/Pesquisa operacional e programação inteira/Projetos/train_route/train_route/input_example.json") as file:
        initial_params = json.load(file)

    total_demands = [
        [12000, 0, 0],
        [0, 32000, 2300],
        [25000, 0, 5400],
        [0, 0, 0]
    ]

    days = 7

    optimizer = OptimizerModel(initial_params=initial_params,
                               total_demands=total_demands)

    best_sol, best_params = optimizer.run_optimization(time_horizon=24 * days)
    print(best_params)
