from pyomo.environ import *

# definição dos dados de entrada
trains = list(range(13))
terminals = list(range(4))
products = list(range(3))
# parâmetros com um índice
num_wagons = [57] * 8 + [43] * 5  # número de vagões por trem
mean_weight = [60, 52, 35]  # peso médio dos produtos
load_times = [8] * len(terminals)  # tempo de carregamento nos terminais
unload_times = [7] * len(terminals)  # tempo de descarregamento nos terminais
max_num_trains = [4, 3, 4, 4]  # número máximo de trens que podem ser enviados pros terminais

# parâmetros com dois índices
tt_list = [[0, 23, 27, 38],
           [23, 0, 37, 22],
           [26, 35, 0, 25],
           [41, 21, 24, 0]]  # tempos de trânsito entre os terminais

demands = [[1200, 0, 0],
           [0, 500, 800],
           [1500, 0, 600],
           [0, 0, 0]]  # demandas dos produtos por terminais

stocks = [[0, 5000, 3600],
          [8000, 0, 0],
          [0, 4500, 0],
          [6000, 0, 1200]]  # estoque dos produtos por terminais

remaining_op = [[0] * len(terminals) for _ in range(len(trains))]
remaining_op[0][0] = 2
remaining_op[1][2] = 3
remaining_op[2][1] = 10
remaining_op[-2][3] = 1.5
remaining_op[-1][1] = 3

train_locations = [[0] * len(terminals) for _ in range(len(trains))]  # localização dos trens por terminal
train_locations[0][0] = 1
train_locations[1][2] = 1
train_locations[2][1] = 1 # add
train_locations[-2][3] = 1
train_locations[-1][1] = 1

max_trains_per_route = [[5] * len(terminals) for _ in range(len(terminals))]  # número máximo de trens por trecho
max_trains_per_route[0][1] = 1
max_trains_per_route[1][2] = 2

# parâmetros com três índices
# tempo restante de viagem por trem e rota
remaining_tt = [[[0] * len(terminals) for _ in range(len(terminals))]
                for _ in range(len(trains))]
remaining_tt[3][0][1] = 9
remaining_tt[4][0][1] = 21
remaining_tt[5][1][0] = 3
remaining_tt[6][1][2] = 18
remaining_tt[7][1][2] = 3
remaining_tt[8][2][3] = 2
remaining_tt[9][2][0] = 3
remaining_tt[10][1][3] = 15

# trens em rota, por trecho
trains_en_route = [[[0] * len(terminals) for _ in range(len(terminals))]
                   for _ in range(len(trains))]
trains_en_route[3][0][1] = 1
trains_en_route[4][0][1] = 1
trains_en_route[5][1][0] = 1
trains_en_route[6][1][2] = 1
trains_en_route[7][1][2] = 1
trains_en_route[8][2][3] = 1
trains_en_route[9][2][0] = 1
trains_en_route[10][1][3] = 1

# construção do modelo
model = ConcreteModel()

# definição das variáveis
model.tau = Var(domain=NonNegativeReals)  # tempo da simulação
model.x = Var(trains, terminals, terminals, within=Binary)
model.nu = Var(trains, terminals, terminals, products, within=NonNegativeIntegers)

# função objetivo (maximizar tempo de simulação e número de vagões despachados)
model.obj = Objective(expr=model.tau + sum(model.nu[t, j, k, ll] for t in trains
                                           for j in terminals
                                           for k in terminals
                                           for ll in products),
                      sense=maximize)

# restrições do problema
model.constrs = ConstraintList()

# restrições de destino único
for t in range(len(trains)):
    a = 1
    model.constrs.add(expr=sum(model.x[t, i, j] for i in terminals for j in terminals) <= 1)

# Restrições de factibilidade
for t in range(len(trains)):
    for j in range(len(terminals)):
        for k in range(len(terminals)):
            a = 1
            model.constrs.add(expr=model.x[t, j, k] <= train_locations[t][j])

# Restrições de chegada em um nó
for k in range(len(terminals)):
    a = 1
    model.constrs.add(expr=sum(model.x[t, j, k] for t in trains for j in terminals) <= max_num_trains[k])

# Restrições de tráfego em um trecho
for j in range(len(terminals)):
    for k in range(len(terminals)):
        a = 1
        model.constrs.add(expr=sum(model.x[t, j, k] for t in trains) <= max_trains_per_route[j][k])

# Restrições de número de vagões por trem
for t in range(len(trains)):
    for j in range(len(terminals)):
        for k in range(len(terminals)):
            a = 1
            model.constrs.add(expr=sum(model.nu[t, j, k, la] for la in products) <= model.x[t, j, k] * num_wagons[t])

# Restrições de estoque por produto
for ll in range(len(products)):
    for j in range(len(terminals)):
        a = 1
        model.constrs.add(
            expr=sum(model.nu[t, j, k, ll] * mean_weight[ll] for t in trains for k in terminals) <= stocks[j][ll])

# Restrições de demanda por produto
for ll in range(len(products)):
    for k in range(len(terminals)):
        a = 1
        model.constrs.add(
            expr=sum(model.nu[t, j, k, ll] * mean_weight[ll] for t in trains for j in terminals) >= demands[k][ll])

# Restrições de tempo  máximo de simulação
for t in range(len(trains)):
    for i in range(len(terminals)):
        for j in range(len(terminals)):
            try:
                model.constrs.add(expr=model.x[t, i, j] * remaining_op[t][i] <= model.tau)
            except ValueError:
                pass
            try:
                model.constrs.add(expr=trains_en_route[t][i][j] * model.tau <= trains_en_route[t][i][j] * (
                        remaining_tt[t][i][j] + unload_times[j]))
            except ValueError:
                continue

for t in range(len(trains)):
    for j in range(len(terminals)):
        for k in range(len(terminals)):
            model.constrs.add(expr=model.tau <= load_times[j] + tt_list[j][k] + 100 * (1 - model.x[t, j, k]))

optimizer = SolverFactory("glpk")
results = optimizer.solve(model, tee=False)
print(results.solver.status)
print(results.solver.termination_condition)
print('\n\n________________________________')
time_ = model.tau.value
total_wagons = model.obj.expr() - time_
print(f'Total de vagões despachados: {int(total_wagons)}')
print(f'Horizonte de tempo simulado: {time_} horas')

print("------")
for t in trains:
    for i in terminals:
        for j in terminals:
            if model.x[t, i, j].value is not None and model.x[t, i, j].value > 0:
                print(f'Trem {t} despachado do Terminal {i} para Terminal {j}')
                for k in products:
                    print(f'{int(model.nu[t, i, j, k].value)} vagões do produto {k}')
                print("------")
