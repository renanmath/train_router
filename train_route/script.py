import json
from copy import deepcopy
from train_route.solver.solver import ModelSolver

##### CALCULAR TEMPO DE FILA #####
with open("input_example.json") as file:
    params = json.load(file)

queue_times = []
for i, trains in enumerate(params['terminals']['trains']):
    max_op = max(train['operation_time'] for train in trains)
    un_time = params['terminals']['unload_times'][i]
    expected_op = un_time*len(trains)
    queue_time = max(0, max_op-expected_op)
    queue_times.append(queue_time)

print(queue_times)

print("OK")
