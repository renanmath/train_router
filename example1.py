from pyomo.environ import *

model = ConcreteModel()
model.x = Var(domain=NonNegativeIntegers, bounds=(2,None))
model.y = Var(domain=NonNegativeIntegers, bounds=(None, 23))
model.obj = Objective(expr=3*model.x - 1.7*model.y, sense=maximize)

model.constrs = ConstraintList()
model.constrs.add(expr=model.y - 0.8*model.x <= 8)
model.constrs.add(expr=5.3*model.x + 2.1*model.y <= 19)

model.pprint()

optimizer = SolverFactory("glpk")
results = optimizer.solve(model)

cost = model.obj.expr()
print(cost)
x_value = model.x.value
y_value = model.y.value
print(x_value,y_value)
