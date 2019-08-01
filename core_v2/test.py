from scipy.optimize import minimize

def fun(x):
    return x**4 + 100

sol = minimize(fun,0)
print(sol)