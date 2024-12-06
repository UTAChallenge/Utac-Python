import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Définir la fonction exponentielle
def exponential_function(x, a, b, c):
    return a * np.exp(-b * x)

# Données
x_data = np.array([120,110,100,86,68, 56, 51, 45, 38, 33,30,28,27,26,25])
y_data = np.array([200,220,250,300,400, 500, 600, 700, 800, 900,1000,1100,1200,1300,1400])
x_vitesse = np.array([1056,256])
y_vitesse = np.array([20,0])

y_frein = np.array([0, 0.10, 0.50, 0.8])
x_frein = np.array([8, 5, 2, 0])
params3, covariance3 = curve_fit(lambda x, a, b, c, d: a*x**2 + b*x + c, x_frein, y_frein)

# Ajuster la fonction polynomiale
params, covariance = curve_fit(lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e, x_data, y_data)
params2, covariance2 = curve_fit(lambda x, a, b: a * x + b, x_vitesse, y_vitesse)

# Générer des points pour la courbe ajustée
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = (lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e)(x_fit, *params)
x_fit2 = np.linspace(min(x_vitesse), max(x_vitesse), 100)
y_fit2 = (lambda x, a, b: a * x + b)(x_fit2, *params2)
# Générer des points pour la courbe ajustée
x_fit3 = np.linspace(min(x_frein), max(x_frein), 100)
y_fit3 = (lambda x, a, b, c, d: a*x**2 + b*x + c)(x_fit3, *params3)

# Tester la fonction avec quelques valeurs de x
x_test = np.array([20,24, 35, 67,80,100,120,130,140])
x_test2 = np.array([1056, 700, 500, 250])
y_predicted = (lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e)(x_test, *params)
y_predicted2 = (lambda x, a, b: a * x + b)(x_test2, *params2)

# Afficher les résultats
for x, y_pred in zip(x_test, y_predicted):
    print(f'Pour x = {x}, y_pred = {y_pred}')

for x, y_pred in zip(x_test2, y_predicted2):
    print(f'Pour x = {x}, y_pred = {y_pred}')

# Afficher les résultats
plt.scatter(x_frein, y_frein, label='Points de données')
plt.plot(x_fit3, y_fit3, label='Fonction ajustée (polynomiale)', color='red')
plt.title('Ajustement polynomiale pour les données principales')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.show()


# Afficher les résultats
plt.scatter(x_vitesse, y_vitesse, label='Points de données')
plt.plot(x_fit2, y_fit2, label='Fonction ajustée (polynomiale)', color='red')
plt.title('Ajustement polynomiale pour les données de vitesse')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.show()

# Afficher les résultats
plt.scatter(x_data, y_data, label='Points de données')
plt.plot(x_fit, y_fit, label='Fonction ajustée (polynomiale)', color='red')
plt.title('Ajustement polynomiale pour les données principales')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.show()