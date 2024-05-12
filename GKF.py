import numpy as np
import matplotlib.pyplot as plt

def gaussian_kernel(x, x_prime, sigma):
    return np.exp(-0.5 * np.sum((x - x_prime) ** 2) / sigma ** 2)

def generate_prediction(x_new, X, y, sigma=0.5):
    y_pred = np.zeros_like(x_new)
    for i, x in enumerate(x_new):
        for j, x_train in enumerate(X):
            y_pred[i] += y[j] * gaussian_kernel(x, x_train, sigma)
    return y_pred

# Generování trénovacích dat
num_train_points = 20
X_train = np.linspace(-3, 3, num_train_points).reshape(-1, 1)  
y_train = np.sin(X_train).ravel() + np.random.normal(0, 0.1, num_train_points)  

# Generování nových bodů pro predikci
num_test_points = 100
x_new = np.linspace(-3, 3, num_test_points).reshape(-1, 1)

# Predikce hodnot pro nové body
y_pred = generate_prediction(x_new, X_train, y_train)

# Vizualizace
plt.figure(figsize=(10, 6))
plt.scatter(X_train, y_train, color='blue', label='Trénovací body')
plt.plot(x_new, y_pred, color='red', label='Predikovaná funkce')
plt.title('Gaussovská kernel funkce')
plt.xlabel('X')
plt.ylabel('Y') 
plt.legend()
plt.grid(True)
plt.show()
