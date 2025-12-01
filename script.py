import subprocess
import matplotlib.pyplot as plt

num_obj = [100000, 1000000, 10000000, 100000000, 1000000000]  # Diferentes cantidades de objetos
num_threads = [1, 10, 20, 30, 40, 50]  # Diferentes cantidades de hilos

# Almacenar resultados
results = {}

for no in num_obj:
    results[no] = []
    for nt in num_threads:
        print(f"\nEjecutando prueba con {no} objetos y {nt} hilos")
        result = subprocess.run(
            ["./build/bin/Verlet-Multithread", str(no), str(nt)],
            capture_output=True,
            text=True
        )

        # Extraer el FPS de la salida
        try:
            fps = float(result.stdout.strip())  # Asumimos que el FPS es lo último que se imprime
            results[no].append((nt, fps))
            print(f"FPS obtenido: {fps}")
        except ValueError:
            print(f"Error al obtener el FPS para {no} objetos y {nt} hilos")

# Graficar los resultados
for no in num_obj:
    threads = [entry[0] for entry in results[no]]
    fps_values = [entry[1] for entry in results[no]]

    plt.plot(threads, fps_values, label=f'{no} objetos')

plt.xlabel('Número de hilos')
plt.ylabel('FPS')
plt.title('FPS vs Número de hilos')
plt.legend()
plt.grid(True)
plt.show()
