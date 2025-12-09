import numpy as np
import matplotlib.pyplot as plt

def cargar_archivo(nombre):
    datos = np.loadtxt(nombre)
    fps = datos[:, 0]
    objetos = datos[:, 1]
    return fps, objetos

# Cargar GPU y CPU
fps_gpu, obj_gpu = cargar_archivo("resultados_gpu.txt")
fps_cpu, obj_cpu = cargar_archivo("resultados_cpu.txt")

plt.figure(figsize=(12, 7))

# GPU
plt.plot(obj_gpu, fps_gpu, label="GPU", linewidth=2)

# CPU
plt.plot(obj_cpu, fps_cpu, label="CPU", linewidth=2)

plt.title("FPS vs Número de elementos (CPU vs GPU)", fontsize=16)
plt.xlabel("Número de elementos", fontsize=14)
plt.ylabel("FPS", fontsize=14)
# Líneas horizontales de referencia
plt.axhline(60, color="green", linestyle="--", linewidth=1, label="60 FPS")
plt.axhline(30, color="red", linestyle="--", linewidth=1, label="30 FPS")


plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(fontsize=14)

plt.tight_layout()
plt.savefig("comparativa_cpu_gpu.png", dpi=200)

plt.show()
