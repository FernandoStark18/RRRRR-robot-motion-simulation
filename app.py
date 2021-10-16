# Espericueta Fonseca Fernando Simón
# Mecatrónica 7mo 6
# Dinámica y control de robots
# Análisis cinemático por la convención Denavit-Hartenberg de un robot RRRRR

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider

fig, ax = plt.subplots()
plt.subplots_adjust(left = 0, bottom = 0.3, right =1, top = 1)
ax = plt.axes(projection = "3d")
# Función para rotar en el eje x
def matriz_rotacion_x(grados):
	rad = grados/180*np.pi
	#Matriz de rotación
	rotacion=np.array([[1,0,0,0],
					   [0,np.cos(rad),-np.sin(rad),0],
					   [0,np.sin(rad),np.cos(rad),0],
					   [0,0,0,1]])
	return rotacion

def matriz_rotacion_y(grados):  # Definimos la función de rotación en y
    rad = grados/180*np.pi  # Conversión a grados
    rotacion = np.array([[np.cos(rad), 0, -np.sin(rad), 0],  # Matriz de rotación primera fila
                                  # Matriz de rotación segunda fila
                                  [0,  1, 0, 0],
                                  # Matriz de rotación terca fila
                                  [np.sin(rad), 0, np.cos(rad), 0],
                                  [0, 0, 0, 1]])  # Cuarta fila de la matriz

    return rotacion  # Devuelvo la matriz de rotación en y

# Función para rotar en el eje z
def matriz_rotacion_z(grados):
	rad = grados/180*np.pi
	#Matriz de rotacuón
	rotacion=np.array([[np.cos(rad),-np.sin(rad),0,0],
					   [np.sin(rad),np.cos(rad),0,0],
					   [0,0,1,0],
					   [0,0,0,1]])
	return rotacion

# Función para trasladar en el eje x
def matriz_traslacion_x(x):
	traslacion = np.array([[1,0,0,x],
						   [0,1,0,0],
						   [0,0,1,0],	   
						   [0,0,0,1]])
	return traslacion

def matriz_traslacion_y(y):  # Definimos la matriz de traslación en y
    traslacion = np.array([[1, 0, 0, 0],  # Primera fila de la matriz de traslación
                           [0, 1, 0, y],  # Segunda fila de la matriz de traslación
                           [0, 0, 1, 0],  # Tercera fila de la matriz de traslación
                           [0, 0, 0, 1]])  # Cuarta fila de la matriz de traslación
    return traslacion  # Devolvemos la matriz de traslación en y

# Función para trasladar en el eje z
def matriz_traslacion_z(z):
	traslacion = np.array([[1,0,0,0],
						   [0,1,0,0],
						   [0,0,1,z],	   
						   [0,0,0,1]])
	return traslacion

# Función para la configuración gráfica
def configuracion_grafica():
	# Título del gráfico
	plt.title("Robót RRRRR", x = 0.03, y = 30)
	# Límites del gráfico
	ax.set_xlim(-2,2)
	ax.set_ylim(-2,2)
	ax.set_zlim(0,2)
	# Estiquetas para identificar los ejes
	ax.set_xlabel("x")
	ax.set_ylabel("y")
	ax.set_zlabel("z")
	# Vista

# Función para la operación de Denavit-Hartenberg
def DH(theta_i, di, ai, alpha_i):
	MT = matriz_rotacion_x(theta_i)@matriz_traslacion_x(di)@matriz_traslacion_z(ai)@matriz_rotacion_z(alpha_i)
	return MT

# Composición de las matrices de transformación homogénea
def Cuadrupedo(theta_1, d1, a1, alpha_1, 
			   theta_2, d2, a2, alpha_2,
			   theta_3, d3, a3, alpha_3,
			   theta_4, d4, a4, alpha_4,
			   theta_5, d5, a5, alpha_5):
	
	# Pata 1 Frente derecha
	A0 = np.eye(4)
	_0A1 = DH(theta_1, d1, a1, alpha_1)
	_1A2 = DH(theta_2, d2, a2, alpha_2)
	_2A3 = DH(theta_3, d3, a3, alpha_3)
	_3A4 = DH(theta_4, d4, a4, alpha_4)
	_4A5 = DH(theta_5, d5, a5, alpha_5)
	_0A2 = _0A1@_1A2
	_0A3 = _0A2@_2A3
	_0A4 = _0A3@_3A4
	_0A5 = _0A4@_4A5

	# Se deibujan los eslabones
	ax.plot3D([A0[0,3],_0A1[0,3]],[A0[1,3],_0A1[1,3]],[A0[2,3],_0A1[2,3]], color = 'black')
	ax.plot3D([_0A1[0,3],_0A2[0,3]],[_0A1[1,3],_0A2[1,3]],[_0A1[2,3],_0A2[2,3]], color = 'blue')
	ax.plot3D([_0A2[0,3],_0A3[0,3]],[_0A2[1,3],_0A3[1,3]],[_0A2[2,3],_0A3[2,3]], color = 'red')
	ax.plot3D([_0A3[0,3],_0A4[0,3]],[_0A3[1,3],_0A4[1,3]],[_0A3[2,3],_0A4[2,3]], color = 'blue')
	ax.plot3D([_0A4[0,3],_0A5[0,3]],[_0A4[1,3],_0A5[1,3]],[_0A4[2,3],_0A5[2,3]], color = 'red')

# Se actualizan las juntas cada vez que se modifica el valor de un ángulo con el slider
def actualizacion_juntas(val):
		ax.cla()
		configuracion_grafica()
		theta_1 = sld_ang_1.val
		theta_2 = sld_ang_2.val
		theta_3 = sld_ang_3.val
		theta_4 = sld_ang_4.val
		theta_5 = sld_ang_5.val

		# Parámetros de Denavit-Hartenberg obtenidos en la tabla
		Cuadrupedo(0, 0, 0.8, theta_1,
				   theta_2, 0, 0.7, 0,
				   theta_3, 0, 0.5, 0,
				   theta_4, 0, 0.2, 0,
				   0, 0, 0.1, theta_5)
		plt.draw()
		plt.pause(1e-3)

# Deiseño de los sliders
ax1 = plt.axes([0.2,0.20,0.65,0.026])
ax2 = plt.axes([0.2,0.18,0.65,0.026])
ax3 = plt.axes([0.2,0.16,0.65,0.026])
ax4 = plt.axes([0.2,0.14,0.65,0.026])
ax5 = plt.axes([0.2,0.12,0.65,0.026])

# Creación de los sliders
sld_ang_1 = Slider(ax1, "Theta_1",0,180,valinit = 90)
sld_ang_2 = Slider(ax2, "Theta_2",0,180,valinit = 90)
sld_ang_3 = Slider(ax3, "Theta_3",0,180,valinit = 90)
sld_ang_4 = Slider(ax4, "Theta_4",0,180,valinit = 90)
sld_ang_5 = Slider(ax5, "Theta_5",0,180,valinit = 90)

# Cada vez que se mueve un slider se actualizan las juntas
configuracion_grafica()
Cuadrupedo(0, 0, 0.8, 90,
		   90, 0, 0.7, 0,
		   90, 0, 0.5, 0,
		   90, 0, 0.2, 0,
		   0, 0, 0.1, 90)
sld_ang_1.on_changed(actualizacion_juntas)
sld_ang_2.on_changed(actualizacion_juntas)
sld_ang_3.on_changed(actualizacion_juntas)
sld_ang_4.on_changed(actualizacion_juntas)
sld_ang_5.on_changed(actualizacion_juntas)

plt.show()
print("Programado por Fernando Espericueta")