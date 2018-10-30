import numpy as np

class KalmanFilter:

	def __init__(self, *args, **kwargs):

		#Atributes: dt, mea_noise, accel_noise, mat_A, mat_B, mat_C, mat_X

		self.dt = kwargs.get('dt', 1) 
		
		self.measurement_noise = kwargs.get('mea_noise', np.random.normal(0, 0.3))
		self.acceleration_noise = kwargs.get('accel_noise', np.random.normal(0, 0.3)) 

		self.A = kwargs.get('mat_A', np.array([[1, self.dt],[0, 1]], dtype=np.float) )
		#self.B = kwargs.get('mat_B', np.array([[(dt**2)/2],[dt]]) )
		self.C = kwargs.get('mat_C', np.array([[1, 0]], dtype=np.float) )
		self.x = kwargs.get('mat_X', np.array([[0],[0]],dtype=np.float) )
		self.y = np.matmul(self.C, self.x)
		self.xhat = self.x

		self.Sz = self.measurement_noise**2 
		#Will it be the measurement noise rather than the acceleration noise?
		self.Sw = self.acceleration_noise**2

		self.P = self.Sw

	def Status(self):
		print("dt",self.dt)
		print("measurement_noise",self.measurement_noise)
		print("acceleration_noise",self.acceleration_noise)
		print("A",self.A)
		print("C",self.C)
		print("x",self.x)
		print("y",self.y)
		print("xhat",self.xhat)
		print("Sz",self.Sz)
		print("Sw",self.Sw)
		print("P",self.P)


	def Update(self, measurement):

		proc_noise = self.acceleration_noise * np.array([[((self.dt**2)/2)*np.random.normal(0, 0.3)],[self.dt*np.random.normal(0, 0.3)]])
		self.x = np.matmul(self.A, self.x) + proc_noise

		mea_noise = self.measurement_noise * np.random.normal(0, 0.3)
		
		self.y = np.matmul(self.C, self.x) +  mea_noise
		self.xhat = np.matmul(self.A, self.xhat)

		innovation_vec = self.y - np.matmul(self.C, self.xhat)

		s_inn = np.matmul(self.C, self.P*np.transpose(self.C)) + self.Sz


		#try:
		#K = np.matmul(self.A, np.matmul(self.P, np.matmul(np.transpose(self.C), np.linalg.inv(s_inn))))

		K = np.linalg.inv(np.matmul(self.C, self.P*np.transpose(self.C))+s_inn)

		K = np.matmul(self.A, np.matmul(self.P*np.transpose(self.C), K))

		#except np.linalg.linalg.LinAlgError:
		#	print("Error!")

		self.xhat = self.xhat + np.matmul(K,innovation_vec)

		# from Kalman_filter import KalmanFilter; kf = KalmanFilter(); import numpy as np; measurement = np.array([[1],[1]], dtype=np.float); kf.Update(measurement)		

		# P comienza como escalar pero luego se vuelve Matriz, ¿Toca inicializarlo como una matriz?

		P_1 = np.matmul(self.A, self.P*np.transpose(self.A))
		P_2 = np.matmul(self.A, self.P*np.matmul(np.transpose(self.C), np.matmul(np.linalg.inv(s_inn), np.matmul(self.C, self.P*np.transpose(self.A)))))
		self.P = P_1 - P_2 + self.Sw

		return self.xhat[0]