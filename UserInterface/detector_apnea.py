import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.sequence import pad_sequences

class DetectorApnea:
    def __init__(self, ruta_modelo="modelo_cnn_lstm_apnea_adaptativo.h5"):
        """
        Inicializa el detector de apnea.
        
        Args:
            ruta_modelo: Ruta al archivo H5 del modelo entrenado
        """
        # Configuración
        self.fs = 10  # Frecuencia de muestreo (10 Hz)
        self.ventana_segundos = 4  # Bloques de 4 segundos
        self.ventana_puntos = self.ventana_segundos * self.fs  # 40 puntos
        self.max_seq_length = 300  # MAX_DURACION_APNEA * FS_NUEVO del entrenamiento
        
        # Cargar el modelo
        try:
            self.modelo = load_model(ruta_modelo)
            print(f"Modelo cargado desde: {ruta_modelo}")
        except Exception as e:
            print(f"Error al cargar el modelo: {e}")
            print("¡IMPORTANTE! Coloca el archivo del modelo en el mismo directorio que este script.")
            print("Usando un modelo simulado para demostración.")
            # Crear un modelo vacío para demonstración si no se encuentra el archivo
            self.crear_modelo_simulado()
        
        # Buffer para almacenar datos
        self.buffer = []
    
    def crear_modelo_simulado(self):
        """Crea un modelo simulado para demostración."""
        from tensorflow.keras.models import Sequential
        from tensorflow.keras.layers import Dense, Input
        
        modelo = Sequential()
        modelo.add(Input(shape=(self.max_seq_length, 1)))
        modelo.add(Dense(1, activation='sigmoid'))
        modelo.compile(optimizer='adam', loss='binary_crossentropy')
        
        self.modelo = modelo
    
    def agregar_dato(self, valor):
        """
        Agrega un nuevo dato al buffer.
        
        Args:
            valor: Valor numérico de la señal
        
        Returns:
            (bool, float): Tupla con (hay_apnea, probabilidad_apnea)
            donde hay_apnea es True si se detectó apnea, False en caso contrario.
            probabilidad_apnea es un valor entre 0 y 1.
        """
        # Agregar dato al buffer
        self.buffer.append(valor)
        
        # Si no tenemos suficientes datos, retornar None
        if len(self.buffer) < self.ventana_puntos:
            return False, 0.0
        
        # Mantener solo los últimos N puntos
        if len(self.buffer) > self.ventana_puntos:
            self.buffer = self.buffer[-self.ventana_puntos:]
        
        # Solo evaluar cuando tenemos exactamente una ventana completa
        if len(self.buffer) == self.ventana_puntos:
            return self.evaluar_ventana()
        
        return False, 0.0
    
    def evaluar_ventana(self):
        """
        Evalúa la ventana actual de datos con el modelo.
        
        Returns:
            (bool, float): Tupla con (hay_apnea, probabilidad_apnea)
        """
        # Preparar datos (normalización igual que en entrenamiento)
        ventana = np.array(self.buffer)
        ventana = (ventana - np.mean(ventana)) / (np.std(ventana) + 1e-8)
        
        # Aplicar padding para que coincida con el formato esperado por el modelo
        X = pad_sequences(
            [ventana], 
            maxlen=self.max_seq_length,
            dtype='float32',
            padding='post',
            truncating='post',
            value=0.0
        )
        
        # Reshape para modelo CNN: [muestras, tiempo, canales]
        X = X.reshape(1, self.max_seq_length, 1)
        
        # Predecir
        try:
            probabilidad = self.modelo.predict(X, verbose=0)[0][0]
            hay_apnea = probabilidad >= 0.5
            return hay_apnea, float(probabilidad)
        except Exception as e:
            print(f"Error al realizar predicción: {e}")
            return False, 0.0
    
    def reiniciar_buffer(self):
        """Reinicia el buffer de datos."""
        self.buffer = []