import sys
import json
import socket
import threading
import time
from datetime import datetime
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, font
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib
import tensorflow as tf
matplotlib.use("TkAgg")

# Importar el detector de apnea
from detector_apnea import DetectorApnea

# Clase principal para la aplicación
class SignalVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        
        # Datos de las señales
        self.max_points = 200
        self.tiempo_data = np.zeros(self.max_points)
        self.valor1_data = np.zeros(self.max_points)
        self.valor2_data = np.zeros(self.max_points)
        self.valor3_data = np.zeros(self.max_points)
        
        # Frecuencia de muestreo
        self.fs = 10  # 10 Hz
        
        # Bloques de datos para análisis de apnea (4 segundos)
        self.bloque_duracion = 8  # segundos
        #self.bloque_duracion = 4  # segundos
        self.bloque_puntos = self.bloque_duracion * self.fs  # 40 puntos
        self.bloque_actual = []
        
        # Inicializar detector de apnea
        self.detector_apnea = DetectorApnea()
        self.apnea_detectada = False
        self.probabilidad_apnea = 0.0
        
        # Modo de análisis automático de apnea
        self.analisis_apnea_activado = True
        
        # Estado de conexión
        self.connected = False
        self.udp_thread = None
        self.udp_socket = None
        
        # Dirección IP y puerto de la ESP32 para enviar señales
        self.esp32_ip = "172.20.10.2"  # IP predeterminada de la ESP32
        self.esp32_port = 12346  # Puerto UDP para enviar comandos
        
        # Configurar la interfaz
        self.title("Visualizador de Señales ESP32 - Detector de Apnea")
        self.geometry("1000x700")
        self.configure(bg="#141414")
        
        # Configurar el estilo
        self.setup_styles()
        
        # Configurar la interfaz
        self.setup_ui()
        
        # Temporizador para actualizar gráficos
        self.after(50, self.update_ui)  # Actualizar cada 50ms
    
    def setup_styles(self):
        """Configura los estilos para los widgets"""
        # Configurar estilos para ttk
        style = ttk.Style()
        style.theme_use('default')
        
        # Configurar colores base
        bg_color = "#141414"
        fg_color = "#E5E5E5"
        accent_color = "#E50914"
        button_bg = "#2D2D2D"
        
        # Configurar estilo para etiquetas
        style.configure("TLabel", background=bg_color, foreground=fg_color)
        
        # Configurar estilo para frames
        style.configure("TFrame", background=bg_color)
        
        # Configurar estilo para checkbox
        style.configure("TCheckbutton", background=bg_color, foreground=fg_color)
        style.map("TCheckbutton",
                 background=[('active', bg_color)],
                 foreground=[('active', fg_color)])
        
        # Fuentes personalizadas
        self.title_font = font.Font(family="Arial", size=18, weight="bold")
        self.subtitle_font = font.Font(family="Arial", size=12)
        self.button_font = font.Font(family="Arial", size=11, weight="bold")
        self.text_font = font.Font(family="Arial", size=11)
        
        # Colores para los gráficos
        self.colors = [
            "#E50914",  # Rojo Netflix para valor1
            "#00B4D8",  # Azul para valor2
            "#FFA500"   # Naranja para valor3
        ]
    
    def setup_ui(self):
        # Configurar colores base
        bg_color = "#141414"
        fg_color = "#E5E5E5"
        accent_color = "#E50914"
        button_bg = "#2D2D2D"
        """Configura la interfaz de usuario"""
        # Panel principal con padding
        main_frame = ttk.Frame(self, padding=15)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Panel superior con controles
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Panel izquierdo - Configuración de conexión
        connection_frame = ttk.Frame(top_frame)
        connection_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        # IP y Puerto
        ip_label = ttk.Label(connection_frame, text="Dirección IP:")
        ip_label.grid(row=0, column=0, sticky=tk.W, pady=5)
        
        self.ip_input = tk.Entry(connection_frame, bg="#333333", fg="white", 
                                 font=self.text_font, width=15)
        self.ip_input.insert(0, "0.0.0.0")  # Recibe en todas las interfaces
        self.ip_input.grid(row=0, column=1, sticky=tk.W, pady=5, padx=5)
        
        port_label = ttk.Label(connection_frame, text="Puerto UDP:")
        port_label.grid(row=1, column=0, sticky=tk.W, pady=5)
        
        self.port_input = tk.Entry(connection_frame, bg="#333333", fg="white",
                                  font=self.text_font, width=15)
        self.port_input.insert(0, "12345")
        self.port_input.grid(row=1, column=1, sticky=tk.W, pady=5, padx=5)
        
        # ESP32 IP y Puerto para enviar señales
        esp32_ip_label = ttk.Label(connection_frame, text="IP ESP32:")
        esp32_ip_label.grid(row=2, column=0, sticky=tk.W, pady=5)
        
        self.esp32_ip_input = tk.Entry(connection_frame, bg="#333333", fg="white", 
                                      font=self.text_font, width=15)
        self.esp32_ip_input.insert(0, self.esp32_ip)
        self.esp32_ip_input.grid(row=2, column=1, sticky=tk.W, pady=5, padx=5)
        
        esp32_port_label = ttk.Label(connection_frame, text="Puerto ESP32:")
        esp32_port_label.grid(row=3, column=0, sticky=tk.W, pady=5)
        
        self.esp32_port_input = tk.Entry(connection_frame, bg="#333333", fg="white",
                                       font=self.text_font, width=15)
        self.esp32_port_input.insert(0, str(self.esp32_port))
        self.esp32_port_input.grid(row=3, column=1, sticky=tk.W, pady=5, padx=5)
        
        # Análisis automático de apnea
        self.apnea_var = tk.BooleanVar(value=self.analisis_apnea_activado)
        self.apnea_checkbox = ttk.Checkbutton(connection_frame, text="Análisis automático de apnea", 
                                             variable=self.apnea_var, 
                                             command=self.toggle_apnea_analysis)
        self.apnea_checkbox.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        # Botones
        button_frame = ttk.Frame(connection_frame)
        button_frame.grid(row=5, column=0, columnspan=2, pady=5)
        
        self.connect_button = tk.Button(button_frame, text="Conectar", 
                                       bg="#E50914", fg="white", 
                                       font=self.button_font,
                                       activebackground="#F40612",
                                       activeforeground="white",
                                       bd=0, padx=10, pady=5,
                                       command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.clear_button = tk.Button(button_frame, text="Limpiar Datos",
                                     bg="#2D2D2D", fg="white", 
                                     font=self.button_font,
                                     activebackground="#3D3D3D",
                                     activeforeground="white",
                                     bd=0, padx=10, pady=5,
                                     command=self.clear_data)
        self.clear_button.pack(side=tk.LEFT, padx=(0, 5))
        
        # NUEVO: Botón de envío de señal de alerta a ESP32
        self.alert_button = tk.Button(button_frame, text="ENVIAR ALERTA",
                                    bg="#FF8000", fg="white", 
                                    font=self.button_font,
                                    activebackground="#FF9900",
                                    activeforeground="white",
                                    bd=0, padx=10, pady=5,
                                    command=self.send_alert_signal)
        self.alert_button.pack(side=tk.LEFT)
        
        # Información de estado
        self.status_label = tk.Label(connection_frame, text="Estado: Desconectado",
                                    bg="#141414", fg="#E50914", 
                                    font=self.text_font)
        self.status_label.grid(row=6, column=0, columnspan=2, sticky=tk.W, pady=5)
        
        # Estado de detección de apnea
        self.apnea_status_label = tk.Label(connection_frame, text="Apnea: No detectada",
                                         bg="#141414", fg="#00FF00", 
                                         font=self.text_font)
        self.apnea_status_label.grid(row=7, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        # Información de probabilidad
        self.apnea_prob_label = tk.Label(connection_frame, text="Probabilidad: 0.00",
                                       bg="#141414", fg=fg_color, 
                                       font=self.text_font)
        self.apnea_prob_label.grid(row=8, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        # Información de última actualización
        self.last_update_label = tk.Label(connection_frame, text="Última actualización: --",
                                        bg="#141414", fg=fg_color, 
                                        font=self.text_font)
        self.last_update_label.grid(row=9, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        # Estado de última alerta enviada
        self.alert_status_label = tk.Label(connection_frame, text="Última alerta: Ninguna",
                                        bg="#141414", fg=fg_color, 
                                        font=self.text_font)
        self.alert_status_label.grid(row=10, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        # Panel central - Título
        title_frame = ttk.Frame(top_frame)
        title_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=20)
        
        title_label = tk.Label(title_frame, text="DETECTOR DE APNEA", 
                             bg="#141414", fg="#E50914", 
                             font=self.title_font)
        title_label.pack(pady=(0, 5))
        
        subtitle_label = tk.Label(title_frame, text="ESP32 Data Streaming",
                                bg="#141414", fg="#CCCCCC", 
                                font=self.subtitle_font)
        subtitle_label.pack()
        
        # Panel de gráficos
        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True)
        
        # Crear gráficos
        self.setup_graphs(graph_frame)
        
        # Barra de estado
        self.status_bar = tk.Label(self, text="Aplicación lista para recibir datos", 
                                 bd=1, relief=tk.SUNKEN, anchor=tk.W,
                                 bg="#080808", fg="white")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def send_alert_signal(self):
        """Envía una señal de alerta a la ESP32"""
        try:
            # Obtener IP y puerto de la ESP32 desde la interfaz
            self.esp32_ip = self.esp32_ip_input.get()
            self.esp32_port = int(self.esp32_port_input.get())
            
            # Crear socket para enviar mensaje
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Crear mensaje de alerta
            alert_msg = {
                "comando": "ALERTA",
                "tiempo": time.time(),
                "mensaje": "¡Alerta activada desde la aplicación!"
            }
            
            # Convertir a JSON y enviar
            alert_data = json.dumps(alert_msg).encode('utf-8')
            sock.sendto(alert_data, (self.esp32_ip, self.esp32_port))
            
            # Actualizar estado
            current_time = datetime.now().strftime("%H:%M:%S")
            self.alert_status_label.config(text=f"Última alerta: {current_time}")
            self.status_bar.config(text=f"Alerta enviada a {self.esp32_ip}:{self.esp32_port}")
            
            # Cerrar socket
            sock.close()
            
        except Exception as e:
            self.status_bar.config(text=f"Error al enviar alerta: {str(e)}")
    
    def setup_graphs(self, parent):
        """Configura los gráficos de matplotlib"""
        # Crear figura y subplots
        self.fig = Figure(figsize=(10, 6), dpi=100, facecolor="#141414")
        
        # Crear subplots
        self.ax1 = self.fig.add_subplot(311)  # 3 filas, 1 columna, posición 1
        self.ax2 = self.fig.add_subplot(312)  # 3 filas, 1 columna, posición 2
        self.ax3 = self.fig.add_subplot(313)  # 3 filas, 1 columna, posición 3
        
        # Configurar estilo de los gráficos
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.set_facecolor("#141414")
            ax.grid(True, alpha=0.3)
            ax.tick_params(colors="white")
            for spine in ax.spines.values():
                spine.set_color("white")
        
        # Gráfico 1 - Con detección de apnea
        self.ax1.set_title("Señal 1 - Detección de Apnea", color="white", fontsize=14)
        self.ax1.set_ylabel("Amplitud", color="white", fontsize=12)
        self.line1, = self.ax1.plot([], [], lw=2, color=self.colors[0])
        
        # Añadir zona sombreada para ventana de análisis
        #self.analysis_region = self.ax1.axvspan(0, 4, alpha=0.2, color='white')
        self.analysis_region = self.ax1.axvspan(0, 8, alpha=0.2, color='white')
        # Texto para indicar apnea
        self.apnea_text = self.ax1.text(0.5, 0.5, "POSIBLE APNEA", 
                                       horizontalalignment='center',
                                       verticalalignment='center',
                                       transform=self.ax1.transAxes,
                                       color='white', fontsize=14, 
                                       bbox=dict(facecolor='red', alpha=0.8))
        self.apnea_text.set_visible(False)  # Inicialmente oculto
        
        # Gráfico 2
        self.ax2.set_title("Señal 2 - Cosenoidal", color="white", fontsize=14)
        self.ax2.set_ylabel("Amplitud", color="white", fontsize=12)
        self.line2, = self.ax2.plot([], [], lw=2, color=self.colors[1])
        
        # Gráfico 3
        self.ax3.set_title("Señal 3 - Compuesta", color="white", fontsize=14)
        self.ax3.set_xlabel("Tiempo (s)", color="white", fontsize=12)
        self.ax3.set_ylabel("Amplitud", color="white", fontsize=12)
        self.line3, = self.ax3.plot([], [], lw=2, color=self.colors[2])
        
        # Ajustar espaciado
        self.fig.tight_layout()
        
        # Crear canvas de matplotlib
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def toggle_apnea_analysis(self):
        """Activa o desactiva el análisis automático de apnea"""
        self.analisis_apnea_activado = self.apnea_var.get()
        if not self.analisis_apnea_activado:
            self.apnea_status_label.config(text="Apnea: Análisis desactivado", fg="#888888")
            self.apnea_text.set_visible(False)
            self.canvas.draw_idle()
        else:
            # Reiniciar detector
            self.detector_apnea.reiniciar_buffer()
            self.bloque_actual = []
            self.apnea_status_label.config(text="Apnea: No detectada", fg="#00FF00")
    
    def toggle_connection(self):
        """Conecta o desconecta el socket UDP"""
        if not self.connected:
            # Intentar conectar
            try:
                port = int(self.port_input.get())
                ip = self.ip_input.get()
                
                # Mostrar mensaje de intento de conexión
                self.status_bar.config(text=f"Intentando escuchar en {ip}:{port}...")
                
                # Iniciar hilo de conexión UDP
                self.start_udp_thread(ip, port)
                
            except ValueError:
                self.status_bar.config(text="Error: Puerto debe ser un número")
                
        else:
            # Desconectar
            self.stop_udp_thread()
    
    def start_udp_thread(self, ip, port):
        """Inicia el hilo para recibir datos UDP"""
        # Crear un socket UDP
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.bind((ip, port))
            self.udp_socket.settimeout(0.5)  # Timeout para que el hilo pueda terminar
            
            # Iniciar hilo para recibir datos
            self.connected = True
            self.connect_button.config(text="Desconectar")
            self.update_connection_status(True, f"Conectado: Escuchando en {ip}:{port}")
            
            # Limpiar datos antes de iniciar
            self.clear_data()
            
            # Crear y iniciar hilo
            self.udp_thread = threading.Thread(target=self.receive_udp_data)
            self.udp_thread.daemon = True
            self.udp_thread.start()
            
        except Exception as e:
            self.status_bar.config(text=f"Error al conectar: {str(e)}")
            if self.udp_socket:
                self.udp_socket.close()
                self.udp_socket = None
    
    def stop_udp_thread(self):
        """Detiene el hilo de recepción UDP"""
        self.connected = False
        self.connect_button.config(text="Conectar")
        self.update_connection_status(False, "Desconectado")
        
        if self.udp_socket:
            self.udp_socket.close()
            self.udp_socket = None
        
        if self.udp_thread:
            self.udp_thread.join(1.0)
            self.udp_thread = None
    
    def receive_udp_data(self):
        """Función que se ejecuta en un hilo separado para recibir datos UDP"""
        while self.connected and self.udp_socket:
            try:
                # Recibir datos
                data, addr = self.udp_socket.recvfrom(1024)
                
                # Procesar los datos JSON
                data_str = data.decode('utf-8')
                json_data = json.loads(data_str)
                
                # Procesar los datos en el hilo principal
                self.after(0, lambda: self.update_plots(json_data))
                
            except socket.timeout:
                # Timeout, seguir escuchando
                continue
            except json.JSONDecodeError:
                # Error en el formato JSON
                self.after(0, lambda: self.status_bar.config(
                    text="Error: Datos recibidos con formato inválido"))
                continue
            except Exception as e:
                # Otros errores
                if self.connected:  # Solo mostrar errores si aún estamos conectados
                    self.after(0, lambda: self.status_bar.config(text=f"Error: {str(e)}"))
                break
    
    def update_plots(self, data):
        """Actualiza los gráficos con nuevos datos"""
        # Desplazar datos antiguos
        self.tiempo_data = np.roll(self.tiempo_data, -1)
        self.valor1_data = np.roll(self.valor1_data, -1)
        self.valor2_data = np.roll(self.valor2_data, -1)
        self.valor3_data = np.roll(self.valor3_data, -1)
        
        # Añadir nuevos datos
        tiempo_actual = data.get('tiempo', 0)
        valor1_actual = data.get('valor1', 0)
        valor2_actual = data.get('valor2', 0)
        valor3_actual = data.get('valor3', 0)
        
        self.tiempo_data[-1] = tiempo_actual
        self.valor1_data[-1] = valor1_actual
        self.valor2_data[-1] = valor2_actual
        self.valor3_data[-1] = valor3_actual
        
        # Actualizar la etiqueta de última actualización
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.last_update_label.config(text=f"Última actualización: {current_time}")
        
        # Mostrar datos en la barra de estado
        self.status_bar.config(text=
            f"Datos recibidos - Tiempo: {tiempo_actual:.2f}s, "
            f"Valor1: {valor1_actual:.2f}, "
            f"Valor2: {valor2_actual:.2f}, "
            f"Valor3: {valor3_actual:.2f}"
        )
        
        # Procesar para detección de apnea (solo si está activado)
        if self.analisis_apnea_activado:
            # Agregar al detector y verificar apnea
            hay_apnea, prob = self.detector_apnea.agregar_dato(valor1_actual)
            
            # Si tenemos una ventana completa, actualizar estado
            if len(self.detector_apnea.buffer) >= self.detector_apnea.ventana_puntos:
                self.update_apnea_status(hay_apnea, prob)
                
                # Actualizar región de análisis
                if len(self.tiempo_data) >= self.bloque_puntos:
                    self.analysis_region.remove()
                    t_min = max(0, self.tiempo_data[-self.bloque_puntos])
                    t_max = self.tiempo_data[-1]
                    self.analysis_region = self.ax1.axvspan(t_min, t_max, alpha=0.2, color='white')
    
    def update_apnea_status(self, hay_apnea, probabilidad):
        """Actualiza el estado de detección de apnea en la interfaz"""
        self.apnea_detectada = hay_apnea
        self.probabilidad_apnea = probabilidad
        
        # Actualizar etiquetas
        self.apnea_prob_label.config(text=f"Probabilidad: {probabilidad:.2f}")
        
        if hay_apnea:
            self.apnea_status_label.config(text="Apnea: ¡POSIBLE APNEA DETECTADA!", fg="#FF0000")
            
            # Mostrar texto de alerta en el gráfico
            if len(self.tiempo_data) > 0 and np.any(self.tiempo_data > 0):
                self.apnea_text.set_text(f"POSIBLE APNEA ({probabilidad:.2f})")
                self.apnea_text.set_visible(True)

        else:
            self.apnea_status_label.config(text="Apnea: No detectada", fg="#00FF00")
            self.apnea_text.set_visible(False)
    
    def update_ui(self):
        """Actualiza la interfaz gráfica periódicamente"""
        # Actualizar gráficos con los datos más recientes
        if len(self.tiempo_data) > 0 and np.any(self.tiempo_data > 0):
            # Actualizar datos de las líneas
            self.line1.set_data(self.tiempo_data, self.valor1_data)
            self.line2.set_data(self.tiempo_data, self.valor2_data)
            self.line3.set_data(self.tiempo_data, self.valor3_data)
            
            # Ajustar límites de los ejes
            for ax in [self.ax1, self.ax2, self.ax3]:
                if np.max(self.tiempo_data) > 0:
                    ax.set_xlim(
                        max(0, np.max(self.tiempo_data) - 20),  # Mostrar últimos 20 segundos
                        np.max(self.tiempo_data)
                    )
                ax.relim()  # Recalcular límites
                ax.autoscale_view(scalex=False)  # Auto escala en Y
            
            # Redibujar el canvas
            self.canvas.draw_idle()
        
        # Programar la próxima actualización
        self.after(50, self.update_ui)
    
    def update_connection_status(self, connected, status_text):
        """Actualiza el estado de conexión en la interfaz"""
        if connected:
            self.status_label.config(text=f"Estado: {status_text}", fg="#00FF00")
        else:
            self.status_label.config(text=f"Estado: {status_text}", fg="#E50914")
    
    def clear_data(self):
        """Limpia todos los datos y reinicia los gráficos"""
        # Reiniciar arrays de datos
        self.tiempo_data = np.zeros(self.max_points)
        self.valor1_data = np.zeros(self.max_points)
        self.valor2_data = np.zeros(self.max_points)
        self.valor3_data = np.zeros(self.max_points)
        
        # Reiniciar detector de apnea
        self.detector_apnea.reiniciar_buffer()
        self.bloque_actual = []
        
        # Actualizar gráficos
        self.line1.set_data(self.tiempo_data, self.valor1_data)
        self.line2.set_data(self.tiempo_data, self.valor2_data)
        self.line3.set_data(self.tiempo_data, self.valor3_data)
        
        # Restablecer etiquetas
        self.apnea_status_label.config(text="Apnea: No detectada", fg="#00FF00")
        self.apnea_prob_label.config(text="Probabilidad: 0.00")
        self.last_update_label.config(text="Última actualización: --")
        self.alert_status_label.config(text="Última alerta: Ninguna")
        
        # Ocultar texto de alerta
        self.apnea_text.set_visible(False)
        
        # Redibujar canvas
        self.canvas.draw_idle()
        
        # Mensaje en barra de estado
        self.status_bar.config(text="Datos reiniciados")

# Punto de entrada de la aplicación
if __name__ == "__main__":
    app = SignalVisualizer()
    app.mainloop()