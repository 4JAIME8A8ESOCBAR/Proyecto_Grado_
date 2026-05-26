#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import numpy as np

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import tf2_ros
from cv_bridge import CvBridge

class DetectorRescateHumbleNode(Node):
    def __init__(self):
        super().__init__('detector_rescate_node')
        
        self.bridge = CvBridge()
        self.mapa_profundidad = None

        # Suscripciones
        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/oak/nn/detections',
            self.detection_callback,
            10)
            
        self.sub_depth = self.create_subscription(
            Image,
            '/oak/stereo/image_raw',
            self.depth_callback,
            10)
            
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # VARIABLES DE PERSISTENCIA TEMPORAL (Humo/Oclusión)
        self.ultima_posicion_valida = None
        self.ultimo_tiempo_detectado = 0.0
        self.tiempo_gracia_humo = 1.5  
        
        # LÓGICA DE CONFIRMACIÓN (Mínimo 2 fotogramas seguidos)
        self.contador_detecciones = 0
        self.umbral_confirmacion = 2  
        
        self.tf_timer = self.create_timer(0.1, self.publicar_tf_persistente)
        
        self.get_logger().info('=== SISTEMA DE RESCATE: SÓLO CONFIRMACIÓN ===')
        self.get_logger().info('Filtro Delta-Z removido. Filtro de doble impacto activo.')

    def depth_callback(self, msg):
        try:
            self.mapa_profundidad = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Error profundidad: {str(e)}')

    def detection_callback(self, msg):
        if self.mapa_profundidad is None or not msg.detections:
            # Si la IA no ve a nadie en este cuadro, bajamos el contador gradualmente
            if self.contador_detecciones > 0:
                self.contador_detecciones -= 1
            return

        for detection in msg.detections:
            for result in detection.results:
                clase_id = "desconocido"
                if hasattr(result, 'hypothesis'):
                    clase_id = getattr(result.hypothesis, 'class_id', getattr(result.hypothesis, 'id', 'obj'))
                elif hasattr(result, 'id'):
                    clase_id = result.id

                # Filtrar solo clase humana (person o ID 15 en VOC/COCO)
                if clase_id != 'person' and clase_id != '15' and clase_id != 15:
                    continue  

                # Extraer centro de la caja delimitadora
                pixel_x = int(detection.bbox.center.position.x)
                pixel_y = int(detection.bbox.center.position.y)

                alto, ancho = self.mapa_profundidad.shape
                if 0 <= pixel_x < ancho and 0 <= pixel_y < alto:
                    
                    # Ventana de reducción de ruido (clipping/ruido estéreo)
                    rango = 7
                    ventana = self.mapa_profundidad[max(0, pixel_y-rango):min(alto, pixel_y+rango+1), 
                                                   max(0, pixel_x-rango):min(ancho, pixel_x+rango+1)]
                    valores_validos = ventana[ventana > 0]
                    
                    if valores_validos.size > 0:
                        distancia_mm = np.median(valores_validos)
                    else:
                        continue 

                    z_metros = distancia_mm / 1000.0

                    # Proyección 3D usando la óptica de la OAK-D
                    fov_h_rad = np.radians(69.0)
                    fov_v_rad = np.radians(54.0)
                    x_metros = (pixel_x - (ancho / 2.0)) * (2.0 * z_metros * np.tan(fov_h_rad / 2.0)) / ancho
                    y_metros = (pixel_y - (alto / 2.0)) * (2.0 * z_metros * np.tan(fov_v_rad / 2.0)) / alto

                    # FILTRO DE RANGO OPERATIVO (Acotado a tus pruebas)
                    if 0.4 < z_metros < 4.20:
                        
                        # Incrementamos el contador de confianza por cada fotograma válido
                        self.contador_detecciones += 1
                        
                        # Si se cumple el doble impacto, actualizamos la posición oficial
                        if self.contador_detecciones >= self.umbral_confirmacion:
                            self.ultima_posicion_valida = (x_metros, y_metros, z_metros)
                            self.ultimo_tiempo_detectado = time.time()
                            
                            self.get_logger().warn(
                                f'¡VÍCTIMA CONFIRMADA! -> Distancia: {z_metros:.2f}m | X: {x_metros:.2f}'
                            )
                            # Previene el crecimiento infinito del contador
                            self.contador_detecciones = self.umbral_confirmacion

    def publicar_tf_persistente(self):
        if self.ultima_posicion_valida is None:
            return
            
        tiempo_transcurrido = time.time() - self.ultimo_tiempo_detectado
        
        # Mantiene la posición en el mapa durante el tiempo de gracia (por si el humo tapa la vista)
        if tiempo_transcurrido <= self.tiempo_gracia_humo:
            x, y, z = self.ultima_posicion_valida
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'oak_rgb_camera_optical_frame'
            t.child_frame_id = 'victima_rescate'
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
        else:
            # Si pasa el tiempo de gracia sin ver a la víctima, limpiamos los datos
            self.ultima_posicion_valida = None
            self.contador_detecciones = 0 

def main(args=None):
    rclpy.init(args=args)
    node = DetectorRescateHumbleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()