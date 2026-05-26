#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FiltroHumoNode(Node):
    def __init__(self):
        super().__init__('filtro_humo_node')
        
        self.bridge = CvBridge()
        
        # Suscripción al video original a color de la OAK-D
        self.sub_image = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10)
            
        # Publicador de la imagen limpia (Dehazed)
        self.pub_image = self.create_publisher(
            Image,
            '/oak/rgb/image_clean',
            10)
            
        # Configurac    ión de CLAHE para mejorar contraste local ante destellos de fuego
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        
        self.get_logger().info('=== PASO 2: NODO DE ELIMINACIÓN DE HUMO ACTIVO ===')

    def aplicar_dehazing_simplificado(self, img):
        """
        Algoritmo optimizado basado en Dark Channel Prior (DCP) 
        para remover capas de humo/niebla en tiempo real.
        """
        # 1. Obtener el Canal Oscuro (estimación del humo)
        canal_oscuro = np.min(img, axis=2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        comprimido = cv2.erode(canal_oscuro, kernel)
        
        # 2. Estimar la luz atmosférica (brillo del humo denso)
        luz_atmosferica = np.max(comprimido)
        if luz_atmosferica == 0: luz_atmosferica = 1
        
        # 3. Calcular el mapa de transmisión (qué tanta luz real pasa a través del humo)
        img_normalizada = img.astype(np.float64) / luz_atmosferica
        canal_oscuro_norm = np.min(img_normalizada, axis=2)
        transmision = 1.0 - 0.85 * cv2.erode(canal_oscuro_norm, kernel)
        
        # Sostener la transmisión mínima para evitar división por cero en zonas ultra densas
        transmision = np.maximum(transmision, 0.1)
        
        # 4. Reconstruir la escena libre de humo
        img_limpia = np.zeros(img.shape, dtype=np.float64)
        for i in range(3):
            img_limpia[:,:,i] = ((img.astype(np.float64)[:,:,i] - luz_atmosferica) / transmision) + luz_atmosferica
            
        # Cortar los límites y regresar a formato estándar de 8 bits
        img_limpia = np.clip(img_limpia, 0, 255).astype(np.uint8)
        return img_limpia

    def image_callback(self, msg):
        try:
            # Convertir mensaje de ROS a OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # --- CAPA 1: CLAHE en el canal de Luminancia (Compensación de Brillo/Fuego) ---
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l_ecualizada = self.clahe.apply(l)
            lab_fusion = cv2.merge((l_ecualizada, a, b))
            frame_contratado = cv2.cvtColor(lab_fusion, cv2.COLOR_LAB2BGR)
            
            # --- CAPA 2: Eliminación de Humo (Dehazing) ---
            frame_limpio = self.aplicar_dehazing_simplificado(frame_contratado)
            
            # Convertir de vuelta a mensaje de ROS y publicar
            msg_salida = self.bridge.cv2_to_imgmsg(frame_limpio, encoding='bgr8')
            msg_salida.header = msg.header
            self.pub_image.publish(msg_salida)
            
        except Exception as e:
            self.get_logger().error(f'Error en el filtrado de humo: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FiltroHumoNode()
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