import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, Point
from rcl_interfaces.msg import SetParametersResult

"""
Utilizando la convencion: Metros, Segundos y Radianes
"""
 
class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.get_logger().info("The Path Generator Node has succesfully initialized...")

        # - PARAMETERS - #
        # Puntos de figuras predefinidos en elplano X,Y
        self.declare_parameter('figure', 'square') # square, path1, path2, path3
        self.fig = self.get_parameter('figure').value

        # Predifined figures
        self.p1 = [4.0, 0.0]
        self.p2 = [2.0, 2.0]
        self.p3 = [0.0, 2.0]
        self.p4 = [0.0, 0.0]
            
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Punto de destino
        self.goal = Point()
        
        # Guadrdamos los puntos en matriz
        self.dots = [self.p1,self.p2,self.p3,self.p4]

        # Contador de iterar sobre puntos
        self.cont = 0

        # Bandera para mandar valor
        self.send_pose = False

        # Publicadores
        self.publisher_goal = self.create_publisher(Point, 'set_point', 10)
        # Suscriptores
        self.finish_sub = self.create_subscription(Bool, 'finish', self.finish_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def finish_callback(self, msg):
        ''' Callback para recibir el mensaje de finish '''
        self.send_pose = msg.data
        if self.send_pose:
            self.cont += 1
            if self.cont > 4:
                self.cont = 0
                self.send_pose = False

    def timer_callback(self):
        ''' Callback para el timer '''
        if self.send_pose:
            self.send_pose = False # Reiniciamos la bandera

            self.goal.x = self.dots[self.cont - 1][0] # Coordenada X
            self.goal.y = self.dots[self.cont - 1][1] # Coordenada Y
                       
            self.publisher_goal.publish(self.goal) # Publicamos el punto
                
            self.get_logger().info(f"Sending Goal: {self.goal.x}, {self.goal.y}")
            self.get_logger().info(f"The Path Generator Node published point number {self.cont}...")

    def parameter_callback(self, params):
        for param in params:
            # Check Signal type
            if param.name in "figure":
                if (param.value != "path1" and param.value != "path2" and param.value != "path3" and param.value != "square") :
                    self.get_logger().warn("Invalid figure name!!!")
                    return SetParametersResult(successful=False, reason="Invalid figure name!!!")
                else:  
                    if param.name == "figure":
                        self.fig = param.value

                        if self.fig == 'square':
                            self.p1 = [4.0, 0.0]
                            self.p2 = [2.0, 2.0]
                            self.p3 = [0.0, 2.0]
                            self.p4 = [0.0, 0.0] 
                            self.get_logger().info(f"Param: {param.name} successfully changed to: {param.value}")
                            self.get_logger().info(f"New points: {self.p1}, {self.p2}, {self.p3}, {self.p4}")
                        
                        elif self.fig == 'path1':
                            self.p1 = [-1.0, 0.0]
                            self.p2 = [-0.5, 1.0]
                            self.p3 = [-1.0, 2.0]
                            self.p4 = [0.0, 2.0] 
                            self.get_logger().info(f"Param: {param.name} successfully changed to: {param.value}")
                            self.get_logger().info(f"New points: {self.p1}, {self.p2}, {self.p3}, {self.p4}")
                        
                        elif self.fig == 'path2':
                            self.p1 = [1.0, 0.0]
                            self.p2 = [1.0, 1.0]
                            self.p3 = [0.0, 1.0]
                            self.p4 = [0.0, 0.0] 
                            self.get_logger().info(f"Param: {param.name} successfully changed to: {param.value}")
                            self.get_logger().info(f"New points: {self.p1}, {self.p2}, {self.p3}, {self.p4}")
                        
                        elif self.fig == 'path3':
                            self.p1 = [1.0, -1.0]
                            self.p2 = [0.0, 1.0]
                            self.p3 = [1.0, 0.0]
                            self.p4 = [0.0, 0.0] 
                            self.get_logger().info(f"Param: {param.name} successfully changed to: {param.value}")
                            self.get_logger().info(f"New points: {self.p1}, {self.p2}, {self.p3}, {self.p4}")
                        
                        else:
                            self.get_logger().warn("Invalid figure name!!! Defaulting to square...")
                            self.p1 = [4.0, 0.0]
                            self.p2 = [2.0, 2.0]
                            self.p3 = [0.0, 2.0]
                            self.p4 = [0.0, 0.0]
                    
        return SetParametersResult(successful=True) 

def main(args=None): 
    rclpy.init(args=args) 
    m_p=PathGenerator() 
    rclpy.spin(m_p) 
    m_p.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main() 