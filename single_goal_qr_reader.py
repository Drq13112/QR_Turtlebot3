import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import queue

# Definimos una cola para la comunicación entre hilos
message_queue = queue.Queue()

class QR_SUB(Node):
    def __init__(self):
        super().__init__('QR_SUB')
        self.a_group_=ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            Bool,
            '/qr_detected',
            self.listener_callback,
            qos_profile_sensor_data)
        print("init finished")

    def listener_callback(self, data):
        print("reading")
        if data:
            print("Se recibio: True")
            message_queue.put(data)

class NAV(Node):
    def __init__(self):
        super().__init__('NAV')
        self.b_group_=ReentrantCallbackGroup()
        self.timer_1=self.create_timer(1.0,self.callback)

    def callback(self):
        navigator = BasicNavigator()
        print("waitting")

        navigator.waitUntilNav2Active()
        print("inside")
        security_route = [
        [3.0,-2.6],
        [3.5,1.0],
        [-0.6,2.7],
        [-2.74,1.0],
        [-2.62,-1.2],
        [-2.54,-3.63],
        [2.64,-3.45],
        [0.0,0.0]]
        
        QR_points=[
        [1.0,2.2,0.0],
        [0.0,-0.6,0.0],
        [3.0,-4.0,0.0],
        [0.5,-1.0,-0.707],
        [-2.5,-1.0,1.0],
        [-2.2,1.0,0.707]]

        for pt in QR_points:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 0.0
            goal_pose.pose.position.y = 0.0
            #goal_pose.pose.orientation.w = 0.707
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.position.x = pt[0]
            goal_pose.pose.position.y = pt[1]
            goal_pose.pose.orientation.z = pt[2]
            print("sending new goal")
            navigator.goToPose(goal_pose)
            i = 0
            print("here2")
            while not navigator.isTaskComplete():
                try:
                    # Intentamos obtener un mensaje de la cola
                    msg = message_queue.get(timeout=1)  # Espera un segundo para recibir un mensaje
                    # Hacemos algo con el mensaje recibido
                    print("Mensaje recibido en el hilo:", msg)
                except queue.Empty:
                    # Si no hay mensajes en la cola, continuamos
                    print("no msg")


                # Do something with the feedback
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                        navigator.cancelTask()

            # Do something depending on the return code
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()

def main(args=None):
    rclpy.init(args=args)
    nav=NAV()
    qr_reader=QR_SUB()
    executor = MultiThreadedExecutor()
    executor.add_node(nav)
    executor.add_node(qr_reader)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
