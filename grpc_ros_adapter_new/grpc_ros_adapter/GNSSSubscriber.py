import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams["figure.figsize"] = (5,5)
from matplotlib.ticker import StrMethodFormatter
import threading, time

larvae_data = {'longitude': [], 'latitude': [], 'altitude': [], 'count': []} #for plotting only

class GNSSSubscriber(Node):
    def __init__(self, floater_name):
        super().__init__(f'{floater_name}_gnss_subscriber')
        
        # self.plotter = plotter

        self.subscription = self.create_subscription(
            NavSatFix,
            f'/{floater_name}/gnss',
            self.gnss_callback,
            10
        )
        self.subscription_larvae = self.create_subscription(
            Int32,
            f'/{floater_name}/number_larvae',
            self.larvae_callback,
            10
        )

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        global larvae_data 
        self.flag = 0
        self.colorbar = None

    def gnss_callback(self, msg):
        if self.flag == 1:
            larvae_data['longitude'].append(float("{:.5f}".format(msg.longitude)))
            larvae_data['latitude'].append(float("{:.5f}".format(msg.latitude)))
            larvae_data['altitude'].append(msg.altitude)
            self.flag = 0
            time.sleep(0.5)
            # if len(larvae_data['longitude']) == len(larvae_data['count']):
            #     self.get_logger().info(f'Update Plot')
                # self.plotter.update_plot(
                #     larvae_data['longitude'],
                #     larvae_data['latitude'],
                #     larvae_data['altitude'],
                #     larvae_data['count'],
                # )
            
    def larvae_callback(self, msg):
        if self.flag == 0:
            larvae_data['count'].append(int(msg.data))
            self.get_logger().info(f'Larvae Count: {msg.data} {self.flag}')
            self.flag = 1
            time.sleep(0.5)

class Plotter:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.colorbar = None

    def update_plot(self, longitude, latitude, altitude, count):
        self.ax.clear()
        sc = self.ax.scatter(
            longitude,
            latitude,
            altitude,
            c=count,
            cmap='plasma',
            marker='o',
            s=50
        )

        self.ax.get_yaxis().get_major_formatter().set_useOffset(False)
        self.ax.get_xaxis().get_major_formatter().set_useOffset(False)

        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.set_zlabel('Altitude')
        self.ax.set_title('Larvae concentration')
        self.ax.autoscale()

        if self.colorbar:
            self.colorbar.remove()

        self.colorbar = self.fig.colorbar(sc, ax=self.ax)
        self.colorbar.set_label('Larvae Concentration /mL')
        plt.pause(1)


def main(args=None):
    rclpy.init(args=args)

    floater_names = ['floater', 'floater_2']  # Add more floater names as needed
    plotter = Plotter()
    gnss_subscribers = {}
    executor = rclpy.executors.MultiThreadedExecutor()
    
    for name in floater_names:
        gnss_subscribers[name] = GNSSSubscriber(name)
        executor.add_node(gnss_subscribers[name])


    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = gnss_subscribers[floater_names[0]].create_rate(2)

    try:
        while rclpy.ok():
            rate.sleep()
            if len(larvae_data['longitude']) == len(larvae_data['count']):
                plotter.update_plot(
                    larvae_data['longitude'],
                    larvae_data['latitude'],
                    larvae_data['altitude'],
                    larvae_data['count'],
                )
    except KeyboardInterrupt:
        pass
    executor_thread.join()

    for name in floater_names:
        gnss_subscribers[name].destroy_node()

    rclpy.shutdown()   

if __name__ == '__main__':
    main()
