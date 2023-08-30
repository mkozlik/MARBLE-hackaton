import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams["figure.figsize"] = (5,5)
from matplotlib.ticker import StrMethodFormatter

class GNSSSubscriber(Node):
    def __init__(self):
        super().__init__('gnss_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/floater/gnss',
            self.gnss_callback,
            10
        )
        self.subscription_larvae = self.create_subscription(
            Int32,
            '/floater/number_larvae',
            self.larvae_callback,
            10
        )

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.larvae_data = {'longitude': [], 'latitude': [], 'altitude': [], 'count': []}
        self.flag = 0
        self.colorbar = None

    def gnss_callback(self, msg):
        if self.flag == 1:
            self.larvae_data['longitude'].append(float("{:.5f}".format(msg.longitude)))
            self.larvae_data['latitude'].append(float("{:.5f}".format(msg.latitude)))
            self.larvae_data['altitude'].append(msg.altitude)
            self.flag = 0
            if len(self.larvae_data['longitude']) == len(self.larvae_data['count']):
                self.get_logger().info(f'Update Plot')
                self.update_plot()
            
    def larvae_callback(self, msg):
        if self.flag == 0:
            self.larvae_data['count'].append(int(msg.data))
            self.get_logger().info(f'Larvae Count: {msg.data} {self.flag}')
            self.flag = 1

    def update_plot(self):
        self.ax.clear()
        sc = self.ax.scatter(
            self.larvae_data['longitude'],
            self.larvae_data['latitude'],
            self.larvae_data['altitude'],
            c=self.larvae_data['count'],  # Use count values for color mapping
            cmap='plasma',
            marker='o',
            s=50  # Adjust the size of the points as needed
            
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
    gnss_subscriber = GNSSSubscriber()

    rclpy.spin(gnss_subscriber)

    gnss_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
