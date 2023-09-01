import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import torch
import torchvision
from torchvision.ops import nms
from torchvision.transforms import Resize, Grayscale, transforms, ToTensor
from PIL import Image as PILImage
import numpy as np
import io
import threading


class LarvaeCountNode(Node):

    def __init__(self, floater_name):
        super().__init__(f'{floater_name}_larvae_count_node')
        self.floater_name = floater_name
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.new_device_model = self.load_pretrained_model()
        self.new_device_model.to(self.device)
        self.new_device_model.eval()
        self.publisher = self.create_publisher(Int32, f'/{floater_name}/number_larvae', 10)
        self.subscription = self.create_subscription(
            Image,
            f'/{floater_name}/microscope',
            self.microscope_callback,
            10
        )

    def load_pretrained_model(self):
        new_device_model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=False, num_classes=3)
        #add the location of the saved model
        saved_state_dict = torch.load("src/grpc_ros_adapter/grpc_ros_adapter/saved_model_29_08_23-09_12_31.pt", map_location=self.device)
        new_device_model.load_state_dict(saved_state_dict)
        return new_device_model

    def microscope_callback(self, msg):
        # Convert ROS Image message to PIL Image
        pil_image = self.convert_image_message_to_pil(msg)
        
        # Perform larvae counting using the loaded model
        larvae_count = self.count_larvae(self.new_device_model, pil_image)
        
        # Publish the larvae count
        larvae_count_msg = Int32()
        larvae_count_msg.data = larvae_count
        self.publisher.publish(larvae_count_msg)
        
        self.get_logger().info(f'Larvae Count: {larvae_count}')

    def convert_image_message_to_pil(self, image_msg):
        image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        return PILImage.fromarray(image)

    def count_larvae(self, model, pil_image, iou_threshold=0.15):
        model.eval()

        tfs = transforms.Compose([
            transforms.Resize((150, 150)),
            transforms.Grayscale(),
            transforms.ToTensor()
        ])
        img = tfs(pil_image).unsqueeze(0).to(self.device)

        output = model(img)
        output_boxes = output[0]["boxes"].cpu().detach()
        output_scores = output[0]["scores"].cpu().detach()
        output_labels = output[0]["labels"].cpu().detach()

        keep = nms(output_boxes, output_scores, iou_threshold)

        filtered_labels = output_labels[keep]

        num_larvae = (filtered_labels == 1).sum().item()

        return num_larvae

def main(args=None):
    rclpy.init(args=args)
    
    floater_names = ['floater', 'floater_2']  # Add more floater names as needed
   
    LarvaeCountNodes = {}
    executor = rclpy.executors.MultiThreadedExecutor()

    for name in floater_names:
        LarvaeCountNodes[name] = LarvaeCountNode(name)
        executor.add_node(LarvaeCountNodes[name])


    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = LarvaeCountNodes[floater_names[0]].create_rate(2)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    executor_thread.join()

    for name in floater_names:
        LarvaeCountNodes[name].destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()





