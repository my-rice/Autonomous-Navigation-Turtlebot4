# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node
import cv_bridge
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from sig_rec.qreader import QReader



class Image2Code(Node):

    def __init__(self, node_name,**kwargs) -> None:
        super().__init__(node_name,**kwargs)
        
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")
        self._bridge = cv_bridge.CvBridge()
        self._detector = QReader(model_size="n")

        self._command_pub = self.create_publisher(String, "/output_command",10)
        self._image_sub = self.create_subscription(CompressedImage, "/oakd/rgb/preview/image_raw/compressed", self.on_imageread, 10)
        self.publishing = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.publishing = True
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'")
        self.publishing = False
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State):
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'cleaning up'")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'")
        return TransitionCallbackReturn.SUCCESS
    
    
        
    def on_imageread(self, msg):
        if(self.publishing == True):
          image_msg = msg
          cv_image = self._bridge.compressed_imgmsg_to_cv2(image_msg)
        
          decoded_qrs, locations = self._detector.detect_and_decode(
              image=cv_image, return_detections=True
          )
          # Print the results
          #print(f"Image: {frame} -> {len(decoded_qrs)} QRs detected.")
          if len(decoded_qrs) == 0:
              out = String()
              out.data = "NOCODE"
              self.get_logger().info(out.data)
              self._command_pub.publish(out)
          else:
              for content, location in zip(decoded_qrs, locations):
                  #print(f"Content: {content}. Position: {tuple(location[BBOX_XYXY])}"
                  out = String()
                  out.data = "None" if (content is None) else content
                  self.get_logger().info(out.data)
                  self._command_pub.publish(out)
        



def main(args=None) -> None:
    
    rclpy.init(args=args)
    decoder = Image2Code("lifecycle_perception")
    rclpy.spin(decoder)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

