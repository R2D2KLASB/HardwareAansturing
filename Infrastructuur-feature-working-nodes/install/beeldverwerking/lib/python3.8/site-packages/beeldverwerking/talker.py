## @package beeldverwerking.talker
# Beeldverwerking interface and ROS2 talker.
#
# An online interface voor uploading image's that will generate and send g-code in an ROS2 publisher node.
# Setup
# =====
# - Event Handler
# - ROS2 Publisher Node
# - Image editor
# - Web interface
# 
# Todo
# ====
# - Live PI camera.
# - Better looking interface.
#

from .web.app import App
from .handler.eventhandler import eventHandler
from .image.editor import *
from .image.gcode import *
from .publisher_node.publisherNode import Publisher
import rclpy
import sys
import time
from os.path import exists
from ament_index_python.packages import get_package_share_directory
# from .pi.camera import Camera


## Talker main
def main(args=None):

    # Command-line parameters
    par = sys.argv

    # Create ROS2 Node
    rclpy.init(args=args)
    publisher = Publisher()

    # Get package path
    rp = get_package_share_directory('beeldverwerking')

    # Setup eventHandler for image editing and ros2 communication
    _eventHandler = eventHandler()

    ## Add functions to evendHandler 
    #Image edge detection
    _eventHandler.createEvent('edit', lambda file: imageToEdge(file))
        #Image to SVG
    _eventHandler.createEvent('svg', lambda file: edgeToSVG(file))
        #SVG to Gcode
    _eventHandler.createEvent('gcode', lambda path, name: svgToGcode(path, name))
        #Send over ROS2
    _eventHandler.createEvent('send', lambda image: publisher.send(image))

    if len(par) > 1:
        # Camera
        if len(par) == 2:
            if par[1] == 'camera':
                # Setup camera
                piCamera = Camera(int(par[1]))
                while(True):
                    # Check if button pressed
                    if piCamera.checkButton():
                        # Create Picture
                        piCamera.createPic(rp, 'pi.jpg')
                        # Get gcode from image
                        response = _eventHandler.runEvent('upload')(rp + "/pi.jpg")
                        # Send gcode over ROS2
                        _eventHandler.runEvent('send')(response['gcode'])
                        time.sleep(2)


        # Command-Line
        else:
        # Check if file excist
            file = par[1]
            if exists(file):
                extensions = ['jpg', 'jpeg', 'png']
                # Check if file is an image
                if file.split('.')[-1] in extensions:
                    with open(file, "rb") as imageFile:
                        # Edit image
                        images = _eventHandler.runEvent('upload')(imageFile)
                    # Send edited image over ROS2
                    _eventHandler.runEvent('send')(images[-1]['jpg'].decode("utf-8"))
                else:
                    print("File not supported")        
            else:
                print("File doesn't excist")

    # Web Interface
    else:
        #Run Web Interface
        webApp = App(_eventHandler, rp)
        webApp.run(8080)

    #Destroy ROS2 Node
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()