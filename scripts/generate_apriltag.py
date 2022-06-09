import os
from importlib_metadata import version
import requests
from xml.etree import ElementTree
from xml.dom import minidom



from numpy import save

class ApriltagGenerator:
    def __init__(self):
        self.main_url = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master"
        self.families = ["tag16h5", "tag25h9", "tag36h11", "tagCircle21h7", "tagCircle49h12", 
                         "tagCustom48h12", "tagStandard41h12", "tagStandard52h13"]
    
    def generateTag(self, save_path, family="tag36h11", id=0):
        if family not in self.families:
            print("TAG family not exist. Available families are:")
            for f in self.families: print(f, end=" ")
            print()
        
        else:
            
            if not os.path.exists(save_path):
                os.mkdir(save_path)

            tag_name = family.replace('h', '_') + "_%05d" % id
            save_path = os.path.join(save_path, tag_name)

            if not os.path.exists(save_path):
                print(f"Creating {save_path}")
                os.mkdir(save_path)
            
            tag_url = os.path.join(self.main_url, family, tag_name+".png")
            tag_img_path = os.path.join(save_path, "tag.png")

            tag_img_data = requests.get(tag_url).content
            with open(tag_img_path, 'wb') as handler:
                handler.write(tag_img_data)

    def generateSDF(self, save_path, family="tag36h11", id=0):
        ElementTree.ElementTree("")
        root = ElementTree.Element("sdf", version="1.6")



if __name__ == "__main__":
    tag_generator = ApriltagGenerator()
    tag_generator.generateTag(save_path="../models")
