import os, shutil
import requests
from xml.etree import ElementTree
from xml.dom import minidom


class ApriltagGenerator:
    def __init__(self):
        self.main_url = "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master"
        self.families = ["tag16h5", "tag25h9", "tag36h11", "tagCircle21h7", "tagCircle49h12", 
                         "tagCustom48h12", "tagStandard41h12", "tagStandard52h13"]
    
    def generateTagModel(self, save_path, family="tag36h11", id=0, tag_size=1):

        if family not in self.families:
            print("TAG family not exist. Available families are:")
            for f in self.families: print(f, end=" ")
            print()
        
        else:
            
            if not os.path.exists(save_path):
                os.mkdir(save_path)

            tag_name = family.replace('h', '_') + "_%05d" % id
            save_path = os.path.join(save_path, tag_name)

            if os.path.exists(save_path):
                shutil.rmtree(save_path)
            
            print(f"Creating {save_path}")
            os.mkdir(save_path)
            materials_path = os.path.join(save_path, "materials")
            os.mkdir(materials_path)
            
            self.getTag(family=family, tag_name=tag_name, save_path=save_path)
            self.generateSDF(tag_name=tag_name, tag_size=tag_size, save_path=save_path)
            self.generateConfig(tag_name=tag_name, save_path=save_path)
            self.generateMaterial(save_path=save_path, tag_name=tag_name)
            
    def getTag(self, family, tag_name, save_path):
        tag_url = os.path.join(self.main_url, family, tag_name+".png")
        tag_save_path = os.path.join(save_path, "materials", "textures")
        os.mkdir(tag_save_path)
        tag_img_path = os.path.join(tag_save_path, tag_name+".png")

        tag_img_data = requests.get(tag_url).content
        with open(tag_img_path, 'wb') as handler:
            handler.write(tag_img_data)

    def generateSDF(self, tag_name, tag_size, save_path):
        ElementTree.ElementTree("")
        root = ElementTree.Element("sdf", version="1.6")

        model = ElementTree.SubElement(root, "model", name=tag_name)
        static = ElementTree.SubElement(model, "static")
        static.text = "true"
        
        link = ElementTree.SubElement(model, "link")
        
        pose = ElementTree.SubElement(link, "pose")
        pose.text = "0 0 0 0 0 0"
        
        visual = ElementTree.SubElement(link, "visual", name="visual")

        geometry = ElementTree.SubElement(visual, "geometry")
        box = ElementTree.SubElement(geometry, "box")
        size = ElementTree.SubElement(box, "size")
        size.text = f"{tag_size} {tag_size} {0.01*tag_size}"

        material = ElementTree.SubElement(visual, "material")
        script = ElementTree.SubElement(material, "script")
        uri_scripts = ElementTree.SubElement(script, "uri")
        uri_scripts.text = f"model://{tag_name}/materials/scripts"
        uri_textures = ElementTree.SubElement(script, "uri")
        uri_textures.text = f"model://{tag_name}/materials/textures"
        name = ElementTree.SubElement(script, "name")
        name.text = tag_name

        # Save model.sdf
        rough_string = ElementTree.tostring(root, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        root = ElementTree.fromstring(reparsed.toprettyxml(indent="  "))
        tree = ElementTree.ElementTree(root)

        tree.write( os.path.join(save_path, "model.sdf"), xml_declaration=True, encoding="utf-8" )

    def generateConfig(self, tag_name, save_path):
        ElementTree.ElementTree("")
        root = ElementTree.Element("model")

        name = ElementTree.SubElement(root, "name")
        name.text = tag_name
        version = ElementTree.SubElement(root, "version")
        version.text = "1.0"
        sdf = ElementTree.SubElement(root, "sdf", version="1.6")
        sdf.text = "model.sdf"

        author = ElementTree.SubElement(root, "author")
        author_name = ElementTree.SubElement(author, "name")
        author_name.text = "Felipe Mohr"
        author_email = ElementTree.SubElement(author, "email")
        author_email.text = "felipe18mohr@gmail.com"

        description = ElementTree.SubElement(root, "description")
        description.text = f"Apriltag {tag_name} model"

        # Save model.config
        rough_string = ElementTree.tostring(root, 'utf-8')
        reparsed = minidom.parseString(rough_string)
        root = ElementTree.fromstring(reparsed.toprettyxml(indent="  "))
        tree = ElementTree.ElementTree(root)

        tree.write( os.path.join(save_path, "model.config"), xml_declaration=True, encoding="utf-8" )

    def generateMaterial(self, save_path, tag_name):
        materials_path = os.path.join(save_path, "materials")
        
        script_text = f"material {tag_name}\n" \
                      "{\n" \
                      "technique\n" \
                      "{\n" \
                      "    pass\n" \
                      "    {\n" \
                      "    lighting off\n" \
                      "    texture_unit\n" \
                      "    {\n" \
                      f"        texture {tag_name}.png\n" \
                      "        filtering none none none\n" \
                      "        scale 1.0 1.0\n" \
                      "    }\n" \
                      "    }\n" \
                      "}\n" \
                      "}"
        
        scripts_path = os.path.join(materials_path, "scripts")
        os.mkdir(scripts_path)
        scripts_file_path = os.path.join(scripts_path, "Apriltag.material")
        scripts_file = open(scripts_file_path, "w")
        scripts_file.write(script_text)
        

if __name__ == "__main__":
    tag_generator = ApriltagGenerator()
    tag_generator.generateTagModel(save_path="../models")
