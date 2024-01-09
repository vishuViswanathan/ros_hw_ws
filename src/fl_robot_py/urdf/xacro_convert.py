
import xml.etree.ElementTree as ET
import launch
import launch_ros.actions
import xacro


xacro_f = 'fl_robot01.urdf'
if xacro_f[-5:] == 'xacro':
    print('a xacro file')
    doc = xacro.process_file(xacro_f)
    # , mappings={'radius': '0.9'})
    robot_xml = doc.toprettyxml(indent=' ')
    #file_obj = open(xacro_f + '.urdf', 'w')
    #file_obj.write(robot_xml)
    #file_obj.close()
    #tree = ET.parse(robot_xml)
    root = ET.fromstring(robot_xml)
else:
    print('an urdf file ?')
    tree = ET.parse(xacro_f)
    root = tree.getroot()

print(root)


for plugin in root.iter('plugin'):
    print(plugin.attrib.values())
