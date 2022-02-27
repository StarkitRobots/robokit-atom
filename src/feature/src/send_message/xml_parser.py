from xml import dom
from lxml import etree
from xml.dom import minidom
 
def parseXML(xmlFile):
    #print all joints from robot urdf file with type continous
    
    xmldoc = minidom.parse(xmlFile)
    joint_list = xmldoc.getElementsByTagName('joint')
    # print (len(joint_list))
    # print (joint_list)
    joint_list_continuos = []
    for i in joint_list:
        if ( i.attributes['type'].value == 'continuous'):
            joint_list_continuos.append(i)
    joint_names = []
    for i in joint_list_continuos:
        joint_names.append(i.attributes['name'].value)
    
    print("\"", end='')
    for name in joint_names:
        print(name,end='\", \"')
    #print("\" ", end='')
    
 
if __name__ == "__main__":
    parseXML("robokit-atom.urdf")