from xml.etree.ElementTree import *
import sys




def prettify(elem):
    from xml.etree import ElementTree
    from xml.dom import minidom
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    #return ElementTree(rough_string)

innerModel = Element('innerModel')
transformExt = SubElement(innerModel,'transform')
transformExt.set('id', 'world')
transform = SubElement(transformExt,'transform')
transform.set('id', 'floor')

# scenary ground
plane = SubElement(transform,'plane')
plane.set('id', 'ddG') 
plane.set('ny', '1')
plane.set('px', '-0')
plane.set('py', '0')
plane.set('pz', '0')
plane.set('size', '10000,10000,100')
plane.set('texture', '/home/robocomp/robocomp/files/osgModels/textures/wood.jpg')

# ddRight
plane = SubElement(transform,'plane')
plane.set('id', 'ddR') 
plane.set('nx', '1')
plane.set('px', '5000')
plane.set('py', '200')
plane.set('pz', '0')
plane.set('size', '10000,500,10')
plane.set('texture', '#eeeeee')

# ddLeft
plane = SubElement(transform,'plane')
plane.set('id', 'ddL') 
plane.set('nx', '1')
plane.set('px', '-5000')
plane.set('py', '200')
plane.set('pz', '0')
plane.set('size', '10000,500,10')
plane.set('texture', '#eeeeee')

# ddFront
plane = SubElement(transform,'plane')
plane.set('id', 'ddF') 
plane.set('nx', '1')
plane.set('px', '5000')
plane.set('py', '200')
plane.set('pz', '0')
plane.set('size', '10000,500,10')
plane.set('texture', '#555555')

# ddBack
plane = SubElement(transform,'plane')
plane.set('id', 'ddB') 
plane.set('nx', '1')
plane.set('px', '-5000')
plane.set('py', '200')
plane.set('pz', '0')
plane.set('size', '10000,500,10')
plane.set('texture', '#555555')

# ddRight2
plane = SubElement(transform,'plane')
plane.set('id', 'ddR2') 
plane.set('nx', '1')
plane.set('px', '5000')
plane.set('py', '200')
plane.set('pz', '0')
plane.set('size', '10000,500,13')
plane.set('texture', '#eeeeee')

# ddLeft2
plane = SubElement(transform,'plane')
plane.set('id', 'ddL2') 
plane.set('nx', '1')
plane.set('px', '-5000')
plane.set('py', '200')
plane.set('pz', '0')
plane.set('size', '10000,500,13')
plane.set('texture', '#eeeeee')

# ddFront2
plane = SubElement(transform,'plane')
plane.set('id', 'ddF2') 
plane.set('nz', '1')
plane.set('pz', '5000')
plane.set('py', '200')
plane.set('px', '0')
plane.set('size', '10000,500,13')
plane.set('texture', '#555555')

# ddBack2
plane = SubElement(transform,'plane')
plane.set('id', 'ddB2') 
plane.set('nz', '1')
plane.set('pz', '-5000')
plane.set('py', '200')
plane.set('px', '0')
plane.set('size', '10000,500,13')
plane.set('texture', '#555555')


port_prefix = 10004
robot_size = 0
inverter = False
count = 0
for unity in range(0,16):
    
    differentialrobot =  SubElement(transformExt,'differentialrobot')
    differentialrobot.set('id', 'base'+str(unity))
    differentialrobot.set('port', str(port_prefix + count))

    mesh  = SubElement(differentialrobot, 'mesh')
    mesh.set('id', 'base_robex'+str(unity))
    mesh.set('file', "/home/robocomp/robocomp/files/osgModels/robex/robex.ive")
    mesh.set('tx', str(robot_size)) #+-2300
    mesh.set('ty', '0')
    mesh.set('tz', str(robot_size)) #+-2300
    mesh.set('scale', '1000')
    mesh.set('collide', '1')

    translation = SubElement(differentialrobot, 'translation')
    translation.set('id', 'laserPose' + str(unity))
    translation.set('tx', '0')
    translation.set('ty', '140')
    translation.set('tz', '200')

    laser = SubElement(translation, 'laser')
    laser.set('id', 'laser' + str(unity))
    laser.set('port', str(port_prefix + count + 1))
    laser.set('measures', '100')
    laser.set('min', '100')
    laser.set('max', '10000')
    laser.set('angle', '3')
    laser.set('ifconfig', '10000')

    plane = SubElement(translation, 'plane')
    plane.set('id', 'sensorL' + str(unity))
    plane.set('nz', '1')
    plane.set('pz', '-200')
    plane.set('size', '100')
    plane.set('repeat', '1')
    plane.set('texture', '#ff0000')

    count = (count + 2)
    
    if(inverter):
        robot_size = (robot_size * -1)
        inverter = False
    else:
        robot_size = (robot_size * -1)
        robot_size = robot_size + 550
        inverter = True

document = ElementTree(innerModel)
document.write("robos.xml", xml_declaration=None)


