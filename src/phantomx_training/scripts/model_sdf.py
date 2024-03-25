import xml.etree.ElementTree as ET

def parser_model_sdf(file, new_model_file):

    with open(file) as f:
        data = f.read()

    root = ET.fromstring(data)

    uri_element = root.find('.//uri')

    old_model_file = uri_element.text

    for uri_element in root.findall('.//uri'):
        uri_element.text = uri_element.text.replace(old_model_file.strip(), new_model_file.strip())
    
    with open(file, 'w') as f:
        f.write(ET.tostring(root).decode())    
