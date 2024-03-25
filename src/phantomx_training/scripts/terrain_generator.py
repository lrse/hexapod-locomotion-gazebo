import os 
import sys
import numpy as np
import uuid
import random
from noise import snoise2
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from perlin_noise import PerlinNoise

def parser_model_sdf(file, new_model_file, width=12, height=12, height_z=0.1):

    """ This function is used to parse the model.sdf file and replace the old model file with the new one. """ 

    with open(file) as f:
        data = f.read()

    root = ET.fromstring(data)

    uri_element = root.find('.//uri')

    old_model_file = uri_element.text

    for uri_element in root.findall('.//uri'):
        uri_element.text = uri_element.text.replace(old_model_file.strip(), new_model_file.strip())


    size_element = root.find('.//size')
    for size_element in root.findall('.//size'):
        size_element.text = size_element.text.replace(size_element.text, f"{width} {height} {height_z}")

    with open(file, 'w') as f:
        f.write(ET.tostring(root).decode())    


def octaves_perlin(roughness, frequency, amplitude, octaves=1, lacunarity=1000.02, persistence=0.002, width=129, height=129):

    """ This function is used to generate a perlin noise heightmap using the octaves method.

    Parameters
    ----------
    octaves (int): Number of octaves to use in the perlin noise algorithm.
    frequency (float): Frequency of the perlin noise algorithm.
    amplitude (float): Amplitude of the perlin noise algorithm.
    lacunarity (float): Lacunarity of the perlin noise algorithm.
    persistence (float): Persistence of the perlin noise algorithm.
    width (int): Width of the heightmap.
    height (int): Height of the heightmap. """

    # Initialize Perlin Noise
    noise = PerlinNoise(octaves=1)
    # Create an empty heightmap array
    heightMap = np.zeros((height, width))

    for y in range(height):

        for x in range(width):

            heightMap[y][x] = octave_perlin_at_coordinate(noise, octaves, persistence, roughness, frequency, lacunarity, amplitude, x, y)

    # Display the heightmap using matplotlib and save it in current directory.
    name_to_save =  f"hills_{uuid.uuid4()}_[{round(roughness,3)},{round(frequency,3)},{round(amplitude,3)}].png"
    current_directory = os.path.dirname(os.path.abspath(sys.argv[0]))
    parent_directory = os.path.dirname(current_directory)
    parent_directory = parent_directory + '/models/terrain/terrains_generated/'
    plt.imsave(parent_directory + name_to_save, heightMap, cmap='gray')
    plt.imshow(heightMap, cmap='gray')
    return name_to_save

def octave_perlin_at_coordinate(noise, octaves, persistence, roughness, frequency, amplitude, lacunarity, x, y):
    
    """ This function is used to generate a perlin noise heightmap using the octaves method.
    
     Parameters:
     ----------
        noise (PerlinNoise): PerlinNoise object.
        octaves (int): Number of octaves to use in the perlin noise algorithm.
        frequency (float): Frequency of the perlin noise algorithm.
        amplitude (float): Amplitude of the perlin noise algorithm.
        lacunarity (float): Lacunarity of the perlin noise algorithm.
        persistence (float): Persistence of the perlin noise algorithm.
        x (int): X coordinate of the heightmap.
        y (int): Y coordinate of the heightmap. """

    total = 0 # Total sum of noise
    maxValue = 0  # Used for normalizing result to [0,1]

    # Loop through every octave.
    for i in range(octaves):
        # Perlin(cT_2, cT_3)[i, j] + U(-cT_1, cT_1).
        total +=  amplitude * (noise([x * frequency, y * frequency]) + random.uniform(-roughness, roughness))
        # Update subsequence values
        maxValue  += amplitude # This is used for normalizing the final result
        amplitude *= persistence # Multiply the amplitude by the persistence
        frequency *= lacunarity
    
    # Normalize value
    # Es necesario normalizar entre 0 y 1?
    return total/maxValue


def stairs_generator(stepWidth, stepHeight, heightIncrement, width=129, height=129):

    # Create an empty heightmap array
    heightMap = np.zeros((height, width))
    max = 0
    for x in range(height):
        # If we finished with one step, move on to the next.
        if (x % stepWidth == 0): stepHeight += heightIncrement
        # For each pixel in the current step, fill the heightmap
        for y in range(width):   
            heightMap[y][x] = stepHeight
            if stepHeight > max: max = stepHeight
    # Normalize the heightmap
    for x in range(height):
        for y in range(width):
            heightMap[y][x] /= max
        
    name_to_save = f"stairs_{uuid.uuid4()}_[{round(stepWidth,2)}, {round(stepHeight,2)},{round(heightIncrement,2)}].png"
    current_directory = os.path.dirname(os.path.abspath(sys.argv[0]))
    parent_directory = os.path.dirname(current_directory)
    parent_directory = parent_directory + '/models/terrain/terrains_generated/'
    plt.imsave(parent_directory + name_to_save, heightMap, cmap='gray')
    plt.imshow(heightMap, cmap='gray')
    return name_to_save


"""
PARAMETERS TO DEFINE:

The number of octaves determines how many times I call that function. 

The frequency determines a scaling value I apply to my x,y values before calling it. 

In addition to this we'll usually have persistence and lacunarity parameters, 
determining how the amplitude should diminish and frequency increase as we drop down the octaves. Something like.

Persistence (also called gain) is a value in the range (0, 1) that controls how quickly the later octaves "die out". 
Something around 0.5 is pretty conventional here.

Lacunarity is a value greater than 1 that controls how much finer a scale each subsequent octave should use. 
Something around 2.0 is a conventional choice.


El frecuency (Se controla con lacuranity)se va duplicando por 2
La amplitude (Se controla con persistence) se va diviendo por 2


Frecuency y amplitude valores iniciales.


Ver definicion: https://gamedev.stackexchange.com/questions/197861/how-to-handle-octave-frequency-in-the-perlin-noise-algorithm
Ver imagen: https://adrianb.io/2014/08/09/perlinnoise.html

Necesitamos que la frecuency tenga una pequeña variacion, en numeros enternos se anula el perlin noise.


Default segun picture:

octaves 6
frec 4(debe ser float)
amplitude 128
lacu = 2
persistence = 0.5

print(octaves_perlin(octaves = 8, frequency = 0.08, amplitude = 0.02, lacunarity = 0.2, persistence = 0.2))
"""
