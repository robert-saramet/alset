import csv
from os import path
from os.path import normpath
from PIL import Image

posFile = open('pos.csv', 'r')
i = 'StopSign/'  # Original image path
o = 'Cropped/'  # Cropped image path

readCsv = csv.DictReader(posFile)  # Read CSV to dictionary
for row in readCsv:
    filename = row['filename']
    inPath = normpath(i + filename)
    outPath = normpath(o + filename)

    if path.exists(inPath) and not path.exists(outPath):
        img = Image.open(normpath(inPath))
        coords = (int(row['xMin']), int(row['yMin']), int(row['xMax']), int(row['yMax']))  # Left, top, right, bottom
        cropped = img.crop(coords)
        cropped.save(outPath)

posFile.close()
