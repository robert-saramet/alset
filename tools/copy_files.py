from shutil import copyfile
from os import path
from os.path import normpath

inputPath = r'REPLACE/THIS/WITH/SOURCE/IMAGE/PATH/'
outputPath = r'REPLACE/THIS/WITH/DESTINATION/IMAGE/PATH/'
filename = "output.log"
f = open(filename, "r")

for x in f:
    file = x[:-5] + 'jpg' #change file extension
    src = normpath(inputPath + file)
    dest = normpath(outputPath + file)
    
    if path.exists(src) and not path.exists(dest):
        copyfile(src, dest)
        
f.close()
