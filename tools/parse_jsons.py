import os
import json

stop_sign_labels = [
	# whatever the label in the json file is
]
def parse_json(filename):
	objects = []
	label = ""
	x = 0
	y = 0
	w = 0
	h = 0

	f = open(filename, "r")
	data = f.read()
	j = json.loads(data)
	for o in j["objects"]:
		label = o["label"]
		x = int(o["bbox"]["xmin"])
		x1 = int(o["bbox"]["xmax"])
		y = int(o["bbox"]["ymin"])
		y1 = int(o["bbox"]["ymax"])
		w = x1 - x
		h = y1 - y
		objects.append((label, x, y, w, h))
	return objects


# initialize output_filename with whatever filename you want (pos.txt recommanded)
output_filename = "pos.txt"
output_file = open(output_filename, "w")

# assumes that there is a folder "jsons" next to the positives folder
d = "../jsons"
i = 0
maxi = 43000
for f in os.scandir(d):
	stop_sign_num = 0
	line = ""
	objects = parse_json(d + "/" + f.name)
	# Check if the image contains a stop sign
	for o in objects:
		if o[0] == stop_sign_labels[0] or o[0] == stop_sign_labels[1]:
			stop_sign_num += 1
	
	# If there is a stop sign, create and push a new line
	if stop_sign_num != 0:
		fn = "../pos/" + os.path.splitext(f.name)[0] + ".jpg " 
		if os.path.isfile(fn):
			print(int(i / 43000 * 100), "%: ")
			line = fn + str(stop_sign_num)
			for o in objects:
				label, x, y, w, h = o
				if label == stop_sign_labels[0] or label == stop_sign_labels[1]:
					line += " " + str(x) + " " + str(y) + " " + str(w) + " " + str(h)
			line += "\n"
			print(line[0:-1])
			output_file.write(line)
	i += 1

# Negatives
d = "../neg"
neg_out_file_name = "neg.txt"
neg_out_file = open(neg_out_file_name, "w")
for f in os.scandir(d):
	neg_out_file.write(d + "/" + f.name + "\n")
	print(f.name[0:-1])
