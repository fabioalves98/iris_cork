
f = open("../yaml/ur10_kinect_caljob.yaml", "r")
lines = f.readlines()



for i in range(0, len(lines)):
    if 'joint_values:' in lines[i]:
        temp = lines[i+1]
        lines[i+1] = lines[i+3]
        lines[i+3] = temp

f.close()





file_write = open("../yaml/ur10_kinect_caljob.yaml", "w")

for l in lines:
    file_write.write(l)
file_write.close()

