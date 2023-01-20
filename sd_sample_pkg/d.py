import numpy as np

cx1, cy1 = 0, 0
cx2, cy2 = 2, 3
cx3, cy3 = 3, 5
data = np.array([[cx1, cy1], [cx2, cy2], [cx3, cy3]])
x = data[:, 0]
y = data[:, 1]
print(x)
print(data[0])
print((data[0] != np.array([0, 0])).all())
k = 0
datanew = []
print(data[k][0])
while k<3:
    if data[k][0] == 0 and data[k][1] == 0:
        k+=1
    else:
        datanew.append(data[k])
        k+=1
x = datanew[0]
print(datanew)
print(x)