

import numpy as np

print "begining"



num = np.loadtxt('./dataset/train/test02.txt')

print num.shape

fpa=open('./dataset/train/test02.txt')
indexx=0
result=[]
for linea in fpa.readlines():
    indexx=indexx+1
    result.append(map(float, linea.split(' ')))
    if indexx ==13000:
        break
fpa.close()
# print  indexx
# print  result


xyz=np.array(result)
#print xyz
np.savetxt("./dataset/train/train_50000_02.txt",xyz[0:13000,0:3],fmt='%s',newline='\n')

print "ending"

