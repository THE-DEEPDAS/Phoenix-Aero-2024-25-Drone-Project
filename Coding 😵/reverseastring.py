list=[1,2,4,3,3,1,4,4,1,2,24]
n=len(list)
max=list[0]
for i in list:
    if i>=max:
        max=i
print("max number is : ",max)
