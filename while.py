#transpose using for loop
matrix=[[1,2,3],[4,5,6],[7,8,9]]
transpose=[[0,0,0],[0,0,0],[0,0,0]]
for i in range (3):
    for j in range(3):
        transpose[i][j] = matrix[j][i]
print(transpose)

#decimal to binary
decimal=int(input("enter a decimal : "))
binary=""
while decimal>0:
    binary+=str(decimal%2)
    decimal=decimal//2
print(binary)

#sum of columns in matrix

for i in range (3):
    sum=0
    for j in range(3):
        sum+=matrix[j][i]
    print(sum,end=" ")   

#pattern
rows=6
for i in range(rows):
    start = 1 if i%2==0 else 0
    string=""
    for j in range (i):
        next = 0 if start==1 else 1
        if(j%2==0):
            string += (str(start))
        else:
            string += (str(next))
    print(string)
    print()