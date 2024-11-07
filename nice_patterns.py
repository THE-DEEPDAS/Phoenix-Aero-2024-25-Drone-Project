def pattern1():
    n=14
    for i in range(1,n+1):
        if i%2==1 and i%7!=0:
            print('    *')
        if i%7==0:
            print("*"*9)
pattern1()

def pattern2():
    n = 9  
    for i in range(n):
        for j in range(n):
            if j == 0 or j == n-1 or i == j or i + j == n - 1:
                print('*', end=' ')
            else:
                print(' ', end=' ')
        print()
    
pattern2()





def pattern3():
    n=5
    for i in range(n):
        print(' '*(n-i-1)+'*'*(2*i+1))
    for i in range(1,n):
        print(' '*i+'*'*(2*(n-i)-1))

pattern3()

def pattern4():
    n=4
    for i in range(n):
        spaces=n-i
        betweenspace=2*i-1
        if(i==0):
            print("    1")
        else:
            print(' '*(spaces)+chr(i+49)+' '*(betweenspace)+chr(i+49))
    print("1 2 3 4 5")
pattern4()

def pattern5():
    n=5
    for i in range(n):
        spaces=n-i-1
        betweenspace=2*i-1
        if(chr(65+i)=='A'):
            print("    A")
        else:
            print(' '*spaces+chr(65+i)+' '*betweenspace+chr(65+i))

pattern5()

def pattern6():
    n = 5
    for i in range(1,n+1):
        for j in range(1,i+1):
            print(j, end="")
        print()  
pattern6()

def pattern7():
    n = 5
    for i in range(1,n+1):
        for j in range(1,n+1):
            if i == 1 or i == n or j == 1 or j == n:
                print("*", end="") 
            else:
                print(" ", end="") 
        print()  

pattern7()
