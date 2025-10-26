integer = int(input("Enter integer: "))
how = "small" if 1 <=integer<=10 else "medium" if 11<=integer<=20 else "large"
print("The integer is", how)
