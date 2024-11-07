n = 4

for i in range(n):
    stars = "*" * (i + 1) + " " * (n - i + 1)
    stars += " " * (n - i -1) + "*" * (i + 1)
    print(stars)
print("**********")
for i in range(n):
    stars = "*" * (n - i) + " " * i
    stars += " " * (i+2) + "*" * (n - i)
    print(stars)
