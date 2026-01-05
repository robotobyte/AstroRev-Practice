
import random

# Ask user for a valid number
while True:
    n = int(input("Enter number of coin flips: "))
    if n > 0:
        break
    print("Number must be positive. Try again.")

flips = []
for _ in range(n):
    flips.append(random.choice(['H', 'T']))

# Count heads and tails
heads = flips.count('H')
tails = flips.count('T')

# Percentages
heads_pct = (heads / n) * 100
tails_pct = (tails / n) * 100

# Consecutive counts
max_h = 0
max_t = 0
current_h = 0
current_t = 0

for f in flips:
    if f == 'H':
        current_h += 1
        current_t = 0
    else:
        current_t += 1
        current_h = 0
    max_h = max(max_h, current_h)
    max_t = max(max_t, current_t)

# Output
print("Flips:", ''.join(flips))
print("Heads:", heads)
print("Tails:", tails)
print(f"Heads percentage: {heads_pct:.2f}%")
print(f"Tails percentage: {tails_pct:.2f}%")
print("Most consecutive heads:", max_h)
print("Most consecutive tails:", max_t)1
