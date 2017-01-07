import numpy as np
print("   ", end='')
nums = []
for idx, deg in enumerate(range(360)):
    x = np.sin(deg * np.pi/180)
    y = 1024 * (x+1) + 1024
    nums.append(y)
    print("{: 5d},".format(int(y)), end='')
    if idx % 12 == 11:
        print("\n   ", end='')

import matplotlib.pyplot as plt
plt.plot(nums)
plt.show()
