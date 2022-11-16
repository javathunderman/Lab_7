import matplotlib.pyplot as plt
import pandas as pd

main_table = pd.read_csv("square.csv")
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(xs=main_table['x'], ys=main_table['y'], zs=main_table['z'])
plt.show()