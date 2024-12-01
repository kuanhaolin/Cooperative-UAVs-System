import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

# Create the figure and axis
fig, ax = plt.subplots(figsize=(10, 6))

# Draw system components as rectangles
components = {
    "姿態角控制": (1, 4),
    "Carrot Chasing": (3, 4),
    "人工勢場法": (5, 4),
    "領導者-跟隨者模型": (3, 2),
    "Unreal Engine\n& AirSim 模擬環境": (3, 0.5)
}
rect_width, rect_height = 1.5, 0.5

for comp, (x, y) in components.items():
    ax.add_patch(plt.Rectangle((x - rect_width / 2, y - rect_height / 2),
                                rect_width, rect_height, fill=True, color="#A9CCE3", edgecolor="black"))
    ax.text(x, y, comp, fontsize=10, ha="center", va="center", wrap=True)

# Add arrows between components
arrows = [
    ((1, 4), (3, 2)),  # 姿態角控制 -> 領導者-跟隨者模型
    ((3, 4), (3, 2)),  # Carrot Chasing -> 領導者-跟隨者模型
    ((5, 4), (3, 2)),  # 人工勢場法 -> 領導者-跟隨者模型
    ((3, 2), (3, 0.5))  # 領導者-跟隨者模型 -> 模擬環境
]

for start, end in arrows:
    ax.add_patch(FancyArrowPatch(start, end, connectionstyle="arc3", arrowstyle="->", color="black", lw=1.5))

# Add a title
ax.set_title("無人機協同控制架構圖", fontsize=14)

# Configure axis
ax.set_xlim(0, 6)
ax.set_ylim(0, 5)
ax.axis("off")

# Display the diagram
plt.tight_layout()
plt.show()
