import matplotlib.pyplot as plt

def GetPoints(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    points = [[float(coord) for coord in line.strip().split()] for line in lines]

    points.append(points[0])

    x = [point[0] for point in points]
    y = [point[1] for point in points]
    return x, y

if __name__ == "__main__":
    s1x, s1y = GetPoints("s1_hull.txt")
    s2x, s2y = GetPoints("s2_hull.txt")
    
    plt.plot(s1x, s1y)
    plt.fill(s1x, s1y, alpha=0.3) 

    plt.plot(s2x, s2y)
    plt.fill(s2x, s2y, alpha=0.3)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Polygon')

    plt.grid(True)
    plt.axis('equal')

    plt.show()