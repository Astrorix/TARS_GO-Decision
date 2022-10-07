import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def readgrid():
    grid = []
    # with open('E:\\Grid.txt', 'r') as file:
    with open('Grid.data', 'r') as file:
        data = file.readlines()
        # print(data[0].split())
        # print(type(data[0]))
        x_line = float(data[0])
        y_line = float(data[1])
        disperse = float(data[2])
        # print(x_line)
    for line in data[3:]:
        if(line != '\n'):
            # print(grid_data)
            grid_data = float(line.strip())
            print(grid_data)
            grid.append(grid_data)
    return x_line, y_line, disperse, grid


def plot(x: numpy.ndarray, y: numpy.ndarray, grid: numpy.ndarray):
    xx, yy = numpy.meshgrid(x, y)
    # 等势图
    plt.contourf(x, y, grid)
    # ax.contourf(xx,yy,grid,zdir='z', offset=-2, cmap=plt.get_cmap('rainbow'))
    # grid梯度图
    fig = plt.figure()
    ax = Axes3D(fig)
    plt.xlabel(r'x_line', fontsize=20, color='cyan')
    plt.ylabel(r'y_line', fontsize=20, color='cyan')
    ax.plot_surface(xx, yy, grid, rstride=1, cstride=1,
                    cmap=plt.get_cmap('rainbow'))
    plt.show()


if __name__ == "__main__":
    x_line, y_line, disperse, grids = readgrid()
    x = numpy.arange(0, x_line*disperse, disperse)
    y = numpy.arange(0, y_line*disperse, disperse)
    grid = numpy.array(grids).reshape(int(y_line), int(x_line))
    plot(  x, y,grid)