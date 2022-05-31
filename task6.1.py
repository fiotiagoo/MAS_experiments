import matplotlib.pyplot as plt
import matplotlib.animation as animation
from boids import *
width, height = 640, 480
N = 50
boids = Boids(N)
    # set up plot
fig = plt.figure(1)
ax = plt.axes(xlim=(0, width), ylim=(0, height))
#paint the birds
pts, = plt.plot([], [], markersize=6, c='b', marker='o', ls='None')
beak, = plt.plot([], [], markersize=2, c='r', marker='>', ls='None')
aveVels = []
aveDists = []
frameNums = []

def update(frameNum, boids):
        # update function for animation
        # print(frameNum)
    if frameNum % 50 == 0:
        aveVels.append(boids.aveVel)
        aveDists.append(boids.aveDist)
        frameNums.append(frameNum)
    boids.move()
    pts.set_data(boids.pos.reshape(2 * boids.N)[::2],
                     boids.pos.reshape(2 * boids.N)[1::2])
    vec = boids.pos + 4 * boids.vel / boids.maxVel
    beak.set_data(vec.reshape(2 * boids.N)[::2],vec.reshape(2 * boids.N)[1::2])
    plt.plot(boids.pos[0][0], boids.pos[0][1], c='y', marker=',', markersize=2)
    plt.plot(boids.pos[10][0], boids.pos[10][1], c='b', marker=',', markersize=2)
    plt.plot(boids.pos[20][0], boids.pos[20][1], c='r', marker=',', markersize=2)
    return pts, beak

#display
anim = animation.FuncAnimation(fig=fig, func=update, fargs=[boids], interval=20)
plt.show()