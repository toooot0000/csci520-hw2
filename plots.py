from itertools import repeat
import numpy as np
import matplotlib.pyplot as plt
import sys
from collections.abc import Callable


class Data:
    def __init__(self, title: str, x: list = [], plots: list[list] = [], labels: list[str] = []):
        self.x: list[int] = x
        self.plots: list[list[float]] = plots
        self.labels: list[str] = labels
        self.title: str = title
        pass
    pass


def plot(data: Data):
    x = data.x

    fig = plt.figure()
    fig, ax = plt.subplots()
    for i in range(len(data.plots)):
        # Plot some data on the axes.
        ax.plot(x, data.plots[i], label=data.labels[i])

    ax.set_xlabel("Frame")  # Add an x-label to the axes.
    ax.set_ylabel("Angle/deg")  # Add a y-label to the axes.
    ax.set_title(data.title)  # Add a title to the axes.
    ax.legend()  # Add a legend.
    plt.savefig(f"{data.title}.png")


def readFile(filename: str):
    with open(file=filename, mode="r", encoding="utf-8") as f:
        lines = f.readlines()[3:]
        for i in range(0, len(lines), 30):
            yield parseLine(lines[i:i+30])


class Posture:
    def __init__(self, frame: int, pos: list[float], bones: dict[str:list[float]]):
        self.frame = frame
        self.pos = pos
        self.bones = bones
        pass

    def __str__(self):
        return f"{self.frame}: {self.pos}, {self.bones}"


def parseLine(lines: list[str]):
    frame = int(lines[0])
    bones = dict()
    for line in lines[1:]:
        split = line.split(' ')
        bones[split[0]] = [float(i) for i in split[1:4]]
    pos = bones['root'][3:6]
    return Posture(frame, pos, bones)


def makeData(title: str, comp1: str, comp2: str):
    return Data(title, labels=["Input", comp1, comp2], plots=[[], [], []])


def makeSample(lo: int, hi: int, boneName: str, axisInd: int) -> Callable[Posture, Data, int]:
    def sample(pos: Posture, data: Data, plotsInd: int):
        if(pos.frame not in range(lo, hi+1)):
            return
        data.plots[plotsInd].append(pos.bones[boneName][axisInd])
    return sample


def graph1():
    data = makeData("1# Linear Euler vs. Bezier Euler", "Linear", "Bezier")
    data.x = list(range(600, 801))

    sample = makeSample(600, 800, "lfemur", 0)
    for pos in readFile("131_04-dance.amc"):
        sample(pos, data, 0)

    for pos in readFile("1-l-e.amc"):
        sample(pos, data, 1)

    for pos in readFile("1-b-e.amc"):
        sample(pos, data, 2)

    plot(data)


def graph2():
    data = makeData("2# Slerp Quaternion vs. Bezier Quaternion",
                    "Slerp", "Bezier")
    data.x = list(range(600, 801))

    sample = makeSample(600, 800, "lfemur", 0)
    for pos in readFile("131_04-dance.amc"):
        sample(pos, data, 0)
    for pos in readFile("2-l-q.amc"):
        sample(pos, data, 1)
    for pos in readFile("2-b-q.amc"):
        sample(pos, data, 2)
    plot(data)


def graph3():
    data = makeData("3# Linear Euler vs. Slerp Quaternion",
                    "Euler", "Slerp")
    data.x = list(range(200, 501))

    sample = makeSample(200, 500, "root", 2)
    for pos in readFile("131_04-dance.amc"):
        sample(pos, data, 0)
    for pos in readFile("3-l-e.amc"):
        sample(pos, data, 1)
    for pos in readFile("3-l-q.amc"):
        sample(pos, data, 2)
    plot(data)


def graph4():
    data = makeData("4# Bezier Euler vs. Bezier Quaternion",
                    "Euler", "Slerp")
    data.x = list(range(200, 501))

    sample = makeSample(200, 500, "root", 2)
    for pos in readFile("131_04-dance.amc"):
        sample(pos, data, 0)
    for pos in readFile("4-b-e.amc"):
        sample(pos, data, 1)
    for pos in readFile("4-b-q.amc"):
        sample(pos, data, 2)
    plot(data)


if __name__ == "__main__":
    graph1()
    graph2()
    graph3()
    graph4()
    pass
