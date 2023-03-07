from itertools import repeat
import numpy as np
import matplotlib.pyplot as plt
import sys


class Data:
    def __init__(self, title: str, x: list = [], plots: list[list] = [], labels: list[str] = []):
        self.x = x
        self.plots = plots
        self.labels = labels
        self.title = title
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
    plt.savefig("CPUPlot.png")


def readFile(filename: str, data: Data):
    with open(file=filename, mode="r", encoding="utf-8") as f:
        lines = f.readlines()[4:]
        for i in range(0, len(lines), 30):
            yield parseLine(lines[i:i+30])


class Posture:
    def __init__(self, frame: int, pos: list[float], bones: dict[str:list[float]]):
        self.frame = frame
        self.pos = pos
        self.bones = bones
        pass


def parseLine(lines: list[str]):
    frame = int(lines[0])
    bones = dict()
    for line in lines[1:]:
        split = line.split(' ')
        bones[split[0]] = [float(i) for i in split[1:]]
    pos = bones['root'][3:6]
    return Posture(frame, pos, bones)


def makeData(title: str, comp1: str, comp2: str):
    return Data(title, labels=["Input", comp1, comp2], plots=[[], [], []])


if __name__ == "__main__":
    pass
