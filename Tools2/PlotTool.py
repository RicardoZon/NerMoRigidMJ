import matplotlib.pyplot as plt

def plot_simple(datas: list, title=None, leg=None, xlabel="X", ylabel="Y", figsize=(10, 5)):
    plt.figure(figsize=figsize)
    for data in datas:
        plt.plot(data)
    plt.xlabel(xlabel)  # 为x轴命名为“x”
    plt.ylabel(ylabel)  # 为y轴命名为“y”
    if title is not None:
        plt.title(title)
    if leg is not None:
        plt.legend(leg)
    # plt.axis('equal')
    plt.grid()
    plt.show()