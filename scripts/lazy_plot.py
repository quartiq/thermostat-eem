
from matplotlib import pyplot as plt
import csv

def main():
    f = open('data.csv', 'r')
    reader = csv.reader(f)
    data = []
    header = next(reader)
    for row in reader:
        data.append(float(row[0]))

    plt.plot(data)
    plt.title("10k REF data")
    plt.xlabel("sample nr")
    plt.ylabel("raw ADC value")
    plt.grid()
    plt.show()



if __name__ == "__main__":
    main()
