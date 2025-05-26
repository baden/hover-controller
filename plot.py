import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

# 🔧 Налаштування
SERIAL_PORT = '/dev/tty.usbmodem22302'          # Заміни на свій порт, напр. /dev/ttyUSB0
BAUD_RATE = 115200
MAX_DATA_POINTS = 200         # Макс. кількість точок на графіку

# 🌀 Порт
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# 📊 Дані
data_buffers = {}
axes = {}
lines = {}
line_pattern = re.compile(r'(\w+):([+-]?(\d+(\.\d*)?|\.\d+))')

# 🎨 Початкова фігура
fig, axs = plt.subplots(nrows=1, ncols=1)
fig.subplots_adjust(hspace=0.5)

def update(frame):
    global data_buffers, axes, lines, fig, axs

    try:
        line = ser.readline().decode('utf-8').strip()
        matches = line_pattern.findall(line)

        if not matches:
            return

        changed = False

        for var, value, *_ in matches:
            value = float(value)
            if var not in data_buffers:
                # Нова змінна: створити subplot
                data_buffers[var] = deque(maxlen=MAX_DATA_POINTS)

                # Перебудувати subplot-и
                fig.clf()
                axs = fig.subplots(nrows=len(data_buffers), ncols=1, sharex=True)
                if len(data_buffers) == 1:
                    axs = [axs]

                lines.clear()
                for i, (vname, buf) in enumerate(data_buffers.items()):
                    axes[vname] = axs[i]
                    lines[vname], = axs[i].plot([], [], label=vname)
                    axs[i].set_ylabel(vname)
                    axs[i].legend(loc='upper left')

                changed = True

            data_buffers[var].append(value)

        if changed:
            return  # дочекаймось наступного кадру з новою графікою

        for var, buf in data_buffers.items():
            y_data = list(buf)
            x_data = list(range(len(y_data)))
            lines[var].set_data(x_data, y_data)
            axes[var].relim()
            axes[var].autoscale_view()

    except Exception as e:
        print(f"Помилка: {e}")

ani = animation.FuncAnimation(fig, update, interval=100)
plt.tight_layout()
plt.show()
