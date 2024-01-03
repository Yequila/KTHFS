import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ma
from matplotlib.widgets import Button

class SignalVisualizer:
    def __init__(self):
        # Set up the figure, the axis, and the plot element
        self.fig, self.ax = plt.subplots()
        self.fig.subplots_adjust(bottom=0.25) 
        self.fig.suptitle('Signal', fontsize=20)
        self.ax.set_xlabel('Time', fontsize=14)
        self.ax.set_ylabel('Signal', fontsize=14)
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1400)
        self.ax.grid(linestyle=':')
        self.line, = self.ax.plot([], [], 'orangered', lw=2)
        self.running = False
        self.t = 0  # Time starts at 0

        # Add buttons for controlling the animation
        self.button_start_ax = self.fig.add_axes([0.59, 0.08, 0.1, 0.05])
        self.button_start = Button(self.button_start_ax, 'Start', color='lightgreen', hovercolor='0.975')

        self.button_stop_ax = self.fig.add_axes([0.7, 0.08, 0.1, 0.05])  # left, bottom, width, height
        self.button_stop = Button(self.button_stop_ax, 'Stop', color='lightgoldenrodyellow', hovercolor='0.975')
        
        self.button_reset_ax = self.fig.add_axes([0.81, 0.08, 0.1, 0.05])
        self.button_reset = Button(self.button_reset_ax, 'Reset', color='lightcoral', hovercolor='0.975')

    def init(self):
        # Initialize an empty plot/frame
        self.line.set_data([], [])
        return self.line,

    def update(self, frame):
        if self.running:
            self.t += 0.01
            lambda_t = 5 * np.sin(2 * np.pi * self.t)
            v = 3 * np.pi * np.exp(-lambda_t)
            x, y = self.line.get_data()
            x.append(self.t)
            y.append(v)
            x_min, x_max = self.ax.get_xlim()
            if self.t >= x_max:
                self.ax.set_xlim(self.t - (x_max - x_min), self.t)
                self.ax.figure.canvas.draw()
            self.line.set_data(x, y)
        return self.line,

    def data_gen(self):
        # Generate data
        t = 0
        while True:
            lambda_t = 5 * np.sin(2 * np.pi * t)
            v = 3 * np.pi * np.exp(-lambda_t)
            yield t, v
            t += 0.01

    def run(self):
        # Start the animation
        self.ani = ma.FuncAnimation(self.fig, self.update, self.data_gen, blit=False,
                                    interval=5, init_func=self.init, repeat=False)
        self.button_start.on_clicked(self.start)
        self.button_stop.on_clicked(self.stop)
        self.button_reset.on_clicked((self.reset))
        plt.show()

    def start(self, event):
        # Stop the animation when the button is clicked
        self.running = True
        self.ani.event_source.start()

    def stop(self, event):
        # Stop the animation when the button is clicked
        self.running = False
        self.ani.event_source.stop()

    def reset(self,event):
        self.running = False
        self.t = 0  # Reset time to 0
        self.line.set_data([], [])
        self.ax.set_xlim(0, 1)
        self.ax.figure.canvas.draw()
        self.ani.event_source.stop()  # Stop the animation

# Create an instance of the SignalVisualizer class
visualizer = SignalVisualizer()
visualizer.run()